#include <camodocal/pose_graph/PoseGraph.h>

#include <boost/thread.hpp>
#include <camodocal/sparse_graph/SparseGraphUtils.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/core/eigen.hpp>

#include "../gpl/EigenQuaternionParameterization.h"
#include "../location_recognition/LocationRecognition.h"
#include "../pose_estimation/P3P.h"
#include "PoseGraphError.h"

#ifdef VCHARGE_VIZ
#include "../../../../visualization/overlay/GLOverlayExtended.h"
#endif

namespace camodocal
{

PoseGraph::PoseGraph(CameraSystem& cameraSystem,
                     SparseGraph& graph,
                     float maxDistanceRatio,
                     int minLoopCorrespondences2D3D,
                     int nImageMatches)
 : m_cameraSystem(cameraSystem)
 , m_graph(graph)
 , k_lossWidth(0.01)
 , k_minLoopCorrespondences2D3D(minLoopCorrespondences2D3D)
 , k_maxDistanceRatio(maxDistanceRatio)
 , k_nImageMatches(nImageMatches)
 , m_verbose(false)
{

}

void
PoseGraph::setVerbose(bool onoff)
{
    m_verbose = onoff;
}

void
PoseGraph::buildEdges(void)
{
    if (m_verbose)
    {
        std::cout << "# INFO: Building odometry edges..." << std::endl;
    }

    m_odometryEdges = findOdometryEdges();

    if (m_verbose)
    {
        std::cout << "# INFO: Built " << m_odometryEdges.size() << " odometry edges." << std::endl;
    }

    if (m_verbose)
    {
        std::cout << "# INFO: Building loop closure edges..." << std::endl;
    }

    findLoopClosures(m_loopClosureEdges, m_correspondences2D3D);

    if (m_verbose)
    {
        std::cout << "# INFO: Built " << m_loopClosureEdges.size() << " loop closure edges." << std::endl;
    }

    m_loopClosureEdgeSwitches.assign(m_loopClosureEdges.size(), ON);
}

void
PoseGraph::optimize(bool useRobustOptimization)
{
    // This implementation is based on the following paper:
    // G.H. Lee, F. Fraundorfer, and M. Pollefeys,
    // Robust Pose-Graph Loop-Closures with Expectation-Maximization,
    // In International Conference on Intelligent Robots and Systems, 2013.
    if (useRobustOptimization)
    {
        for (int i = 0; i < 20; ++i)
        {
            if (!iterateEM(true))
            {
#ifdef VCHARGE_VIZ
                visualizeLoopClosureEdges();
#endif
                return;
            }
        }
    }
    else
    {
        iterateEM(false);
    }
}

std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> >
PoseGraph::getCorrespondences2D3D(void) const
{
    // return 2D-3D correspondences from switched-on loop closure edges
    std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > correspondences2D3D;

    for (size_t i = 0; i < m_correspondences2D3D.size(); ++i)
    {
        if (m_loopClosureEdgeSwitches.at(i) != ON)
        {
            continue;
        }

        correspondences2D3D.insert(correspondences2D3D.end(),
                                   m_correspondences2D3D.at(i).begin(),
                                   m_correspondences2D3D.at(i).end());
    }

    return correspondences2D3D;
}

std::vector<PoseGraph::Edge, Eigen::aligned_allocator<PoseGraph::Edge> >
PoseGraph::findOdometryEdges(void) const
{
    std::vector<PoseGraph::Edge, Eigen::aligned_allocator<PoseGraph::Edge> > edges;

    for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
    {
        const FrameSetSegment& segment = m_graph.frameSetSegment(i);

        if (segment.size() <= 1)
        {
            continue;
        }

        for (size_t j = 0; j < segment.size() - 1; ++j)
        {
            edges.push_back(Edge(segment.at(j)->systemPose(),
                                 segment.at(j + 1)->systemPose()));

            edges.back().type() = EDGE_ODOMETRY;

            Eigen::Matrix4d H_01 = segment.at(j + 1)->odometryMeasurement()->toMatrix().inverse() *
                                   segment.at(j)->odometryMeasurement()->toMatrix();

            edges.back().property().rotation() = Eigen::Quaterniond(H_01.block<3,3>(0,0));
            edges.back().property().translation() = H_01.block<3,1>(0,3);
            edges.back().weight().assign(6, 1.0);
        }
    }

    return edges;
}

void
PoseGraph::findLoopClosures(std::vector<PoseGraph::Edge, Eigen::aligned_allocator<PoseGraph::Edge> >& loopClosureEdges,
                            std::vector<std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > >& correspondences2D3D,
                            double reprojErrorThresh) const
{
    boost::shared_ptr<LocationRecognition> locRec(new LocationRecognition);
    locRec->setup(m_graph);

    for (int i = 0; i < (int)m_graph.frameSetSegments().size(); ++i)
    {
        const FrameSetSegment& segment = m_graph.frameSetSegment(i);

        for (int j = 0; j < (int)segment.size(); ++j)
        {
            const FrameSetPtr& frameSet = segment.at(j);

            std::vector<boost::shared_ptr<boost::thread> > threads(frameSet->frames().size());
            std::vector<PoseGraph::Edge> edges(frameSet->frames().size());
            /// @todo std::vector<std::vector<>> will be quite slow! consider boost::static_vector<boost::static_vector<>> or a single 2d data type allocation
            std::vector<std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > > corr2D3D(frameSet->frames().size());

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                const FramePtr& frame = frameSet->frames().at(k);

                if (frame.get() == 0)
                {
                    continue;
                }

                FrameTag frameTag;
                frameTag.frameSetSegmentId = i;
                frameTag.frameSetId = j;
                frameTag.frameId = k;

                threads[k].reset(new boost::thread(boost::bind(&PoseGraph::findLoopClosuresHelper, this,
                                                               frameTag, locRec,
                                                               &edges[k], &corr2D3D[k],
                                                               reprojErrorThresh)));
            }

            for (int k = 0; k < (int)frameSet->frames().size(); ++k)
            {
                if (threads[k].get() == 0)
                {
                    continue;
                }
                threads[k]->join();

                if (!corr2D3D[k].empty())
                {
                    loopClosureEdges.push_back(edges[k]);
                    correspondences2D3D.push_back(corr2D3D[k]);
                }
            }
        }
    }
}

void
PoseGraph::findLoopClosuresHelper(FrameTag frameTagQuery,
                                  boost::shared_ptr<LocationRecognition> locRec,
                                  PoseGraph::Edge* edge,
                                  std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> >* correspondences2D3D,
                                  double reprojErrorThresh) const
{
    FramePtr& frameQuery = m_graph.frameSetSegment(frameTagQuery.frameSetSegmentId).at(frameTagQuery.frameSetId)->frames().at(frameTagQuery.frameId);

    if (frameQuery.get() == 0)
    {
        return;
    }

    Pose T_cam_odo(m_cameraSystem.getGlobalCameraPose(frameQuery->cameraId()));

    // find closest matching images
    std::vector<FrameTag> frameTags;
    locRec->knnMatch(frameQuery, k_nImageMatches, frameTags);

    std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > corr2D3DBest;
    Transform transformBest;
    FramePtr frameBest;
    FrameTag frameTagBest;
    for (size_t i = 0; i < frameTags.size(); ++i)
    {
        FrameTag frameTag = frameTags.at(i);

        const FramePtr& frame = m_graph.frameSetSegment(frameTag.frameSetSegmentId).at(frameTag.frameSetId)->frames().at(frameTag.frameId);

        // find 2D-3D correspondences between frame pair
        std::vector<cv::DMatch> matches = matchFeatures(frame->features2D(), frameQuery->features2D(), k_maxDistanceRatio);

        if ((int)matches.size() < k_minLoopCorrespondences2D3D)
        {
            continue;
        }

        // find camera pose from P3P RANSAC
        Eigen::Matrix4d H;
        std::vector<cv::DMatch> inliers;
        solveP3PRansac(frame, frameQuery, matches,
                       H, inliers, reprojErrorThresh);

        std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > corr2D3D;
        for (size_t j = 0; j < inliers.size(); ++j)
        {
            const cv::DMatch& match = inliers.at(j);

            Point3DFeaturePtr& p3D = frame->features2D().at(match.queryIdx)->feature3D();
            Point2DFeaturePtr& p2D = frameQuery->features2D().at(match.trainIdx);

            corr2D3D.push_back(std::make_pair(p2D, p3D));
        }

        int nInliers = inliers.size();

        if (nInliers < k_minLoopCorrespondences2D3D)
        {
            continue;
        }

        if (nInliers > (int)corr2D3DBest.size())
        {
            frameBest = frame;
            frameTagBest = frameTag;

            // compute loop closure constraint
            Eigen::Matrix4d H_01 = frame->systemPose()->toMatrix().inverse() * H;

            transformBest.rotation() = Eigen::Quaterniond(H_01.block<3,3>(0,0));
            transformBest.translation() = H_01.block<3,1>(0,3);

            corr2D3DBest = corr2D3D;
        }
    }

    if (!corr2D3DBest.empty())
    {
        edge->inVertex() = frameQuery->systemPose();
        edge->outVertex() = frameBest->systemPose();
        edge->type() = EDGE_LOOP_CLOSURE;
        edge->property() = transformBest;

        edge->weight().assign(6, 1.0);

        *(correspondences2D3D) = corr2D3DBest;

        if (m_verbose)
        {
            std::cout << "# INFO: Image match: " << frameTagQuery.frameSetSegmentId
                      << "," << frameTagQuery.frameSetId
                      << "," << frameTagQuery.frameId
                      << " -> " << frameTagBest.frameSetSegmentId
                      << "," << frameTagBest.frameSetId
                      << "," << frameTagBest.frameId
                      << " with " << correspondences2D3D->size()
                      << " 2D-3D correspondences." << std::endl;
        }
    }
}

bool
PoseGraph::iterateEM(bool useRobustOptimization)
{
    ceres::Problem problem;

    // odometry edges
    for (size_t i = 0; i < m_odometryEdges.size(); ++i)
    {
        Edge& edge = m_odometryEdges.at(i);

        ceres::CostFunction* costFunction =
            new ceres::AutoDiffCostFunction<PoseGraphError, 6, 3, 3, 3, 3>(
                new PoseGraphError(edge.property(), edge.weight()));

        OdometryPtr pose1, pose2;
        pose1 = edge.inVertex().lock();
        pose2 = edge.outVertex().lock();

        problem.AddResidualBlock(costFunction, NULL,
                                 pose1->positionData(), pose1->attitudeData(),
                                 pose2->positionData(), pose2->attitudeData());

        if (i == 0)
        {
            problem.SetParameterBlockConstant(pose1->positionData());
            problem.SetParameterBlockConstant(pose1->attitudeData());
        }
    }

    // loop closure edges
    for (size_t i = 0; i < m_loopClosureEdges.size(); ++i)
    {
        if (m_loopClosureEdgeSwitches.at(i) != ON)
        {
            continue;
        }

        Edge& edge = m_loopClosureEdges.at(i);

        ceres::CostFunction* costFunction =
            new ceres::AutoDiffCostFunction<PoseGraphError, 6, 3, 3, 3, 3>(
                new PoseGraphError(edge.property(), edge.weight()));

        OdometryPtr pose1, pose2;
        pose1 = edge.inVertex().lock();
        pose2 = edge.outVertex().lock();

        ceres::LossFunction* lossFunction = 0;
        if (useRobustOptimization)
        {
            lossFunction = new ceres::CauchyLoss(k_lossWidth);
        }

        problem.AddResidualBlock(costFunction, lossFunction,
                                 pose1->positionData(), pose1->attitudeData(),
                                 pose2->positionData(), pose2->attitudeData());
    }

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (m_verbose)
    {
        std::cout << summary.BriefReport() << std::endl;
    }

    int nIterations = summary.num_successful_steps + summary.num_unsuccessful_steps;

    if (nIterations != 0 && useRobustOptimization)
    {
        classifySwitches();
    }

    return (nIterations != 0);
}

void
PoseGraph::classifySwitches(void)
{
    size_t nSwitchesOn = 0;
    std::map<double, EdgeSwitchState*> edgeSwitchMap;

    for (size_t i = 0; i < m_loopClosureEdges.size(); ++i)
    {
        if (m_loopClosureEdgeSwitches.at(i) == DISABLED)
        {
            continue;
        }

        Edge& edge = m_loopClosureEdges.at(i);

        OdometryPtr pose1, pose2;
        pose1 = edge.inVertex().lock();
        pose2 = edge.outVertex().lock();

        Eigen::Matrix4d H_01 = pose2->toMatrix().inverse() * pose1->toMatrix();
        Eigen::Matrix4d H_01_meas = edge.property().toMatrix();

        // compute error
        Eigen::Matrix4d H_err = H_01_meas.inverse() * H_01;

        Eigen::Matrix3d R_err = H_err.block<3,3>(0,0);
        double r, p, y;
        mat2RPY(R_err, r, p, y);

        Eigen::Matrix<double,6,1> vec_err;
        vec_err.topRows(3) << r, p, y;
        vec_err.bottomRows(3) = H_err.block<3,1>(0,3);

        // compute weight
        double l2 = k_lossWidth * k_lossWidth;
        double w = l2 / (l2 + vec_err.transpose() * vec_err);

        if (w < 0.001 * (k_lossWidth / 0.01))
        {
            m_loopClosureEdgeSwitches.at(i) = OFF;
        }
        else
        {
            m_loopClosureEdgeSwitches.at(i) = ON;

            ++nSwitchesOn;
        }

        edgeSwitchMap.insert(std::make_pair(w, &(m_loopClosureEdgeSwitches.at(i))));
    }

    // disable edge switches corresponding to the first 10% of edges
    // sorted by increasing weight
    size_t nMaxSwitchesDisabled = edgeSwitchMap.size() / 10;
    int nSwitchesDisabled = 0;
    for (std::map<double, EdgeSwitchState*>::iterator it = edgeSwitchMap.begin();
         it != edgeSwitchMap.end(); ++it)
    {
        if (*(it->second) == ON)
        {
            break;
        }

        *(it->second) = DISABLED;

        ++nSwitchesDisabled;
        if (nSwitchesDisabled >= (int)nMaxSwitchesDisabled)
        {
            break;
        }
    }

    if (m_verbose)
    {
        std::cout << "# INFO: # switches turned on: " << nSwitchesOn
                  << "/" << edgeSwitchMap.size() << std::endl;
        std::cout << "# INFO: # switches disabled: " << nSwitchesDisabled << std::endl;
    }
}

void
PoseGraph::solveP3PRansac(const FrameConstPtr& frame1,
                          const FrameConstPtr& frame2,
                          const std::vector<cv::DMatch>& matches,
                          Eigen::Matrix4d& H,
                          std::vector<cv::DMatch>& inliers,
                          double reprojErrorThresh) const
{
    inliers.clear();

    double p = 0.99; // probability that at least one set of random samples does not contain an outlier
    double v = 0.6; // probability of observing an outlier

    double u = 1.0 - v;
    int N = static_cast<int>(log(1.0 - p) / log(1.0 - u * u * u) + 0.5);

    std::vector<size_t> indices;
    for (size_t i = 0; i < matches.size(); ++i)
    {
        indices.push_back(i);
    }

    const std::vector<Point2DFeaturePtr>& features1 = frame1->features2D();
    const std::vector<Point2DFeaturePtr>& features2 = frame2->features2D();

    const CameraConstPtr& camera2 = m_cameraSystem.getCamera(frame2->cameraId());

    // run RANSAC to find best H
    Eigen::Matrix4d H_best;
    std::vector<size_t> inlierIds_best;
    for (int i = 0; i < N; ++i)
    {
        std::random_shuffle(indices.begin(), indices.end());

        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > rays(3);
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > worldPoints(3);
        for (int j = 0; j < 3; ++j)
        {
            const cv::DMatch& match = matches.at(indices.at(j));

            worldPoints.at(j) = features1.at(match.queryIdx)->feature3D()->point();

            const cv::KeyPoint& kpt2 = features2.at(match.trainIdx)->keypoint();

            Eigen::Vector3d ray;
            camera2->liftSphere(Eigen::Vector2d(kpt2.pt.x, kpt2.pt.y), ray);

            rays.at(j) = ray;
        }

        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > solutions;
        if (!solveP3P(rays, worldPoints, solutions))
        {
            continue;
        }

        for (size_t j = 0; j < solutions.size(); ++j)
        {
            Eigen::Matrix4d H_inv = solutions.at(j).inverse();

            std::vector<size_t> inliersIds;
            for (size_t k = 0; k < matches.size(); ++k)
            {
                const cv::DMatch& match = matches.at(k);

                Eigen::Vector3d P1 = features1.at(match.queryIdx)->feature3D()->point();

                Eigen::Vector3d P2 = transformPoint(H_inv, P1);
                Eigen::Vector2d p2_pred;
                camera2->spaceToPlane(P2, p2_pred);

                const Point2DFeatureConstPtr& f2 = features2.at(match.trainIdx);

                double err = hypot(f2->keypoint().pt.x - p2_pred(0),
                                   f2->keypoint().pt.y - p2_pred(1));
                if (err > reprojErrorThresh)
                {
                    continue;
                }

                inliersIds.push_back(k);
            }

            if (inliersIds.size() > inlierIds_best.size())
            {
                H_best = solutions.at(j);
                inlierIds_best = inliersIds;
            }
        }
    }

    for (size_t i = 0; i < inlierIds_best.size(); ++i)
    {
        inliers.push_back(matches.at(inlierIds_best.at(i)));
    }

    H = H_best * m_cameraSystem.getGlobalCameraPose(frame2->cameraId()).inverse();
}

cv::Mat
PoseGraph::buildDescriptorMat(const std::vector<Point2DFeaturePtr>& features,
                              std::vector<size_t>& indices,
                              bool hasScenePoint) const
{
    for (size_t i = 0; i < features.size(); ++i)
    {
        if (hasScenePoint && !features.at(i)->feature3D())
        {
            continue;
        }

        indices.push_back(i);
    }

    cv::Mat dtor(indices.size(), features.at(0)->descriptor().cols, features.at(0)->descriptor().type());

    for (size_t i = 0; i < indices.size(); ++i)
    {
        features.at(indices.at(i))->descriptor().copyTo(dtor.row(i));
    }

    return dtor;
}

std::vector<cv::DMatch>
PoseGraph::matchFeatures(const std::vector<Point2DFeaturePtr>& features1,
                         const std::vector<Point2DFeaturePtr>& features2,
                         float maxDistanceRatio) const
{
    cv::BFMatcher descriptorMatcher(cv::NORM_L2, false);

    std::vector<size_t> indices1, indices2;
    cv::Mat dtor1 = buildDescriptorMat(features1, indices1, true);
    cv::Mat dtor2 = buildDescriptorMat(features2, indices2, false);

    std::vector<std::vector<cv::DMatch> > candidateMatches;
    descriptorMatcher.knnMatch(dtor1, dtor2, candidateMatches, 2);

    std::vector<cv::DMatch> matches;
    for (size_t i = 0; i < candidateMatches.size(); ++i)
    {
        std::vector<cv::DMatch>& match = candidateMatches.at(i);

        if (match.size() < 2)
        {
            continue;
        }

        float distanceRatio = match.at(0).distance / match.at(1).distance;

        if (distanceRatio < maxDistanceRatio)
        {
            matches.push_back(cv::DMatch(indices1.at(match.at(0).queryIdx),
                                         indices2.at(match.at(0).trainIdx),
                                         match.at(0).distance));
        }
    }

    return matches;
}

#ifdef VCHARGE_VIZ

void
PoseGraph::visualizeLoopClosureEdges(void)
{
    vcharge::GLOverlayExtended overlay("loop-closure-edges", VCharge::COORDINATE_FRAME_LOCAL);

    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    if (OdometryPtr o = m_odometryEdges.front().inVertex().lock())
    {
        origin = o->position();
    }

    overlay.setOrigin(origin(0), origin(1), origin(2));

    // visualize loop closure edges
    overlay.clear();
    overlay.lineWidth(2.0f);

    for (size_t i = 0; i < m_loopClosureEdges.size(); ++i)
    {
        Edge& edge = m_loopClosureEdges.at(i);

        OdometryPtr pose1, pose2;
        pose1 = edge.inVertex().lock();
        pose2 = edge.outVertex().lock();

        if (m_loopClosureEdgeSwitches.at(i) == ON)
        {
            overlay.color3f(0.0f, 1.0f, 0.0f);
        }
        else
        {
            overlay.color3f(1.0f, 0.0f, 0.0f);
        }

        overlay.begin(VCharge::LINES);
        overlay.vertex3d(pose1->x() - origin(0), pose1->y() - origin(1), pose1->z() - origin(2));
        overlay.vertex3d(pose2->x() - origin(0), pose2->y() - origin(1), pose2->z() - origin(2));
        overlay.end();
    }

    overlay.publish();
}

#endif

}
