#include "CameraRigBA.h"

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/unordered_set.hpp>
#include <camodocal/calib/PlanarHandEyeCalibration.h>
#include <camodocal/pose_graph/PoseGraph.h>
#include <camodocal/sparse_graph/SparseGraphUtils.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ceres/ceres.h"
#include "ceres/covariance.h"
#include "../camera_models/CostFunctionFactory.h"
#include "../features2d/SurfGPU.h"
#include "../gpl/EigenQuaternionParameterization.h"
#include "../gpl/EigenUtils.h"
#include "../location_recognition/LocationRecognition.h"
#include "../npoint/five-point/five-point.hpp"
#include "../visual_odometry/SlidingWindowBA.h"

#ifdef VCHARGE_VIZ
#include "../../../../library/gpl/CameraEnums.h"
#include "../../../../visualization/overlay/GLOverlayExtended.h"
#endif

namespace camodocal
{

CameraRigBA::CameraRigBA(CameraSystem& cameraSystem,
                         SparseGraph& graph)
 : m_cameraSystem(cameraSystem)
 , m_graph(graph)
 , k_localMapWindowDistance(3.0)
 , k_maxDistanceRatio(0.7f)
 , k_maxPoint3DDistance(20.0)
 , k_maxReprojErr(2.0)
 , k_minLoopCorrespondences2D3D(50)
 , k_minInterCorrespondences2D2D(8)
 , k_nearestImageMatches(15)
 , k_nominalFocalLength(300.0)
 , m_verbose(false)
{

}

void
CameraRigBA::run(int beginStage, bool optimizeIntrinsics,
                 bool saveWorkingData, std::string dataDir)
{
    // stage 1 - triangulate 3D points with feature correspondences from mono VO and run BA
    // stage 2 - run robust pose graph SLAM and find inlier 2D-3D correspondences from loop closures
    // stage 3 - find local inter-camera 3D-3D correspondences
    // stage 4 - run BA
    // stage 5 - run hand-eye calibration

    if (m_verbose)
    {
        std::cout << "# INFO: # segments = " << m_graph.frameSetSegments().size() << std::endl;
        for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
        {
            std::cout << "# INFO:   Segment " << i << ": # frame sets = " << m_graph.frameSetSegment(i).size() << std::endl;
        }
    }

#ifdef VCHARGE_VIZ
    // visualize graph produced by previous stage
    switch (beginStage)
    {
    case 1:
        visualize("unopt-", ODOMETRY);
        visualizeExtrinsics("unopt-extrinsics");
        break;
    case 2:
        visualize("opt1-BA-", ODOMETRY);
        visualizeExtrinsics("opt1-extrinsics");
        break;
    case 3:
        visualize("opt2-BA-", ODOMETRY);
        visualizeExtrinsics("opt2-extrinsics");
        break;
    case 4:
        visualize("opt3-BA-", ODOMETRY);
        visualizeExtrinsics("opt3-extrinsics");
        break;
    default:
        visualize("opt4-BA-", ODOMETRY);
        visualizeExtrinsics("opt4-extrinsics");
    }

//    m_cameraSystem.readFromFile("../config/calib/calib_camera_kermit/2012_09_03/extrinsic.txt");
//    visualizeExtrinsics("ref-extrinsics");

#endif

    // stage 1
    if (beginStage <= 1)
    {
        prune(PRUNE_BEHIND_CAMERA, CAMERA);

        if (m_verbose)
        {
            std::cout << "# INFO: Triangulating feature correspondences... " << std::endl;
        }

        triangulateFeatureCorrespondences();

        if (m_verbose)
        {
            std::cout << "# INFO: Checking the validity of the graph..." << std::endl;
        }

        if (!validateGraph())
        {
            std::cout << "# ERROR: Graph is not valid." << std::endl;
            exit(1);
        }

        if (m_verbose)
        {
            std::cout << "# INFO: Finished checking the validity of the graph." << std::endl;
        }

        prune(PRUNE_BEHIND_CAMERA, ODOMETRY);

        double minError, maxError, avgError;
        size_t featureCount;

        reprojectionError(minError, maxError, avgError, featureCount, ODOMETRY);

        if (m_verbose)
        {
            std::cout << "# INFO: Reprojection error after triangulation: avg = " << avgError
                      << " px | max = " << maxError << " px" << std::endl;
            std::cout << "# INFO: # 2D feature points: " << featureCount << std::endl;
            std::cout << "# INFO: # 3D scene points: " << m_graph.scenePointCount() << std::endl;
        }

#ifdef VCHARGE_VIZ
        visualize("unopt-", ODOMETRY);
#endif

        if (m_verbose)
        {
            std::cout << "# INFO: Running BA on odometry data... " << std::endl;
        }

        // optimize camera extrinsics and 3D scene points
        optimize(CAMERA_ODOMETRY_EXTRINSICS | POINT_3D, false);
//        optimize(POINT_3D, false);

        prune(PRUNE_BEHIND_CAMERA, ODOMETRY); // | PRUNE_FARAWAY | PRUNE_HIGH_REPROJ_ERR, ODOMETRY);

        reprojectionError(minError, maxError, avgError, featureCount, ODOMETRY);

        if (m_verbose)
        {
            std::cout << "# INFO: Done." << std::endl;
            std::cout << "# INFO: Reprojection error after BA (odometry): avg = " << avgError
                      << " px | max = " << maxError << " px" << std::endl;
            std::cout << "# INFO: # 2D feature points: " << featureCount << std::endl;
            std::cout << "# INFO: # 3D scene points: " << m_graph.scenePointCount() << std::endl;
        }

#ifdef VCHARGE_VIZ
        visualize("opt1-BA-", ODOMETRY);

        visualizeExtrinsics("opt1-extrinsics");
#endif

        if (saveWorkingData)
        {
            boost::filesystem::path extrinsicPath(dataDir);
            extrinsicPath /= "tmp_extrinsic_1.txt";
            m_cameraSystem.writePosesToTextFile(extrinsicPath.string());

            boost::filesystem::path graphPath(dataDir);
            graphPath /= "frames_1.sg";
            m_graph.writeToBinaryFile(graphPath.string());
        }
    }

    // stage 2
    if (beginStage <= 2)
    {
        if (m_verbose)
        {
            std::cout << "# INFO: Running robust pose graph optimization... " << std::endl;
        }

        PoseGraph poseGraph(m_cameraSystem, m_graph,
                            k_maxDistanceRatio,
                            k_minLoopCorrespondences2D3D,
                            k_nearestImageMatches,
                            k_nominalFocalLength);
        poseGraph.setVerbose(m_verbose);
        poseGraph.buildEdges();
        poseGraph.optimize(true);

        if (m_verbose)
        {
            std::cout << "# INFO: Triangulating feature correspondences... " << std::endl;
        }

        triangulateFeatureCorrespondences();

        std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > correspondences2D3D;
        correspondences2D3D = poseGraph.getCorrespondences2D3D();

        if (m_verbose)
        {
            std::cout << "# INFO: # inlier 2D-3D correspondences: " << correspondences2D3D.size() << std::endl;
        }

        size_t nMerged3DScenePoints = 0;
        for (size_t i = 0; i < correspondences2D3D.size(); ++i)
        {
            Point3DFeaturePtr f3D1 = correspondences2D3D.at(i).first->feature3D();
            Point3DFeaturePtr f3D2 = correspondences2D3D.at(i).second;

            if (f3D1.get() == 0)
            {
                continue;
            }

            bool merge = false;
            for (size_t j = 0; j < f3D2->features2D().size(); ++j)
            {
                Point2DFeaturePtr f2D2 = f3D2->features2D().at(j).lock();
                if (f2D2.get() == 0)
                {
                    continue;
                }

                bool found = false;
                for (size_t k = 0; k < f3D1->features2D().size(); ++k)
                {
                    Point2DFeaturePtr f2D1 = f3D1->features2D().at(k).lock();
                    if (f2D1.get() == 0)
                    {
                        continue;
                    }

                    if (f2D1.get() == f2D2.get())
                    {
                        found = true;
                        break;
                    }
                }

                if (!found)
                {
                    f3D1->features2D().push_back(f2D2);
                    merge = true;
                }
            }

            for (size_t j = 0; j < f3D1->features2D().size(); ++j)
            {
                if (Point2DFeaturePtr feature2D = f3D1->features2D().at(j).lock())
                {
                    feature2D->feature3D() = f3D1;
                }
            }

            if (merge)
            {
                ++nMerged3DScenePoints;
            }
        }

        if (m_verbose)
        {
            std::cout << "# INFO: Merged " << nMerged3DScenePoints << " 3D scene points." << std::endl;
        }

        prune(PRUNE_BEHIND_CAMERA, ODOMETRY);

        double minError, maxError, avgError;
        size_t featureCount;

        reprojectionError(minError, maxError, avgError, featureCount, ODOMETRY);

        if (m_verbose)
        {
            std::cout << "# INFO: Reprojection error after robust pose-graph optimization: avg = " << avgError
                      << " px | max = " << maxError << " px" << std::endl;
            std::cout << "# INFO: # 2D feature points: " << featureCount << std::endl;
            std::cout << "# INFO: # 3D scene points: " << m_graph.scenePointCount() << std::endl;
        }

#ifdef VCHARGE_VIZ
        visualize("opt2-", ODOMETRY);
#endif

        if (saveWorkingData)
        {
            boost::filesystem::path extrinsicPath(dataDir);
            extrinsicPath /= "tmp_extrinsic_2.txt";
            m_cameraSystem.writePosesToTextFile(extrinsicPath.string());

            boost::filesystem::path graphPath(dataDir);
            graphPath /= "frames_2.sg";
            m_graph.writeToBinaryFile(graphPath.string());
        }
    }

    // stage 3
    if (beginStage <= 3)
    {
        if (m_verbose)
        {
            std::cout << "# INFO: Finding inter-map 3D-3D correspondences... " << std::endl;
        }

        prune(PRUNE_BEHIND_CAMERA, ODOMETRY);

        // find local inter-map 3D-3D correspondences
        std::vector<Correspondence2D2D> localInterMap2D2D;
        findLocalInterMap2D2DCorrespondences(localInterMap2D2D);

        if (m_verbose)
        {
            std::cout << "# INFO: # local inter-map 3D-3D correspondences = "
                      << localInterMap2D2D.size() << std::endl;

            size_t featureCount[m_cameraSystem.cameraCount()];
            for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
            {
                featureCount[i] = 0;
            }

            for (size_t i = 0; i < localInterMap2D2D.size(); ++i)
            {
                if (FramePtr frame = localInterMap2D2D.at(i).first->frame().lock())
                {
                    ++featureCount[frame->cameraId()];
                }
                else
                {
                    std::cout << "# WARNING: Parent frame is missing." << std::endl;
                }
                if (FramePtr frame = localInterMap2D2D.at(i).second->frame().lock())
                {
                    ++featureCount[frame->cameraId()];
                }
                else
                {
                    std::cout << "# WARNING: Parent frame is missing." << std::endl;
                }
            }

            for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
            {
                std::cout << "# INFO: # features seen in camera " << i
                          << ": " << featureCount[i] << std::endl;
            }
        }

        std::vector<std::pair<FramePtr, FramePtr> > localInterMapFrameFrame;
        for (size_t i = 0; i < localInterMap2D2D.size(); ++i)
        {
            FramePtr f1 = localInterMap2D2D.at(i).first->frame().lock();
            FramePtr f2 = localInterMap2D2D.at(i).second->frame().lock();

            if (f1.get() == 0 || f2.get() == 0)
            {
                std::cout << "# WARNING: Parent frame is missing." << std::endl;
                continue;
            }

            localInterMapFrameFrame.push_back(std::make_pair(f1, f2));
        }

        for (size_t i = 0; i < localInterMap2D2D.size(); ++i)
        {
            Point3DFeaturePtr f3D1 = localInterMap2D2D.at(i).first->feature3D();
            Point3DFeaturePtr f3D2 = localInterMap2D2D.at(i).second->feature3D();

            for (size_t j = 0; j < f3D1->features2D().size(); ++j)
            {
                Point2DFeaturePtr f2D1 = f3D1->features2D().at(j).lock();
                if (f2D1.get() == 0)
                {
                    continue;
                }

                bool found = false;
                for (size_t k = 0; k < f3D2->features2D().size(); ++k)
                {
                    Point2DFeaturePtr f2D2 = f3D2->features2D().at(k).lock();
                    if (f2D2.get() == 0)
                    {
                        continue;
                    }

                    if (f2D1.get() == f2D2.get())
                    {
                        found = true;
                        break;
                    }
                }

                if (!found)
                {
                    f3D2->features2D().push_back(f2D1);
                }
            }

            for (size_t j = 0; j < f3D2->features2D().size(); ++j)
            {
                if (Point2DFeaturePtr feature2D = f3D2->features2D().at(j).lock())
                {
                    feature2D->feature3D() = f3D2;
                }
            }
            f3D2->attributes() = Point3DFeature::LOCALLY_OBSERVED_BY_DIFFERENT_CAMERAS;
        }

#ifdef VCHARGE_VIZ
        visualizeFrameFrameCorrespondences("local-inter-p-p", localInterMapFrameFrame);
        visualize3D3DCorrespondences("local-inter-3d-3d", localInterMap2D2D);
#endif

        prune(PRUNE_BEHIND_CAMERA, ODOMETRY);

//        std::cout << "# INFO: Running BA on odometry data... " << std::endl;
//
//        optimize(CAMERA_ODOMETRY_EXTRINSICS | ODOMETRY_6D_EXTRINSICS | POINT_3D, true);
//
//        prune(PRUNE_BEHIND_CAMERA, ODOMETRY);

        double minError, maxError, avgError;
        size_t featureCount;

        reprojectionError(minError, maxError, avgError, featureCount, ODOMETRY);

        if (m_verbose)
        {
            std::cout << "# INFO: Done." << std::endl;
            std::cout << "# INFO: Reprojection error after local matching: avg = " << avgError
                      << " px | max = " << maxError << " px" << std::endl;
            std::cout << "# INFO: # 2D feature points: " << featureCount << std::endl;
            std::cout << "# INFO: # 3D scene points: " << m_graph.scenePointCount() << std::endl;
        }

#ifdef VCHARGE_VIZ
        visualize("opt3-BA-", ODOMETRY);

        visualizeExtrinsics("opt3-extrinsics");
#endif

        if (saveWorkingData)
        {
            boost::filesystem::path extrinsicPath(dataDir);
            extrinsicPath /= "tmp_extrinsic_3.txt";
            m_cameraSystem.writePosesToTextFile(extrinsicPath.string());

            boost::filesystem::path graphPath(dataDir);
            graphPath /= "frames_3.sg";
            m_graph.writeToBinaryFile(graphPath.string());
        }
    }

    if (beginStage <= 4)
    {
        reweightScenePoints();

        // read chessboard data used for intrinsic calibration
        bool isChessboardDataComplete = true;

        std::vector<boost::shared_ptr<CameraCalibration> > cameraCalibrations(m_cameraSystem.cameraCount());
        for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
        {
            cameraCalibrations.at(i).reset(new CameraCalibration);

            boost::filesystem::path chessboardDataPath(dataDir);
            std::ostringstream oss;
            oss << m_cameraSystem.getCamera(i)->cameraName() << "_chessboard_data.dat";
            chessboardDataPath /= oss.str();

            if (!cameraCalibrations.at(i)->readChessboardData(chessboardDataPath.string()))
            {
                isChessboardDataComplete = false;
                break;
            }
        }

        if (isChessboardDataComplete)
        {
            m_cameraCalibrations = cameraCalibrations;
        }

        std::cout << "# INFO: Running BA on odometry data... " << std::endl;

        // perform BA to optimize intrinsics, extrinsics and scene points
        if (optimizeIntrinsics)
        {
            // perform BA to optimize intrinsics, extrinsics, odometry poses, and scene points
            optimize(CAMERA_INTRINSICS | CAMERA_ODOMETRY_EXTRINSICS | ODOMETRY_6D_EXTRINSICS | POINT_3D, true);
        }
        else
        {
            // perform BA to optimize extrinsics, odometry poses, and scene points
            optimize(CAMERA_ODOMETRY_EXTRINSICS | ODOMETRY_6D_EXTRINSICS | POINT_3D, true);
        }

        prune(PRUNE_BEHIND_CAMERA, ODOMETRY);

        double minError, maxError, avgError;
        size_t featureCount;

        reprojectionError(minError, maxError, avgError, featureCount, ODOMETRY);

        prune(PRUNE_BEHIND_CAMERA | PRUNE_FARAWAY | PRUNE_HIGH_REPROJ_ERR, ODOMETRY);

        if (m_verbose)
        {
            std::cout << "# INFO: Done." << std::endl;
            std::cout << "# INFO: Reprojection error after full BA: avg = " << avgError
                      << " px | max = " << maxError << " px" << std::endl;
            std::cout << "# INFO: # 2D feature points: " << featureCount << std::endl;
            std::cout << "# INFO: # 3D scene points: " << m_graph.scenePointCount() << std::endl;
        }

#ifdef VCHARGE_VIZ
        visualize("opt4-BA-", ODOMETRY);

        visualizeExtrinsics("opt4-extrinsics");
#endif

        if (saveWorkingData)
        {
            boost::filesystem::path extrinsicPath(dataDir);
            extrinsicPath /= "tmp_extrinsic_4.txt";
            m_cameraSystem.writePosesToTextFile(extrinsicPath.string());

            boost::filesystem::path graphPath(dataDir);
            graphPath /= "frames_4.sg";
            m_graph.writeToBinaryFile(graphPath.string());
        }
    }

    if (beginStage <= 5)
    {
        estimateCameraOdometryTransforms();

        double zGround = 0.0;
        if (estimateAbsoluteGroundHeight(zGround))
        {
            if (m_verbose)
            {
                std::cout << "# INFO: Found ground plane: z = " << zGround << std::endl;
            }

            for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
            {
                Eigen::Matrix4d cameraPose = m_cameraSystem.getGlobalCameraPose(i);
                cameraPose(2,3) -= zGround;

                m_cameraSystem.setGlobalCameraPose(i, cameraPose);
            }
        }
        else
        {
            if (m_verbose)
            {
                std::cout << "# INFO: Did not find ground plane." << std::endl;
            }
        }

#ifdef VCHARGE_VIZ
        visualizeExtrinsics("camodocal-extrinsics");
#endif
    }
}

void
CameraRigBA::setVerbose(bool verbose)
{
    m_verbose = verbose;
}

void
CameraRigBA::frameReprojectionError(const FramePtr& frame,
                                    const CameraConstPtr& camera,
                                    const Pose& T_cam_odo,
                                    double& minError, double& maxError, double& avgError,
                                    size_t& featureCount,
                                    int type) const
{
    minError = std::numeric_limits<double>::max();
    maxError = std::numeric_limits<double>::min();

    size_t count = 0;
    double totalError = 0.0;

    const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

    for (size_t i = 0; i < features2D.size(); ++i)
    {
        const Point2DFeatureConstPtr& feature2D = features2D.at(i);
        const Point3DFeatureConstPtr& feature3D = feature2D->feature3D();

        if (feature3D.get() == 0)
        {
            continue;
        }

        if (isnan(feature3D->point()(0)) || isnan(feature3D->point()(1)) ||
            isnan(feature3D->point()(2)))
        {
            continue;
        }

        double error = 0.0;

        if (type == ODOMETRY)
        {
            error = reprojectionError(camera, feature3D->point(),
                                      T_cam_odo.rotation(),
                                      T_cam_odo.translation(),
                                      frame->systemPose()->position(),
                                      frame->systemPose()->attitude(),
                                      Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y));
        }
        else
        {
            error = camera->reprojectionError(feature3D->point(),
                                              frame->cameraPose()->rotation(),
                                              frame->cameraPose()->translation(),
                                              Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y));
        }

        if (minError > error)
        {
            minError = error;
        }
        if (maxError < error)
        {
            maxError = error;
        }
        totalError += error;
        ++count;
    }

    if (count == 0)
    {
        avgError = 0.0;
        minError = 0.0;
        maxError = 0.0;
        featureCount = count;

        return;
    }

    avgError = totalError / count;
    featureCount = count;
}

void
CameraRigBA::reprojectionError(double& minError, double& maxError,
                               double& avgError, size_t& featureCount,
                               int type) const
{
    minError = std::numeric_limits<double>::max();
    maxError = std::numeric_limits<double>::min();

    size_t count = 0;
    double totalError = 0.0;

    Pose T_cam_odo[m_cameraSystem.cameraCount()];
    for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
    {
        T_cam_odo[i] = m_cameraSystem.getGlobalCameraPose(i);
    }

    for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
    {
        const FrameSetSegment& segment = m_graph.frameSetSegment(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            const FrameSetPtr& frameSet = segment.at(j);

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                const FramePtr& frame = frameSet->frames().at(k);

                if (frame.get() == 0)
                {
                    continue;
                }

                double frameMinError;
                double frameMaxError;
                double frameAvgError;
                size_t frameFeatureCount;

                frameReprojectionError(frame,
                                       m_cameraSystem.getCamera(frame->cameraId()),
                                       T_cam_odo[frame->cameraId()],
                                       frameMinError, frameMaxError, frameAvgError, frameFeatureCount,
                                       type);

                if (minError > frameMinError)
                {
                    minError = frameMinError;
                }
                if (maxError < frameMaxError)
                {
                    maxError = frameMaxError;
                }
                totalError += frameAvgError * frameFeatureCount;
                count += frameFeatureCount;
            }
        }
    }

    if (count == 0)
    {
        avgError = 0.0;
        minError = 0.0;
        maxError = 0.0;
        featureCount = 0;

        return;
    }

    avgError = totalError / count;
    featureCount = count;
}

double
CameraRigBA::reprojectionError(const CameraConstPtr& camera,
                               const Eigen::Vector3d& P,
                               const Eigen::Quaterniond& cam_odo_q,
                               const Eigen::Vector3d& cam_odo_t,
                               const Eigen::Vector3d& odo_p,
                               const Eigen::Vector3d& odo_att,
                               const Eigen::Vector2d& observed_p) const
{
    Eigen::Quaterniond q_z_inv(cos(odo_att(0) / 2.0), 0.0, 0.0, -sin(odo_att(0) / 2.0));
    Eigen::Quaterniond q_y_inv(cos(odo_att(1) / 2.0), 0.0, -sin(odo_att(1) / 2.0), 0.0);
    Eigen::Quaterniond q_x_inv(cos(odo_att(2) / 2.0), -sin(odo_att(2) / 2.0), 0.0, 0.0);

    Eigen::Quaterniond q_world_odo = q_x_inv * q_y_inv * q_z_inv;
    Eigen::Quaterniond q_cam = cam_odo_q.conjugate() * q_world_odo;

    Eigen::Vector3d t_cam = - q_cam.toRotationMatrix() * odo_p - cam_odo_q.conjugate().toRotationMatrix() * cam_odo_t;

    return camera->reprojectionError(P, q_cam, t_cam, observed_p);
}

void
CameraRigBA::triangulateFeatureCorrespondences(void)
{
    // remove 3D scene points
    for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
    {
        FrameSetSegment& segment = m_graph.frameSetSegment(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            FrameSetPtr& frameSet = segment.at(j);

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                FramePtr& frame = frameSet->frames().at(k);

                if (frame.get() == 0)
                {
                    continue;
                }

                std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    Point2DFeaturePtr& pf = features2D.at(l);

                    if (pf->feature3D().get() != 0)
                    {
                        pf->feature3D() = Point3DFeaturePtr();
                    }
                }
            }
        }
    }

    // triangulate feature correspondences to get 3D scene points in odometry frame
    for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
    {
        Pose T_cam_odo(m_cameraSystem.getGlobalCameraPose(i));

        for (size_t j = 0; j < m_graph.frameSetSegments().size(); ++j)
        {
            FrameSetSegment& segment = m_graph.frameSetSegment(j);

            std::vector<std::vector<FramePtr> > frameSegments;
            frameSegments.resize(1);

            for (size_t k = 0; k < segment.size(); ++k)
            {
                FramePtr& frame = segment.at(k)->frames().at(i);

                if (frame.get() == 0)
                {
                    frameSegments.resize(frameSegments.size() + 1);
                }
                else
                {
                    frameSegments.back().push_back(frame);
                }
            }

            for (size_t k = 0; k < frameSegments.size(); ++k)
            {
                std::vector<FramePtr>& frameSegment = frameSegments.at(k);

                if (frameSegment.size() < 3)
                {
                    continue;
                }

                for (size_t l = 2; l < frameSegment.size(); ++l)
                {
                    triangulateFeatures(frameSegment.at(l-2), frameSegment.at(l-1), frameSegment.at(l),
                                        m_cameraSystem.getCamera(i), T_cam_odo);
                }
            }
        }
    }
}

void
CameraRigBA::triangulateFeatures(FramePtr& frame1, FramePtr& frame2, FramePtr& frame3,
                                 const CameraConstPtr& camera,
                                 const Pose& T_cam_odo)
{
    // triangulate new feature correspondences seen in last 3 frames
    std::vector<std::vector<Point2DFeaturePtr> > featureCorrespondences;

    // use features that are seen in frames 0, 1, and 2
    find2D2DCorrespondences(frame3->features2D(), 3, featureCorrespondences);

//    if (m_verbose)
//    {
//        std::cout << "# INFO: Found " << featureCorrespondences.size() << " feature correspondences in last 3 frames." << std::endl;
//    }

    std::vector<cv::Point2f> ipoints[3];

    std::vector<std::vector<Point2DFeaturePtr> > untriFeatureCorrespondences;
    for (size_t i = 0; i < featureCorrespondences.size(); ++i)
    {
        std::vector<Point2DFeaturePtr>& fc = featureCorrespondences.at(i);

        Point2DFeaturePtr& f0 = fc.at(0);
        Point2DFeaturePtr& f1 = fc.at(1);
        Point2DFeaturePtr& f2 = fc.at(2);

        if (f0->feature3D().get() == 0 && f1->feature3D().get() == 0)
        {
            ipoints[0].push_back(f0->keypoint().pt);
            ipoints[1].push_back(f1->keypoint().pt);
            ipoints[2].push_back(f2->keypoint().pt);

            untriFeatureCorrespondences.push_back(fc);
        }

        if (f0->feature3D().get() != 0 && f1->feature3D().get() != 0)
        {
            f2->feature3D() = f1->feature3D();
            f2->feature3D()->features2D().push_back(f2);
        }
    }

//    if (m_verbose)
//    {
//        std::cout << "# INFO: Found " << untriFeatureCorrespondences.size() << " untriangulated feature correspondences." << std::endl;
//    }

    if (!untriFeatureCorrespondences.empty())
    {
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points3D;
        std::vector<size_t> indices;

        Eigen::Matrix4d H_odo_cam = T_cam_odo.toMatrix().inverse();

        Eigen::Matrix4d H1 = H_odo_cam * frame1->systemPose()->toMatrix().inverse();
        Eigen::Matrix4d H2 = H_odo_cam * frame2->systemPose()->toMatrix().inverse();
        Eigen::Matrix4d H3 = H_odo_cam * frame3->systemPose()->toMatrix().inverse();

        tvt(camera,
            H1, ipoints[0],
            H2, ipoints[1],
            H3, ipoints[2],
            points3D, indices);

//        if (m_verbose)
//        {
//            std::cout << "# INFO: Triangulated " << points3D.size() << " new points." << std::endl;
//
//            if (!points3D.empty())
//            {
//                size_t count = 0;
//                double errorTotal = 0.0;
//                double errorMax = std::numeric_limits<double>::min();
//
//                for (size_t i = 0; i < points3D.size(); ++i)
//                {
//                    const cv::Point2f& feature2D = ipoints[0].at(indices.at(i));
//
//                    const Eigen::Vector3d& feature3D = points3D.at(i);
//
//                    OdometryReprojectionError reprojErr(cameraParameters,
//                                                        feature2D.x, feature2D.y);
//
//                    double residuals[2];
//                    reprojErr(T_cam_odo.rotationData(),
//                              T_cam_odo.translationData(),
//                              frame1->systemPose()->positionData(),
//                              frame1->systemPose()->attitudeData(),
//                              feature3D.data(), residuals);
//
//                    double error = hypot(residuals[0], residuals[1]);
//                    errorTotal += error;
//
//                    if (error > errorMax)
//                    {
//                        errorMax = error;
//                    }
//
//                    ++count;
//                }
//
//                double errorAvg = errorTotal / count;
//
//                std::cout << "# INFO: Reprojection error in frame n-2: avg = " << errorAvg
//                          << " px | max = " << errorMax << " px." << std::endl;
//
//                count = 0;
//                errorTotal = 0.0;
//                errorMax = std::numeric_limits<double>::min();
//
//                for (size_t i = 0; i < points3D.size(); ++i)
//                {
//                    const cv::Point2f& feature2D = ipoints[1].at(indices.at(i));
//
//                    const Eigen::Vector3d& feature3D = points3D.at(i);
//
//                    OdometryReprojectionError reprojErr(cameraParameters,
//                                                        feature2D.x, feature2D.y);
//
//                    double residuals[2];
//                    reprojErr(T_cam_odo.rotationData(),
//                              T_cam_odo.translationData(),
//                              frame2->systemPose()->positionData(),
//                              frame2->systemPose()->attitudeData(),
//                              feature3D.data(), residuals);
//
//                    double error = hypot(residuals[0], residuals[1]);
//                    errorTotal += error;
//
//                    if (error > errorMax)
//                    {
//                        errorMax = error;
//                    }
//
//                    ++count;
//                }
//
//                errorAvg = errorTotal / count;
//
//                std::cout << "# INFO: Reprojection error in frame n-1: avg = " << errorAvg
//                          << " px | max = " << errorMax << " px." << std::endl;
//
//                count = 0;
//                errorTotal = 0.0;
//                errorMax = std::numeric_limits<double>::min();
//
//                for (size_t i = 0; i < points3D.size(); ++i)
//                {
//                    const cv::Point2f& feature2D = ipoints[2].at(indices.at(i));
//
//                    const Eigen::Vector3d& feature3D = points3D.at(i);
//
//                    OdometryReprojectionError reprojErr(cameraParameters,
//                                                        feature2D.x, feature2D.y);
//
//                    double residuals[2];
//                    reprojErr(T_cam_odo.rotationData(),
//                              T_cam_odo.translationData(),
//                              frame3->systemPose()->positionData(),
//                              frame3->systemPose()->attitudeData(),
//                              feature3D.data(), residuals);
//
//                    double error = hypot(residuals[0], residuals[1]);
//                    errorTotal += error;
//
//                    if (error > errorMax)
//                    {
//                        errorMax = error;
//                    }
//
//                    ++count;
//                }
//
//                errorAvg = errorTotal / count;
//
//                std::cout << "# INFO: Reprojection error in frame n: avg = " << errorAvg
//                          << " px | max = " << errorMax << " px." << std::endl;
//            }
//        }

        for (size_t i = 0; i < points3D.size(); ++i)
        {
            Point3DFeaturePtr point3D(new Point3DFeature);

            point3D->point() = points3D.at(i);

            std::vector<Point2DFeaturePtr>& fc = untriFeatureCorrespondences.at(indices.at(i));

            for (int j = 0; j < 3; ++j)
            {
                Point2DFeaturePtr& pt = fc.at(j);

                point3D->features2D().push_back(pt);
                pt->feature3D() = point3D;
            }
        }
    }

//    if (m_verbose)
//        double minError, maxError, avgError;
//        size_t featureCount;
//
//        frameReprojectionError(frame3, cameraParameters, T_cam_odo, minError, maxError, avgError, featureCount);
//
//        std::cout << "# INFO: Frame reprojection error: min = " << minError << " | max = " << maxError << " | avg = " << avgError << std::endl;
//    }
}

void
CameraRigBA::find2D2DCorrespondences(const std::vector<Point2DFeaturePtr>& features,
                                     int nViews,
                                     std::vector<std::vector<Point2DFeaturePtr> >& correspondences) const
{
    // find feature correspondences across n views starting backward from
    // specified feature set in nth view
    if (nViews < 2)
    {
        return;
    }

    correspondences.reserve(features.size());

    for (size_t i = 0; i < features.size(); ++i)
    {
        Point2DFeaturePtr pt[nViews];

        pt[nViews - 1] = features.at(i);
        bool foundCorrespondences = true;

        for (int j = nViews - 1; j > 0; --j)
        {
            if (pt[j]->prevMatches().empty() || pt[j]->bestPrevMatchId() == -1)
            {
                foundCorrespondences = false;
                break;
            }

            pt[j - 1] = pt[j]->prevMatch().lock();

            if (pt[j - 1].get() == 0)
            {
                foundCorrespondences = false;
                break;
            }
        }

        if (!foundCorrespondences)
        {
            continue;
        }

        std::vector<Point2DFeaturePtr> correspondence(nViews);
        for (int j = 0; j < nViews; ++j)
        {
            correspondence.at(j) = pt[j];
        }

        correspondences.push_back(correspondence);
    }
}

void
CameraRigBA::findLocalInterMap2D2DCorrespondences(std::vector<Correspondence2D2D>& correspondences2D2D,
                                                  double reprojErrorThresh)
{
    int segmentId = 0;
    int frameSetId = 0;

    std::list<FramePtr> windows[m_cameraSystem.cameraCount()];

    while (segmentId < m_graph.frameSetSegments().size())
    {
        FrameSetPtr& frameSet = m_graph.frameSetSegment(segmentId).at(frameSetId);

        for (int cameraId = 0; cameraId < m_cameraSystem.cameraCount(); ++cameraId)
        {
            std::list<FramePtr>& window = windows[cameraId];

            if (frameSet->frames().at(cameraId).get() != 0)
            {
                windows[cameraId].push_front(frameSet->frames().at(cameraId));
            }

            std::list<FramePtr>::iterator itWindow = windows[cameraId].begin();
            bool init = false;
            Eigen::Vector3d lastOdoPos;

            double dist = 0.0;
            while (itWindow != windows[cameraId].end() && dist < k_localMapWindowDistance)
            {
                if (!init)
                {
                    lastOdoPos = (*itWindow)->systemPose()->position();

                    init = true;

                    ++itWindow;

                    continue;
                }

                dist += ((*itWindow)->systemPose()->position() - lastOdoPos).norm();

                lastOdoPos = (*itWindow)->systemPose()->position();

                if (dist < k_localMapWindowDistance)
                {
                    ++itWindow;
                }
            }

            if (itWindow != windows[cameraId].end())
            {
                windows[cameraId].erase(itWindow, windows[cameraId].end());
            }
        }

        if (m_verbose)
        {
            std::cout << "# INFO: Entering frame set node " << segmentId
                      << "-" << frameSetId << " [ ";

            for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
            {
                std::cout << windows[i].size() << " ";
            }
            std::cout << "]" << std::endl;
        }

        // for each camera, match current image with each image in each other camera's buffer
        for (int cameraId1 = 0; cameraId1 < m_cameraSystem.cameraCount(); ++cameraId1)
        {
            FramePtr& frame1 = frameSet->frames().at(cameraId1);

            if (frame1.get() == 0)
            {
                continue;
            }

            if (frame1->features2D().empty())
            {
                continue;
            }

            boost::shared_ptr<boost::thread> threads[m_cameraSystem.cameraCount()];
            std::vector<Correspondence2D2D> subCorr2D2D[m_cameraSystem.cameraCount()];
            for (int cameraId2 = 0; cameraId2 < m_cameraSystem.cameraCount(); ++cameraId2)
            {
                if (cameraId1 == cameraId2)
                {
                    continue;
                }

                std::vector<FramePtr> window;
                window.insert(window.end(), windows[cameraId2].begin(), windows[cameraId2].end());
                threads[cameraId2].reset(new boost::thread(boost::bind(&CameraRigBA::matchFrameToWindow, this,
                                                                       frame1, window,
                                                                       &subCorr2D2D[cameraId2],
                                                                       reprojErrorThresh)));
            }

            for (int cameraId2 = 0; cameraId2 < m_cameraSystem.cameraCount(); ++cameraId2)
            {
                if (threads[cameraId2].get() == 0)
                {
                    continue;
                }
                threads[cameraId2]->join();

                correspondences2D2D.insert(correspondences2D2D.end(), subCorr2D2D[cameraId2].begin(), subCorr2D2D[cameraId2].end());
            }
        }

        ++frameSetId;
        if (frameSetId >= m_graph.frameSetSegment(segmentId).size())
        {
            ++segmentId;
            frameSetId = 0;
        }
    }

//    std::vector<Correspondence2D2D>::iterator it = correspondences2D2D.begin();
//    while (it != correspondences2D2D.end())
//    {
//        Point2DFeaturePtr& p1 = it->first;
//        Point2DFeaturePtr& p2 = it->second;
//
//        FramePtr frame1 = p1->frame().lock();
//        FramePtr frame2 = p2->frame().lock();
//
//        if (frame1.get() == 0 || frame2.get() == 0)
//        {
//            std::cout << "# WARNING: Parent frame is missing." << std::endl;
//            continue;
//        }
//
//        if (p1->feature3D().get() == 0 && p2->feature3D().get() == 0)
//        {
//            Eigen::Vector3d scenePoint;
//            if (!triangulate3DPoint(p1, p2, scenePoint))
//            {
//                it = correspondences2D2D.erase(it);
//                continue;
//            }
//
//            Point3DFeaturePtr feature3D(new Point3DFeature);
//            feature3D->point() = scenePoint;
//            feature3D->features2D().push_back(p1);
//            feature3D->features2D().push_back(p2);
//
//            p1->feature3D() = feature3D;
//            p2->feature3D() = feature3D;
//        }
//        else
//        {
//            if (p1->feature3D().get() == 0)
//            {
//                p1->feature3D() = p2->feature3D();
//                p1->feature3D()->features2D().push_back(p1);
//            }
//
//            if (p2->feature3D().get() == 0)
//            {
//                p2->feature3D() = p1->feature3D();
//                p2->feature3D()->features2D().push_back(p1);
//            }
//        }
//
//        ++it;
//    }
}

void
CameraRigBA::matchFrameToWindow(FramePtr& frame1,
                                std::vector<FramePtr>& window,
                                std::vector<Correspondence2D2D>* correspondences2D2D,
                                double reprojErrorThresh)
{
    if (window.empty())
    {
        return;
    }

    if (frame1->image().empty())
    {
        return;
    }

    boost::shared_ptr<boost::thread> threads[window.size()];
    std::vector<std::pair<Point2DFeaturePtr, Point2DFeaturePtr> > corr2D2D[window.size()];

    int windowId = -1;
    for (std::vector<FramePtr>::iterator itFrame2 = window.begin(); itFrame2 != window.end(); ++itFrame2)
    {
        ++windowId;
        FramePtr& frame2 = *itFrame2;

        threads[windowId].reset(new boost::thread(boost::bind(&CameraRigBA::matchFrameToFrame, this,
                                                              frame1, frame2,
                                                              &corr2D2D[windowId],
                                                              reprojErrorThresh)));
    }

    int windowIdBest = -1;
    std::vector<std::pair<Point2DFeaturePtr, Point2DFeaturePtr> > corr2D2DBest;

    for (windowId = 0; windowId < window.size(); ++windowId)
    {
        threads[windowId]->join();

        if (corr2D2D[windowId].size() > corr2D2DBest.size())
        {
            windowIdBest = windowId;
            corr2D2DBest = corr2D2D[windowId];
        }
    }

    if (!corr2D2DBest.empty())
    {
        FramePtr& frame2 = window.at(windowIdBest);

        correspondences2D2D->insert(correspondences2D2D->end(),
                                    corr2D2DBest.begin(), corr2D2DBest.end());

        if (m_verbose)
        {
            std::cout << "# INFO: Best: cam " << frame1->cameraId()
                      << " - cam " << frame2->cameraId()
                      << ": window " << windowIdBest
                      << " | " << correspondences2D2D->size() << " 2D-2D" << std::endl;
        }

#ifdef VCHARGE_VIZ
        visualize3D3DCorrespondences("local-inter-3d-3d", corr2D2DBest);
#endif
    }
}

void
CameraRigBA::matchFrameToFrame(FramePtr& frame1, FramePtr& frame2,
                               std::vector<Correspondence2D2D>* corr2D2D,
                               double reprojErrorThresh)
{
    std::vector<size_t> inliers2D2D;

    if (frame2.get() == 0)
    {
        return;
    }

    if (frame2->features2D().empty())
    {
        return;
    }

    if (frame2->image().empty())
    {
        return;
    }

    int cameraId1 = frame1->cameraId();
    int cameraId2 = frame2->cameraId();

    Pose T_odo_cam1(m_cameraSystem.getGlobalCameraPose(cameraId1).inverse());
    Pose T_odo_cam2(m_cameraSystem.getGlobalCameraPose(cameraId2).inverse());

    Eigen::Matrix4d H_cam1 = T_odo_cam1.toMatrix() * frame1->systemPose()->toMatrix().inverse();
    Eigen::Matrix4d H_cam2 = T_odo_cam2.toMatrix() * frame2->systemPose()->toMatrix().inverse();

    // compute average rotation between camera pair
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H_cam1.block<3,3>(0,0) +
                                          H_cam2.block<3,3>(0,0),
                                          Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d avgR = svd.matrixU() * svd.matrixV().transpose();

    Eigen::Matrix3d R1 = avgR * H_cam1.block<3,3>(0,0).transpose();
    Eigen::Matrix3d R2 = avgR * H_cam2.block<3,3>(0,0).transpose();

    cv::Mat R1_cv, R2_cv;
    cv::eigen2cv(R1, R1_cv);
    cv::eigen2cv(R2, R2_cv);

    cv::Mat mapX1, mapY1, mapX2, mapY2;
    if (m_cameraSystem.getCamera(cameraId1)->modelType() == Camera::PINHOLE)
    {
        m_cameraSystem.getCamera(cameraId1)->initUndistortRectifyMap(mapX1, mapY1,
                                                                     -1.0f, -1.0f,
                                                                     cv::Size(0, 0),
                                                                     -1.0f, -1.0f, R1_cv);
    }
    else
    {
        m_cameraSystem.getCamera(cameraId1)->initUndistortRectifyMap(mapX1, mapY1,
                                                                     k_nominalFocalLength, k_nominalFocalLength,
                                                                     cv::Size(0, 0),
                                                                     -1.0f, -1.0f, R1_cv);
    }
    if (m_cameraSystem.getCamera(cameraId2)->modelType() == Camera::PINHOLE)
    {
        m_cameraSystem.getCamera(cameraId2)->initUndistortRectifyMap(mapX2, mapY2,
                                                                     -1.0f, -1.0f,
                                                                     cv::Size(0, 0),
                                                                     -1.0f, -1.0f, R2_cv);
    }
    else
    {
        m_cameraSystem.getCamera(cameraId2)->initUndistortRectifyMap(mapX2, mapY2,
                                                                     k_nominalFocalLength, k_nominalFocalLength,
                                                                     cv::Size(0, 0),
                                                                     -1.0f, -1.0f, R2_cv);
    }

    cv::Mat rimg1, rimg2;
    cv::remap(frame1->image(), rimg1, mapX1, mapY1, cv::INTER_LINEAR);
    cv::remap(frame2->image(), rimg2, mapX2, mapY2, cv::INTER_LINEAR);

    cv::Mat rmask1, rmask2;
    if (m_cameraSystem.getCamera(cameraId1)->mask().empty())
    {
        rmask1 = rimg1 > 0;
    }
    else
    {
        cv::remap(m_cameraSystem.getCamera(cameraId1)->mask(), rmask1, mapX1, mapY1, cv::INTER_NEAREST);
    }
    if (m_cameraSystem.getCamera(cameraId2)->mask().empty())
    {
        rmask2 = rimg2 > 0;
    }
    else
    {
        cv::remap(m_cameraSystem.getCamera(cameraId2)->mask(), rmask2, mapX2, mapY2, cv::INTER_NEAREST);
    }

    cv::Ptr<SurfGPU> surf = SurfGPU::instance(200.0);

    std::vector<cv::KeyPoint> rkeypoints1, rkeypoints2;
    cv::Mat rdtors1, rdtors2;
    std::vector<cv::DMatch> rmatches;

    surf->match(rimg1, rkeypoints1, rdtors1, rmask1,
                rimg2, rkeypoints2, rdtors2, rmask2, rmatches);

    if (rmatches.size() < k_minInterCorrespondences2D2D)
    {
        return;
    }

    std::vector<cv::Point2f> rpoints1, rpoints2;
    for (size_t i = 0; i < rmatches.size(); ++i)
    {
        cv::DMatch& match = rmatches.at(i);

        rpoints1.push_back(rkeypoints1.at(match.queryIdx).pt);
        rpoints2.push_back(rkeypoints2.at(match.trainIdx).pt);
    }

    cv::Mat E, inlierMat;
    E = findEssentialMat(rpoints1, rpoints2, k_nominalFocalLength,
                         cv::Point2d(rimg1.cols / 2, rimg1.rows / 2),
                         CV_FM_RANSAC, 0.99, reprojErrorThresh, 1000, inlierMat);

    if (cv::countNonZero(inlierMat) < k_minInterCorrespondences2D2D)
    {
        return;
    }

    if (0)
    {
        static boost::mutex rmutex;
        rmutex.lock();

        static int rcount = 0;
        std::ostringstream oss;
        oss << "r-inter-map-matches-" << cameraId1 << "-" << cameraId2 << "-" << rcount << ".png";

        std::vector<cv::DMatch> inlierRMatches;
        for (size_t i = 0; i < rmatches.size(); ++i)
        {
            if (inlierMat.at<unsigned char>(0,i) != 0)
            {
                inlierRMatches.push_back(rmatches.at(i));
            }
        }

        cv::Mat sketch;
        cv::drawMatches(rimg1, rkeypoints1, rimg2, rkeypoints2, inlierRMatches, sketch);

        cv::imwrite(oss.str(), sketch);

        oss.str(""); oss.clear();
        oss << "r" << cameraId1 << "img" << rcount << ".png";
        cv::imwrite(oss.str(), frame1->image());

        oss.str(""); oss.clear();
        oss << "r" << cameraId2 << "img" << rcount << ".png";
        cv::imwrite(oss.str(), frame2->image());

        ++rcount;

        rmutex.unlock();
    }

    Eigen::Matrix4d H1 = Eigen::Matrix4d::Identity();
    H1.block<3,3>(0,0) = R1;
    Eigen::Matrix4d H_rcam1 = H1 * H_cam1;

    Eigen::Matrix4d H2 = Eigen::Matrix4d::Identity();
    H2.block<3,3>(0,0) = R2;
    Eigen::Matrix4d H_rcam2 = H2 * H_cam2;

    std::vector<std::pair<Point2DFeaturePtr, Point2DFeaturePtr> > candidateCorr2D2D(rmatches.size());
    for (size_t i = 0; i < rmatches.size(); ++i)
    {
        if (inlierMat.at<unsigned char>(0,i) == 0)
        {
            continue;
        }

        cv::Point2f& rp = rpoints1.at(i);

        // compute corresponding image point in original image in camera 1
        Eigen::Vector3d ray;
        ray(0) = (rp.x - m_cameraSystem.getCamera(cameraId1)->imageWidth() / 2.0) / k_nominalFocalLength;
        ray(1) = (rp.y - m_cameraSystem.getCamera(cameraId1)->imageHeight() / 2.0) / k_nominalFocalLength;
        ray(2) = 1.0;

        ray = R1.transpose() * ray;

        Eigen::Vector2d ep;
        m_cameraSystem.getCamera(cameraId1)->spaceToPlane(ray, ep);

        // find closest image point to computed image point
        std::vector<Point2DFeaturePtr> candidateImagePoints1;
        Point2DFeaturePtr p2DBest;
        for (size_t j = 0; j < frame1->features2D().size(); ++j)
        {
            Point2DFeaturePtr& p2D = frame1->features2D().at(j);

            float kpDist = hypot(ep(0) - p2D->keypoint().pt.x,
                                 ep(1) - p2D->keypoint().pt.y);
            if (kpDist < 1.0f)
            {
                candidateImagePoints1.push_back(p2D);
            }
        }

        if (candidateImagePoints1.size() != 1)
        {
            continue;
        }

        candidateCorr2D2D.at(i).first = candidateImagePoints1.front();

        rp = rpoints2.at(i);

        // compute corresponding image point in original image in camera 2
        ray(0) = (rp.x - m_cameraSystem.getCamera(cameraId2)->imageWidth() / 2.0) / k_nominalFocalLength;
        ray(1) = (rp.y - m_cameraSystem.getCamera(cameraId2)->imageHeight() / 2.0) / k_nominalFocalLength;
        ray(2) = 1.0;

        ray = R2.transpose() * ray;

        m_cameraSystem.getCamera(cameraId2)->spaceToPlane(ray, ep);

        // find closest image point to computed image point
        std::vector<Point2DFeaturePtr> candidateImagePoints2;
        for (size_t j = 0; j < frame2->features2D().size(); ++j)
        {
            Point2DFeaturePtr& p2D = frame2->features2D().at(j);

            float kpDist = hypot(ep(0) - p2D->keypoint().pt.x,
                                 ep(1) - p2D->keypoint().pt.y);
            if (kpDist < 1.0f)
            {
                candidateImagePoints2.push_back(p2D);
            }
        }

        if (candidateImagePoints2.size() != 1)
        {
            continue;
        }

        candidateCorr2D2D.at(i).second = candidateImagePoints2.front();
    }

    for (size_t i = 0; i < candidateCorr2D2D.size(); ++i)
    {
        Point2DFeaturePtr& p1 = candidateCorr2D2D.at(i).first;
        Point2DFeaturePtr& p2 = candidateCorr2D2D.at(i).second;

        if (p1.get() == 0 || p2.get() == 0)
        {
            continue;
        }

        if (p1->feature3D().get() == 0 || p2->feature3D().get() == 0)
        {
            continue;
        }

        corr2D2D->push_back(candidateCorr2D2D.at(i));
    }

    if (0 && !corr2D2D->empty())
    {
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        std::vector<cv::DMatch> matches;
        for (size_t i = 0; i < corr2D2D->size(); ++i)
        {
            Point2DFeaturePtr& p1 = corr2D2D->at(i).first;
            Point2DFeaturePtr& p2 = corr2D2D->at(i).second;

            if (p1.get() == 0 || p2.get() == 0)
            {
                continue;
            }

            keypoints1.push_back(p1->keypoint());
            keypoints2.push_back(p2->keypoint());

            cv::DMatch match;
            match.queryIdx = keypoints1.size() - 1;
            match.trainIdx = keypoints2.size() - 1;
            matches.push_back(match);
        }

        static boost::mutex mutex;
        mutex.lock();

        static int count = 0;
        std::ostringstream oss;
        oss << "ur-inter-map-matches-" << cameraId1 << "-" << cameraId2 << "-" << count << ".png";

        cv::Mat sketch;
        cv::drawMatches(frame1->image(), keypoints1, frame2->image(), keypoints2, matches, sketch);

        cv::imwrite(oss.str(), sketch);

        oss.str(""); oss.clear();
        oss << "ur" << cameraId1 << "img" << count << ".png";
        cv::imwrite(oss.str(), frame1->image());

        oss.str(""); oss.clear();
        oss << "ur" << cameraId2 << "img" << count << ".png";
        cv::imwrite(oss.str(), frame2->image());

        ++count;

        mutex.unlock();
    }
}

bool
CameraRigBA::project3DPoint(const CameraConstPtr& camera,
                            const Eigen::Matrix4d& H,
                            const Eigen::Vector4d& src, Eigen::Vector3d& dst) const
{
    // transform point from world frame to camera frame
    Eigen::Vector3d P = H.block<3,3>(0,0) * (src.block<3,1>(0,0) / src(3)) + H.block<3,1>(0,3);

    // check if point is behind camera
    if (P(2) < 0.0)
    {
        return false;
    }

    Eigen::Vector2d p;
    camera->spaceToPlane(P, p);
    dst << p, 1.0;

    return true;
}

void
CameraRigBA::tvt(const CameraConstPtr& camera,
                 const Eigen::Matrix4d& H1,
                 const std::vector<cv::Point2f>& imagePoints1,
                 const Eigen::Matrix4d& H2,
                 const std::vector<cv::Point2f>& imagePoints2,
                 const Eigen::Matrix4d& H3,
                 const std::vector<cv::Point2f>& imagePoints3,
                 std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points3D,
                 std::vector<size_t>& inliers) const
{
    Eigen::Matrix<double, 3, 4> P1 = H1.block<3,4>(0,0);
    Eigen::Matrix<double, 3, 4> P2 = H2.block<3,4>(0,0);
    Eigen::Matrix<double, 3, 4> P3 = H3.block<3,4>(0,0);

    Eigen::Matrix4d H12 = H2 * H1.inverse();
    Eigen::Matrix4d H23 = H3 * H2.inverse();
    Eigen::Matrix4d H13 = H3 * H1.inverse();

    Eigen::Matrix3d E12 = skew(Eigen::Vector3d(H12.block<3,1>(0,3))) * H12.block<3,3>(0,0);
    Eigen::Matrix3d E23 = skew(Eigen::Vector3d(H23.block<3,1>(0,3))) * H23.block<3,3>(0,0);
    Eigen::Matrix3d E13 = skew(Eigen::Vector3d(H13.block<3,1>(0,3))) * H13.block<3,3>(0,0);

    // linear triangulation
    for (size_t i = 0; i < imagePoints1.size(); ++i)
    {
        const cv::Point2f& p1_cv = imagePoints1.at(i);
        const cv::Point2f& p2_cv = imagePoints2.at(i);
        const cv::Point2f& p3_cv = imagePoints3.at(i);

        cv::Point2f rect_p1_cv, rect_p2_cv, rect_p3_cv;
        rectifyImagePoint(camera, p1_cv, rect_p1_cv);
        rectifyImagePoint(camera, p2_cv, rect_p2_cv);
        rectifyImagePoint(camera, p3_cv, rect_p3_cv);

        Eigen::Matrix4d J;
        J.row(0) = P2.row(2) * rect_p2_cv.x - P2.row(0);
        J.row(1) = P2.row(2) * rect_p2_cv.y - P2.row(1);
        J.row(2) = P3.row(2) * rect_p3_cv.x - P3.row(0);
        J.row(3) = P3.row(2) * rect_p3_cv.y - P3.row(1);

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Vector4d scenePoint = svd.matrixV().block<4,1>(0,3);

        scenePoint /= scenePoint(3);

        // validate scene point
        Eigen::Vector3d p1, p2, p3;
        if (!project3DPoint(camera, H1, scenePoint, p1))
        {
            continue;
        }
        if (!project3DPoint(camera, H2, scenePoint, p2))
        {
            continue;
        }
        if (!project3DPoint(camera, H3, scenePoint, p3))
        {
            continue;
        }

        points3D.push_back(scenePoint.block<3,1>(0,0));
        inliers.push_back(i);
    }
}

bool
CameraRigBA::triangulate3DPoint(const Point2DFeatureConstPtr& p1,
                                const Point2DFeatureConstPtr& p2,
                                Eigen::Vector3d& scenePoint,
                                double reprojErrorThresh) const
{
    FrameConstPtr frame1 = p1->frame().lock();
    FrameConstPtr frame2 = p2->frame().lock();

    if (frame1.get() == 0 || frame2.get() == 0)
    {
        std::cout << "# WARNING: Parent frame is missing." << std::endl;
        return false;
    }

    const CameraPtr& cam1 = m_cameraSystem.getCamera(frame1->cameraId());
    const CameraPtr& cam2 = m_cameraSystem.getCamera(frame2->cameraId());

    const cv::Point2f& pt1 = p1->keypoint().pt;
    const cv::Point2f& pt2 = p2->keypoint().pt;

    // projective rays
    Eigen::Vector3d ray1, ray2;
    cam1->liftProjective(Eigen::Vector2d(pt1.x, pt1.y), ray1);
    cam2->liftProjective(Eigen::Vector2d(pt2.x, pt2.y), ray2);

    ray1 = m_cameraSystem.getGlobalCameraPose(frame1->cameraId()).block<3,3>(0,0) * ray1;
    ray2 = m_cameraSystem.getGlobalCameraPose(frame2->cameraId()).block<3,3>(0,0) * ray2;

    ray1.normalize();
    ray2.normalize();

    // compute Plucker line correspondence
    Eigen::Matrix<double,6,1> l1, l2;
    l1 << ray1, m_cameraSystem.getGlobalCameraPose(frame1->cameraId()).block<3,1>(0,3).cross(ray1);
    l2 << ray2, m_cameraSystem.getGlobalCameraPose(frame2->cameraId()).block<3,1>(0,3).cross(ray2);

    Eigen::Vector3d q1 = l1.head(3);
    Eigen::Vector3d q1p = l1.tail(3);

    Eigen::Vector3d q2 = l2.head(3);
    Eigen::Vector3d q2p = l2.tail(3);

    Eigen::Vector3d q1xq1p = q1.cross(q1p);

    Eigen::Matrix4d H;
    H = frame2->systemPose()->toMatrix().inverse() *
        frame1->systemPose()->toMatrix();

    Eigen::MatrixXd A(3,2);
    A.col(0) = H.block<3,3>(0,0) * q1;
    A.col(1) = -q2;

    Eigen::Vector3d b = q2.cross(q2p) - H.block<3,3>(0,0) * q1xq1p - H.block<3,1>(0,3);

    Eigen::Vector2d gamma = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    scenePoint = q1xq1p + gamma(0) * q1;

    if (isnan(scenePoint(0)) || isnan(scenePoint(1)) || isnan(scenePoint(2)))
    {
        return false;
    }

    scenePoint = frame1->systemPose()->toMatrix().block<3,3>(0,0) * scenePoint + frame1->systemPose()->toMatrix().block<3,1>(0,3);

    Pose T_cam1_odo(m_cameraSystem.getGlobalCameraPose(frame1->cameraId()));
    double e1 = reprojectionError(cam1, scenePoint, T_cam1_odo.rotation(), T_cam1_odo.translation(),
                                  frame1->systemPose()->position(), frame1->systemPose()->attitude(),
                                  Eigen::Vector2d(pt1.x, pt1.y));

    Pose T_cam2_odo(m_cameraSystem.getGlobalCameraPose(frame2->cameraId()));
    double e2 = reprojectionError(cam2, scenePoint, T_cam2_odo.rotation(), T_cam2_odo.translation(),
                                  frame2->systemPose()->position(), frame2->systemPose()->attitude(),
                                  Eigen::Vector2d(pt2.x, pt2.y));

    if (e1 > reprojErrorThresh || e2 > reprojErrorThresh)
    {
        return false;
    }

    return true;
}

void
CameraRigBA::prune(int flags, int poseType)
{
    std::vector<Pose, Eigen::aligned_allocator<Pose> > T_cam_odo(m_cameraSystem.cameraCount());
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > H_odo_cam(m_cameraSystem.cameraCount());
    for (size_t i = 0; i < m_cameraSystem.cameraCount(); ++i)
    {
        T_cam_odo.at(i) = m_cameraSystem.getGlobalCameraPose(i);

        H_odo_cam.at(i) = T_cam_odo.at(i).toMatrix().inverse();
    }

    // prune points that are too far away or behind a camera
    for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
    {
        FrameSetSegment& segment = m_graph.frameSetSegment(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            FrameSetPtr& frameSet = segment.at(j);

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                FramePtr& frame = frameSet->frames().at(k);

                if (frame.get() == 0)
                {
                    continue;
                }

                int cameraId = frame->cameraId();

                std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                Eigen::Matrix4d H_cam = Eigen::Matrix4d::Identity();
                if (frame->cameraPose().get() != 0)
                {
                    H_cam = frame->cameraPose()->toMatrix();
                }

                Eigen::Matrix4d H_odo_inv = frame->systemPose()->toMatrix().inverse();

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    Point2DFeaturePtr& pf = features2D.at(l);

                    if (pf->feature3D().get() == 0)
                    {
                        continue;
                    }

                    Eigen::Vector4d P;
                    P << pf->feature3D()->point(), 1.0;

                    Eigen::Vector4d P_cam;
                    if (poseType == CAMERA)
                    {
                        P_cam = H_cam * P;
                    }
                    else
                    {
                        P_cam = (H_odo_cam.at(cameraId) * H_odo_inv) * P;
                    }

                    bool prune = false;

                    if ((flags & PRUNE_BEHIND_CAMERA) == PRUNE_BEHIND_CAMERA &&
                        P_cam(2) < 0.0)
                    {
                        prune = true;
                    }

                    if ((flags & PRUNE_FARAWAY) == PRUNE_FARAWAY &&
                        P_cam.block<3,1>(0,0).norm() > k_maxPoint3DDistance)
                    {
                        prune = true;
                    }

                    if ((flags & PRUNE_HIGH_REPROJ_ERR) == PRUNE_HIGH_REPROJ_ERR)
                    {
                        double error = 0.0;

                        if (poseType == CAMERA)
                        {
                            error = m_cameraSystem.getCamera(cameraId)->reprojectionError(pf->feature3D()->point(),
                                                                                          frame->cameraPose()->rotation(),
                                                                                          frame->cameraPose()->translation(),
                                                                                          Eigen::Vector2d(pf->keypoint().pt.x, pf->keypoint().pt.y));
                        }
                        else
                        {
                            error = reprojectionError(m_cameraSystem.getCamera(cameraId),
                                                      pf->feature3D()->point(),
                                                      T_cam_odo.at(cameraId).rotation(),
                                                      T_cam_odo.at(cameraId).translation(),
                                                      frame->systemPose()->position(),
                                                      frame->systemPose()->attitude(),
                                                      Eigen::Vector2d(pf->keypoint().pt.x, pf->keypoint().pt.y));
                        }

                        if (error > k_maxReprojErr)
                        {
                            prune = true;
                        }
                    }

                    if (prune)
                    {
                        // delete entire feature track
                        std::vector<Point2DFeatureWPtr> features2D = pf->feature3D()->features2D();

                        for (size_t m = 0; m < features2D.size(); ++m)
                        {
                            if (Point2DFeaturePtr feature2D = features2D.at(m).lock())
                            {
                                feature2D->feature3D() = Point3DFeaturePtr();
                            }
                        }
                    }
                }
            }
        }
    }
}

void
CameraRigBA::optimize(int flags, bool optimizeZ, int nIterations)
{
    size_t nPoints = 0;
    size_t nPointsMultipleCams = 0;
    boost::unordered_set<Point3DFeature*> scenePointSet;
    for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
    {
        FrameSetSegment& segment = m_graph.frameSetSegment(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            FrameSetPtr& frameSet = segment.at(j);

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                FramePtr& frame = frameSet->frames().at(k);

                if (frame.get() == 0)
                {
                    continue;
                }

                const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    if (features2D.at(l)->feature3D().get() != 0)
                    {
                        if (features2D.at(l)->feature3D()->attributes() & Point3DFeature::LOCALLY_OBSERVED_BY_DIFFERENT_CAMERAS)
                        {
                            ++nPointsMultipleCams;
                        }

                        ++nPoints;

                        scenePointSet.insert(features2D.at(l)->feature3D().get());
                    }
                }
            }
        }
    }

    bool includeChessboardData = (flags & CAMERA_INTRINSICS) && !m_cameraCalibrations.empty();
    size_t nResidualsChessboard = 0;
    double wResidualChessboard = 0.0;
    if (includeChessboardData)
    {
        for (size_t i = 0; i < m_cameraCalibrations.size(); ++i)
        {
            nResidualsChessboard += m_cameraCalibrations.at(i)->imagePoints().size() *
                                    m_cameraCalibrations.at(i)->imagePoints().front().size();

            if (m_verbose)
            {
                std::vector<cv::Mat> rvecs(m_cameraCalibrations.at(i)->scenePoints().size());
                std::vector<cv::Mat> tvecs(m_cameraCalibrations.at(i)->scenePoints().size());

                for (int j = 0; j < m_cameraCalibrations.at(i)->cameraPoses().rows; ++j)
                {
                    cv::Mat rvec(3, 1, CV_64F);
                    rvec.at<double>(0) =  m_cameraCalibrations.at(i)->cameraPoses().at<double>(j,0);
                    rvec.at<double>(1) =  m_cameraCalibrations.at(i)->cameraPoses().at<double>(j,1);
                    rvec.at<double>(2) =  m_cameraCalibrations.at(i)->cameraPoses().at<double>(j,2);

                    cv::Mat tvec(3, 1, CV_64F);
                    tvec.at<double>(0) =  m_cameraCalibrations.at(i)->cameraPoses().at<double>(j,3);
                    tvec.at<double>(1) =  m_cameraCalibrations.at(i)->cameraPoses().at<double>(j,4);
                    tvec.at<double>(2) =  m_cameraCalibrations.at(i)->cameraPoses().at<double>(j,5);

                    rvecs.at(j) = rvec;
                    tvecs.at(j) = tvec;
                }

                double err = m_cameraSystem.getCamera(i)->reprojectionError(m_cameraCalibrations.at(i)->scenePoints(),
                                                                            m_cameraCalibrations.at(i)->imagePoints(),
                                                                            rvecs, tvecs);
                std::cout << "# INFO: "
                          << "[" << m_cameraSystem.getCamera(i)->cameraName()
                          << "] Initial reprojection error (chessboard): "
                          << err << " pixels" << std::endl;
            }
        }

        wResidualChessboard = static_cast<double>(nPoints - nPointsMultipleCams) / static_cast<double>(nResidualsChessboard);

        if (m_verbose)
        {
            std::cout << "# INFO: Added " << nResidualsChessboard << " residuals from chessboard data." << std::endl;
            std::cout << "# INFO: Assigned weight of " << wResidualChessboard << " to each residual." << std::endl;
        }
    }

    ceres::Problem problem;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = nIterations;
    options.num_threads = 8;
    options.num_linear_solver_threads = 8;

    // intrinsics
    std::vector<double> intrinsicParams[m_cameraSystem.cameraCount()];

    for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
    {
        m_cameraSystem.getCamera(i)->writeParameters(intrinsicParams[i]);
    }

    // extrinsics
    std::vector<Pose, Eigen::aligned_allocator<Pose> > T_cam_odo(m_cameraSystem.cameraCount());
    for (size_t i = 0; i < m_cameraSystem.cameraCount(); ++i)
    {
        T_cam_odo.at(i) = Pose(m_cameraSystem.getGlobalCameraPose(i));
    }

    boost::dynamic_bitset<> optimizeExtrinsics(m_cameraSystem.cameraCount());

    for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
    {
        FrameSetSegment& segment = m_graph.frameSetSegment(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            FrameSetPtr& frameSet = segment.at(j);

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                FramePtr& frame = frameSet->frames().at(k);

                if (frame.get() == 0)
                {
                    continue;
                }

                int cameraId = frame->cameraId();

                std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    Point2DFeaturePtr& feature2D = features2D.at(l);
                    Point3DFeaturePtr& feature3D = feature2D->feature3D();

                    if (feature3D.get() == 0)
                    {
                        continue;
                    }

                    if (isnan(feature3D->point()(0)) || isnan(feature3D->point()(1)) ||
                        isnan(feature3D->point()(2)))
                    {
                        continue;
                    }

                    optimizeExtrinsics[cameraId] = 1;

                    ceres::LossFunction* lossFunction = new ceres::ScaledLoss(new ceres::HuberLoss(1.0), feature3D->weight(), ceres::TAKE_OWNERSHIP);

                    ceres::CostFunction* costFunction;
                    switch (flags)
                    {
                    case POINT_3D:
                    {
                        costFunction
                            = CostFunctionFactory::instance()->generateCostFunction(m_cameraSystem.getCamera(cameraId),
                                                                                    T_cam_odo.at(cameraId).rotation(),
                                                                                    T_cam_odo.at(cameraId).translation(),
                                                                                    frame->systemPose()->position(),
                                                                                    frame->systemPose()->attitude(),
                                                                                    Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y),
                                                                                    flags);

                        problem.AddResidualBlock(costFunction, lossFunction,
                                                 feature3D->pointData());

                        break;
                    }
                    case CAMERA_ODOMETRY_EXTRINSICS | POINT_3D:
                    {
                        costFunction
                            = CostFunctionFactory::instance()->generateCostFunction(m_cameraSystem.getCamera(cameraId),
                                                                                    frame->systemPose()->position(),
                                                                                    frame->systemPose()->attitude(),
                                                                                    Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y),
                                                                                    flags,
                                                                                    optimizeZ);

                        problem.AddResidualBlock(costFunction, lossFunction,
                                                 T_cam_odo.at(cameraId).rotationData(),
                                                 T_cam_odo.at(cameraId).translationData(),
                                                 feature3D->pointData());

                        break;
                    }
                    case CAMERA_ODOMETRY_EXTRINSICS | ODOMETRY_3D_EXTRINSICS | POINT_3D:
                    case CAMERA_ODOMETRY_EXTRINSICS | ODOMETRY_6D_EXTRINSICS | POINT_3D:
                    {
                        costFunction
                            = CostFunctionFactory::instance()->generateCostFunction(m_cameraSystem.getCamera(cameraId),
                                                                                    Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y),
                                                                                    flags,
                                                                                    optimizeZ);

                        problem.AddResidualBlock(costFunction, lossFunction,
                                                 T_cam_odo.at(cameraId).rotationData(),
                                                 T_cam_odo.at(cameraId).translationData(),
                                                 frame->systemPose()->positionData(),
                                                 frame->systemPose()->attitudeData(),
                                                 feature3D->pointData());

                        break;
                    }
                    case CAMERA_INTRINSICS | CAMERA_ODOMETRY_EXTRINSICS | ODOMETRY_3D_EXTRINSICS | POINT_3D:
                    case CAMERA_INTRINSICS | CAMERA_ODOMETRY_EXTRINSICS | ODOMETRY_6D_EXTRINSICS | POINT_3D:
                    {
                        costFunction
                            = CostFunctionFactory::instance()->generateCostFunction(m_cameraSystem.getCamera(cameraId),
                                                                                    Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y),
                                                                                    flags,
                                                                                    optimizeZ);

                        problem.AddResidualBlock(costFunction, lossFunction,
                                                 intrinsicParams[cameraId].data(),
                                                 T_cam_odo.at(cameraId).rotationData(),
                                                 T_cam_odo.at(cameraId).translationData(),
                                                 frame->systemPose()->positionData(),
                                                 frame->systemPose()->attitudeData(),
                                                 feature3D->pointData());

                        break;
                    }
                    case CAMERA_EXTRINSICS | POINT_3D:
                    {
                        costFunction
                            = CostFunctionFactory::instance()->generateCostFunction(m_cameraSystem.getCamera(cameraId),
                                                                                    Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y),
                                                                                    flags);

                        problem.AddResidualBlock(costFunction, lossFunction,
                                                 frame->cameraPose()->rotationData(),
                                                 frame->cameraPose()->translationData(),
                                                 feature3D->pointData());

                        break;
                    }
                    }
                }

                if (flags & CAMERA_EXTRINSICS)
                {
                    ceres::LocalParameterization* quaternionParameterization =
                        new EigenQuaternionParameterization;

                    problem.SetParameterization(frame->cameraPose()->rotationData(), quaternionParameterization);
                }
            }
        }
    }

    if (flags & CAMERA_ODOMETRY_EXTRINSICS)
    {
        for (size_t i = 0; i < m_cameraSystem.cameraCount(); ++i)
        {
            if (optimizeExtrinsics[i])
            {
                ceres::LocalParameterization* quaternionParameterization =
                    new EigenQuaternionParameterization;

                problem.SetParameterization(T_cam_odo.at(i).rotationData(), quaternionParameterization);
            }
        }
    }

    std::vector<std::vector<double> > chessboardCameraPoses[m_cameraSystem.cameraCount()];

    if (includeChessboardData)
    {
        for (size_t i = 0; i < m_cameraCalibrations.size(); ++i)
        {
            cv::Mat& cameraPoses = m_cameraCalibrations.at(i)->cameraPoses();
            chessboardCameraPoses[i].resize(cameraPoses.rows);

            std::vector<std::vector<cv::Point2f> >& imagePoints = m_cameraCalibrations.at(i)->imagePoints();
            std::vector<std::vector<cv::Point3f> >& scenePoints = m_cameraCalibrations.at(i)->scenePoints();

            // create residuals for each observation
            for (size_t j = 0; j < imagePoints.size(); ++j)
            {
                chessboardCameraPoses[i].at(j).resize(7);

                Eigen::Vector3d rvec(cameraPoses.at<double>(j,0),
                                     cameraPoses.at<double>(j,1),
                                     cameraPoses.at<double>(j,2));

                AngleAxisToQuaternion(rvec, chessboardCameraPoses[i].at(j).data());

                chessboardCameraPoses[i].at(j).at(4) = cameraPoses.at<double>(j,3);
                chessboardCameraPoses[i].at(j).at(5) = cameraPoses.at<double>(j,4);
                chessboardCameraPoses[i].at(j).at(6) = cameraPoses.at<double>(j,5);

                for (size_t k = 0; k < imagePoints.at(j).size(); ++k)
                {
                    const cv::Point3f& spt = scenePoints.at(j).at(k);
                    const cv::Point2f& ipt = imagePoints.at(j).at(k);

                    ceres::CostFunction* costFunction =
                        CostFunctionFactory::instance()->generateCostFunction(m_cameraSystem.getCamera(i),
                                                                              Eigen::Vector3d(spt.x, spt.y, spt.z),
                                                                              Eigen::Vector2d(ipt.x, ipt.y),
                                                                              CAMERA_INTRINSICS | CAMERA_EXTRINSICS);

                    ceres::LossFunction* lossFunction = new ceres::ScaledLoss(new ceres::CauchyLoss(1.0), wResidualChessboard, ceres::TAKE_OWNERSHIP);
                    problem.AddResidualBlock(costFunction, lossFunction,
                                             intrinsicParams[i].data(),
                                             chessboardCameraPoses[i].at(j).data(),
                                             chessboardCameraPoses[i].at(j).data() + 4);
                }

                ceres::LocalParameterization* quaternionParameterization =
                    new EigenQuaternionParameterization;

                problem.SetParameterization(chessboardCameraPoses[i].at(j).data(),
                                            quaternionParameterization);
            }
        }
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (m_verbose)
    {
        std::cout << summary.BriefReport() << std::endl;
    }

    bool computeCovariances = false;
    if (flags & ODOMETRY_6D_EXTRINSICS)
    {
        computeCovariances = true;
    }
    if (flags & POINT_3D)
    {
        computeCovariances = true;
    }

    if (0)
//    if (computeCovariances)
    {
        if (m_verbose)
        {
            std::cout << "# INFO: Computing covariances... " << std::flush;
        }

        // compute covariances
        ceres::Covariance::Options covOptions;
        covOptions.num_threads = 8;
        covOptions.algorithm_type = ceres::SPARSE_QR;
        covOptions.apply_loss_function = true;

        ceres::Covariance covariance(covOptions);

        std::vector<std::pair<const double*, const double*> > covarianceBlocks;

        if (flags & ODOMETRY_6D_EXTRINSICS)
        {
            for (size_t i = 0; i < m_cameraSystem.cameraCount(); ++i)
            {
                covarianceBlocks.push_back(std::make_pair(T_cam_odo.at(i).rotationData(),
                                                          T_cam_odo.at(i).rotationData()));
                covarianceBlocks.push_back(std::make_pair(T_cam_odo.at(i).rotationData(),
                                                          T_cam_odo.at(i).translationData()));
                covarianceBlocks.push_back(std::make_pair(T_cam_odo.at(i).translationData(),
                                                          T_cam_odo.at(i).translationData()));
            }
        }

        if (flags & POINT_3D)
        {
            for (boost::unordered_set<Point3DFeature*>::iterator it = scenePointSet.begin();
                    it != scenePointSet.end(); ++it)
            {
                covarianceBlocks.push_back(std::make_pair((*it)->pointData(),
                                                          (*it)->pointData()));
            }
        }

        if (covariance.Compute(covarianceBlocks, &problem))
        {
            if (flags & ODOMETRY_6D_EXTRINSICS)
            {
                for (size_t i = 0; i < m_cameraSystem.cameraCount(); ++i)
                {
                    double covariance_rr[4*4];
                    double covariance_rt[4*3];
                    double covariance_tt[3*3];

                    covariance.GetCovarianceBlock(T_cam_odo.at(i).rotationData(),
                                                  T_cam_odo.at(i).rotationData(),
                                                  covariance_rr);
                    covariance.GetCovarianceBlock(T_cam_odo.at(i).rotationData(),
                                                  T_cam_odo.at(i).translationData(),
                                                  covariance_rt);
                    covariance.GetCovarianceBlock(T_cam_odo.at(i).translationData(),
                                                  T_cam_odo.at(i).translationData(),
                                                  covariance_tt);

                    memcpy(T_cam_odo.at(i).covariance().block<4,4>(0,0).data(), covariance_rr, sizeof(double) * 16);
                    memcpy(T_cam_odo.at(i).covariance().block<4,3>(0,4).data(), covariance_rt, sizeof(double) * 12);
                    memcpy(T_cam_odo.at(i).covariance().block<3,3>(4,4).data(), covariance_tt, sizeof(double) * 9);
                    T_cam_odo.at(i).covariance().block<3,4>(4,0) = T_cam_odo.at(i).covariance().block<4,3>(0,4).transpose();
                }
            }

            if (flags & POINT_3D)
            {
                for (boost::unordered_set<Point3DFeature*>::iterator it = scenePointSet.begin();
                        it != scenePointSet.end(); ++it)
                {
                    double covariance_PP[3*3];

                    covariance.GetCovarianceBlock((*it)->pointData(),
                                                  (*it)->pointData(),
                                                  covariance_PP);

                    memcpy((*it)->pointCovarianceData(), covariance_PP, sizeof(double) * 9);
                }
            }

            if (m_verbose)
            {
                std::cout << "Finished." << std::endl;
            }
        }
        else
        {
            std::cout << std::endl << "# ERROR: Ceres was unable to compute the covariances." << std::endl;
        }
    }

    if (flags & CAMERA_ODOMETRY_EXTRINSICS)
    {
        for (size_t i = 0; i < m_cameraSystem.cameraCount(); ++i)
        {
            m_cameraSystem.setGlobalCameraPose(i, T_cam_odo.at(i).toMatrix());
        }
    }

    if (flags & CAMERA_INTRINSICS)
    {
        for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
        {
            m_cameraSystem.getCamera(i)->readParameters(intrinsicParams[i]);
        }
    }

    if (includeChessboardData && m_verbose)
    {
        for (size_t i = 0; i < m_cameraCalibrations.size(); ++i)
        {
            std::vector<cv::Mat> rvecs(m_cameraCalibrations.at(i)->scenePoints().size());
            std::vector<cv::Mat> tvecs(m_cameraCalibrations.at(i)->scenePoints().size());

            for (size_t j = 0; j < chessboardCameraPoses[i].size(); ++j)
            {
                Eigen::Vector3d rvec;
                QuaternionToAngleAxis(chessboardCameraPoses[i].at(j).data(), rvec);
                cv::eigen2cv(rvec, rvecs.at(j));

                tvecs.at(j) = cv::Mat(3, 1, CV_64F);
                cv::Mat& tvec = tvecs.at(j);
                tvec.at<double>(0) = chessboardCameraPoses[i].at(j).at(4);
                tvec.at<double>(1) = chessboardCameraPoses[i].at(j).at(5);
                tvec.at<double>(2) = chessboardCameraPoses[i].at(j).at(6);
            }

            double err = m_cameraSystem.getCamera(i)->reprojectionError(m_cameraCalibrations.at(i)->scenePoints(),
                                                                        m_cameraCalibrations.at(i)->imagePoints(),
                                                                        rvecs, tvecs);
            std::cout << "# INFO: "
                      << "[" << m_cameraSystem.getCamera(i)->cameraName()
                      << "] Final reprojection error (chessboard): "
                      << err << " pixels" << std::endl;
        }
    }
}

void
CameraRigBA::reweightScenePoints(void)
{
    // Assign high weights to scene points observed by multiple cameras.
    // Note that the observation condition only applies to scene points
    // *locally* observed by multiple cameras.

    size_t nPoints = 0;
    size_t nPointsMultipleCams = 0;
    boost::unordered_set<Point3DFeature*> scenePointSet;
    for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
    {
        FrameSetSegment& segment = m_graph.frameSetSegment(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            FrameSetPtr& frameSet = segment.at(j);

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                FramePtr& frame = frameSet->frames().at(k);

                if (frame.get() == 0)
                {
                    continue;
                }

                const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    if (features2D.at(l)->feature3D().get() != 0)
                    {
                        if (features2D.at(l)->feature3D()->attributes() & Point3DFeature::LOCALLY_OBSERVED_BY_DIFFERENT_CAMERAS)
                        {
                            ++nPointsMultipleCams;
                        }

                        ++nPoints;

                        scenePointSet.insert(features2D.at(l)->feature3D().get());
                    }
                }
            }
        }
    }

    double weightM = static_cast<double>(nPoints - nPointsMultipleCams) / static_cast<double>(nPointsMultipleCams);

    for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
    {
        FrameSetSegment& segment = m_graph.frameSetSegment(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            FrameSetPtr& frameSet = segment.at(j);

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                FramePtr& frame = frameSet->frames().at(k);

                if (frame.get() == 0)
                {
                    continue;
                }

                const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    Point3DFeaturePtr& feature3D = features2D.at(l)->feature3D();

                    if (feature3D.get() == 0)
                    {
                        continue;
                    }

                    if (feature3D->attributes() & Point3DFeature::LOCALLY_OBSERVED_BY_DIFFERENT_CAMERAS)
                    {
                        feature3D->weight() = weightM;
                    }
                    else
                    {
                        feature3D->weight() = 1.0;
                    }
                }
            }
        }
    }

    if (m_verbose)
    {
        std::cout << "# INFO: Assigned weight of " << weightM
                  << " to scene points observed by multiple cameras."
                  << std::endl;
    }
}

bool
CameraRigBA::estimateCameraOdometryTransforms(void)
{
    PlanarHandEyeCalibration calib;
    calib.setVerbose(m_verbose);

    for (int segmentId = 0; segmentId < m_graph.frameSetSegments().size(); ++segmentId)
    {
        FrameSetSegment& segment = m_graph.frameSetSegment(segmentId);

        bool init = false;
        Eigen::Matrix4d odoMeasPrev, odoEstPrev;
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > H_odoMeas, H_odoEst;

        for (size_t frameSetId = 0; frameSetId < segment.size(); ++frameSetId)
        {
            FrameSetPtr& frameSet = segment.at(frameSetId);

            Eigen::Matrix4d odoMeas = frameSet->odometryMeasurement()->toMatrix();
            Eigen::Matrix4d odoEst = frameSet->systemPose()->toMatrix();

            if (init)
            {
                H_odoMeas.push_back(odoMeas.inverse() * odoMeasPrev);
                H_odoEst.push_back(odoEst.inverse() * odoEstPrev);
            }
            else
            {
                init = true;
            }

            odoMeasPrev = odoMeas;
            odoEstPrev = odoEst;
        }

        calib.addMotions(H_odoEst, H_odoMeas);
    }

    Eigen::Matrix4d H_odoEst_odoMeas;
    if (!calib.calibrate(H_odoEst_odoMeas))
    {
        return false;
    }

    for (size_t i = 0; i < m_cameraSystem.cameraCount(); ++i)
    {
        Eigen::Matrix4d H_cam_odoEst = m_cameraSystem.getGlobalCameraPose(i);

        Eigen::Matrix4d H_cam_odoMeas = H_odoEst_odoMeas * H_cam_odoEst;

        m_cameraSystem.setGlobalCameraPose(i, H_cam_odoMeas);
    }

    return true;
}

bool
CameraRigBA::estimateAbsoluteGroundHeight(double& zGround) const
{
    return false;
}

#ifdef VCHARGE_VIZ
void
CameraRigBA::visualize(const std::string& overlayPrefix, int type)
{
    for (size_t i = 0; i < m_cameraSystem.cameraCount(); ++i)
    {
        std::ostringstream oss;
        oss << overlayPrefix << i + 1;

        std::vector<OdometryPtr> odometry;
        std::vector<PosePtr> cameraPoses;
        boost::unordered_set<Point3DFeature*> scenePointSet;

        for (size_t j = 0; j < m_graph.frameSetSegments().size(); ++j)
        {
            FrameSetSegment& segment = m_graph.frameSetSegment(j);

            for (size_t k = 0; k < segment.size(); ++k)
            {
                FrameSetPtr& frameSet = segment.at(k);

                FramePtr& frame = frameSet->frames().at(i);

                if (frame.get() == 0)
                {
                    continue;
                }

                if (type == ODOMETRY)
                {
                    odometry.push_back(frame->systemPose());
                }
                else
                {
                    cameraPoses.push_back(frame->cameraPose());
                }

                std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    if (features2D.at(l)->feature3D().get() == 0)
                    {
                        continue;
                    }

                    if (features2D.at(l)->feature3D()->point().norm() < 1000.0)
                    {
                        scenePointSet.insert(features2D.at(l)->feature3D().get());
                    }
                }
            }
        }

        Eigen::Vector3d origin;
        if (type == ODOMETRY)
        {
            origin = odometry.front()->position();
        }
        else
        {
            origin = cameraPoses.front()->translation();
        }

        vcharge::GLOverlayExtended overlay(oss.str(), VCharge::COORDINATE_FRAME_LOCAL);

        // visualize camera poses and 3D scene points
        overlay.clear();
        overlay.setOrigin(origin(0), origin(1), origin(2));
        overlay.pointSize(2.0f);
        overlay.lineWidth(1.0f);

        if (type == ODOMETRY)
        {
            for (size_t j = 0; j < odometry.size(); ++j)
            {
                OdometryPtr& odo = odometry.at(j);

                Eigen::Matrix4d H_odo = odo->toMatrix();
                Eigen::Matrix4d H_cam = H_odo * m_cameraSystem.getGlobalCameraPose(i);

                double xBound = 0.1;
                double yBound = 0.1;
                double zFar = 0.2;

                std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > frustum;
                frustum.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
                frustum.push_back(Eigen::Vector3d(-xBound, -yBound, zFar));
                frustum.push_back(Eigen::Vector3d(xBound, -yBound, zFar));
                frustum.push_back(Eigen::Vector3d(xBound, yBound, zFar));
                frustum.push_back(Eigen::Vector3d(-xBound, yBound, zFar));

                for (size_t k = 0; k < frustum.size(); ++k)
                {
                    frustum.at(k) = transformPoint(H_cam, frustum.at(k)) - origin;
                }

                overlay.color4f(1.0f, 1.0f, 1.0f, 1.0f);
                overlay.begin(VCharge::LINES);

                for (int k = 1; k < 5; ++k)
                {
                    overlay.vertex3f(frustum.at(0)(0), frustum.at(0)(1), frustum.at(0)(2));
                    overlay.vertex3f(frustum.at(k)(0), frustum.at(k)(1), frustum.at(k)(2));
                }

                overlay.end();

                switch (i)
                {
                case vcharge::CAMERA_FRONT:
                    overlay.color4f(1.0f, 0.0f, 0.0f, 0.5f);
                    break;
                case vcharge::CAMERA_LEFT:
                    overlay.color4f(0.0f, 1.0f, 0.0f, 0.5f);
                    break;
                case vcharge::CAMERA_REAR:
                    overlay.color4f(0.0f, 1.0f, 1.0f, 0.5f);
                    break;
                case vcharge::CAMERA_RIGHT:
                    overlay.color4f(1.0f, 1.0f, 0.0f, 0.5f);
                    break;
                default:
                    overlay.color4f(1.0f, 1.0f, 1.0f, 0.5f);
                }

                overlay.begin(VCharge::POLYGON);

                for (int k = 1; k < 5; ++k)
                {
                    overlay.vertex3f(frustum.at(k)(0), frustum.at(k)(1), frustum.at(k)(2));
                }

                overlay.end();
            }

            overlay.lineWidth(1.0f);
            overlay.pointSize(2.0f);
        }
        else
        {
            for (size_t j = 0; j < cameraPoses.size(); ++j)
            {
                PosePtr& cameraPose = cameraPoses.at(j);

                Eigen::Matrix4d H_cam = cameraPose->toMatrix().inverse();

                double xBound = 0.1;
                double yBound = 0.1;
                double zFar = 0.2;

                std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > frustum;
                frustum.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
                frustum.push_back(Eigen::Vector3d(-xBound, -yBound, zFar));
                frustum.push_back(Eigen::Vector3d(xBound, -yBound, zFar));
                frustum.push_back(Eigen::Vector3d(xBound, yBound, zFar));
                frustum.push_back(Eigen::Vector3d(-xBound, yBound, zFar));

                for (size_t k = 0; k < frustum.size(); ++k)
                {
                    frustum.at(k) = transformPoint(H_cam, frustum.at(k)) - origin;
                }

                overlay.color4f(1.0f, 1.0f, 1.0f, 1.0f);
                overlay.begin(VCharge::LINES);

                for (int k = 1; k < 5; ++k)
                {
                    overlay.vertex3f(frustum.at(0)(0), frustum.at(0)(1), frustum.at(0)(2));
                    overlay.vertex3f(frustum.at(k)(0), frustum.at(k)(1), frustum.at(k)(2));
                }

                overlay.end();

                switch (i)
                {
                case vcharge::CAMERA_FRONT:
                    overlay.color4f(1.0f, 0.0f, 0.0f, 0.5f);
                    break;
                case vcharge::CAMERA_LEFT:
                    overlay.color4f(0.0f, 1.0f, 0.0f, 0.5f);
                    break;
                case vcharge::CAMERA_REAR:
                    overlay.color4f(0.0f, 1.0f, 1.0f, 0.5f);
                    break;
                case vcharge::CAMERA_RIGHT:
                    overlay.color4f(1.0f, 1.0f, 0.0f, 0.5f);
                    break;
                default:
                    overlay.color4f(1.0f, 1.0f, 1.0f, 0.5f);
                }

                overlay.begin(VCharge::POLYGON);

                for (int k = 1; k < 5; ++k)
                {
                    overlay.vertex3f(frustum.at(k)(0), frustum.at(k)(1), frustum.at(k)(2));
                }

                overlay.end();
            }
        }

        // draw 3D scene points
        switch (i)
        {
        case vcharge::CAMERA_FRONT:
            overlay.color4f(1.0f, 0.0f, 0.0f, 0.5f);
            break;
        case vcharge::CAMERA_LEFT:
            overlay.color4f(0.0f, 1.0f, 0.0f, 0.5f);
            break;
        case vcharge::CAMERA_REAR:
            overlay.color4f(0.0f, 1.0f, 1.0f, 0.5f);
            break;
        case vcharge::CAMERA_RIGHT:
            overlay.color4f(1.0f, 1.0f, 0.0f, 0.5f);
            break;
        default:
            overlay.color4f(1.0f, 1.0f, 1.0f, 0.5f);
        }

        overlay.begin(VCharge::POINTS);

        for (boost::unordered_set<Point3DFeature*>::iterator it = scenePointSet.begin();
                 it != scenePointSet.end(); ++it)
        {
            Eigen::Vector3d p = (*it)->point() - origin;

            overlay.vertex3f(p(0), p(1), p(2));
        }

        overlay.end();

        overlay.publish();

        usleep(50000);
    }

    std::ostringstream oss;
    oss << overlayPrefix << "odo";

    visualizeSystemPoses(oss.str());

    usleep(50000);
}

void
CameraRigBA::visualizeExtrinsics(const std::string& overlayName)
{
    vcharge::GLOverlayExtended overlay(overlayName, VCharge::COORDINATE_FRAME_GLOBAL);

    // visualize extrinsics
    overlay.clear();
    overlay.lineWidth(1.0f);

    // x-axis
    overlay.color4f(1.0f, 0.0f, 0.0f, 1.0f);
    overlay.begin(VCharge::LINES);
    overlay.vertex3f(0.0f, 0.0f, 0.0f);
    overlay.vertex3f(0.3f, 0.0f, 0.0f);
    overlay.end();

    // y-axis
    overlay.color4f(0.0f, 1.0f, 0.0f, 1.0f);
    overlay.begin(VCharge::LINES);
    overlay.vertex3f(0.0f, 0.0f, 0.0f);
    overlay.vertex3f(0.0f, 0.3f, 0.0f);
    overlay.end();

    // z-axis
    overlay.color4f(0.0f, 0.0f, 1.0f, 1.0f);
    overlay.begin(VCharge::LINES);
    overlay.vertex3f(0.0f, 0.0f, 0.0f);
    overlay.vertex3f(0.0f, 0.0f, 0.3f);
    overlay.end();

    double z_ref = 0.0;
    for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
    {
        Eigen::Matrix4d H_cam = m_cameraSystem.getGlobalCameraPose(i);

        z_ref += H_cam(2,3);
    }
    z_ref /= m_cameraSystem.cameraCount();

    for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
    {
        Eigen::Matrix4d H_cam = m_cameraSystem.getGlobalCameraPose(i);

        double xBound = 0.1;
        double yBound = 0.1;
        double zFar = 0.2;

        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > frustum;
        frustum.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
        frustum.push_back(Eigen::Vector3d(-xBound, -yBound, zFar));
        frustum.push_back(Eigen::Vector3d(xBound, -yBound, zFar));
        frustum.push_back(Eigen::Vector3d(xBound, yBound, zFar));
        frustum.push_back(Eigen::Vector3d(-xBound, yBound, zFar));

        for (size_t k = 0; k < frustum.size(); ++k)
        {
            frustum.at(k) = transformPoint(H_cam, frustum.at(k));
            frustum.at(k)(2) -= z_ref;
        }

        overlay.color4f(1.0f, 1.0f, 1.0f, 1.0f);
        overlay.begin(VCharge::LINES);

        for (int k = 1; k < 5; ++k)
        {
            overlay.vertex3f(frustum.at(0)(0), frustum.at(0)(1), frustum.at(0)(2));
            overlay.vertex3f(frustum.at(k)(0), frustum.at(k)(1), frustum.at(k)(2));
        }

        overlay.end();

        switch (i)
        {
        case vcharge::CAMERA_FRONT:
            overlay.color4f(1.0f, 0.0f, 0.0f, 0.5f);
            break;
        case vcharge::CAMERA_LEFT:
            overlay.color4f(0.0f, 1.0f, 0.0f, 0.5f);
            break;
        case vcharge::CAMERA_REAR:
            overlay.color4f(0.0f, 1.0f, 1.0f, 0.5f);
            break;
        case vcharge::CAMERA_RIGHT:
            overlay.color4f(1.0f, 1.0f, 0.0f, 0.5f);
            break;
        default:
            overlay.color4f(1.0f, 1.0f, 1.0f, 0.5f);
        }

        overlay.begin(VCharge::POLYGON);

        for (int k = 1; k < 5; ++k)
        {
            overlay.vertex3f(frustum.at(k)(0), frustum.at(k)(1), frustum.at(k)(2));
        }

        overlay.end();
    }

    overlay.publish();
}

void
CameraRigBA::visualizeFrameFrameCorrespondences(const std::string& overlayName,
                                                const std::vector<std::pair<FramePtr, FramePtr> >& correspondencesFrameFrame) const
{
    vcharge::GLOverlayExtended overlay(overlayName, VCharge::COORDINATE_FRAME_GLOBAL);

    // visualize camera poses and 3D scene points
    overlay.clear();
    overlay.lineWidth(2.0f);

    for (size_t i = 0; i < correspondencesFrameFrame.size(); ++i)
    {
        const FramePtr& frame1 = correspondencesFrameFrame.at(i).first;
        const FramePtr& frame2 = correspondencesFrameFrame.at(i).second;
        int cameraId1 = frame1->cameraId();
        int cameraId2 = frame2->cameraId();

        const Eigen::Matrix4d& H_cam_odo1 = m_cameraSystem.getGlobalCameraPose(cameraId1);
        const Eigen::Matrix4d& H_cam_odo2 = m_cameraSystem.getGlobalCameraPose(cameraId2);

        Eigen::Matrix4d H_cam1 = frame1->systemPose()->toMatrix() * H_cam_odo1;
        Eigen::Matrix4d H_cam2 = frame2->systemPose()->toMatrix() * H_cam_odo2;

        overlay.begin(VCharge::LINES);

        switch (cameraId1)
        {
        case vcharge::CAMERA_FRONT:
            overlay.color4f(1.0f, 0.0f, 0.0f, 0.5f);
            break;
        case vcharge::CAMERA_LEFT:
            overlay.color4f(0.0f, 1.0f, 0.0f, 0.5f);
            break;
        case vcharge::CAMERA_REAR:
            overlay.color4f(0.0f, 1.0f, 1.0f, 0.5f);
            break;
        case vcharge::CAMERA_RIGHT:
            overlay.color4f(1.0f, 1.0f, 0.0f, 0.5f);
            break;
        default:
            overlay.color4f(1.0f, 1.0f, 1.0f, 0.5f);
        }

        overlay.vertex3d(H_cam1(0,3), H_cam1(1,3), H_cam1(2,3));

        switch (cameraId2)
        {
        case vcharge::CAMERA_FRONT:
            overlay.color4f(1.0f, 0.0f, 0.0f, 0.5f);
            break;
        case vcharge::CAMERA_LEFT:
            overlay.color4f(0.0f, 1.0f, 0.0f, 0.5f);
            break;
        case vcharge::CAMERA_REAR:
            overlay.color4f(0.0f, 1.0f, 1.0f, 0.5f);
            break;
        case vcharge::CAMERA_RIGHT:
            overlay.color4f(1.0f, 1.0f, 0.0f, 0.5f);
            break;
        default:
            overlay.color4f(1.0f, 1.0f, 1.0f, 0.5f);
        }

        overlay.vertex3d(H_cam2(0,3), H_cam2(1,3), H_cam2(2,3));
        overlay.end();
    }

    overlay.publish();
}

void
CameraRigBA::visualizeSystemPoses(const std::string& overlayName)
{
    std::vector<OdometryPtr> odometryVec;

    for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
    {
        FrameSetSegment& segment = m_graph.frameSetSegment(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            FrameSetPtr& frameSet = segment.at(j);

            odometryVec.push_back(segment.at(j)->systemPose());
        }
    }

    vcharge::GLOverlayExtended overlay(overlayName, VCharge::COORDINATE_FRAME_LOCAL);

    Eigen::Vector3d origin = odometryVec.front()->position();

    overlay.clear();
    overlay.setOrigin(origin(0), origin(1), origin(2));
    overlay.lineWidth(1.0f);
    overlay.color3f(0.7f, 0.7f, 0.7f);

    double w_2 = 0.05;
    double l_2 = 0.1;

    double vertices[4][3] = {{-l_2, -w_2, 0.0},
                             {l_2, -w_2, 0.0},
                             {l_2, w_2, 0.0},
                             {-l_2, w_2, 0.0}};

    for (std::vector<OdometryPtr>::iterator it = odometryVec.begin();
             it != odometryVec.end(); ++it)
    {
        OdometryPtr& odo = *it;

        Eigen::Matrix4d H = odo->toMatrix();

        overlay.begin(VCharge::LINE_LOOP);

        for (int i = 0; i < 4; ++i)
        {
            Eigen::Vector3d p;
            p << vertices[i][0], vertices[i][1], vertices[i][2];

            p = transformPoint(H, p) - origin;

            overlay.vertex3f(p(0), p(1), p(2));
        }

        overlay.end();

        Eigen::Vector3d p0(0.0, 0.0, 0.0);
        Eigen::Vector3d p1(l_2, 0.0, 0.0);

        p0 = transformPoint(H, p0) - origin;
        p1 = transformPoint(H, p1) - origin;

        overlay.begin(VCharge::LINES);

        overlay.vertex3f(p0(0), p0(1), p0(2));
        overlay.vertex3f(p1(0), p1(1), p1(2));

        overlay.end();
    }

    overlay.publish();
}

void
CameraRigBA::visualize2D3DCorrespondences(const std::string& overlayName,
                                          const std::vector<Correspondence2D3D>& correspondences) const
{
    vcharge::GLOverlayExtended overlay(overlayName, VCharge::COORDINATE_FRAME_GLOBAL);

    // visualize 3D-3D correspondences
    overlay.clear();
    overlay.lineWidth(1.0f);

    overlay.begin(VCharge::LINES);

    for (size_t i = 0; i < correspondences.size(); ++i)
    {
        const FramePtr& frame1 = correspondences.at(i).get<0>();
        const FramePtr& frame2 = correspondences.at(i).get<1>();

        int cameraId1 = frame1->cameraId();
        int cameraId2 = frame2->cameraId();

        Eigen::Vector3d p2 = correspondences.at(i).get<3>()->point();

        const Eigen::Matrix4d& H_cam_odo1 = m_cameraSystem.getGlobalCameraPose(cameraId1);
        const Eigen::Matrix4d& H_cam_odo2 = m_cameraSystem.getGlobalCameraPose(cameraId2);

        Eigen::Matrix4d H_cam1 = frame1->systemPose()->toMatrix() * H_cam_odo1;
        Eigen::Matrix4d H_cam2 = frame2->systemPose()->toMatrix() * H_cam_odo2;

        switch (cameraId1)
        {
        case vcharge::CAMERA_FRONT:
            overlay.color4f(1.0f, 0.0f, 0.0f, 0.3f);
            break;
        case vcharge::CAMERA_LEFT:
            overlay.color4f(0.0f, 1.0f, 0.0f, 0.3f);
            break;
        case vcharge::CAMERA_REAR:
            overlay.color4f(0.0f, 1.0f, 1.0f, 0.3f);
            break;
        case vcharge::CAMERA_RIGHT:
            overlay.color4f(1.0f, 1.0f, 0.0f, 0.3f);
            break;
        default:
            overlay.color4f(1.0f, 1.0f, 1.0f, 0.3f);
        }

        overlay.vertex3d(H_cam1(0,3), H_cam1(1,3), H_cam1(2,3));

        switch (cameraId2)
        {
        case vcharge::CAMERA_FRONT:
            overlay.color4f(1.0f, 0.0f, 0.0f, 0.7f);
            break;
        case vcharge::CAMERA_LEFT:
            overlay.color4f(0.0f, 1.0f, 0.0f, 0.7f);
            break;
        case vcharge::CAMERA_REAR:
            overlay.color4f(0.0f, 1.0f, 1.0f, 0.7f);
            break;
        case vcharge::CAMERA_RIGHT:
            overlay.color4f(1.0f, 1.0f, 0.0f, 0.7f);
            break;
        default:
            overlay.color4f(1.0f, 1.0f, 1.0f, 0.7f);
        }

        overlay.vertex3d(p2(0), p2(1), p2(2));
    }

    overlay.end();

    overlay.publish();
}

void
CameraRigBA::visualize3D3DCorrespondences(const std::string& overlayName,
                                          const std::vector<Correspondence3D3D>& correspondences) const
{
    vcharge::GLOverlayExtended overlay(overlayName, VCharge::COORDINATE_FRAME_GLOBAL);

    // visualize 3D-3D correspondences
    overlay.clear();
    overlay.lineWidth(1.0f);

    overlay.begin(VCharge::LINES);

    for (size_t i = 0; i < correspondences.size(); ++i)
    {
        const FramePtr& frame1 = correspondences.at(i).get<0>();
        const FramePtr& frame2 = correspondences.at(i).get<1>();

        int cameraId1 = frame1->cameraId();
        int cameraId2 = frame2->cameraId();

        Eigen::Vector3d p1 = correspondences.at(i).get<2>()->point();
        Eigen::Vector3d p2 = correspondences.at(i).get<3>()->point();

        const Eigen::Matrix4d& H_cam_odo1 = m_cameraSystem.getGlobalCameraPose(cameraId1);
        const Eigen::Matrix4d& H_cam_odo2 = m_cameraSystem.getGlobalCameraPose(cameraId2);

        Eigen::Matrix4d H_cam1 = frame1->systemPose()->toMatrix() * H_cam_odo1;
        Eigen::Matrix4d H_cam2 = frame2->systemPose()->toMatrix() * H_cam_odo2;

        switch (cameraId1)
        {
        case vcharge::CAMERA_FRONT:
            overlay.color4f(1.0f, 0.0f, 0.0f, 0.3f);
            break;
        case vcharge::CAMERA_LEFT:
            overlay.color4f(0.0f, 1.0f, 0.0f, 0.3f);
            break;
        case vcharge::CAMERA_REAR:
            overlay.color4f(0.0f, 1.0f, 1.0f, 0.3f);
            break;
        case vcharge::CAMERA_RIGHT:
            overlay.color4f(1.0f, 1.0f, 0.0f, 0.3f);
            break;
        default:
            overlay.color4f(1.0f, 1.0f, 1.0f, 0.3f);
        }

        overlay.vertex3d(H_cam1(0,3), H_cam1(1,3), H_cam1(2,3));
        overlay.vertex3d(p1(0), p1(1), p1(2));

        overlay.color3f(1.0f, 1.0f, 1.0f);

        overlay.vertex3d(p1(0), p1(1), p1(2));
        overlay.vertex3d(p2(0), p2(1), p2(2));

        switch (cameraId2)
        {
        case vcharge::CAMERA_FRONT:
            overlay.color4f(1.0f, 0.0f, 0.0f, 0.7f);
            break;
        case vcharge::CAMERA_LEFT:
            overlay.color4f(0.0f, 1.0f, 0.0f, 0.7f);
            break;
        case vcharge::CAMERA_REAR:
            overlay.color4f(0.0f, 1.0f, 1.0f, 0.7f);
            break;
        case vcharge::CAMERA_RIGHT:
            overlay.color4f(1.0f, 1.0f, 0.0f, 0.7f);
            break;
        default:
            overlay.color4f(1.0f, 1.0f, 1.0f, 0.7f);
        }

        overlay.vertex3d(p2(0), p2(1), p2(2));
        overlay.vertex3d(H_cam2(0,3), H_cam2(1,3), H_cam2(2,3));
    }

    overlay.end();

    overlay.publish();
}

void
CameraRigBA::visualize3D3DCorrespondences(const std::string& overlayName,
                                          const std::vector<Correspondence2D2D>& correspondences2D2D) const
{
    std::vector<Correspondence3D3D> correspondences3D3D;
    correspondences3D3D.reserve(correspondences2D2D.size());

    for (size_t i = 0; i < correspondences2D2D.size(); ++i)
    {
        const Point2DFeaturePtr& p1 = correspondences2D2D.at(i).first;
        const Point2DFeaturePtr& p2 = correspondences2D2D.at(i).second;

        if (p1->feature3D().get() == 0 || p2->feature3D().get() == 0)
        {
            continue;
        }

        correspondences3D3D.push_back(boost::make_tuple(p1->frame(), p2->frame(),
                                                        p1->feature3D(), p2->feature3D()));
    }

    visualize3D3DCorrespondences(overlayName, correspondences3D3D);
}

void
CameraRigBA::visualizeGroundPoints(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points) const
{
    vcharge::GLOverlayExtended overlay("ground-pts", VCharge::COORDINATE_FRAME_GLOBAL);

    // visualize 3D-3D correspondences
    overlay.clear();
    overlay.pointSize(3.0f);
    overlay.color3f(0.0f, 1.0f, 0.0f);

    overlay.begin(VCharge::POINTS);

    for (size_t i = 0; i < points.size(); ++i)
    {
        const Eigen::Vector3d& p = points.at(i);

        overlay.vertex3d(p(0), p(1), p(2));
    }

    overlay.end();

    overlay.publish();
}

#endif

bool
CameraRigBA::validateGraph(void) const
{
    bool valid = true;

    for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
    {
        const FrameSetSegment& segment = m_graph.frameSetSegment(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            const FrameSetPtr& frameSet = segment.at(j);

            if (j < segment.size() - 1)
            {
                if (segment.at(j)->systemPose().get() == segment.at(j + 1)->systemPose().get())
                {
                    std::cout << "# WARNING: Adjacent frame sets " << j + 1
                              << " and " << j + 2
                              << " have the same system pose reference." << std::endl;
                    valid = false;
                }
            }

            OdometryConstPtr systemPose;
            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                const FramePtr& frame = frameSet->frames().at(k);

                if (frame.get() == 0)
                {
                    continue;
                }

                if (systemPose.get() == 0)
                {
                    systemPose = frame->systemPose();
                }
                else
                {
                    if (systemPose.get() != frame->systemPose().get())
                    {
                        std::cout << "# WARNING: Frames in frame set " << j + 1 << " do not have the same system pose reference." << std::endl;
                        valid = false;
                    }
                }

                if (frame->gpsInsMeasurement().get() != frameSet->gpsInsMeasurement().get())
                {
                    std::cout << "# WARNING: Frame and its parent frameset ("
                              << j + 1 << ") do not have the same gpsInsMeasurement() reference." << std::endl;
                    valid = false;
                }

                if (frame->odometryMeasurement().get() != frameSet->odometryMeasurement().get())
                {
                    std::cout << "# WARNING: Frame and its parent frameset ("
                              << j + 1 << ") do not have the same odometryMeasurement() reference." << std::endl;
                    valid = false;
                }

                if (frame->systemPose().get() != frameSet->systemPose().get())
                {
                    std::cout << "# WARNING: Frame and its parent frameset ("
                              << j + 1 << ") do not have the same systemPose() reference." << std::endl;
                    valid = false;
                }

                if (frame->cameraId() != k)
                {
                    std::cout << "# WARNING: Frame's camera ID does not match expected value." << std::endl;
                    valid = false;
                }

                const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    const Point2DFeatureConstPtr& feature2D = features2D.at(l);
                    const Point3DFeatureConstPtr& feature3D = feature2D->feature3D();

                    FrameConstPtr parentFrame = feature2D->frame().lock();

                    if (parentFrame.get() == 0)
                    {
                        std::cout << "# WARNING: Parent frame is missing." << std::endl;
                        valid = false;
                    }

                    if (parentFrame.get() != frame.get())
                    {
                        std::cout << "# WARNING: Container frame and parent frame do not match for 2D feature." << std::endl;
                        valid = false;
                    }

                    if (feature3D.get() == 0)
                    {
                        continue;
                    }
                }
            }
        }
    }

    return valid;
}

}
