#include <cmath>

#include "CameraRigBA.h"

#include <boost/dynamic_bitset.hpp>
#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/unordered_set.hpp>
#include <boost/lexical_cast.hpp>
#include <camodocal/calib/PlanarHandEyeCalibration.h>
#include <camodocal/camera_models/CataCamera.h>
#include <camodocal/camera_models/PinholeCamera.h>
#include <camodocal/pose_graph/PoseGraph.h>
#include <camodocal/sparse_graph/SparseGraphUtils.h>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ceres/ceres.h"
#include "../camera_models/CostFunctionFactory.h"
#include "../features2d/SurfGPU.h"
#include "../gpl/EigenQuaternionParameterization.h"
#include "camodocal/EigenUtils.h"
#include "../npoint/five-point/five-point.hpp"
#include "../visual_odometry/SlidingWindowBA.h"
#include "OdometryError.h"

#ifdef VCHARGE_D3D
#include "d3d_base/exception.h"
#include "d3d_base/fishEyeCameraMatrix.h"
#include "d3d_cudaBase/cudaFishEyeImageProcessor.h"
#include "d3d_stereo/cudaFishEyePlaneSweep.h"
#include "d3d_base/fishEyeDepthMap.h"
#endif

#ifdef VCHARGE_VIZ
#include "../../../../library/gpl/CameraEnums.h"
#include "../../../../visualization/overlay/GLOverlayExtended.h"
#endif

namespace camodocal
{

CameraRigBA::CameraRigBA(CameraSystem& cameraSystem,
                         SparseGraph& graph,
                         double windowDistance)
 : m_cameraSystem(cameraSystem)
 , m_graph(graph)
 , k_windowDistance(windowDistance)
 , k_maxDistanceRatio(0.7f)
 , k_maxPoint3DDistance(20.0)
 , k_maxReprojErr(2.0)
 , k_minLoopCorrespondences2D3D(50)
 , k_minWindowCorrespondences2D2D(8)
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
    // stage 5 - run fish-eye plane sweep to find ground plane height

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
    case 0:
        break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    {
        std::ostringstream oss;
        oss << "stage" << beginStage - 1 << "-";

        visualize(oss.str(), ODOMETRY);

        oss << "extrinsics";
        visualizeExtrinsics(oss.str());
        break;
    }
    default:
        visualize("final-", ODOMETRY);
        visualizeExtrinsics("final-extrinsics");
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
            double minError, maxError, avgError;
            size_t featureCount;

            reprojectionError(minError, maxError, avgError, featureCount, CAMERA);

            printf("# INFO: Reprojection error: avg = %.2f px | max = %.2f px | # obs = %lu\n",
                   avgError, maxError, featureCount);

            std::cout << "# INFO: Triangulating feature correspondences... " << std::endl;
        }

        triangulateFeatureCorrespondences();

        if (m_verbose)
        {
            double minError, maxError, avgError;
            size_t featureCount;

            reprojectionError(minError, maxError, avgError, featureCount, ODOMETRY);

            printf("# INFO: Reprojection error after triangulation: avg = %.2f px | max = %.2f px | # obs = %lu\n",
                   avgError, maxError, featureCount);
            std::cout << "# INFO: # 3D scene points: " << m_graph.scenePointCount() << std::endl;
        }

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

        if (m_verbose)
        {
            double minError, maxError, avgError;
            size_t featureCount;

            reprojectionError(minError, maxError, avgError, featureCount, ODOMETRY);

            printf("# INFO: Reprojection error after pruning: avg = %.2f px | max = %.2f px | # obs = %lu\n",
                   avgError, maxError, featureCount);
            std::cout << "# INFO: # 3D scene points: " << m_graph.scenePointCount() << std::endl;
        }

#ifdef VCHARGE_VIZ
        visualize("stage0-", ODOMETRY);

        visualizeExtrinsics("stage0-extrinsics");
#endif

        if (m_verbose)
        {
            std::cout << "# INFO: Running BA on odometry data... " << std::endl;
        }

        for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
        {
            Eigen::Matrix4d H = m_cameraSystem.getGlobalCameraPose(i);

            H(2,3) = 0.0;

            m_cameraSystem.setGlobalCameraPose(i, H);
        }

        // optimize camera extrinsics and 3D scene points
        optimize(CAMERA_ODOMETRY_TRANSFORM | POINT_3D, false);

        prune(PRUNE_BEHIND_CAMERA, ODOMETRY); // | PRUNE_FARAWAY | PRUNE_HIGH_REPROJ_ERR, ODOMETRY);

        if (m_verbose)
        {
            double minError, maxError, avgError;
            size_t featureCount;

            reprojectionError(minError, maxError, avgError, featureCount, ODOMETRY);

            std::cout << "# INFO: Done." << std::endl;
            printf("# INFO: Reprojection error after BA (odometry): avg = %.2f px | max = %.2f px | # obs = %lu\n",
                   avgError, maxError, featureCount);
            std::cout << "# INFO: # 3D scene points: " << m_graph.scenePointCount() << std::endl;
        }

#ifdef VCHARGE_VIZ
        visualize("stage1-", ODOMETRY);

        visualizeExtrinsics("stage1-extrinsics");
#endif

        if (saveWorkingData)
        {
            boost::filesystem::path extrinsicPath(dataDir);
            extrinsicPath /= "extrinsic_1";
            m_cameraSystem.writeToDirectory(extrinsicPath.string());

            boost::filesystem::path graphPath(dataDir);
            graphPath /= "frames_1.sg";
            m_graph.writeToBinaryFile(graphPath.string());

            boost::filesystem::path pointcloudPath(dataDir);
            pointcloudPath /= "pointcloud_1";
            dumpPointCloud(pointcloudPath.string());
        }
    }

    // stage 2
    if (beginStage <= 2)
    {
        if (m_verbose)
        {
            std::cout << "# INFO: Running robust pose graph optimization... " << std::endl;
        }

        // For each scene point, record its coordinates with respect to the
        // first camera it was observed in.

        boost::unordered_map<Point3DFeature*, Eigen::Vector3d, boost::hash<Point3DFeature*>, std::equal_to<Point3DFeature*>, Eigen::aligned_allocator<std::pair<Point3DFeature* const, Eigen::Vector3d> > > scenePointMap;

        for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
        {
            FrameSetSegment& segment = m_graph.frameSetSegment(i);

            for (size_t j = 0; j < segment.size(); ++j)
            {
                FrameSetPtr& frameSet = segment.at(j);

                Eigen::Matrix4d H = frameSet->systemPose()->toMatrix().inverse();

                for (size_t k = 0; k < frameSet->frames().size(); ++k)
                {
                    FramePtr& frame = frameSet->frames().at(k);

                    if (!frame)
                    {
                        continue;
                    }

                    std::vector<Point2DFeaturePtr>& features = frame->features2D();
                    for (size_t l = 0; l < features.size(); ++l)
                    {
                        Point3DFeaturePtr& scenePoint = features.at(l)->feature3D();

                        if (!scenePoint)
                        {
                            continue;
                        }

                        if (scenePointMap.find(scenePoint.get()) != scenePointMap.end())
                        {
                            continue;
                        }

                        Eigen::Vector3d P = transformPoint(H, scenePoint->point());

                        scenePointMap.insert(std::make_pair(scenePoint.get(), P));
                    }
                }
            }
        }

        PoseGraph poseGraph(m_cameraSystem, m_graph,
                            k_maxDistanceRatio,
                            k_minLoopCorrespondences2D3D,
                            k_nearestImageMatches);
        poseGraph.setVerbose(m_verbose);
        poseGraph.buildEdges();
        poseGraph.optimize(true);

        // For each scene point, compute its new global coordinates.

        boost::unordered_set<Point3DFeature*> scenePointSet;
        for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
        {
            FrameSetSegment& segment = m_graph.frameSetSegment(i);

            for (size_t j = 0; j < segment.size(); ++j)
            {
                FrameSetPtr& frameSet = segment.at(j);

                Eigen::Matrix4d H = frameSet->systemPose()->toMatrix();

                for (size_t k = 0; k < frameSet->frames().size(); ++k)
                {
                    FramePtr& frame = frameSet->frames().at(k);

                    if (!frame)
                    {
                        continue;
                    }

                    std::vector<Point2DFeaturePtr>& features = frame->features2D();
                    for (size_t l = 0; l < features.size(); ++l)
                    {
                        Point3DFeaturePtr& scenePoint = features.at(l)->feature3D();

                        if (!scenePoint)
                        {
                            continue;
                        }

                        if (scenePointSet.find(scenePoint.get()) != scenePointSet.end())
                        {
                            continue;
                        }

                        Eigen::Vector3d P = scenePointMap[scenePoint.get()];
                        scenePoint->point() = H.block<3,3>(0,0) * P + H.block<3,1>(0,3);

                        scenePointSet.insert(scenePoint.get());
                    }
                }
            }
        }

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

            if (!f3D1)
            {
                continue;
            }

            bool merge = false;
            for (size_t j = 0; j < f3D2->features2D().size(); ++j)
            {
                Point2DFeaturePtr f2D2 = f3D2->features2D().at(j).lock();
                if (!f2D2)
                {
                    continue;
                }

                bool found = false;
                for (size_t k = 0; k < f3D1->features2D().size(); ++k)
                {
                    Point2DFeaturePtr f2D1 = f3D1->features2D().at(k).lock();
                    if (!f2D1)
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
                f3D1->point() = 0.5 * (f3D1->point() + f3D2->point());

                ++nMerged3DScenePoints;
            }
        }

        if (m_verbose)
        {
            std::cout << "# INFO: Merged " << nMerged3DScenePoints << " 3D scene points." << std::endl;
        }

        prune(PRUNE_BEHIND_CAMERA, ODOMETRY);

        std::cout << "# INFO: Running BA on odometry data... " << std::endl;

        optimize(CAMERA_ODOMETRY_TRANSFORM | POINT_3D, true);

        prune(PRUNE_BEHIND_CAMERA, ODOMETRY);

        if (m_verbose)
        {
            double minError, maxError, avgError;
            size_t featureCount;

            reprojectionError(minError, maxError, avgError, featureCount, ODOMETRY);

            printf("# INFO: Reprojection error after robust pose-graph optimization: avg = %.2f px | max = %.2f px | # obs = %lu\n",
                   avgError, maxError, featureCount);
            std::cout << "# INFO: # 3D scene points: " << m_graph.scenePointCount() << std::endl;
        }

#ifdef VCHARGE_VIZ
        visualize("stage2-", ODOMETRY);

        visualizeExtrinsics("stage2-extrinsics");
#endif

        if (saveWorkingData)
        {
            boost::filesystem::path extrinsicPath(dataDir);
            extrinsicPath /= "extrinsic_2";
            m_cameraSystem.writeToDirectory(extrinsicPath.string());

            boost::filesystem::path graphPath(dataDir);
            graphPath /= "frames_2.sg";
            m_graph.writeToBinaryFile(graphPath.string());

            boost::filesystem::path pointcloudPath(dataDir);
            pointcloudPath /= "pointcloud_2";
            dumpPointCloud(pointcloudPath.string());
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

            if (!f1 || !f2)
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
                if (!f2D1)
                {
                    continue;
                }

                bool found = false;
                for (size_t k = 0; k < f3D2->features2D().size(); ++k)
                {
                    Point2DFeaturePtr f2D2 = f3D2->features2D().at(k).lock();
                    if (!f2D2)
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

            f3D2->point() = 0.5 * (f3D1->point() + f3D2->point());
            f3D2->attributes() = Point3DFeature::LOCALLY_OBSERVED_BY_DIFFERENT_CAMERAS;
        }

#ifdef VCHARGE_VIZ
        visualizeFrameFrameCorrespondences("local-inter-p-p", localInterMapFrameFrame);
        visualize3D3DCorrespondences("local-inter-3d-3d", localInterMap2D2D);
#endif

        prune(PRUNE_BEHIND_CAMERA, ODOMETRY);

        if (m_verbose)
        {
            double minError, maxError, avgError;
            size_t featureCount;

            reprojectionError(minError, maxError, avgError, featureCount, ODOMETRY);

            std::cout << "# INFO: Done." << std::endl;
            printf("# INFO: Reprojection error after local matching: avg = %.2f px | max = %.2f px | # obs = %lu\n",
                   avgError, maxError, featureCount);
            std::cout << "# INFO: # 3D scene points: " << m_graph.scenePointCount() << std::endl;
        }

#ifdef VCHARGE_VIZ
        visualize("stage3-BA-", ODOMETRY);

        visualizeExtrinsics("stage3-extrinsics");
#endif

        if (saveWorkingData)
        {
            boost::filesystem::path extrinsicPath(dataDir);
            extrinsicPath /= "extrinsic_3";
            m_cameraSystem.writeToDirectory(extrinsicPath.string());

            boost::filesystem::path graphPath(dataDir);
            graphPath /= "frames_3.sg";
            m_graph.writeToBinaryFile(graphPath.string());

            boost::filesystem::path pointcloudPath(dataDir);
            pointcloudPath /= "pointcloud_3";
            dumpPointCloud(pointcloudPath.string());
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

        // perform BA to optimize intrinsics, extrinsics and scene points
        if (optimizeIntrinsics)
        {
            std::cout << "# INFO: Running BA on odometry data -> optimize CAMERA_INTRINSICS | CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_6D_POSE | POINT_3D ..." << std::endl;

            // perform BA to optimize intrinsics, extrinsics, odometry poses, and scene points
            optimize(CAMERA_INTRINSICS | CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_6D_POSE | POINT_3D, true);
        }
        else
        {
            std::cout << "# INFO: Running BA on odometry data -> optimize CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_6D_POSE | POINT_3D ..." << std::endl;

            // perform BA to optimize extrinsics, odometry poses, and scene points
            optimize(CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_6D_POSE | POINT_3D, true);
        }

        prune(PRUNE_BEHIND_CAMERA | PRUNE_FARAWAY | PRUNE_HIGH_REPROJ_ERR, ODOMETRY);

        if (m_verbose)
        {
            double minError, maxError, avgError;
            size_t featureCount;

            reprojectionError(minError, maxError, avgError, featureCount, ODOMETRY);

            std::cout << "# INFO: Done." << std::endl;
            printf("# INFO: Reprojection error after full BA: avg = %.2f px | max = %.2f px | # obs = %lu\n",
                   avgError, maxError, featureCount);
            std::cout << "# INFO: # 3D scene points: " << m_graph.scenePointCount() << std::endl;
        }

#ifdef VCHARGE_VIZ
        visualize("stage4-", ODOMETRY);

        visualizeExtrinsics("stage4-extrinsics");
#endif

        if (saveWorkingData)
        {
            boost::filesystem::path extrinsicPath(dataDir);
            extrinsicPath /= "extrinsic_4";
            m_cameraSystem.writeToDirectory(extrinsicPath.string());

            boost::filesystem::path graphPath(dataDir);
            graphPath /= "frames_4.sg";
            m_graph.writeToBinaryFile(graphPath.string());

            boost::filesystem::path pointcloudPath(dataDir);
            pointcloudPath /= "pointcloud_4";
            dumpPointCloud(pointcloudPath.string());
        }
    }

    if (beginStage <= 5)
    {
        // estimate ground height using plane sweep stereo
        double zGround = 0.0;
        if (estimateAbsoluteGroundHeight(zGround))
        {
            if (m_verbose)
            {
                std::cout << "# INFO: Found ground plane: z = " << zGround << std::endl;
            }

            // apply odometry-ground transform to extrinsics and system poses
            Eigen::Matrix4d H_odo_ground = Eigen::Matrix4d::Identity();
            H_odo_ground(2,3) = -zGround;

            applyTransform(H_odo_ground, true, true, false);
        }
        else
        {
            if (m_verbose)
            {
                std::cout << "# INFO: Did not find ground plane." << std::endl;
            }
        }

#ifdef VCHARGE_VIZ
        visualize("final-", ODOMETRY);

        visualizeExtrinsics("final-extrinsics");
#endif

        if (saveWorkingData)
        {
            boost::filesystem::path extrinsicPath(dataDir);
            extrinsicPath /= "extrinsic_5";
            m_cameraSystem.writeToDirectory(extrinsicPath.string());

            boost::filesystem::path graphPath(dataDir);
            graphPath /= "frames_5.sg";
            m_graph.writeToBinaryFile(graphPath.string());

            boost::filesystem::path pointcloudPath(dataDir);
            pointcloudPath /= "pointcloud_5";
            dumpPointCloud(pointcloudPath.string());
        }
    }

    if (beginStage <= 6)
    {
        if (saveWorkingData)
        {
            boost::filesystem::path posePath(dataDir);
            posePath /= "poses.txt";
            writePosesToTextFile(posePath.string());
        }
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

        if (!feature3D)
        {
            continue;
        }

        if (std::isnan(feature3D->point()(0)) || std::isnan(feature3D->point()(1)) ||
            std::isnan(feature3D->point()(2)))
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

        /*if (std::isnan(error) || error != error)
        {
            Eigen::Matrix3d R = frame->cameraPose()->rotation().toRotationMatrix();

            Eigen::Vector3d P_cam = R * feature3D->point() + frame->cameraPose()->translation();

            Eigen::Vector2d p;
            camera->spaceToPlane(P_cam, p);

            std::cout << "Rotation:\n" << R << std::endl;
            std::cout << "Translation: " << frame->cameraPose()->translation() << std::endl;
            std::cout << "# 3D point: " << feature3D->point().transpose() << std::endl;
            std::cout << "# 3D point (P_cam): " << P_cam.transpose() << std::endl;
            std::cout << "# 2D point: " << feature2D->keypoint().pt.x << "," << feature2D->keypoint().pt.y << std::endl;
            std::cout << "# 2D point (P_cam): " << p.transpose() << std::endl;
            exit(1);
        }*/

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

    std::vector<Pose> T_cam_odo(m_cameraSystem.cameraCount());
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

                if (!frame)
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

                if (!frame)
                {
                    continue;
                }

                std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    Point2DFeaturePtr& pf = features2D.at(l);

                    if (pf->feature3D())
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

                if (!frame)
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

                if (frameSegment.size() < 2)
                {
                    continue;
                }

                for (size_t l = 1; l < frameSegment.size(); ++l)
                {
                    triangulateFeatures(frameSegment.at(l-1), frameSegment.at(l),
                                        m_cameraSystem.getCamera(i), T_cam_odo);
                }
            }
        }
    }
}

void
CameraRigBA::triangulateFeatures(FramePtr& frame1, FramePtr& frame2,
                                 const CameraConstPtr& camera,
                                 const Pose& T_cam_odo)
{
    // triangulate new feature correspondences seen in last 2 frames
    std::vector<std::vector<Point2DFeaturePtr> > featureCorrespondences;

    // use features that are seen in both frames
    find2D2DCorrespondences(frame2->features2D(), 2, featureCorrespondences);

    std::vector<cv::Point2f> ipoints[2];

    std::vector<std::vector<Point2DFeaturePtr> > untriFeatureCorrespondences;
    for (size_t i = 0; i < featureCorrespondences.size(); ++i)
    {
        std::vector<Point2DFeaturePtr>& fc = featureCorrespondences.at(i);

        Point2DFeaturePtr& f1 = fc.at(0);
        Point2DFeaturePtr& f2 = fc.at(1);

        if (!f1->feature3D())
        {
            ipoints[0].push_back(f1->keypoint().pt);
            ipoints[1].push_back(f2->keypoint().pt);

            untriFeatureCorrespondences.push_back(fc);
        }
        else
        {
            f2->feature3D() = f1->feature3D();
            f2->feature3D()->features2D().push_back(f2);
        }
    }

    if (!untriFeatureCorrespondences.empty())
    {
        Eigen::Matrix4d H_cam_odo = T_cam_odo.toMatrix();

        Eigen::Matrix4d H_cam1 = frame1->systemPose()->toMatrix() * H_cam_odo;
        Eigen::Matrix4d H_cam2 = frame2->systemPose()->toMatrix() * H_cam_odo;
        Eigen::Matrix4d H_cam2_inv = H_cam2.inverse();

        Eigen::Matrix4d H = H_cam2_inv * H_cam1;

        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points3D;
        std::vector<size_t> indices;

        for (size_t i = 0; i < ipoints[0].size(); ++i)
        {
            const cv::Point2f& p1_cv = ipoints[0].at(i);
            const cv::Point2f& p2_cv = ipoints[1].at(i);

            Eigen::Vector3d spt1;
            camera->liftSphere(Eigen::Vector2d(p1_cv.x, p1_cv.y), spt1);

            Eigen::Vector3d spt2;
            camera->liftSphere(Eigen::Vector2d(p2_cv.x, p2_cv.y), spt2);

            Eigen::MatrixXd A(3,2);
            A.col(0) = H.block<3,3>(0,0) * spt1;
            A.col(1) = - spt2;

            Eigen::Vector3d b = - H.block<3,1>(0,3);

            Eigen::Vector2d gamma = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

            // check if scene point is behind camera
            if (gamma(0) < 1e-8 || gamma(1) < 1e-8)
            {
                continue;
            }

            Eigen::Vector3d P = gamma(0) * spt1;
            P = transformPoint(H_cam1, P);

            // validate scene point (Z can't be 0)
            if (transformPoint(H_cam2_inv, P)(2) < 1e-6)
            {
                continue;
            }

            points3D.push_back(P);
            indices.push_back(i);
        }

        for (size_t i = 0; i < points3D.size(); ++i)
        {
            Point3DFeaturePtr point3D = boost::make_shared<Point3DFeature>();

            point3D->point() = points3D.at(i);

            std::vector<Point2DFeaturePtr>& fc = untriFeatureCorrespondences.at(indices.at(i));

            for (int j = 0; j < 2; ++j)
            {
                Point2DFeaturePtr& pt = fc.at(j);

                point3D->features2D().push_back(pt);
                pt->feature3D() = point3D;
            }
        }
    }
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
        std::vector<Point2DFeaturePtr> pt(nViews);

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

    std::vector<std::list<FramePtr> > windows(m_cameraSystem.cameraCount());

    while (segmentId < (int)m_graph.frameSetSegments().size())
    {
        FrameSetPtr& frameSet = m_graph.frameSetSegment(segmentId).at(frameSetId);

        for (int cameraId = 0; cameraId < m_cameraSystem.cameraCount(); ++cameraId)
        {
            //std::list<FramePtr>& window = windows[cameraId];

            if (frameSet->frames().at(cameraId).get() != 0)
            {
                windows[cameraId].push_front(frameSet->frames().at(cameraId));
            }

            std::list<FramePtr>::iterator itWindow = windows[cameraId].begin();
            bool init = false;
            Eigen::Vector3d lastOdoPos;

            double dist = 0.0;
            while (itWindow != windows[cameraId].end() && dist < k_windowDistance)
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

                if (dist < k_windowDistance)
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

            if (!frame1)
            {
                continue;
            }

            if (frame1->features2D().empty())
            {
                continue;
            }

            std::vector<boost::shared_ptr<boost::thread> > threads(m_cameraSystem.cameraCount());
            /// @todo vec<vec<>> is slow! consider alternatives like boost::static_vector multiarray, or even an eigen matrix
            std::vector<std::vector<Correspondence2D2D> > subCorr2D2D(m_cameraSystem.cameraCount());
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
        if (frameSetId >= (int)m_graph.frameSetSegment(segmentId).size())
        {
            ++segmentId;
            frameSetId = 0;
        }
    }
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

    std::vector<boost::shared_ptr<boost::thread> > threads(window.size());
    /// @todo vec<vec<>> is slow! consider alternatives like boost::static_vector multiarray, or even an eigen matrix
    std::vector<std::vector<std::pair<Point2DFeaturePtr, Point2DFeaturePtr> > > corr2D2D(window.size());

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

    for (windowId = 0; windowId < (int)window.size(); ++windowId)
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

    if (!frame2)
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

    if (rmatches.size() < k_minWindowCorrespondences2D2D)
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

    double focalLength1 = k_nominalFocalLength;
    if (m_cameraSystem.getCamera(cameraId1)->modelType() == Camera::PINHOLE)
    {
        PinholeCameraPtr pCam = boost::static_pointer_cast<PinholeCamera>(m_cameraSystem.getCamera(cameraId1));

        focalLength1 = (pCam->getParameters().fx(), pCam->getParameters().fy());
    }

    double focalLength2 = k_nominalFocalLength;
    if (m_cameraSystem.getCamera(cameraId2)->modelType() == Camera::PINHOLE)
    {
        PinholeCameraPtr pCam = boost::static_pointer_cast<PinholeCamera>(m_cameraSystem.getCamera(cameraId2));

        focalLength2 = (pCam->getParameters().fx(), pCam->getParameters().fy());
    }

    cv::Mat E, inlierMat;
    E = findEssentialMat(rpoints1, rpoints2, focalLength1,
                         cv::Point2d(rimg1.cols / 2, rimg1.rows / 2),
                         CV_FM_RANSAC, 0.99, reprojErrorThresh, 1000, inlierMat);

    if (cv::countNonZero(inlierMat) < (int)k_minWindowCorrespondences2D2D)
    {
        return;
    }

    if (1)
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
    //Eigen::Matrix4d H_rcam1 = H1 * H_cam1;

    Eigen::Matrix4d H2 = Eigen::Matrix4d::Identity();
    H2.block<3,3>(0,0) = R2;
    //Eigen::Matrix4d H_rcam2 = H2 * H_cam2;

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
        ray(0) = (rp.x - m_cameraSystem.getCamera(cameraId1)->imageWidth() / 2.0) / focalLength1;
        ray(1) = (rp.y - m_cameraSystem.getCamera(cameraId1)->imageHeight() / 2.0) / focalLength1;
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
        ray(0) = (rp.x - m_cameraSystem.getCamera(cameraId2)->imageWidth() / 2.0) / focalLength2;
        ray(1) = (rp.y - m_cameraSystem.getCamera(cameraId2)->imageHeight() / 2.0) / focalLength2;
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

        if (!p1 || !p2)
        {
            continue;
        }

        if (!p1->feature3D() || !p2->feature3D())
        {
            continue;
        }

        corr2D2D->push_back(candidateCorr2D2D.at(i));
    }

    if (1  && !corr2D2D->empty())
    {
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        std::vector<cv::DMatch> matches;
        for (size_t i = 0; i < corr2D2D->size(); ++i)
        {
            Point2DFeaturePtr& p1 = corr2D2D->at(i).first;
            Point2DFeaturePtr& p2 = corr2D2D->at(i).second;

            if (!p1 || !p2)
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
        //cv::imwrite(oss.str(), frame1->image());

        oss.str(""); oss.clear();
        oss << "ur" << cameraId2 << "img" << count << ".png";
        //cv::imwrite(oss.str(), frame2->image());

        ++count;

        mutex.unlock();
    }
}

void
CameraRigBA::prune(int flags, int poseType)
{
    std::vector<Pose, Eigen::aligned_allocator<Pose> > T_cam_odo(m_cameraSystem.cameraCount());
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > H_odo_cam(m_cameraSystem.cameraCount());
    for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
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

                if (!frame)
                {
                    continue;
                }

                int cameraId = frame->cameraId();

                std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                Eigen::Matrix4d H_cam = Eigen::Matrix4d::Identity();
                if (poseType == CAMERA)
                {
                    H_cam = frame->cameraPose()->toMatrix();
                }
                else
                {
                    H_cam = H_odo_cam.at(cameraId) * frame->systemPose()->toMatrix().inverse();
                }

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    Point2DFeaturePtr& pf = features2D.at(l);

                    if (!pf->feature3D())
                    {
                        continue;
                    }

                    Eigen::Vector3d P_cam = transformPoint(H_cam, pf->feature3D()->point());

                    bool prune = false;

                    if ((flags & PRUNE_BEHIND_CAMERA) &&
                        P_cam(2) < 0.0)
                    {
                        prune = true;
                    }

                    if ((flags & PRUNE_FARAWAY) &&
                        P_cam.block<3,1>(0,0).norm() > k_maxPoint3DDistance)
                    {
                        prune = true;
                    }

                    if (flags & PRUNE_HIGH_REPROJ_ERR)
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
    size_t nOdometryResiduals = 0;
    Eigen::Matrix3d sqrtOdometryPrecisionMat;
    sqrtOdometryPrecisionMat.setIdentity();
    if (flags & ODOMETRY_6D_POSE)
    {
        // compute precision matrix for odometry data
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > errVec;
        Eigen::Vector3d errMean = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
        {
            FrameSetSegment& segment = m_graph.frameSetSegment(i);

            if (segment.size() <= 1)
            {
                continue;
            }

            FrameSetPtr frameSetPrev = segment.front();
            for (size_t j = 1; j < segment.size(); ++j)
            {
                FrameSetPtr frameSet = segment.at(j);

                Eigen::Matrix4d H_odo_meas = frameSet->odometryMeasurement()->toMatrix().inverse() *
                                             frameSetPrev->odometryMeasurement()->toMatrix();


                Eigen::Matrix4d H_err = H_odo_meas *
                                        frameSetPrev->systemPose()->toMatrix().inverse() *
                                        frameSet->systemPose()->toMatrix();

                Eigen::Matrix3d R_err = H_err.block<3,3>(0,0);
                double r, p, y;
                mat2RPY(R_err, r, p, y);

                Eigen::Vector3d err;
                err << H_err(0,3), H_err(1,3), y;

                errVec.push_back(err);

                errMean += err;

                frameSetPrev = frameSet;
            }
        }
        errMean /= static_cast<double>(errVec.size());

        Eigen::Matrix3d odometryCovariance;
        odometryCovariance.setZero();
        for (size_t i = 0; i < errVec.size(); ++i)
        {
            Eigen::Vector3d e = errVec.at(i) - errMean;

            odometryCovariance += e * e.transpose();
        }
        odometryCovariance /= static_cast<double>(errVec.size());

        if (odometryCovariance.trace() < 1e-10)
        {
            // No loop closures are found. Hence, the measurement covariance
            // for odometry data is a zero matrix. In this case,
            // use a reasonable measurement covariance.

            odometryCovariance << 2.5e-5, 0.0, 0.0,
                                  0.0, 6.25e-6, 0.0,
                                  0.0, 0.0, 9e-6;
        }

        Eigen::Matrix3d odometryPrecisionMat = odometryCovariance.inverse();
        sqrtOdometryPrecisionMat = sqrtm(odometryPrecisionMat);

        nOdometryResiduals = errVec.size();

        if (m_verbose)
        {
            std::cerr << "# INFO: Added " << nOdometryResiduals << " residuals from odometry data." << std::endl;
        }
    }

    double fudgeFactor = 1.0;
    Eigen::Matrix2d sqrtKptPrecisionMat = Eigen::Matrix2d::Identity() / sqrt(0.04);

    bool includeChessboardData = (flags & CAMERA_INTRINSICS) && !m_cameraCalibrations.empty();
    size_t nCBCornerResiduals = 0;

    if (includeChessboardData)
    {
        fudgeFactor = 10.0;

        Eigen::Matrix2d kptCovariance = Eigen::Matrix2d::Zero();

        for (size_t i = 0; i < m_cameraCalibrations.size(); ++i)
        {
            size_t nObs = m_cameraCalibrations.at(i)->imagePoints().size() *
                          m_cameraCalibrations.at(i)->imagePoints().front().size();

            nCBCornerResiduals += nObs;

            Eigen::Matrix2d measurementCov = m_cameraCalibrations.at(i)->measurementCovariance();
            kptCovariance += measurementCov * static_cast<double>(nObs);

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
                std::cerr << "# INFO: "
                          << "[" << m_cameraSystem.getCamera(i)->cameraName()
                          << "] Initial reprojection error (chessboard): "
                          << err << " pixels" << std::endl;
            }
        }

        kptCovariance /= static_cast<double>(nCBCornerResiduals) * fudgeFactor;

        Eigen::Matrix2d kptPrecisionMat = kptCovariance.inverse();
        sqrtKptPrecisionMat = sqrtm(kptPrecisionMat);

        if (m_verbose)
        {
            std::cerr << "# INFO: Added " << nCBCornerResiduals << " residuals from chessboard data." << std::endl;
        }
    }

    Eigen::Vector2d e;
    e << 1.0 / sqrt(2.0), 1.0 / sqrt(2.0);
    double lossParam = e.transpose() * sqrtKptPrecisionMat * sqrtKptPrecisionMat.transpose() * e;

    if (m_verbose)
    {
        std::cerr << "# INFO: Loss parameter = " << lossParam << std::endl;
        std::cerr << "# INFO: sqrt precision matrix (keypoint position):" << std::endl;
        std::cerr << sqrtKptPrecisionMat << std::endl;
    }

    ceres::Problem problem;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = nIterations;
    options.num_threads = 8;
    options.num_linear_solver_threads = 8;

    // intrinsics
    /// @todo vec<vec<>> is slow! consider alternatives like boost::static_vector multiarray, or even an eigen matrix
    std::vector<std::vector<double> > intrinsicParams(m_cameraSystem.cameraCount());

    for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
    {
        m_cameraSystem.getCamera(i)->writeParameters(intrinsicParams[i]);
    }

    // extrinsics
    std::vector<Pose, Eigen::aligned_allocator<Pose> > T_cam_odo(m_cameraSystem.cameraCount());
    for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
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

                if (!frame)
                {
                    continue;
                }

                int cameraId = frame->cameraId();

                std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    Point2DFeaturePtr& feature2D = features2D.at(l);
                    Point3DFeaturePtr& feature3D = feature2D->feature3D();

                    if (!feature3D)
                    {
                        continue;
                    }

                    if (std::isnan(feature3D->point()(0)) || std::isnan(feature3D->point()(1)) ||
                        std::isnan(feature3D->point()(2)))
                    {
                        continue;
                    }

                    optimizeExtrinsics[cameraId] = 1;

                    ceres::LossFunction* lossFunction = new ceres::ScaledLoss(new ceres::CauchyLoss(lossParam), feature3D->weight(), ceres::TAKE_OWNERSHIP);

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
                    case CAMERA_ODOMETRY_TRANSFORM | POINT_3D:
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
                    case CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_3D_POSE | POINT_3D:
                    case CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_6D_POSE | POINT_3D:
                    {
                        costFunction
                            = CostFunctionFactory::instance()->generateCostFunction(m_cameraSystem.getCamera(cameraId),
                                                                                    Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y),
                                                                                    sqrtKptPrecisionMat,
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
                    case CAMERA_INTRINSICS | CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_3D_POSE | POINT_3D:
                    case CAMERA_INTRINSICS | CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_6D_POSE | POINT_3D:
                    {
                        costFunction
                            = CostFunctionFactory::instance()->generateCostFunction(m_cameraSystem.getCamera(cameraId),
                                                                                    Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y),
                                                                                    sqrtKptPrecisionMat,
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
                    case CAMERA_POSE | POINT_3D:
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

                if (flags & CAMERA_POSE)
                {
                    ceres::LocalParameterization* quaternionParameterization =
                        new EigenQuaternionParameterization;
                    problem.SetParameterization(frame->cameraPose()->rotationData(), quaternionParameterization);
                }
            }
        }
    }

    if (flags & CAMERA_ODOMETRY_TRANSFORM)
    {
        for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
        {
            if (optimizeExtrinsics[i])
            {
                ceres::LocalParameterization* quaternionParameterization =
                    new EigenQuaternionParameterization;

                problem.SetParameterization(T_cam_odo.at(i).rotationData(), quaternionParameterization);
            }
        }
    }

    if (flags & ODOMETRY_6D_POSE)
    {
        for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
        {
            FrameSetSegment& segment = m_graph.frameSetSegment(i);

            if (segment.size() <= 1)
            {
                continue;
            }

            FrameSetPtr frameSetPrev = segment.front();
            for (size_t j = 1; j < segment.size(); ++j)
            {
                FrameSetPtr frameSet = segment.at(j);

                Eigen::Matrix4d H_odo_meas = frameSet->odometryMeasurement()->toMatrix().inverse() *
                                             frameSetPrev->odometryMeasurement()->toMatrix();

                ceres::CostFunction* costFunction =
                    new ceres::AutoDiffCostFunction<OdometryError, 3, 3, 3, 3, 3>(
                        new OdometryError(H_odo_meas, sqrtOdometryPrecisionMat));

                problem.AddResidualBlock(costFunction, 0,
                                         frameSetPrev->systemPose()->positionData(),
                                         frameSetPrev->systemPose()->attitudeData(),
                                         frameSet->systemPose()->positionData(),
                                         frameSet->systemPose()->attitudeData());

                frameSetPrev = frameSet;
            }
        }
    }

    /// @todo vec<vec<>> is slow! consider alternatives like boost::static_vector multiarray, or even an eigen matrix
    std::vector<std::vector<std::vector<double> > > chessboardCameraPoses(m_cameraSystem.cameraCount());

    if (includeChessboardData)
    {
        for (size_t i = 0; i < m_cameraCalibrations.size(); ++i)
        {
            cv::Mat& cameraPoses = m_cameraCalibrations.at(i)->cameraPoses();
            chessboardCameraPoses[i].resize(cameraPoses.rows);

            std::vector<std::vector<cv::Point2f> >& imagePoints = m_cameraCalibrations.at(i)->imagePoints();
            std::vector<std::vector<cv::Point3f> >& scenePoints = m_cameraCalibrations.at(i)->scenePoints();
            Eigen::Matrix2d measurementCov = m_cameraCalibrations.at(i)->measurementCovariance();
            Eigen::Matrix2d precisionMat = measurementCov.inverse();
            Eigen::Matrix2d sqrtPrecisionMat = sqrtm(precisionMat);

            double lossParamCB = e.transpose() * precisionMat * e;

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
                                                                              sqrtPrecisionMat,
                                                                              CAMERA_INTRINSICS | CAMERA_POSE);

                    ceres::LossFunction* lossFunction = new ceres::CauchyLoss(lossParamCB);
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
        std::cerr << summary.BriefReport() << std::endl;
    }

    if (flags & CAMERA_ODOMETRY_TRANSFORM)
    {
        for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
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

    if ((flags & ODOMETRY_6D_POSE) && m_verbose)
    {
        double errAvg = 0.0;
        double errMax = 0.0;
        size_t count = 0;
        for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
        {
            FrameSetSegment& segment = m_graph.frameSetSegment(i);

            if (segment.size() <= 1)
            {
                continue;
            }

            FrameSetPtr frameSetPrev = segment.front();
            for (size_t j = 1; j < segment.size(); ++j)
            {
                FrameSetPtr frameSet = segment.at(j);

                double scale_meas = (frameSet->odometryMeasurement()->position() -
                                     frameSetPrev->odometryMeasurement()->position()).norm();

                double scale_est = (frameSet->systemPose()->position() -
                                    frameSetPrev->systemPose()->position()).norm();

                double err = fabs(scale_est - scale_meas) / scale_meas;

                errAvg += err;
                ++count;

                if (err > errMax)
                {
                    errMax = err;
                }

                frameSetPrev = frameSet;
            }
        }

        errAvg /= static_cast<double>(count);

        printf("# INFO: Scale error: avg = %.3f%% | max = %.3f%%\n",
               errAvg * 100.0, errMax * 100.0);
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
            std::cerr << "# INFO: "
                      << "[" << m_cameraSystem.getCamera(i)->cameraName()
                      << "] Final reprojection error (chessboard): "
                      << err << " pixels" << std::endl;
        }
    }
}

void
CameraRigBA::reweightScenePoints(void)
{
    // Assign high weights to feature observations associated with scene points
    // observed by multiple cameras.
    // Note that the observation condition only applies to scene points
    // *locally* observed by multiple cameras.

    size_t nObs = 0;
    size_t nObsMultipleCams = 0;
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

                if (!frame)
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
                            ++nObsMultipleCams;
                        }

                        ++nObs;

                        scenePointSet.insert(features2D.at(l)->feature3D().get());
                    }
                }
            }
        }
    }

    double weightS = 0.0;
    if (nObsMultipleCams == 0)
    {
        weightS = 1.0;
    }
    else
    {
        weightS = static_cast<double>(nObsMultipleCams) / static_cast<double>(nObs - nObsMultipleCams);
    }

    for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
    {
        FrameSetSegment& segment = m_graph.frameSetSegment(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            FrameSetPtr& frameSet = segment.at(j);

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                FramePtr& frame = frameSet->frames().at(k);

                if (!frame)
                {
                    continue;
                }

                const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    Point3DFeaturePtr& feature3D = features2D.at(l)->feature3D();

                    if (!feature3D)
                    {
                        continue;
                    }

                    if (feature3D->attributes() & Point3DFeature::LOCALLY_OBSERVED_BY_DIFFERENT_CAMERAS)
                    {
                        feature3D->weight() = 1.0;
                    }
                    else
                    {
                        feature3D->weight() = weightS;
                    }
                }
            }
        }
    }

    if (m_verbose)
    {
        std::cout << "# INFO: Assigned weight of " << weightS
                  << " to scene points observed by a single camera."
                  << std::endl;
    }
}

bool
CameraRigBA::estimateRigOdometryTransform(Eigen::Matrix4d& H_rig_odo) const
{
    PlanarHandEyeCalibration calib;
    calib.setVerbose(m_verbose);

    for (int segmentId = 0; segmentId < (int)m_graph.frameSetSegments().size(); ++segmentId)
    {
        const FrameSetSegment& segment = m_graph.frameSetSegment(segmentId);

        bool init = false;
        Eigen::Matrix4d odoPrev, rigPrev;
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > H_odo, H_rig;

        for (size_t frameSetId = 0; frameSetId < segment.size(); ++frameSetId)
        {
            const FrameSetPtr& frameSet = segment.at(frameSetId);

            Eigen::Matrix4d odo = frameSet->odometryMeasurement()->toMatrix();
            Eigen::Matrix4d rig = frameSet->systemPose()->toMatrix();

            if (init)
            {
                H_odo.push_back(odo.inverse() * odoPrev);
                H_rig.push_back(rig.inverse() * rigPrev);
            }
            else
            {
                init = true;
            }

            odoPrev = odo;
            rigPrev = rig;
        }

        calib.addMotions(H_rig, H_odo);
    }

    return calib.calibrate(H_rig_odo);
}

bool
CameraRigBA::estimateAbsoluteGroundHeight(double& zGround) const
{
#ifdef VCHARGE_D3D
    double iScale = 0.5;

    float minZ = 0.5f;
    float maxZ = 3.5f;
    float minDepth = 0.2f;
    float maxDepth = 5.0f;
    int nGroundPlaneHypots = 128;
    int nImagesPerMatch = 3;

    D3D_CUDA::DeviceImage devImg;
    D3D_CUDA::CudaFishEyeImageProcessor cFEIP;

    std::vector<std::pair<double, size_t> > zGrounds;

    for (size_t cameraId = 0; cameraId < m_cameraSystem.cameraCount(); ++cameraId)
    {
        if (m_cameraSystem.getCamera(cameraId)->modelType() != Camera::MEI)
        {
            continue;
        }

        const CataCameraPtr camera = boost::static_pointer_cast<CataCamera>(m_cameraSystem.getCamera(cameraId));

        double xi = camera->getParameters().xi();
        double k1 = camera->getParameters().k1();
        double k2 = camera->getParameters().k2();
        double p1 = camera->getParameters().p1();
        double p2 = camera->getParameters().p2();

        Eigen::Matrix3d cameraK = Eigen::Matrix3d::Identity();
        cameraK(0,0) = camera->getParameters().gamma1() * iScale;
        cameraK(1,1) = camera->getParameters().gamma2() * iScale;
        cameraK(0,2) = camera->getParameters().u0() * iScale;
        cameraK(1,2) = camera->getParameters().v0() * iScale;

        Eigen::Matrix3d cameraKOrig = Eigen::Matrix3d::Identity();
        cameraKOrig(0,0) = camera->getParameters().gamma1();
        cameraKOrig(1,1) = camera->getParameters().gamma2();
        cameraKOrig(0,2) = camera->getParameters().u0();
        cameraKOrig(1,2) = camera->getParameters().v0();

        CataCamera::Parameters params;
        params = camera->getParameters();
        params.gamma1() *= iScale;
        params.gamma2() *= iScale;
        params.u0() *= iScale;
        params.v0() *= iScale;
        params.k1() = 0.0;
        params.k2() = 0.0;
        params.p1() = 0.0;
        params.p2() = 0.0;

        CataCameraPtr cameraWNoDist = boost::make_shared<CataCamera>();
        cameraWNoDist->setParameters(params);

        Eigen::Matrix4d H_cam_sys = m_cameraSystem.getGlobalCameraPose(cameraId);
        Eigen::Matrix4d H_sys_cam = H_cam_sys.inverse();

        // generate ground plane hypotheses
        float delta = (maxZ - minZ) / static_cast<float>(nGroundPlaneHypots - 1);

        Eigen::Vector3d direction = H_sys_cam.block<3,3>(0,0) * Eigen::Vector3d::UnitZ();

        D3D::Grid<Eigen::Vector4d> groundPlaneHypots;
        groundPlaneHypots.resize(nGroundPlaneHypots, 1, 1);
        for (int i = 0; i < nGroundPlaneHypots; ++i)
        {
            groundPlaneHypots(i,0)(0) = direction(0);
            groundPlaneHypots(i,0)(1) = direction(1);
            groundPlaneHypots(i,0)(2) = direction(2);
            groundPlaneHypots(i,0)(3) = minZ + static_cast<float>(i) * delta;
        }

        std::vector<std::vector<FramePtr> > frames(1);
        std::vector<std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > > poses(1);
        std::vector<std::vector<uint64_t> > timestamps(1);
        for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
        {
            const FrameSetSegment& segment = m_graph.frameSetSegment(i);

            for (size_t j = 0; j < segment.size(); ++j)
            {
                const FrameSetPtr& frameSet = segment.at(j);
                const FramePtr& frame = frameSet->frames().at(cameraId);

                if (frame && !frame->image().empty())
                {
                    frames.back().push_back(frame);

                    Eigen::Matrix4d H_cam = H_sys_cam * frameSet->systemPose()->toMatrix().inverse();
                    poses.back().push_back(H_cam);

                    timestamps.back().push_back(frameSet->systemPose()->timeStamp());
                }
                else
                {
                    if (!frames.back().empty())
                    {
                        frames.resize(frames.size() + 1);
                        poses.resize(poses.size() + 1);
                        timestamps.resize(timestamps.size() + 1);
                    }

                    continue;
                }
            }
        }
        if (frames.back().empty())
        {
            if (frames.size() == 1)
            {
                continue;
            }
            else
            {
                frames.resize(frames.size() - 1);
                poses.resize(frames.size() - 1);
                timestamps.resize(frames.size() - 1);
            }
        }

        for (size_t i = 0; i < frames.size(); ++i)
        {
            if (frames.at(i).size() < nImagesPerMatch)
            {
                continue;
            }

            D3D::CudaFishEyePlaneSweep cFEPS;

            cFEPS.setMatchingCosts(D3D::FISH_EYE_PLANE_SWEEP_ZNCC);
            cFEPS.setZRange(minDepth, maxDepth);
            cFEPS.setMatchWindowSize(9,9);

            if (nImagesPerMatch == 2)
            {
                cFEPS.setOcclusionMode(D3D::FISH_EYE_PLANE_SWEEP_OCCLUSION_NONE);
            }
            else
            {
                cFEPS.setOcclusionMode(D3D::FISH_EYE_PLANE_SWEEP_OCCLUSION_REF_SPLIT);
            }

            cFEPS.setPlaneGenerationMode(D3D::FISH_EYE_PLANE_SWEEP_PLANEMODE_UNIFORM_DISPARITY);
            cFEPS.setSubPixelInterpolationMode(D3D::FISH_EYE_PLANE_SWEEP_SUB_PIXEL_INTERP_INVERSE);
            cFEPS.enableSubPixel();
            cFEPS.enableOutputBestCosts();

            std::list<std::pair<int, uint64_t> > cFEPSImageIds;

            for (size_t j = 0; j < frames.at(i).size(); ++j)
            {
                const cv::Mat& image = frames.at(i).at(j)->image();
                const Eigen::Matrix4d& H_cam = poses.at(i).at(j);
                uint64_t timestamp = timestamps.at(i).at(j);

                D3D::FishEyeCameraMatrix<double> cameraMatUndist(cameraKOrig, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), xi);

                devImg.reallocatePitchedAndUpload(image);
                cFEIP.setInputImg(devImg, cameraMatUndist);

                std::pair<D3D_CUDA::DeviceImage, D3D::FishEyeCameraMatrix<double> > undistRes;
                undistRes = cFEIP.undistort(iScale, 1.0, k1, k2, p1, p2);

                D3D::FishEyeCameraMatrix<double> cameraMat(cameraK, H_cam.block<3,3>(0,0), H_cam.block<3,1>(0,3), xi);
                cFEPSImageIds.push_back(std::make_pair(cFEPS.addDeviceImage(undistRes.first, cameraMat),
                                                       timestamp));

                cv::Mat imageUndist;
                undistRes.first.download(imageUndist);

                cv::imshow("Undistorted Image", imageUndist);
                cv::waitKey(2);

                if (cFEPSImageIds.size() == nImagesPerMatch)
                {
                    std::list<std::pair<int, uint64_t> >::iterator it = cFEPSImageIds.begin();
                    for (int k = 0; k < nImagesPerMatch / 2; ++k)
                    {
                        ++it;
                    }
                    std::pair<int, uint64_t> refId = *it;

                    cFEPS.process(refId.first, groundPlaneHypots);

                    D3D::FishEyeDepthMap<float, double> dMGround;
                    dMGround = cFEPS.getBestDepth();

                    D3D::Grid<float> bestCostsGround;
                    bestCostsGround = cFEPS.getBestCosts();

                    for (unsigned int y = 0; y < dMGround.getHeight(); ++y)
                    {
                        for (unsigned int x = 0; x < dMGround.getWidth(); ++x)
                        {
                            if (bestCostsGround(x,y) > 0.15f)
                            {
                                dMGround(x,y) = -1.0;
                            }
                        }
                    }

                    // display the depth maps
                    dMGround.displayInvDepthColored(minDepth, maxDepth, 2);

//                    std::stringstream fileName;
//                    fileName << "point_clouds/" << timestamp << "_" << camera->cameraName() << ".wrl";
//                    std::ofstream wrlOutStr;
//                    wrlOutStr.open(fileName.str().c_str());
//                    dMGround.pointCloudColoredToVRML(wrlOutStr, image, maxDepth);

                    cFEPS.deleteImage(cFEPSImageIds.begin()->first);
                    cFEPSImageIds.pop_front();

                    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > pointCloud;
                    for (unsigned int y = 0; y < dMGround.getHeight(); ++y)
                    {
                        for (unsigned int x = 0; x < dMGround.getWidth(); ++x)
                        {
                            double depth = dMGround(x,y);

                            if (depth < 0.0 || depth > maxDepth)
                            {
                                continue;
                            }

                            Eigen::Vector3d ray;
                            cameraWNoDist->liftSphere(Eigen::Vector2d(x,y), ray);

                            Eigen::Vector3d P = ray / ray(2) * depth;

                            P = H_cam_sys.block<3,3>(0,0) * P + H_cam_sys.block<3,1>(0,3);

                            pointCloud.push_back(P);
                        }
                    }

                    // find ground plane height

                    // TODO: Better way is to find the mode of a continuous 1D distribution.
                    std::vector<size_t> inlierIdsBest;
                    for (size_t k = 0; k < pointCloud.size(); k += 1000)
                    {
                        double z = pointCloud.at(k)(2);

                        std::vector<size_t> inlierIds;
                        for (size_t l = 0; l < pointCloud.size(); ++l)
                        {
                            if (fabs(z - pointCloud.at(l)(2)) < 0.01)
                            {
                                inlierIds.push_back(l);
                            }
                        }

                        if (inlierIds.size() < 10000)
                        {
                            continue;
                        }

                        if (inlierIds.size() > inlierIdsBest.size())
                        {
                            inlierIdsBest = inlierIds;
                        }
                    }

                    if (inlierIdsBest.empty())
                    {
                        continue;
                    }

                    double zSum = 0.0;
                    for (size_t k = 0; k < inlierIdsBest.size(); ++k)
                    {
                        zSum += pointCloud.at(inlierIdsBest.at(k))(2);
                    }

                    zGrounds.push_back(std::make_pair(zSum, inlierIdsBest.size()));
                }
            }
        }
    }

    cv::destroyAllWindows();

    double zSum = 0.0;
    size_t n = 0;
    for (size_t i = 0; i < zGrounds.size(); ++i)
    {
        zSum += zGrounds.at(i).first;
        n += zGrounds.at(i).second;
    }

    zGround = zSum / static_cast<double>(n);

    return true;
#else
    return false;
#endif
}

void
CameraRigBA::applyTransform(const Eigen::Matrix4d& H_sys_nsys,
                            bool applyToExtrinsics,
                            bool applyToSystemPoses,
                            bool applyToScenePoints)
{
    if (applyToExtrinsics)
    {
        for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
        {
            Eigen::Matrix4d H_cam_sys = m_cameraSystem.getGlobalCameraPose(i);

            Eigen::Matrix4d H_cam_nsys = H_sys_nsys * H_cam_sys;

            m_cameraSystem.setGlobalCameraPose(i, H_cam_nsys);
        }
    }

    boost::unordered_set<Point3DFeature*> scenePointSet;

    for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
    {
        FrameSetSegment& segment = m_graph.frameSetSegment(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            FrameSetPtr& frameSet = segment.at(j);

            if (applyToSystemPoses)
            {
                Eigen::Matrix4d H_sys_world = frameSet->systemPose()->toMatrix();

                Eigen::Matrix4d H_nsys_world = H_sys_world * H_sys_nsys.inverse();
                Eigen::Matrix3d R_nsys_world = H_nsys_world.block<3,3>(0,0);

                double r, p, y;
                mat2RPY(R_nsys_world, r, p, y);

                frameSet->systemPose()->attitude() << y, p, r;
                frameSet->systemPose()->position() = H_nsys_world.block<3,1>(0,3);
            }

            if (!applyToScenePoints)
            {
                continue;
            }

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                FramePtr& frame = frameSet->frames().at(k);

                if (!frame)
                {
                    continue;
                }

                std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    Point3DFeaturePtr& scenePoint = features2D.at(l)->feature3D();

                    if (!scenePoint)
                    {
                        continue;
                    }

                    scenePointSet.insert(scenePoint.get());
                }
            }
        }
    }

    if (!applyToScenePoints)
    {
        return;
    }

    for (boost::unordered_set<Point3DFeature*>::iterator it = scenePointSet.begin();
         it != scenePointSet.end(); ++it)
    {
        Point3DFeature* scenePoint = *it;

        scenePoint->point() = transformPoint(H_sys_nsys, scenePoint->point());
    }
}

void
CameraRigBA::writePosesToTextFile(const std::string& filename) const
{
    std::ofstream ofs(filename.c_str());
    if (!ofs.is_open())
    {
        return;
    }

    ofs << std::fixed << std::setprecision(10);

    for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
    {
        const FrameSetSegment& segment = m_graph.frameSetSegment(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            const FrameSetPtr& frameSet = segment.at(j);

            ofs << frameSet->systemPose()->timeStamp() << " "
                << frameSet->systemPose()->attitude()(2) << " "
                << frameSet->systemPose()->attitude()(1) << " "
                << frameSet->systemPose()->attitude()(0) << " "
                << frameSet->systemPose()->position()(0) << " "
                << frameSet->systemPose()->position()(1) << " "
                << frameSet->systemPose()->position()(2);

            if (frameSet->gpsInsMeasurement())
            {
                Eigen::Matrix3d R = frameSet->gpsInsMeasurement()->rotation().toRotationMatrix();

                double roll, pitch, yaw;
                mat2RPY(R, roll, pitch, yaw);

                ofs << " "
                    << roll << " " << pitch << " " << yaw << " "
                    << frameSet->gpsInsMeasurement()->translation()(0) << " "
                    << frameSet->gpsInsMeasurement()->translation()(1) << " "
                    << frameSet->gpsInsMeasurement()->translation()(2);
            }

            ofs << std::endl;
        }
    }

    ofs.close();
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

                if (!frame)
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
                    if (!features2D.at(l)->feature3D())
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

        if (!p1->feature3D() || !p2->feature3D())
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

                if (!frame)
                {
                    continue;
                }

                if (!systemPose)
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

                if (frame->cameraId() != (int)k)
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

                    if (!parentFrame)
                    {
                        std::cout << "# WARNING: Parent frame is missing." << std::endl;
                        valid = false;
                    }

                    if (parentFrame.get() != frame.get())
                    {
                        std::cout << "# WARNING: Container frame and parent frame do not match for 2D feature." << std::endl;
                        valid = false;
                    }

                    if (!feature3D)
                    {
                        continue;
                    }
                }
            }
        }
    }

    return valid;
}

// --------------------------------------------------------------------------------------
void
CameraRigBA::dumpPointCloud(const std::string& dir, bool dumpPoses)
{
    boost::filesystem::path directory(dir);
    if (!boost::filesystem::is_directory(directory))
        boost::filesystem::create_directory(directory);


    float hw = 0.5;
    float hl = 0.25;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vertex(4);

    vertex[0] = Eigen::Vector3d(hw,hl,0);
    vertex[1] = Eigen::Vector3d(hw,-hl,0);
    vertex[2] = Eigen::Vector3d(-hw,hl*0.5,0);
    vertex[3] = Eigen::Vector3d(-hw,-hl*0.5,0);

    // dump system poses
    if (dumpPoses)
    {
        boost::filesystem::path posePath = directory / ("rig.obj");
        std::ofstream pose_dump(posePath.string());

        int last_vertex_idx = 1;

        for (size_t i = 0; i < m_graph.frameSetSegments().size(); ++i)
        {
            FrameSetSegment& segment = m_graph.frameSetSegment(i);

            for (size_t j = 0; j < segment.size(); ++j)
            {
                Eigen::Isometry3d T(segment.at(j)->systemPose()->toMatrix());
                for(Eigen::Vector3d v : vertex)
                {
                    v = T * v;
                    pose_dump << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
                    last_vertex_idx++;
                }
                pose_dump << "f " << last_vertex_idx-4 << " " << last_vertex_idx-3 << " " << last_vertex_idx-2 << std::endl;
                pose_dump << "f " << last_vertex_idx-2 << " " << last_vertex_idx-3 << " " << last_vertex_idx-1 << std::endl;
            }
        }
    }

    // point cloud
    for (int i = 0; i < m_cameraSystem.cameraCount(); ++i)
    {
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

                if (!frame)
                {
                    continue;
                }

                odometry.push_back(frame->systemPose());
                cameraPoses.push_back(frame->cameraPose());

                std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    if (!features2D.at(l)->feature3D())
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


        boost::filesystem::path camPath = directory / (m_cameraSystem.getCamera(i)->cameraName() + "_pose.obj");
        std::ofstream pose_dump(camPath.string());
        int last_vertex_idx = 1;
        Eigen::Vector3d origin = cameraPoses.front()->translation();

        for (size_t j = 0; j < cameraPoses.size(); ++j)
        {
            PosePtr& cameraPose = cameraPoses.at(j);

            Eigen::Matrix4d H_cam = cameraPose->toMatrix().inverse();
            //Eigen::Isometry3d T = Eigen::Isometry3d(frame->cameraPose()->toMatrix()).inverse();
            for(Eigen::Vector3d vhat : vertex)
            {
                Eigen::Vector3d v = transformPoint(H_cam, vhat) - origin;
                pose_dump << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
                last_vertex_idx++;
            }
            pose_dump << "f " << last_vertex_idx-4 << " " << last_vertex_idx-3 << " " << last_vertex_idx-2 << std::endl;
            pose_dump << "f " << last_vertex_idx-2 << " " << last_vertex_idx-3 << " " << last_vertex_idx-1 << std::endl;
        }

        // dump collected 3d points to a point cloud
        boost::filesystem::path filepath = directory / (m_cameraSystem.getCamera(i)->cameraName() + ".obj");
        std::ofstream file(filepath.c_str());
        for (Point3DFeature* feature : scenePointSet)
        {
            Eigen::Vector3i color(255,255,255);
            color *= feature->pointCovariance().norm();
            file << "v " << feature->point().transpose() << " " << color.transpose() << std::endl;
        }
    }
}

}
