#include "camodocal/infrastr_calib/InfrastructureCalibration.h"

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>
#include <iostream>
#include <opencv2/core/eigen.hpp>

#include "../camera_models/CostFunctionFactory.h"
#include "../features2d/SurfGPU.h"
#include "../gpl/EigenQuaternionParameterization.h"
#include "../../gpl/gpl.h"
#include "camodocal/EigenUtils.h"
#ifndef HAVE_OPENCV3
#include "../gpl/OpenCVUtils.h"
#endif // HAVE_OPENCV3
#include "../location_recognition/LocationRecognition.h"
#include "../pose_estimation/P3P.h"
#include "camodocal/sparse_graph/SparseGraphUtils.h"
#include "ceres/ceres.h"

#ifdef VCHARGE_VIZ
#include <boost/unordered_set.hpp>
#include "../../../../library/gpl/CameraEnums.h"
#endif

namespace camodocal
{

InfrastructureCalibration::InfrastructureCalibration(std::vector<CameraPtr>& cameras,
                                                     bool verbose)
 : m_cameras(cameras)
 , m_x_last(0.0)
 , m_y_last(0.0)
 , m_distance(0.0)
 , m_verbose(verbose)
#ifdef VCHARGE_VIZ
 , m_overlay("cameras", VCharge::COORDINATE_FRAME_GLOBAL)
#endif
 , m_cameraSystem(cameras.size())
 , k_maxDistanceRatio(0.7f)
 , k_minCorrespondences2D3D(25)
 , k_minKeyFrameDistance(0.3)
 , k_nearestImageMatches(10)
 , k_reprojErrorThresh(2.0)
{

}

bool
InfrastructureCalibration::loadMap(const std::string& mapDirectory)
{
    if (m_verbose)
    {
        std::cout << "# INFO: Loading map... " << std::flush;
    }

    boost::filesystem::path graphPath(mapDirectory);
    graphPath /= "frames_5.sg";

    if (!m_refGraph.readFromBinaryFile(graphPath.string()))
    {
        std::cout << std::endl << "# ERROR: Cannot read graph file " << graphPath.string() << "." << std::endl;
        return false;
    }

    if (m_verbose)
    {
        std::cout << "Finished." << std::endl;
    }

#ifdef VCHARGE_VIZ
    visualizeMap("map-ref", REFERENCE_MAP);

    m_overlay.clear();
#endif

    if (m_verbose)
    {
        std::cout << "# INFO: Setting up location recognition... " << std::flush;
    }

    m_locrec = boost::make_shared<LocationRecognition>();
    m_locrec->setup(m_refGraph);

    if (m_verbose)
    {
        std::cout << "Finished." << std::endl;
    }

    reset();

    return true;
}

void
InfrastructureCalibration::addFrameSet(const std::vector<cv::Mat>& images,
                                       uint64_t timestamp,
                                       bool preprocess)
{
    if (images.size() != m_cameras.size())
    {
        std::cout << "# WARNING: Number of images does not match camera count." << std::endl;
        return;
    }

    std::vector<boost::shared_ptr<boost::thread> > threads(m_cameras.size());
    std::vector<FramePtr> frames(m_cameras.size());

    // estimate camera pose corresponding to each image
    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        frames.at(i) = boost::make_shared<Frame>();
        frames.at(i)->cameraId() = i;

        threads.at(i) = boost::make_shared<boost::thread>(&InfrastructureCalibration::estimateCameraPose,
                                                          this, images.at(i), timestamp,
                                                          frames.at(i), preprocess);
    }

    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        threads.at(i)->join();
    }

    FrameSet frameset;
    frameset.timestamp = timestamp;

    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        if (frames.at(i)->cameraPose().get() == 0)
        {
            continue;
        }

        frameset.frames.push_back(frames.at(i));
    }

    if (frameset.frames.size() < 2)
    {
        return;
    }

    std::vector<FramePtr> framePrev(m_cameras.size());
    std::vector<FramePtr> frameCurr(m_cameras.size());

    bool addFrameSet = false;

    if (m_framesets.empty())
    {
        addFrameSet = true;
    }

    if (!addFrameSet)
    {
        // estimate keyframe distance by using the minimum of the norm of the
        // translation of the ith camera between two frames
        for (size_t i = 0; i < m_framesets.back().frames.size(); ++i)
        {
            FramePtr& frame = m_framesets.back().frames.at(i);

            framePrev.at(frame->cameraId()) = frame;
        }

        for (size_t i = 0; i < frameset.frames.size(); ++i)
        {
            FramePtr& frame = frameset.frames.at(i);

            frameCurr.at(frame->cameraId()) = frame;
        }

        double keyFrameDist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < m_cameras.size(); ++i)
        {
            if (framePrev.at(i).get() == 0 || frameCurr.at(i).get() == 0)
            {
                continue;
            }

            double d = (frameCurr.at(i)->cameraPose()->toMatrix().inverse().block<3,1>(0,3) -
                        framePrev.at(i)->cameraPose()->toMatrix().inverse().block<3,1>(0,3)).norm();

            if (d < keyFrameDist)
            {
                keyFrameDist = d;
            }
        }

        if (keyFrameDist != std::numeric_limits<double>::max() &&
            keyFrameDist > k_minKeyFrameDistance)
        {
            addFrameSet = true;
        }
        else
        {
            if (m_verbose)
            {
                std::cout << "# INFO: Skipping frame set as inter-frame distance is too small." << std::endl;
            }
        }
    }

    if (!addFrameSet)
    {
        return;
    }

    m_framesets.push_back(frameset);

    if (m_verbose)
    {
        std::cout << "# INFO: Added frame set " << m_framesets.size()
                  << " [ ";
        for (size_t i = 0; i < frameset.frames.size(); ++i)
        {
            std::cout << frameset.frames.at(i)->cameraId() << " ";
        }
        std::cout << "] ts = " << frameset.timestamp << std::endl;
    }

#ifdef VCHARGE_VIZ
    visualizeCameraPoses(true);
    for (size_t i = 0; i < frameset.frames.size(); ++i)
    {
        visualizeCameraPose(frameset.frames.at(i), true);
    }
    m_overlay.publish();

    usleep(100000);
    visualizeMap("map-opt", REFERENCE_POINTS);
#endif
}

void
InfrastructureCalibration::addOdometry(double x, double y, double yaw,
                                       uint64_t timestamp)
{
    if (m_x_last != 0.0 || m_y_last != 0.0)
    {
        m_distance += hypot(x - m_x_last, y - m_y_last);
    }

    m_x_last = x;
    m_y_last = y;
}

void
InfrastructureCalibration::reset(void)
{
    m_feature3DMap.clear();
    m_framesets.clear();
    m_x_last = 0.0;
    m_y_last = 0.0;
    m_distance = 0.0;

    m_cameraSystem = CameraSystem(m_cameras.size());

#ifdef VCHARGE_VIZ
    m_overlay.clear();
#endif
}

void
InfrastructureCalibration::run(void)
{
#ifdef VCHARGE_VIZ
    visualizeCameraPoses(true);
#endif

    if (0)
    {
        // show 2D point features with 3D correspondences in images
        for (size_t i = 0; i < m_framesets.size(); ++i)
        {
            FrameSet& frameset = m_framesets.at(i);

            for (size_t j = 0; j < frameset.frames.size(); ++j)
            {
                FramePtr& frame = frameset.frames.at(j);

                std::vector<cv::KeyPoint> keypoints;
                for (size_t k = 0; k < frame->features2D().size(); ++k)
                {
                    Point2DFeaturePtr& feature2D = frame->features2D().at(k);
                    Point3DFeaturePtr& feature3D = feature2D->feature3D();

                    if (feature3D.get() == 0)
                    {
                        continue;
                    }

                    cv::KeyPoint keypoint = feature2D->keypoint();
                    keypoint.pt.x /= 2.0f;
                    keypoint.pt.y /= 2.0f;

                    keypoints.push_back(keypoint);
                }

                cv::Mat sketch;
                cv::cvtColor(frame->image(), sketch, CV_GRAY2BGR);
                cv::resize(sketch, sketch, cv::Size(), 0.5, 0.5);
                cv::drawKeypoints(sketch, keypoints, sketch);

                std::ostringstream oss;
                oss << "cam" << frame->cameraId();

                cv::imshow(oss.str(), sketch);
            }

            cv::waitKey(0);
        }
    }

    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        m_cameraSystem.setCamera(i, m_cameras.at(i));
    }

    if (m_verbose)
    {
        double sumError = 0.0;
        size_t sumFeatureCount = 0;
        for (size_t i = 0; i < m_framesets.size(); ++i)
        {
            FrameSet& frameset = m_framesets.at(i);

            for (size_t j = 0; j < frameset.frames.size(); ++j)
            {
                FramePtr& frame = frameset.frames.at(j);

                double minError, maxError, avgError;
                size_t featureCount;

                frameReprojectionError(frame,
                                       m_cameraSystem.getCamera(frame->cameraId()),
                                       minError, maxError, avgError,
                                       featureCount);

                sumError += avgError * featureCount;
                sumFeatureCount += featureCount;
            }
        }

        std::cout << "# INFO: Average reprojection error over all frames: "
                  << sumError / sumFeatureCount << " px" << std::endl;

        size_t nFrames = 0;
        for (size_t i = 0; i < m_framesets.size(); ++i)
        {
            FrameSet& frameset = m_framesets.at(i);

            nFrames += frameset.frames.size();
        }

        std::cout << "# INFO: Average number of frames per set: "
                  << static_cast<double>(nFrames) / static_cast<double>(m_framesets.size())
                  << std::endl;
    }

    // without loss of generality, mark camera 0 as the reference frame
    m_cameraSystem.setGlobalCameraPose(0, Eigen::Matrix4d::Identity());

    // find initial estimates for camera extrinsics

    // in each iteration over complete frame sets,
    // compute the relative camera poses with respect to camera 0,
    // and use these extrinsics to compute the average reprojection error
    // over all frame sets. We use the extrinsics with the lowest average
    // reprojection error as the initial estimate.
    double minReprojError = std::numeric_limits<double>::max();
    std::vector<Pose, Eigen::aligned_allocator<Pose> > best_T_cam_ref;
    for (size_t i = 0; i < m_framesets.size(); ++i)
    {
        FrameSet& frameset = m_framesets.at(i);

        if (frameset.frames.size() < m_cameras.size())
        {
            continue;
        }

        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > poses(m_cameras.size());
        for (size_t j = 0; j < frameset.frames.size(); ++j)
        {
            int cameraIdx = frameset.frames.at(j)->cameraId();

            poses.at(cameraIdx) = frameset.frames.at(j)->cameraPose()->toMatrix();
        }

        std::vector<Pose, Eigen::aligned_allocator<Pose> > T_cam_ref(m_cameras.size());

        T_cam_ref.at(0).rotation() = Eigen::Quaterniond::Identity();
        T_cam_ref.at(0).translation() = Eigen::Vector3d::Zero();

        for (size_t j = 1; j < m_cameras.size(); ++j)
        {
            Eigen::Matrix4d H_cam_ref = poses.at(0) * poses.at(j).inverse();

            T_cam_ref.at(j).rotation() = Eigen::Quaterniond(H_cam_ref.block<3,3>(0,0));
            T_cam_ref.at(j).translation() = H_cam_ref.block<3,1>(0,3);

            m_cameraSystem.setGlobalCameraPose(j, H_cam_ref);
        }

        for (size_t j = 0; j < m_framesets.size(); ++j)
        {
            // estimate odometry
            Eigen::Vector3d pos = Eigen::Vector3d::Zero();
            std::vector<Eigen::Quaterniond> att;

            for (size_t k = 0; k < m_framesets.at(j).frames.size(); ++k)
            {
                int cameraIdx = m_framesets.at(j).frames.at(k)->cameraId();
                PosePtr& cameraPose = m_framesets.at(j).frames.at(k)->cameraPose();

                Eigen::Matrix4d H = cameraPose->toMatrix().inverse() * T_cam_ref.at(cameraIdx).toMatrix().inverse();

                pos += H.block<3,1>(0,3);
                att.push_back(Eigen::Quaterniond(H.block<3,3>(0,0)));
            }

            pos /= m_framesets.at(j).frames.size();

            OdometryPtr odometry = boost::make_shared<Odometry>();
            odometry->timeStamp() = m_framesets.at(j).frames.at(0)->cameraPose()->timeStamp();
            odometry->position() = pos;

            Eigen::Quaterniond qAvg = quaternionAvg(att);

            double roll, pitch, yaw;
            mat2RPY(qAvg.toRotationMatrix(), roll, pitch, yaw);

            odometry->attitude() = Eigen::Vector3d(yaw, pitch, roll);

            for (size_t k = 0; k < m_framesets.at(j).frames.size(); ++k)
            {
                m_framesets.at(j).frames.at(k)->systemPose() = odometry;
            }
        }

        // compute average reprojection error over all frame sets
        double minError, maxError, avgError;
        size_t featureCount;
        reprojectionError(minError, maxError, avgError, featureCount);

        if (avgError < minReprojError)
        {
            minReprojError = avgError;

            best_T_cam_ref = T_cam_ref;
        }
    }

    if (minReprojError == std::numeric_limits<double>::max())
    {
        std::cout << "# ERROR: No complete frame sets were found." << std::endl;
        return;
    }

    for (size_t i = 1; i < m_cameras.size(); ++i)
    {
        m_cameraSystem.setGlobalCameraPose(i, best_T_cam_ref.at(i).toMatrix());
    }

    // extrinsics
    std::vector<Pose, Eigen::aligned_allocator<Pose> > T_cam_ref = best_T_cam_ref;

    for (size_t i = 0; i < m_framesets.size(); ++i)
    {
        FrameSet& frameset = m_framesets.at(i);

        // estimate odometry
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        std::vector<Eigen::Quaterniond> att;

        for (size_t j = 0; j < frameset.frames.size(); ++j)
        {
            int cameraIdx = frameset.frames.at(j)->cameraId();
            PosePtr& cameraPose = frameset.frames.at(j)->cameraPose();

            Eigen::Matrix4d H = cameraPose->toMatrix().inverse() * T_cam_ref.at(cameraIdx).toMatrix().inverse();

            pos += H.block<3,1>(0,3);
            att.push_back(Eigen::Quaterniond(H.block<3,3>(0,0)));
        }

        pos /= frameset.frames.size();

        OdometryPtr& odometry = frameset.frames.at(0)->systemPose();
        odometry->position() = pos;

        Eigen::Quaterniond qAvg = quaternionAvg(att);

        double roll, pitch, yaw;
        mat2RPY(qAvg.toRotationMatrix(), roll, pitch, yaw);

        odometry->attitude() = Eigen::Vector3d(yaw, pitch, roll);

        for (size_t j = 0; j < frameset.frames.size(); ++j)
        {
            FramePtr& frame = frameset.frames.at(j);

            frame->systemPose() = odometry;
        }
    }

    // run non-linear optimization to optimize odometry poses and camera extrinsics
    optimize(false);

    if (m_verbose)
    {
        std::cout << "# INFO: Odometry distance: " << m_distance << " m" << std::endl;
    }

#ifdef VCHARGE_VIZ
    visualizeExtrinsics();
    sleep(1);
    visualizeOdometry();
    sleep(1);
    visualizeMap("map-opt", REFERENCE_POINTS);
    sleep(1);
#endif
}

void
InfrastructureCalibration::loadFrameSets(const std::string& filename)
{
    m_framesets.clear();

    SparseGraph graph;

    graph.readFromBinaryFile(filename);

    for (size_t i = 0; i < graph.frameSetSegment(0).size(); ++i)
    {
        FrameSet frameset;
        uint64_t timestamp = 0;

        for (int j = 0; j < (int)graph.frameSetSegment(0).at(i)->frames().size(); ++j)
        {
            FramePtr& frame = graph.frameSetSegment(0).at(i)->frames().at(j);

            if (frame.get() == 0)
            {
                continue;
            }

            timestamp = frame->cameraPose()->timeStamp();

            frameset.frames.push_back(frame);
        }

        frameset.timestamp = timestamp;

        m_framesets.push_back(frameset);
    }

    if (m_verbose)
    {
        std::cout << "# INFO: Loaded " << m_framesets.size() << " frame sets from " << filename << std::endl;
    }

#ifdef VCHARGE_VIZ
    visualizeCameraPoses(false);
#endif
}

void
InfrastructureCalibration::saveFrameSets(const std::string& filename) const
{
    SparseGraph graph;

    graph.frameSetSegments().resize(1);
    graph.frameSetSegment(0).resize(m_framesets.size());

    for (size_t i = 0; i < m_framesets.size(); ++i)
    {
        const FrameSet& frameset = m_framesets.at(i);

        graph.frameSetSegment(0).at(i) = boost::make_shared<camodocal::FrameSet>();
        graph.frameSetSegment(0).at(i)->frames().resize(m_cameras.size());

        for (size_t j = 0; j < frameset.frames.size(); ++j)
        {
            int cameraId = frameset.frames.at(j)->cameraId();

            graph.frameSetSegment(0).at(i)->frames().at(cameraId) = frameset.frames.at(j);
        }
    }

    graph.writeToBinaryFile(filename);

    if (m_verbose)
    {
        std::cout << "# INFO: Wrote " << m_framesets.size() << " frame sets to " << filename << std::endl;
    }
}

void
InfrastructureCalibration::estimateCameraPose(const cv::Mat& image,
                                              uint64_t timestamp,
                                              FramePtr& frame,
                                              bool preprocess)
{
    cv::Mat imageProc;
    if (preprocess)
    {


#ifdef HAVE_CUDA
    //////////////////
    // CUDA + OPENCV2 + OPENCV3
    //////////////////

#ifdef HAVE_OPENCV3
   typedef cv::cuda::GpuMat CUDAMat;
#else // HAVE_OPENCV3
   typedef cv::gpu::GpuMat CUDAMat;
#endif // HAVE_OPENCV3


        CUDAMat gpuImage, gpuImageProc;
        gpuImage.upload(image);
        cv::equalizeHist(gpuImage, gpuImageProc);


        gpuImageProc.download(imageProc);
#else // HAVE_CUDA
    //////////////////
    // OPENCV2 + OPENCV3
    //////////////////
        cv::Mat gpuImage, gpuImageProc;
        cv::equalizeHist(gpuImage, gpuImageProc);
#endif
    }
    else
    {
        image.copyTo(imageProc);
    }

    double tsStart = timeInSeconds();

    // compute keypoints and descriptors
    cv::Ptr<SurfGPU> surf = SurfGPU::instance(300.0);

    std::vector<cv::KeyPoint> keypoints;
    surf->detect(imageProc, keypoints);

    cv::Mat descriptors;
    surf->compute(imageProc, keypoints, descriptors);

//    image.copyTo(frame->image());

    for (size_t i = 0; i < keypoints.size(); ++i)
    {
        Point2DFeaturePtr feature2D = boost::make_shared<Point2DFeature>();

        feature2D->keypoint() = keypoints.at(i);
        descriptors.row(i).copyTo(feature2D->descriptor());
        feature2D->index() = i;
        feature2D->frame() = frame;

        frame->features2D().push_back(feature2D);
    }

    // find k closest matches in vocabulary tree
    std::vector<FrameTag> candidates;
    m_locrec->knnMatch(frame, k_nearestImageMatches, candidates);

    // find match with highest number of inlier 2D-2D correspondences
    int bestInlierCount = 0;
    std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > bestCorr2D3D;
    Eigen::Matrix4d bestH;

    for (size_t i = 0; i < candidates.size(); ++i)
    {
        FrameTag tag = candidates.at(i);

        FramePtr& trainFrame = m_refGraph.frameSetSegment(tag.frameSetSegmentId).at(tag.frameSetId)->frames().at(tag.frameId);

        // find 2D-3D correspondences
        std::vector<cv::DMatch> matches = matchFeatures(frame->features2D(), trainFrame->features2D());

        if ((int)matches.size() < k_minCorrespondences2D3D)
        {
            continue;
        }

        // find camera pose from P3P RANSAC
        Eigen::Matrix4d H;
        std::vector<cv::DMatch> inliers;
        solveP3PRansac(frame, trainFrame, matches, H, inliers);

        int nInliers = inliers.size();

        if (nInliers < k_minCorrespondences2D3D)
        {
            continue;
        }

        if (nInliers > bestInlierCount)
        {
            bestInlierCount = nInliers;

            bestCorr2D3D.clear();
            for (size_t j = 0; j < inliers.size(); ++j)
            {
                const cv::DMatch& match = inliers.at(j);

                bestCorr2D3D.push_back(std::make_pair(frame->features2D().at(match.queryIdx),
                                                      trainFrame->features2D().at(match.trainIdx)->feature3D()));
            }

            bestH = H;
        }
    }

    if (bestInlierCount < k_minCorrespondences2D3D)
    {
        return;
    }

    if (m_verbose)
    {
        std::cout << "# INFO: [Cam " << frame->cameraId() <<  "] Found " << bestInlierCount
                  << " inlier 2D-3D correspondences from nearest image."
                  << std::endl;
    }

    PosePtr pose = boost::make_shared<Pose>(bestH);
    pose->timeStamp() = timestamp;

    frame->cameraPose() = pose;

    // store inlier 2D-3D correspondences
    for (size_t i = 0; i < bestCorr2D3D.size(); ++i)
    {
        Point2DFeaturePtr& p2D = bestCorr2D3D.at(i).first;
        Point3DFeaturePtr& p3D = bestCorr2D3D.at(i).second;

        boost::lock_guard<boost::mutex> lock(m_feature3DMapMutex);
        boost::unordered_map<Point3DFeature*, Point3DFeaturePtr>::iterator it = m_feature3DMap.find(p3D.get());

        Point3DFeaturePtr feature3D;
        if (it == m_feature3DMap.end())
        {
            feature3D = boost::make_shared<Point3DFeature>();
            feature3D->point() = p3D->point();

            m_feature3DMap.insert(std::make_pair(p3D.get(), feature3D));
        }
        else
        {
            feature3D = it->second;
        }

        feature3D->features2D().push_back(p2D);
        p2D->feature3D() = feature3D;
    }

    // prune features that are not associated to a scene point
    std::vector<Point2DFeaturePtr>::iterator it = frame->features2D().begin();
    while (it != frame->features2D().end())
    {
        if ((*it)->feature3D().get() == 0)
        {
            frame->features2D().erase(it);
        }
        else
        {
            ++it;
        }
    }

    if (m_verbose)
    {
        std::cout << "# INFO: [Cam " << frame->cameraId() <<  "] Estimated camera pose" << std::endl;
        std::cout << bestH << std::endl;
        std::cout << "           time: " << timeInSeconds() - tsStart << " s" << std::endl;

        double minError, maxError, avgError;
        size_t featureCount;

        frameReprojectionError(frame, m_cameras.at(frame->cameraId()),
                               minError, maxError, avgError, featureCount);

        std::cout << "          reproj: " << avgError << std::endl;
        std::cout << "              ts: " << pose->timeStamp() << std::endl;
    }
}

const CameraSystem&
InfrastructureCalibration::cameraSystem(void) const
{
    return m_cameraSystem;
}

void
InfrastructureCalibration::optimize(bool optimizeScenePoints)
{
    // extrinsics
    std::vector<Pose, Eigen::aligned_allocator<Pose> > T_cam_ref(m_cameras.size());
    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        T_cam_ref.at(i) = Pose(m_cameraSystem.getGlobalCameraPose(i));
    }

    if (m_verbose)
    {
        double minError, maxError, avgError;
        size_t featureCount;
        reprojectionError(minError, maxError, avgError, featureCount);

        std::cout << "# INFO: Initial reprojection error: avg = " << avgError
                << " px | max = " << maxError << " px | count = " << featureCount << std::endl;
    }

    double tsStart = timeInSeconds();

    ceres::Problem problem;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 1000;
    options.num_threads = 8;

    for (size_t i = 0; i < m_framesets.size(); ++i)
    {
        FrameSet& frameset = m_framesets.at(i);

        for (size_t j = 0; j < frameset.frames.size(); ++j)
        {
            FramePtr& frame = frameset.frames.at(j);

            for (size_t k = 0; k < frame->features2D().size(); ++k)
            {
                Point2DFeaturePtr& feature2D = frame->features2D().at(k);

                if (feature2D->feature3D().get() == 0)
                {
                    continue;
                }

                ceres::LossFunction* lossFunction = new ceres::CauchyLoss(1.0);

                if (optimizeScenePoints)
                {
                    ceres::CostFunction* costFunction
                        = CostFunctionFactory::instance()->generateCostFunction(m_cameraSystem.getCamera(frame->cameraId()),
                                                                                Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y),
                                                                                CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_6D_POSE | POINT_3D);

                    problem.AddResidualBlock(costFunction, lossFunction,
                                             T_cam_ref.at(frame->cameraId()).rotationData(),
                                             T_cam_ref.at(frame->cameraId()).translationData(),
                                             frame->systemPose()->positionData(),
                                             frame->systemPose()->attitudeData(),
                                             feature2D->feature3D()->pointData());
                }
                else
                {
                    ceres::CostFunction* costFunction
                        = CostFunctionFactory::instance()->generateCostFunction(m_cameraSystem.getCamera(frame->cameraId()),
                                                                                feature2D->feature3D()->point(),
                                                                                Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y),
                                                                                CAMERA_ODOMETRY_TRANSFORM | ODOMETRY_6D_POSE);

                    problem.AddResidualBlock(costFunction, lossFunction,
                                             T_cam_ref.at(frame->cameraId()).rotationData(),
                                             T_cam_ref.at(frame->cameraId()).translationData(),
                                             frame->systemPose()->positionData(),
                                             frame->systemPose()->attitudeData());
                }
            }
        }
    }

    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        ceres::LocalParameterization* quaternionParameterization =
            new EigenQuaternionParameterization;

        problem.SetParameterization(T_cam_ref.at(i).rotationData(), quaternionParameterization);
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (m_verbose)
    {
        std::cout << summary.BriefReport() << std::endl;
    }

    if (m_verbose)
    {
        std::cout << "# INFO: Optimization took "
                  << timeInSeconds() - tsStart << " s." << std::endl;
    }

    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        m_cameraSystem.setGlobalCameraPose(i, T_cam_ref.at(i).toMatrix());
    }

    if (m_verbose)
    {
        double minError, maxError, avgError;
        size_t featureCount;
        reprojectionError(minError, maxError, avgError, featureCount);

        std::cout << "# INFO: Final reprojection error: avg = " << avgError
                << " px | max = " << maxError << " px | count = " << featureCount << std::endl;
    }
}

cv::Mat
InfrastructureCalibration::buildDescriptorMat(const std::vector<Point2DFeaturePtr>& features,
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
InfrastructureCalibration::matchFeatures(const std::vector<Point2DFeaturePtr>& queryFeatures,
                                         const std::vector<Point2DFeaturePtr>& trainFeatures) const
{
    std::vector<size_t> queryIndices, trainIndices;
    cv::Mat queryDtor = buildDescriptorMat(queryFeatures, queryIndices, false);
    cv::Mat trainDtor = buildDescriptorMat(trainFeatures, trainIndices, true);

    if (queryDtor.cols != trainDtor.cols)
    {
        std::cout << "# WARNING: Descriptor lengths do not match." << std::endl;
        return std::vector<cv::DMatch>();
    }

    if (queryDtor.type() != trainDtor.type())
    {
        std::cout << "# WARNING: Descriptor types do not match." << std::endl;
        return std::vector<cv::DMatch>();
    }

    cv::Ptr<SurfGPU> surf = SurfGPU::instance(300.0);

    std::vector<std::vector<cv::DMatch> > candidateFwdMatches;
    surf->knnMatch(queryDtor, trainDtor, candidateFwdMatches, 2);

    std::vector<std::vector<cv::DMatch> > candidateRevMatches;
    surf->knnMatch(trainDtor, queryDtor, candidateRevMatches, 2);

    std::vector<std::vector<cv::DMatch> > fwdMatches(candidateFwdMatches.size());
    for (size_t i = 0; i < candidateFwdMatches.size(); ++i)
    {
        std::vector<cv::DMatch>& match = candidateFwdMatches.at(i);

        if (match.size() < 2)
        {
            continue;
        }

        float distanceRatio = match.at(0).distance / match.at(1).distance;

        if (distanceRatio < k_maxDistanceRatio)
        {
            fwdMatches.at(i).push_back(match.at(0));
        }
    }

    std::vector<std::vector<cv::DMatch> > revMatches(candidateRevMatches.size());
    for (size_t i = 0; i < candidateRevMatches.size(); ++i)
    {
        std::vector<cv::DMatch>& match = candidateRevMatches.at(i);

        if (match.size() < 2)
        {
            continue;
        }

        float distanceRatio = match.at(0).distance / match.at(1).distance;

        if (distanceRatio < k_maxDistanceRatio)
        {
            revMatches.at(i).push_back(match.at(0));
        }
    }

    // cross-check
    std::vector<cv::DMatch> matches;
    for (size_t i = 0; i < fwdMatches.size(); ++i)
    {
        if (fwdMatches.at(i).empty())
        {
            continue;
        }

        cv::DMatch& fwdMatch = fwdMatches.at(i).at(0);

        if (revMatches.at(fwdMatch.trainIdx).empty())
        {
            continue;
        }

        cv::DMatch& revMatch = revMatches.at(fwdMatch.trainIdx).at(0);

        if (fwdMatch.queryIdx == revMatch.trainIdx &&
            fwdMatch.trainIdx == revMatch.queryIdx)
        {
            cv::DMatch match;
            match.queryIdx = queryIndices.at(fwdMatch.queryIdx);
            match.trainIdx = trainIndices.at(revMatch.queryIdx);

            matches.push_back(match);
        }
    }

    return matches;
}

void
InfrastructureCalibration::solveP3PRansac(const FrameConstPtr& frame1,
                                          const FrameConstPtr& frame2,
                                          const std::vector<cv::DMatch>& matches,
                                          Eigen::Matrix4d& H,
                                          std::vector<cv::DMatch>& inliers) const
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

    const CameraConstPtr& camera1 = m_cameraSystem.getCamera(frame1->cameraId());

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

            worldPoints.at(j) = features2.at(match.trainIdx)->feature3D()->point();

            const cv::KeyPoint& kpt1 = features1.at(match.queryIdx)->keypoint();

            Eigen::Vector3d ray;
            camera1->liftProjective(Eigen::Vector2d(kpt1.pt.x, kpt1.pt.y), ray);

            rays.at(j) = ray.normalized();
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

                Eigen::Vector3d P2 = features2.at(match.trainIdx)->feature3D()->point();

                Eigen::Vector3d P1 = transformPoint(H_inv, P2);
                Eigen::Vector2d p1_pred;
                camera1->spaceToPlane(P1, p1_pred);

                const Point2DFeatureConstPtr& f1 = features1.at(match.queryIdx);

                double err = hypot(f1->keypoint().pt.x - p1_pred(0),
                                   f1->keypoint().pt.y - p1_pred(1));
                if (err > k_reprojErrorThresh)
                {
                    continue;
                }

                inliersIds.push_back(k);
            }

            if (inliersIds.size() > inlierIds_best.size())
            {
                H_best = H_inv;
                inlierIds_best = inliersIds;
            }
        }
    }

    for (size_t i = 0; i < inlierIds_best.size(); ++i)
    {
        inliers.push_back(matches.at(inlierIds_best.at(i)));
    }

    H = H_best;
}

double
InfrastructureCalibration::reprojectionError(const CameraConstPtr& camera,
                                             const Eigen::Vector3d& P,
                                             const Eigen::Quaterniond& cam_ref_q,
                                             const Eigen::Vector3d& cam_ref_t,
                                             const Eigen::Vector3d& ref_p,
                                             const Eigen::Vector3d& ref_att,
                                             const Eigen::Vector2d& observed_p) const
{
    Eigen::Quaterniond q_z_inv(cos(ref_att(0) / 2.0), 0.0, 0.0, -sin(ref_att(0) / 2.0));
    Eigen::Quaterniond q_y_inv(cos(ref_att(1) / 2.0), 0.0, -sin(ref_att(1) / 2.0), 0.0);
    Eigen::Quaterniond q_x_inv(cos(ref_att(2) / 2.0), -sin(ref_att(2) / 2.0), 0.0, 0.0);

    Eigen::Quaterniond q_world_ref = q_x_inv * q_y_inv * q_z_inv;
    Eigen::Quaterniond q_cam = cam_ref_q.conjugate() * q_world_ref;

    Eigen::Vector3d t_cam = - q_cam.toRotationMatrix() * ref_p - cam_ref_q.conjugate().toRotationMatrix() * cam_ref_t;

    return camera->reprojectionError(P, q_cam, t_cam, observed_p);
}

void
InfrastructureCalibration::frameReprojectionError(const FramePtr& frame,
                                                  const CameraConstPtr& camera,
                                                  const Pose& T_cam_ref,
                                                  double& minError, double& maxError, double& avgError,
                                                  size_t& featureCount) const
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

        double error
            = reprojectionError(camera, feature3D->point(),
                                T_cam_ref.rotation(),
                                T_cam_ref.translation(),
                                frame->systemPose()->position(),
                                frame->systemPose()->attitude(),
                                Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y));

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
InfrastructureCalibration::frameReprojectionError(const FramePtr& frame,
                                                  const CameraConstPtr& camera,
                                                  double& minError, double& maxError, double& avgError,
                                                  size_t& featureCount) const
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

        double error = camera->reprojectionError(feature3D->point(),
                                                 frame->cameraPose()->rotation(),
                                                 frame->cameraPose()->translation(),
                                                 Eigen::Vector2d(feature2D->keypoint().pt.x, feature2D->keypoint().pt.y));

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
InfrastructureCalibration::reprojectionError(double& minError, double& maxError,
                                             double& avgError, size_t& featureCount) const
{
    minError = std::numeric_limits<double>::max();
    maxError = std::numeric_limits<double>::min();

    size_t count = 0;
    double totalError = 0.0;

    for (size_t i = 0; i < m_framesets.size(); ++i)
    {
        const FrameSet& frameset = m_framesets.at(i);

        for (size_t j = 0; j < frameset.frames.size(); ++j)
        {
            const FramePtr& frame = frameset.frames.at(j);

            Pose T_cam_ref(m_cameraSystem.getGlobalCameraPose(frame->cameraId()));

            double frameMinError;
            double frameMaxError;
            double frameAvgError;
            size_t frameFeatureCount;

            frameReprojectionError(frame,
                                   m_cameraSystem.getCamera(frame->cameraId()),
                                   T_cam_ref,
                                   frameMinError, frameMaxError, frameAvgError, frameFeatureCount);

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

#ifdef VCHARGE_VIZ
void
InfrastructureCalibration::visualizeMap(const std::string& overlayName, MapType type) const
{
    vcharge::GLOverlayExtended overlay(overlayName, VCharge::COORDINATE_FRAME_GLOBAL);

    // visualize camera poses
    overlay.pointSize(2.0f);
    overlay.lineWidth(1.0f);

    if (type == REFERENCE_MAP)
    {
        for (size_t i = 0; i < m_cameras.size(); ++i)
        {
            boost::unordered_set<Point3DFeature*> scenePointSet;

            for (size_t j = 0; j < m_refGraph.frameSetSegments().size(); ++j)
            {
                const FrameSetSegment& segment = m_refGraph.frameSetSegment(j);

                for (size_t k = 0; k < segment.size(); ++k)
                {
                    const FrameSetPtr& frameSet = segment.at(k);
                    const FramePtr& frame = frameSet->frames().at(i);

                    if (frame.get() == 0)
                    {
                        continue;
                    }

                    const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                    for (size_t l = 0; l < features2D.size(); ++l)
                    {
                        if (features2D.at(l)->feature3D().get() == 0)
                        {
                            continue;
                        }

                        scenePointSet.insert(features2D.at(l)->feature3D().get());
                    }
                }
            }

            // visualize 3D scene points
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
                Eigen::Vector3d p = (*it)->point();

                overlay.vertex3f(p(0), p(1), p(2));
            }

            overlay.end();
        }
    }
    else
    {
        boost::unordered_set<Point3DFeature*> scenePointSet[m_cameras.size()];
        for (size_t i = 0; i < m_framesets.size(); ++i)
        {
            const FrameSet& frameset = m_framesets.at(i);

            for (size_t j = 0; j < frameset.frames.size(); ++j)
            {
                const FramePtr& frame = frameset.frames.at(j);

                const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    if (features2D.at(l)->feature3D().get() == 0)
                    {
                        continue;
                    }

                    if (features2D.at(l)->feature3D()->point().norm() < 1000.0)
                    {
                        scenePointSet[frame->cameraId()].insert(features2D.at(l)->feature3D().get());
                    }
                }
            }
        }

        for (size_t i = 0; i < m_cameras.size(); ++i)
        {
            // visualize 3D scene points
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

            for (boost::unordered_set<Point3DFeature*>::iterator it = scenePointSet[i].begin();
                     it != scenePointSet[i].end(); ++it)
            {
                Eigen::Vector3d p = (*it)->point();

                overlay.vertex3f(p(0), p(1), p(2));
            }

            overlay.end();
        }
    }

    overlay.publish();
}

void
InfrastructureCalibration::visualizeCameraPose(const FrameConstPtr& frame,
                                               bool showScenePoints)
{
    Eigen::Matrix4d H_cam = frame->cameraPose()->toMatrix().inverse();

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
    }

    m_overlay.color4f(1.0f, 1.0f, 1.0f, 1.0f);
    m_overlay.begin(VCharge::LINES);

    for (int k = 1; k < 5; ++k)
    {
        m_overlay.vertex3f(frustum.at(0)(0), frustum.at(0)(1), frustum.at(0)(2));
        m_overlay.vertex3f(frustum.at(k)(0), frustum.at(k)(1), frustum.at(k)(2));
    }

    m_overlay.end();

    switch (frame->cameraId())
    {
    case vcharge::CAMERA_FRONT:
        m_overlay.color4f(1.0f, 0.0f, 0.0f, 0.5f);
        break;
    case vcharge::CAMERA_LEFT:
        m_overlay.color4f(0.0f, 1.0f, 0.0f, 0.5f);
        break;
    case vcharge::CAMERA_REAR:
        m_overlay.color4f(0.0f, 1.0f, 1.0f, 0.5f);
        break;
    case vcharge::CAMERA_RIGHT:
        m_overlay.color4f(1.0f, 1.0f, 0.0f, 0.5f);
        break;
    default:
        m_overlay.color4f(1.0f, 1.0f, 1.0f, 0.5f);
    }

    m_overlay.begin(VCharge::POLYGON);

    for (int k = 1; k < 5; ++k)
    {
        m_overlay.vertex3f(frustum.at(k)(0), frustum.at(k)(1), frustum.at(k)(2));
    }

    m_overlay.end();

    if (!showScenePoints)
    {
        return;
    }

    m_overlay.begin(VCharge::LINES);
    for (size_t i = 0; i < frame->features2D().size(); ++i)
    {
        const Point2DFeaturePtr& p2D = frame->features2D().at(i);
        if (p2D->feature3D().get() == 0)
        {
            continue;
        }

        Eigen::Vector3d scenePoint = p2D->feature3D()->point();

        m_overlay.vertex3f(H_cam(0,3), H_cam(1,3), H_cam(2,3));
        m_overlay.vertex3f(scenePoint(0), scenePoint(1), scenePoint(2));
    }
    m_overlay.end();
}

void
InfrastructureCalibration::visualizeCameraPoses(bool showScenePoints)
{
    m_overlay.clear();

    for (size_t i = 0; i < m_framesets.size(); ++i)
    {
        FrameSet& frameset = m_framesets.at(i);

        for (size_t j = 0; j < frameset.frames.size(); ++j)
        {
            FramePtr& frame = frameset.frames.at(j);

            Eigen::Matrix4d H_cam = frame->cameraPose()->toMatrix().inverse();

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
            }

            m_overlay.color4f(1.0f, 1.0f, 1.0f, 1.0f);
            m_overlay.begin(VCharge::LINES);

            for (int k = 1; k < 5; ++k)
            {
                m_overlay.vertex3f(frustum.at(0)(0), frustum.at(0)(1), frustum.at(0)(2));
                m_overlay.vertex3f(frustum.at(k)(0), frustum.at(k)(1), frustum.at(k)(2));
            }

            m_overlay.end();

            if (!showScenePoints)
            {
                continue;
            }

            switch (frame->cameraId())
            {
            case vcharge::CAMERA_FRONT:
                m_overlay.color4f(1.0f, 0.0f, 0.0f, 0.5f);
                break;
            case vcharge::CAMERA_LEFT:
                m_overlay.color4f(0.0f, 1.0f, 0.0f, 0.5f);
                break;
            case vcharge::CAMERA_REAR:
                m_overlay.color4f(0.0f, 1.0f, 1.0f, 0.5f);
                break;
            case vcharge::CAMERA_RIGHT:
                m_overlay.color4f(1.0f, 1.0f, 0.0f, 0.5f);
                break;
            default:
                m_overlay.color4f(1.0f, 1.0f, 1.0f, 0.5f);
            }

            m_overlay.begin(VCharge::POLYGON);

            for (int k = 1; k < 5; ++k)
            {
                m_overlay.vertex3f(frustum.at(k)(0), frustum.at(k)(1), frustum.at(k)(2));
            }

            m_overlay.end();

            m_overlay.begin(VCharge::POINTS);
            for (size_t i = 0; i < frame->features2D().size(); ++i)
            {
                const Point2DFeaturePtr& p2D = frame->features2D().at(i);
                if (p2D->feature3D().get() == 0)
                {
                    continue;
                }

                Eigen::Vector3d scenePoint = p2D->feature3D()->point();

                m_overlay.vertex3f(scenePoint(0), scenePoint(1), scenePoint(2));
            }
            m_overlay.end();
        }
    }

    m_overlay.publish();
}

void
InfrastructureCalibration::visualizeExtrinsics(void) const
{
    vcharge::GLOverlayExtended overlay("infra-extrinsics", VCharge::COORDINATE_FRAME_GLOBAL);

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

    for (int i = 0; i < m_cameras.size(); ++i)
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
        }

        overlay.color4f(1.0f, 1.0f, 1.0f, 1.0f);
        overlay.begin(VCharge::LINES);

        for (int k = 1; k < 5; ++k)
        {
            overlay.vertex3f(frustum.at(0)(2), -frustum.at(0)(0), -frustum.at(0)(1));
            overlay.vertex3f(frustum.at(k)(2), -frustum.at(k)(0), -frustum.at(k)(1));
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
            overlay.vertex3f(frustum.at(k)(2), -frustum.at(k)(0), -frustum.at(k)(1));
        }

        overlay.end();
    }

    overlay.publish();
}

void
InfrastructureCalibration::visualizeOdometry(void) const
{
    vcharge::GLOverlayExtended overlay("infra-odo", VCharge::COORDINATE_FRAME_GLOBAL);

    overlay.lineWidth(1.0f);
    overlay.color3f(0.7f, 0.7f, 0.7f);

    double w_2 = 0.05;
    double l_2 = 0.1;

    double vertices[4][3] = {{-w_2, 0.0, -l_2},
                             {-w_2, 0.0, l_2},
                             {w_2, 0.0, l_2},
                             {w_2, 0.0, -l_2}};

    for (size_t i = 0; i < m_framesets.size(); ++i)
    {
        const FrameSet& frameset = m_framesets.at(i);

        const OdometryPtr& odometry = frameset.frames.at(0)->systemPose();

        Eigen::Matrix4d H = odometry->toMatrix();

        overlay.begin(VCharge::LINE_LOOP);

        for (int i = 0; i < 4; ++i)
        {
            Eigen::Vector3d p;
            p << vertices[i][0], vertices[i][1], vertices[i][2];

            p = transformPoint(H, p);

            overlay.vertex3f(p(0), p(1), p(2));
        }

        overlay.end();

        Eigen::Vector3d p0(0.0, 0.0, 0.0);
        Eigen::Vector3d p1(0.0, 0.0, l_2);

        p0 = transformPoint(H, p0);
        p1 = transformPoint(H, p1);

        overlay.begin(VCharge::LINES);

        overlay.vertex3f(p0(0), p0(1), p0(2));
        overlay.vertex3f(p1(0), p1(1), p1(2));

        overlay.end();
    }

    overlay.publish();
}

#endif

}
