#include "SlidingWindowBA.h"

#include <boost/unordered_set.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../ceres-solver/include/ceres/ceres.h"
#include "../gpl/EigenUtils.h"
#include "../npoint/five-point/five-point.hpp"
#include "ReprojectionError.h"

namespace camodocal
{

SlidingWindowBA::SlidingWindowBA(const CataCamera::Parameters& cameraParameters,
                                 int N, int n, int mode,
                                 Eigen::Matrix4d globalCameraPose)
 : mCamera(cameraParameters)
 , m_N(N)
 , m_n(n)
 , mMode(mode)
 , kMinDisparity(3.0)
 , kNominalFocalLength(300.0)
 , kReprojErrorThresh(1.0)
 , kTVTReprojErrorThresh(3.0)
 , mFrameCount(0)
 , mVerbose(false)
 , kMin2D2DFeatureCorrespondences(10)
 , kMin2D3DFeatureCorrespondences(10)
{
    const Eigen::Matrix4d& H_cam_odo = globalCameraPose;

    m_T_cam_odo.rotation() = Eigen::Quaterniond(H_cam_odo.block<3,3>(0,0));
    m_T_cam_odo.translation() = Eigen::Vector3d(H_cam_odo.block<3,1>(0,3));
}

Eigen::Matrix4d
SlidingWindowBA::globalCameraPose(void)
{
    return m_T_cam_odo.pose();
}

bool
SlidingWindowBA::addFrame(FramePtr& frame,
                          const Eigen::Matrix3d& R_rel, const Eigen::Vector3d& t_rel,
                          Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    FramePtr frameCurr = frame;

    if (mMode == VO)
    {
        frameCurr->camera() = PosePtr(new Pose);
    }

    frameCurr->id() = mFrameCount;

    mWindow.push_back(frameCurr);
    while (mWindow.size() > m_N)
    {
        mWindow.pop_front();
    }

    ++mFrameCount;

    if (mVerbose)
    {
        std::cout << "# INFO: Added frame " << mFrameCount - 1 << "." << std::endl;
    }

    if (mFrameCount == 1)
    {
        if (mMode == VO)
        {
            frameCurr->camera()->rotation() = Eigen::Quaterniond::Identity();
            frameCurr->camera()->translation().setZero();

            R = frameCurr->camera()->rotation().toRotationMatrix();
            t = frameCurr->camera()->translation();
        }

        return true;
    }

    FramePtr framePrev = *(++mWindow.rbegin());

    if (mMode == VO)
    {
        frameCurr->camera()->rotation() = Eigen::Quaterniond(R_rel) * framePrev->camera()->rotation();
        frameCurr->camera()->translation() = R_rel * framePrev->camera()->translation() + t_rel;
    }

    if (mFrameCount < 3)
    {
        if (mMode == VO)
        {
            R = frameCurr->camera()->rotation().toRotationMatrix();
            t = frameCurr->camera()->translation();
        }

        return true;
    }

    std::vector<std::vector<Point2DFeaturePtr> > featureCorrespondencesToCheck;

    if (mFrameCount == 3)
    {
        // compute pose in frame 2 relative to frame 0
        std::vector<std::vector<Point2DFeaturePtr> > featureCorrespondences;

        // use features that are seen in frames 0, 1, and 2
        findFeatureCorrespondences(frameCurr->features2D(), 3, featureCorrespondences);

        if (mVerbose)
        {
            std::cout << "# INFO: Found " << featureCorrespondences.size() << " feature correspondences in last 3 frames." << std::endl;
        }

        std::vector<cv::Point2f> imagePoints[3];
        for (size_t i = 0; i < featureCorrespondences.size(); ++i)
        {
            std::vector<Point2DFeaturePtr>& fc = featureCorrespondences.at(i);

            for (size_t j = 0; j < fc.size(); ++j)
            {
                imagePoints[j].push_back(fc.at(j)->keypoint().pt);
            }
        }

        if (imagePoints[0].size() < kMin2D2DFeatureCorrespondences)
        {
            if (mVerbose)
            {
                std::cout << "# INFO: Insufficient number of 2D-2D correspondences for BA initialization." << std::endl;
            }

            return false;
        }

        std::vector<cv::Point2f> rectImagePoints[3];
        for (size_t i = 0; i < 3; ++i)
        {
            rectifyImagePoints(imagePoints[i], rectImagePoints[i], kNominalFocalLength);
        }

        cv::Mat inliers;
        if (mMode == VO)
        {
            cv::Point2f pp(mCamera.parameters().imageWidth() / 2.0f, mCamera.parameters().imageHeight() / 2.0f);
            cv::Mat E, R_cv, t_cv;
            E = findEssentialMat(rectImagePoints[0], rectImagePoints[2], kNominalFocalLength, pp,
                                 CV_FM_RANSAC, 0.99, kReprojErrorThresh, 100, inliers);
            recoverPose(E, rectImagePoints[0], rectImagePoints[2], R_cv, t_cv, kNominalFocalLength, pp, inliers);

            if (mVerbose)
            {
                std::cout << "# INFO: Computed pose in frame 0 wrt pose in frame 2 with " << inliers.cols << " inliers:" << std::endl;
                std::cout << R_cv << std::endl;
                std::cout << t_cv << std::endl;
            }

            cv::cv2eigen(R_cv, R);
            cv::cv2eigen(t_cv, t);

            frameCurr->camera()->rotation() = Eigen::Quaterniond(R);
            frameCurr->camera()->translation() = t;
        }
        else
        {
            inliers = cv::Mat(1, featureCorrespondences.size(), CV_8U);
            inliers = cv::Scalar(1);
        }

        std::vector<std::vector<Point2DFeaturePtr> > inlierFeatureCorrespondences;
        for (int i = 0; i < inliers.cols; ++i)
        {
            if (inliers.at<unsigned char>(0,i) == 0)
            {
                continue;
            }

            inlierFeatureCorrespondences.push_back(featureCorrespondences.at(i));
        }

        for (int i = 0; i < 3; ++i)
        {
            imagePoints[i].clear();
        }
        for (size_t i = 0; i < inlierFeatureCorrespondences.size(); ++i)
        {
            std::vector<Point2DFeaturePtr>& fc = inlierFeatureCorrespondences.at(i);

            for (size_t j = 0; j < fc.size(); ++j)
            {
                imagePoints[j].push_back(fc.at(j)->keypoint().pt);
            }
        }

        for (size_t i = 0; i < 3; ++i)
        {
            rectifyImagePoints(imagePoints[i], rectImagePoints[i], kNominalFocalLength);
        }

        // triangulate scene points
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points3D;
        std::vector<size_t> indices;

        if (mMode == VO)
        {
            triangulatePoints(mWindow.front()->camera()->rotation(), mWindow.front()->camera()->translation(), imagePoints[0],
                              frameCurr->camera()->rotation(), frameCurr->camera()->translation(), imagePoints[2],
                              points3D, indices);
        }
        else
        {
            Eigen::Matrix4d H_odo_cam = m_T_cam_odo.pose().inverse();

            Eigen::Matrix4d H1 = H_odo_cam * mWindow.front()->odometer()->pose().inverse();
            Eigen::Matrix4d H2 = H_odo_cam * frameCurr->odometer()->pose().inverse();

            triangulatePoints(Eigen::Quaterniond(H1.block<3,3>(0,0)), Eigen::Vector3d(H1.block<3,1>(0,3)), imagePoints[0],
                              Eigen::Quaterniond(H2.block<3,3>(0,0)), Eigen::Vector3d(H2.block<3,1>(0,3)), imagePoints[2],
                              points3D, indices);
        }

        if (mVerbose)
        {
            std::cout << "# INFO: Triangulated " << points3D.size() << " points." << std::endl;

            size_t count = 0;
            double errorTotal = 0.0;
            double errorMax = std::numeric_limits<double>::min();

            for (size_t i = 0; i < points3D.size(); ++i)
            {
                const cv::Point2f& feature2D = imagePoints[0].at(indices.at(i));

                const Eigen::Vector3d& feature3D = points3D.at(i);

                double error;
                if (mMode == VO)
                {
                    CameraReprojectionError reprojErr(mCamera.parameters(), feature2D.x, feature2D.y);

                    double residuals[2];
                    reprojErr(mWindow.front()->camera()->rotationData(),
                              mWindow.front()->camera()->translationData(),
                              feature3D.data(), residuals);

                    error = hypot(residuals[0], residuals[1]);
                }
                else
                {
                    OdometerReprojectionError reprojErr(mCamera.parameters(), feature2D.x, feature2D.y);

                    double residuals[2];
                    reprojErr(m_T_cam_odo.rotationData(),
                              m_T_cam_odo.translationData(),
                              mWindow.front()->odometer()->positionData(),
                              mWindow.front()->odometer()->yawData(),
                              feature3D.data(), residuals);

                    error = hypot(residuals[0], residuals[1]);
                }

                errorTotal += error;

                if (error > errorMax)
                {
                    errorMax = error;
                }

                ++count;
            }

            double errorAvg = errorTotal / count;

            std::cout << "# INFO: Reprojection error in frame 0: avg = " << errorAvg
                     << " px | max = " << errorMax << " px." << std::endl;

            count = 0;
            errorTotal = 0.0;
            errorMax = std::numeric_limits<double>::min();

            for (size_t i = 0; i < points3D.size(); ++i)
            {
                const cv::Point2f& feature2D = imagePoints[2].at(indices.at(i));

                const Eigen::Vector3d& feature3D = points3D.at(i);

                double error;
                if (mMode == VO)
                {
                    CameraReprojectionError reprojErr(mCamera.parameters(), feature2D.x, feature2D.y);

                    double residuals[2];
                    reprojErr(frameCurr->camera()->rotationData(),
                              frameCurr->camera()->translationData(),
                              feature3D.data(), residuals);

                    error = hypot(residuals[0], residuals[1]);
                }
                else
                {
                    OdometerReprojectionError reprojErr(mCamera.parameters(), feature2D.x, feature2D.y);

                    double residuals[2];
                    reprojErr(m_T_cam_odo.rotationData(),
                              m_T_cam_odo.translationData(),
                              frameCurr->odometer()->positionData(),
                              frameCurr->odometer()->yawData(),
                              feature3D.data(), residuals);

                    error = hypot(residuals[0], residuals[1]);
                }

                errorTotal += error;

                if (error > errorMax)
                {
                    errorMax = error;
                }

                ++count;
            }

            errorAvg = errorTotal / count;

            std::cout << "# INFO: Reprojection error in frame 2: avg = " << errorAvg
                     << " px | max = " << errorMax << " px." << std::endl;
        }

        if (points3D.size() < kMin2D3DFeatureCorrespondences)
        {
            if (mVerbose)
            {
                std::cout << "# INFO: Insufficient number of 2D-3D correspondences for BA initialization." << std::endl;
            }

            return false;
        }

        // find pose in frame 1 from 3D-2D point correspondences
        std::vector<cv::Point3f> scenePoints;

        for (size_t i = 0; i < points3D.size(); ++i)
        {
            const Eigen::Vector3d& p = points3D.at(i);

            scenePoints.push_back(cv::Point3f(p(0), p(1), p(2)));
        }

        std::vector<cv::Point2f> tmpImagePoints;
        for (size_t i = 0; i < indices.size(); ++i)
        {
            size_t idx = indices.at(i);

            tmpImagePoints.push_back(rectImagePoints[1].at(idx));
        }
        rectImagePoints[1] = tmpImagePoints;

        if (mMode == VO)
        {
            cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
            cameraMatrix.at<double>(0,0) = kNominalFocalLength;
            cameraMatrix.at<double>(1,1) = kNominalFocalLength;
            cameraMatrix.at<double>(0,2) = mCamera.parameters().imageWidth() / 2.0;
            cameraMatrix.at<double>(1,2) = mCamera.parameters().imageHeight() / 2.0;

            cv::Mat rvec_cv, tvec_cv;
            cv::solvePnPRansac(scenePoints, rectImagePoints[1], cameraMatrix, cv::noArray(),
                               rvec_cv, tvec_cv, false, 100, kReprojErrorThresh, 100, cv::noArray(), CV_EPNP);

            if (mVerbose)
            {
                std::cout << "# INFO: Using " << scenePoints.size() << " scene points to compute pose in frame 1 via PnP:" << std::endl;

                cv::Mat R_cv;
                cv::Rodrigues(rvec_cv, R_cv);
                std::cout << R_cv << std::endl;
                std::cout << tvec_cv << std::endl;
            }

            Eigen::Vector3d rvec, tvec;
            cv::cv2eigen(rvec_cv, rvec);
            cv::cv2eigen(tvec_cv, tvec);

            framePrev->camera()->rotation() = AngleAxisToQuaternion(rvec);
            framePrev->camera()->translation() = tvec;
        }

        for (size_t i = 0; i < points3D.size(); ++i)
        {
            size_t idx = indices.at(i);

            std::vector<Point2DFeaturePtr>& fc = inlierFeatureCorrespondences.at(idx);

            Point2DFeaturePtr& f0 = fc.at(0);
            Point2DFeaturePtr& f1 = fc.at(1);
            Point2DFeaturePtr& f2 = fc.at(2);

            double error;
            if (mMode == VO)
            {
                CameraReprojectionError reprojErr(mCamera.parameters(), f1->keypoint().pt.x, f1->keypoint().pt.y);

                double residuals[2];
                reprojErr(framePrev->camera()->rotationData(),
                          framePrev->camera()->translationData(),
                          points3D.at(i).data(), residuals);

                error = hypot(residuals[0], residuals[1]);
            }
            else
            {
                OdometerReprojectionError reprojErr(mCamera.parameters(), f1->keypoint().pt.x, f1->keypoint().pt.y);

                double residuals[2];
                reprojErr(m_T_cam_odo.rotationData(),
                          m_T_cam_odo.translationData(),
                          framePrev->odometer()->positionData(),
                          framePrev->odometer()->yawData(),
                          points3D.at(i).data(), residuals);

                error = hypot(residuals[0], residuals[1]);
            }

            // remove feature correspondences marked as outliers in PnP RANSAC
           if (mMode == VO && error > kReprojErrorThresh)
           {
               f0->bestNextMatchIdx() = -1;
               f1->bestPrevMatchIdx() = -1;
               f1->bestNextMatchIdx() = -1;
               f2->bestPrevMatchIdx() = -1;
           }
           else
           {
               Point3DFeaturePtr point3D(new Point3DFeature);

               point3D->point() = points3D.at(i);

               for (int j = 0; j < 3; ++j)
               {
                   Point2DFeaturePtr& pt = fc.at(j);

                   point3D->features2D().push_back(pt);
                   pt->feature3D() = point3D;
               }

               frameCurr->features3D().push_back(point3D);
           }
        }

        // remove untriangulated feature correspondences
        for (size_t i = 0; i < featureCorrespondences.size(); ++i)
        {
            std::vector<Point2DFeaturePtr>& fc = featureCorrespondences.at(i);

            Point2DFeaturePtr& f0 = fc.at(0);
            Point2DFeaturePtr& f1 = fc.at(1);
            Point2DFeaturePtr& f2 = fc.at(2);

            if (f2->feature3D().get() != 0)
            {
                continue;
            }

            f0->bestNextMatchIdx() = -1;
            f1->bestNextMatchIdx() = -1;
            f1->bestPrevMatchIdx() = -1;
            f2->bestPrevMatchIdx() = -1;
        }
    }
    else
    {
        std::vector<cv::Point3f> scenePoints;
        std::vector<cv::Point2f> imagePoints;

        // use features that are seen in both previous and current frames,
        // and have associated 3D scene points
        std::vector<std::vector<Point2DFeaturePtr> > featureCorrespondences;
        findFeatureCorrespondences(frameCurr->features2D(), 3, featureCorrespondences);

        if (mVerbose)
        {
            std::cout << "# INFO: Found " << featureCorrespondences.size() << " feature correspondences in last 3 frames." << std::endl;
        }

        std::vector<std::vector<Point2DFeaturePtr> > triFeatureCorrespondences;
        std::vector<std::vector<Point2DFeaturePtr> > untriFeatureCorrespondences;
        for (size_t i = 0; i < featureCorrespondences.size(); ++i)
        {
            std::vector<Point2DFeaturePtr>& fc = featureCorrespondences.at(i);

            Point2DFeaturePtr& f0 = fc.at(0);
            Point2DFeaturePtr& f1 = fc.at(1);
            Point2DFeaturePtr& f2 = fc.at(2);

            if (f0->feature3D().get() == 0 || f1->feature3D().get() == 0)
            {
                if (f0->feature3D().get() == 0 && f1->feature3D().get() == 0)
                {
                    untriFeatureCorrespondences.push_back(fc);
                }

                continue;
            }

            triFeatureCorrespondences.push_back(fc);

            const Eigen::Vector3d& p = f1->feature3D()->point();
            scenePoints.push_back(cv::Point3f(p(0), p(1), p(2)));

            imagePoints.push_back(f2->keypoint().pt);
        }

        if (mMode == VO)
        {
            if (scenePoints.size() < kMin2D3DFeatureCorrespondences)
            {
                if (mVerbose)
                {
                    std::cout << "# INFO: Insufficient number of 2D-3D correspondences (#" << scenePoints.size() << ") for PnP RANSAC." << std::endl;
                }

                return false;
            }

            if (mVerbose)
            {
                std::cout << "# INFO: Using " << scenePoints.size() << " scene points to compute pose via PnP RANSAC." << std::endl;
            }

            cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
            cameraMatrix.at<double>(0,0) = kNominalFocalLength;
            cameraMatrix.at<double>(1,1) = kNominalFocalLength;
            cameraMatrix.at<double>(0,2) = mCamera.parameters().imageWidth() / 2.0;
            cameraMatrix.at<double>(1,2) = mCamera.parameters().imageHeight() / 2.0;

            std::vector<cv::Point2f> rectImagePoints;
            rectifyImagePoints(imagePoints, rectImagePoints, kNominalFocalLength);

            Eigen::Vector3d rvec, tvec;
            cv::Mat rvec_cv, tvec_cv;

            Eigen::Matrix3d R_est = R_rel * framePrev->camera()->rotation();
            rvec = RotationToAngleAxis(R_est);
            cv::eigen2cv(rvec, rvec_cv);

            cv::eigen2cv(framePrev->camera()->translation(), tvec_cv);

            cv::solvePnPRansac(scenePoints, rectImagePoints, cameraMatrix, cv::noArray(),
                               rvec_cv, tvec_cv, true, 100, kReprojErrorThresh, 100, cv::noArray(), CV_ITERATIVE);

            cv::cv2eigen(rvec_cv, rvec);
            cv::cv2eigen(tvec_cv, tvec);

            if (mVerbose)
            {
                std::cout << "# INFO: Computed pose in frame " << mFrameCount - 1 << ":" << std::endl;

                cv::Mat R_cv;
                cv::Rodrigues(rvec_cv, R_cv);
                std::cout << R_cv << std::endl;
                std::cout << tvec_cv << std::endl;
            }

            frameCurr->camera()->rotation() = AngleAxisToQuaternion(rvec);
            frameCurr->camera()->translation() = tvec;
        }

        // remove feature correspondences marked as outliers in PnP RANSAC
        for (size_t i = 0; i < triFeatureCorrespondences.size(); ++i)
        {
            std::vector<Point2DFeaturePtr>& fc = triFeatureCorrespondences.at(i);

            Point2DFeaturePtr& f0 = fc.at(0);
            Point2DFeaturePtr& f1 = fc.at(1);
            Point2DFeaturePtr& f2 = fc.at(2);

            double error;
            if (mMode == VO)
            {
                CameraReprojectionError reprojErr(mCamera.parameters(), f2->keypoint().pt.x, f2->keypoint().pt.y);

                double residuals[2];
                reprojErr(frameCurr->camera()->rotationData(),
                          frameCurr->camera()->translationData(),
                          f1->feature3D()->pointData(), residuals);

                error = hypot(residuals[0], residuals[1]);
            }
            else
            {
                OdometerReprojectionError reprojErr(mCamera.parameters(), f2->keypoint().pt.x, f2->keypoint().pt.y);

                double residuals[2];
                reprojErr(m_T_cam_odo.rotationData(),
                          m_T_cam_odo.translationData(),
                          frameCurr->odometer()->positionData(),
                          frameCurr->odometer()->yawData(),
                          f1->feature3D()->pointData(), residuals);

                error = hypot(residuals[0], residuals[1]);
            }

            if (mMode == VO && error > kReprojErrorThresh)
            {
                f1->bestNextMatchIdx() = -1;
                f2->bestPrevMatchIdx() = -1;
            }
            else
            {
                f2->feature3D() = f1->feature3D();
                f1->feature3D()->features2D().push_back(f2);

                frameCurr->features3D().push_back(f2->feature3D());
            }
        }

        if (mVerbose)
        {
            size_t count = 0;
            double totalError = 0.0;

            for (size_t i = 0; i < scenePoints.size(); ++i)
            {
                const cv::Point2f& feature2D = imagePoints.at(i);

                const cv::Point3f& feature3D = scenePoints.at(i);
                double point3D[3] = {feature3D.x, feature3D.y, feature3D.z};

                double error;
                if (mMode == VO)
                {
                    CameraReprojectionError reprojErr(mCamera.parameters(), feature2D.x, feature2D.y);

                    double residuals[2];
                    reprojErr(frameCurr->camera()->rotationData(),
                              frameCurr->camera()->translationData(),
                              point3D, residuals);

                    error = hypot(residuals[0], residuals[1]);
                }
                else
                {
                    OdometerReprojectionError reprojErr(mCamera.parameters(), feature2D.x, feature2D.y);

                    double residuals[2];
                    reprojErr(m_T_cam_odo.rotationData(),
                              m_T_cam_odo.translationData(),
                              frameCurr->odometer()->positionData(),
                              frameCurr->odometer()->yawData(),
                              point3D, residuals);

                    error = hypot(residuals[0], residuals[1]);
                }

                totalError += error;
                ++count;
            }

            double avgError = totalError / count;

            std::cout << "# INFO: Reprojection error with computed pose: " << avgError << " px." << std::endl;
        }

        // triangulate new feature correspondences seen in last 3 frames
        std::vector<cv::Point2f> ipoints[3];

        for (size_t i = 0; i < untriFeatureCorrespondences.size(); ++i)
        {
            std::vector<Point2DFeaturePtr>& fc = untriFeatureCorrespondences.at(i);

            Point2DFeaturePtr& f0 = fc.at(0);
            Point2DFeaturePtr& f1 = fc.at(1);
            Point2DFeaturePtr& f2 = fc.at(2);

            ipoints[0].push_back(f0->keypoint().pt);
            ipoints[1].push_back(f1->keypoint().pt);
            ipoints[2].push_back(f2->keypoint().pt);
        }

        if (mVerbose)
        {
            std::cout << "# INFO: Found " << untriFeatureCorrespondences.size() << " new feature correspondences." << std::endl;
        }

        if (!untriFeatureCorrespondences.empty())
        {
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points3D;
            std::vector<size_t> indices;

            std::list<FramePtr>::reverse_iterator it = mWindow.rbegin();
            ++it; ++it;
            FramePtr& framePrevPrev = *it;

            if (mMode == VO)
            {
                tvt(framePrevPrev->camera()->rotation(), framePrevPrev->camera()->translation(), ipoints[0],
                    framePrev->camera()->rotation(), framePrev->camera()->translation(), ipoints[1],
                    frameCurr->camera()->rotation(), frameCurr->camera()->translation(), ipoints[2],
                    points3D, indices);
            }
            else
            {
                Eigen::Matrix4d H_odo_cam = m_T_cam_odo.pose().inverse();

                Eigen::Matrix4d H1 = H_odo_cam * framePrevPrev->odometer()->pose().inverse();
                Eigen::Matrix4d H2 = H_odo_cam * framePrev->odometer()->pose().inverse();
                Eigen::Matrix4d H3 = H_odo_cam * frameCurr->odometer()->pose().inverse();

                tvt(Eigen::Quaterniond(H1.block<3,3>(0,0)), Eigen::Vector3d(H1.block<3,1>(0,3)), ipoints[0],
                    Eigen::Quaterniond(H2.block<3,3>(0,0)), Eigen::Vector3d(H2.block<3,1>(0,3)), ipoints[1],
                    Eigen::Quaterniond(H3.block<3,3>(0,0)), Eigen::Vector3d(H3.block<3,1>(0,3)), ipoints[2],
                    points3D, indices);
            }

            if (mVerbose)
            {
                std::cout << "# INFO: Triangulated " << points3D.size() << " new points." << std::endl;

                if (!points3D.empty())
                {
                    size_t count = 0;
                    double errorTotal = 0.0;
                    double errorMax = std::numeric_limits<double>::min();

                    for (size_t i = 0; i < points3D.size(); ++i)
                    {
                        const cv::Point2f& feature2D = ipoints[0].at(indices.at(i));

                        const Eigen::Vector3d& feature3D = points3D.at(i);

                        double error;
                        if (mMode == VO)
                        {
                            CameraReprojectionError reprojErr(mCamera.parameters(), feature2D.x, feature2D.y);

                            double residuals[2];
                            reprojErr(framePrevPrev->camera()->rotationData(),
                                      framePrevPrev->camera()->translationData(),
                                      feature3D.data(), residuals);

                            error = hypot(residuals[0], residuals[1]);
                        }
                        else
                        {
                            OdometerReprojectionError reprojErr(mCamera.parameters(), feature2D.x, feature2D.y);

                            double residuals[2];
                            reprojErr(m_T_cam_odo.rotationData(),
                                      m_T_cam_odo.translationData(),
                                      framePrevPrev->odometer()->positionData(),
                                      framePrevPrev->odometer()->yawData(),
                                      feature3D.data(), residuals);

                            error = hypot(residuals[0], residuals[1]);
                        }

                        errorTotal += error;

                        if (error > errorMax)
                        {
                            errorMax = error;
                        }

                        ++count;
                    }

                    double errorAvg = errorTotal / count;

                    std::cout << "# INFO: Reprojection error in frame n-2: avg = " << errorAvg
                              << " px | max = " << errorMax << " px." << std::endl;

                    count = 0;
                    errorTotal = 0.0;
                    errorMax = std::numeric_limits<double>::min();

                    for (size_t i = 0; i < points3D.size(); ++i)
                    {
                        const cv::Point2f& feature2D = ipoints[1].at(indices.at(i));

                        const Eigen::Vector3d& feature3D = points3D.at(i);

                        double error;
                        if (mMode == VO)
                        {
                            CameraReprojectionError reprojErr(mCamera.parameters(), feature2D.x, feature2D.y);

                            double residuals[2];
                            reprojErr(framePrev->camera()->rotationData(),
                                      framePrev->camera()->translationData(),
                                      feature3D.data(), residuals);

                            error = hypot(residuals[0], residuals[1]);
                        }
                        else
                        {
                            OdometerReprojectionError reprojErr(mCamera.parameters(), feature2D.x, feature2D.y);

                            double residuals[2];
                            reprojErr(m_T_cam_odo.rotationData(),
                                      m_T_cam_odo.translationData(),
                                      framePrev->odometer()->positionData(),
                                      framePrev->odometer()->yawData(),
                                      feature3D.data(), residuals);

                            error = hypot(residuals[0], residuals[1]);
                        }

                        errorTotal += error;

                        if (error > errorMax)
                        {
                            errorMax = error;
                        }

                        ++count;
                    }

                    errorAvg = errorTotal / count;

                    std::cout << "# INFO: Reprojection error in frame n-1: avg = " << errorAvg
                              << " px | max = " << errorMax << " px." << std::endl;

                    count = 0;
                    errorTotal = 0.0;
                    errorMax = std::numeric_limits<double>::min();

                    for (size_t i = 0; i < points3D.size(); ++i)
                    {
                        const cv::Point2f& feature2D = ipoints[2].at(indices.at(i));

                        const Eigen::Vector3d& feature3D = points3D.at(i);

                        double error;
                        if (mMode == VO)
                        {
                            CameraReprojectionError reprojErr(mCamera.parameters(), feature2D.x, feature2D.y);

                            double residuals[2];
                            reprojErr(frameCurr->camera()->rotationData(),
                                      frameCurr->camera()->translationData(),
                                      feature3D.data(), residuals);

                            error = hypot(residuals[0], residuals[1]);
                        }
                        else
                        {
                            OdometerReprojectionError reprojErr(mCamera.parameters(), feature2D.x, feature2D.y);

                            double residuals[2];
                            reprojErr(m_T_cam_odo.rotationData(),
                                      m_T_cam_odo.translationData(),
                                      frameCurr->odometer()->positionData(),
                                      frameCurr->odometer()->yawData(),
                                      feature3D.data(), residuals);

                            error = hypot(residuals[0], residuals[1]);
                        }

                        errorTotal += error;

                        if (error > errorMax)
                        {
                            errorMax = error;
                        }

                        ++count;
                    }

                    errorAvg = errorTotal / count;

                    std::cout << "# INFO: Reprojection error in frame n: avg = " << errorAvg
                              << " px | max = " << errorMax << " px." << std::endl;
                }
            }

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

                frameCurr->features3D().push_back(point3D);

                featureCorrespondencesToCheck.push_back(fc);
            }

            // remove untriangulated feature correspondences
            for (size_t i = 0; i < untriFeatureCorrespondences.size(); ++i)
            {
                std::vector<Point2DFeaturePtr>& fc = untriFeatureCorrespondences.at(i);

                Point2DFeaturePtr& f0 = fc.at(0);
                Point2DFeaturePtr& f1 = fc.at(1);
                Point2DFeaturePtr& f2 = fc.at(2);

                if (f2->feature3D().get() != 0)
                {
                    continue;
                }

                f0->bestNextMatchIdx() = -1;
                f1->bestNextMatchIdx() = -1;
                f1->bestPrevMatchIdx() = -1;
                f2->bestPrevMatchIdx() = -1;
            }
        }
    }

    if (mVerbose)
    {
        double minError, maxError, avgError;

        windowReprojectionError(minError, maxError, avgError);

        std::cout << "# INFO: Window reprojection error before optimization: min = " << minError << " | max = " << maxError << " | avg = " << avgError << std::endl;
    }

    bool runOptimization = false;
    for (std::list<FramePtr>::const_iterator it = mWindow.begin(); it != mWindow.end(); ++it)
    {
        const FrameConstPtr& frame = *it;

        const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

        for (size_t i = 0; i < features2D.size(); ++i)
        {
            const Point2DFeatureConstPtr& feature2D = features2D.at(i);
            const Point3DFeatureConstPtr& feature3D = feature2D->feature3D();

            if (feature2D->feature3D().get() != 0)
            {
                runOptimization = true;
                break;
            }
        }

        if (runOptimization)
        {
            break;
        }
    }

    // perform BA to optimize camera poses and scene points
    if (runOptimization)
    {
        optimize();
    }

//    // prune triangulated scene points with high reprojection error
//    size_t nPrunedScenePoints = 0;
//    for (size_t i = 0; i < featureCorrespondencesToCheck.size(); ++i)
//    {
//        std::vector<Point2DFeaturePtr>& fc = featureCorrespondencesToCheck.at(i);
//
//        bool prune = false;
//        for (int j = 0; j < 3; ++j)
//        {
//            Point2DFeaturePtr& f = fc.at(j);
//
//            ReprojectionError reprojErr(mCamera.parameters(), f->keypoint().pt.x, f->keypoint().pt.y);
//
//            double residuals[2];
//            reprojErr(frameCurr->camera()->rotationData(),
//                      frameCurr->camera()->translationData(),
//                      f->feature3D()->pointData(), residuals);
//
//            if (hypot(residuals[0], residuals[1]) > kTVTReprojErrorThresh)
//            {
//                prune = true;
//                break;
//            }
//        }
//
//        if (prune)
//        {
//            Point2DFeaturePtr& f0 = fc.at(0);
//            Point2DFeaturePtr& f1 = fc.at(1);
//            Point2DFeaturePtr& f2 = fc.at(2);
//
//            std::vector<Point3DFeaturePtr>::iterator itF3D;
//            itF3D = std::find(frameCurr->features3D().begin(),
//                              frameCurr->features3D().end(),
//                              f0->feature3D());
//            frameCurr->features3D().erase(itF3D);
//
//            f0->feature3D() = Point3DFeaturePtr();
//            f1->feature3D() = Point3DFeaturePtr();
//            f2->feature3D() = Point3DFeaturePtr();
//
//            f0->bestNextMatchIdx() = -1;
//            f1->bestNextMatchIdx() = -1;
//            f1->bestPrevMatchIdx() = -1;
//            f2->bestPrevMatchIdx() = -1;
//
//            ++nPrunedScenePoints;
//        }
//    }
//
//    if (mVerbose)
//    {
//        if (nPrunedScenePoints > 0)
//        {
//            std::cout << "# INFO: Pruned " << nPrunedScenePoints << " scene points that had too high reprojection errors." << std::endl;
//        }
//    }

    // prune scene points that are behind cameras
    size_t nPrunedScenePoints = 0;
    for (std::list<FramePtr>::iterator it = mWindow.begin(); it != mWindow.end(); ++it)
    {
        FramePtr& frame = *it;

        std::vector<Point3DFeaturePtr>& features3D = frame->features3D();

        std::vector<Point3DFeaturePtr>::iterator itF3D = features3D.begin();
        while (itF3D != features3D.end())
        {
            Point3DFeaturePtr feature3D = *itF3D;

            if (feature3D.get() == 0)
            {
                ++itF3D;
                continue;
            }

            Eigen::Vector3d P_cam;
            if (mMode == VO)
            {
                P_cam = frame->camera()->rotation().toRotationMatrix() * feature3D->point() + frame->camera()->translation();

            }
            else
            {
                Eigen::Vector4d P;
                P << feature3D->point(), 1.0;

                P = (m_T_cam_odo.pose().inverse() * frame->odometer()->pose().inverse()) * P;

                P_cam = P.block<3,1>(0,0);
            }

            bool prune = false;
            if (P_cam(2) < 0.0)
            {
                prune = true;
            }

            if (prune)
            {
                std::vector<Point2DFeaturePtr> features2D = feature3D->features2D();

                for (size_t i = 0; i < features2D.size(); ++i)
                {
                    features2D.at(i)->feature3D() = Point3DFeaturePtr();
                }

                features3D.erase(itF3D);

                ++nPrunedScenePoints;
            }
            else
            {
                ++itF3D;
            }
        }
    }

    if (mVerbose)
    {
        if (nPrunedScenePoints > 0)
        {
            std::cout << "# INFO: Pruned " << nPrunedScenePoints << " scene points that were behind cameras." << std::endl;
        }

        double minError, maxError, avgError;

        windowReprojectionError(minError, maxError, avgError);

        std::cout << "# INFO: Window reprojection error after optimization: min = " << minError << " | max = " << maxError << " | avg = " << avgError << std::endl;
    }

    if (mMode == VO)
    {
        R = frameCurr->camera()->rotation().toRotationMatrix();
        t = frameCurr->camera()->translation();
    }

    return true;
}

void
SlidingWindowBA::clear(void)
{
    mFrameCount = 0;
    mWindow.clear();
}

bool
SlidingWindowBA::empty(void) const
{
    return mWindow.empty();
}

size_t
SlidingWindowBA::windowSize(void) const
{
    return mWindow.size();
}

void
SlidingWindowBA::setVerbose(bool verbose)
{
    mVerbose = verbose;
}

int
SlidingWindowBA::N(void)
{
    return m_N;
}

int
SlidingWindowBA::n(void)
{
    return m_n;
}

FramePtr&
SlidingWindowBA::currentFrame(void)
{
    return mWindow.back();
}

std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >
SlidingWindowBA::poses(void) const
{
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > poses;

    for (std::list<FramePtr>::const_iterator it = mWindow.begin(); it != mWindow.end(); ++it)
    {
        const FrameConstPtr& frame = *it;

        Eigen::Matrix4d pose;
        pose.setIdentity();

        pose.block<3,3>(0,0) = frame->camera()->rotation().toRotationMatrix();
        pose.block<3,1>(0,3) = frame->camera()->translation();

        poses.push_back(pose);
    }

    return poses;
}

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >
SlidingWindowBA::scenePoints(void) const
{
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > scenePoints;

    boost::unordered_set<Point3DFeature*> set;

    for (std::list<FramePtr>::const_iterator it = mWindow.begin(); it != mWindow.end(); ++it)
    {
        const FrameConstPtr& frame = *it;

        const std::vector<Point3DFeaturePtr>& features3D = frame->features3D();

        for (size_t i = 0; i < features3D.size(); ++i)
        {
            set.insert(features3D.at(i).get());
        }
    }

    for (boost::unordered_set<Point3DFeature*>::iterator it = set.begin(); it != set.end(); ++it)
    {
        scenePoints.push_back((*it)->point());
    }

    return scenePoints;
}

void
SlidingWindowBA::frameReprojectionError(int windowIdx, double& minError, double& maxError, double& avgError) const
{
    minError = std::numeric_limits<double>::max();
    maxError = std::numeric_limits<double>::min();

    size_t count = 0;
    double totalError = 0.0;

    std::list<FramePtr>::const_iterator it = mWindow.begin();
    std::advance(it, windowIdx);

    const FrameConstPtr& frame = *it;

    const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

    for (size_t i = 0; i < features2D.size(); ++i)
    {
        const Point2DFeatureConstPtr& feature2D = features2D.at(i);
        const Point3DFeatureConstPtr& feature3D = feature2D->feature3D();

        if (feature3D.get() == 0)
        {
            continue;
        }

        double error;
        if (mMode == VO)
        {
            CameraReprojectionError reprojErr(mCamera.parameters(), feature2D->keypoint().pt.x, feature2D->keypoint().pt.y);

            double residuals[2];
            reprojErr(frame->camera()->rotationData(),
                      frame->camera()->translationData(),
                      feature3D->pointData(), residuals);

            error = hypot(residuals[0], residuals[1]);
        }
        else
        {
            OdometerReprojectionError reprojErr(mCamera.parameters(), feature2D->keypoint().pt.x, feature2D->keypoint().pt.y);

            double residuals[2];
            reprojErr(m_T_cam_odo.rotationData(),
                      m_T_cam_odo.translationData(),
                      frame->odometer()->positionData(),
                      frame->odometer()->yawData(),
                      feature3D->pointData(), residuals);

            error = hypot(residuals[0], residuals[1]);
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

        return;
    }

    avgError = totalError / count;
}

void
SlidingWindowBA::windowReprojectionError(double& minError, double& maxError, double& avgError) const
{
    minError = std::numeric_limits<double>::max();
    maxError = std::numeric_limits<double>::min();

    size_t count = 0;
    double totalError = 0.0;

    for (std::list<FramePtr>::const_iterator it = mWindow.begin(); it != mWindow.end(); ++it)
    {
        const FrameConstPtr& frame = *it;

        const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

        for (size_t i = 0; i < features2D.size(); ++i)
        {
            const Point2DFeatureConstPtr& feature2D = features2D.at(i);
            const Point3DFeatureConstPtr& feature3D = feature2D->feature3D();

            if (feature3D.get() == 0)
            {
                continue;
            }

            double error;
            if (mMode == VO)
            {
                CameraReprojectionError reprojErr(mCamera.parameters(), feature2D->keypoint().pt.x, feature2D->keypoint().pt.y);

                double residuals[2];
                reprojErr(frame->camera()->rotationData(),
                          frame->camera()->translationData(),
                          feature3D->pointData(), residuals);

                error = hypot(residuals[0], residuals[1]);
            }
            else
            {
                OdometerReprojectionError reprojErr(mCamera.parameters(), feature2D->keypoint().pt.x, feature2D->keypoint().pt.y);

                double residuals[2];
                reprojErr(m_T_cam_odo.rotationData(),
                          m_T_cam_odo.translationData(),
                          frame->odometer()->positionData(),
                          frame->odometer()->yawData(),
                          feature3D->pointData(), residuals);

                error = hypot(residuals[0], residuals[1]);
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
    }

    if (count == 0)
    {
        avgError = 0.0;
        minError = 0.0;
        maxError = 0.0;

        return;
    }

    avgError = totalError / count;
}

void
SlidingWindowBA::findFeatureCorrespondences(const std::vector<Point2DFeaturePtr>& features,
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
            if (pt[j]->prevMatches().empty() || pt[j]->bestPrevMatchIdx() == -1)
            {
                foundCorrespondences = false;
                break;
            }

            pt[j - 1] = pt[j]->prevMatch();
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

bool
SlidingWindowBA::project3DPoint(const Eigen::Quaterniond& q, const Eigen::Vector3d& t,
                                const Eigen::Vector4d& src, Eigen::Vector3d& dst) const
{
    // transform point from world frame to camera frame
    Eigen::Vector3d P = q.toRotationMatrix() * (src.block<3,1>(0,0) / src(3)) + t;

    // check if point is behind camera
    if (P(2) < 0.0)
    {
        return false;
    }

    mCamera.space2plane(P(0), P(1), P(2), &dst(0), &dst(1));
    dst(2) = 1.0;

    return true;
}

void
SlidingWindowBA::rectifyImagePoint(const cv::Point2f& src, cv::Point2f& dst, double focal) const
{
    Eigen::Vector3d P;

    mCamera.liftProjective(src.x, src.y, &P(0), &P(1), &P(2));

    P /= P(2);

    dst.x = P(0) * focal + mCamera.parameters().imageWidth() / 2.0f;
    dst.y = P(1) * focal + mCamera.parameters().imageHeight() / 2.0f;
}

void
SlidingWindowBA::rectifyImagePoint(const Eigen::Vector2d& src, Eigen::Vector2d& dst, double focal) const
{
    Eigen::Vector3d P;

    mCamera.liftProjective(src(0), src(1), &P(0), &P(1), &P(2));

    P /= P(2);

    Eigen::Vector2d pp(mCamera.parameters().imageWidth() / 2.0, mCamera.parameters().imageHeight() / 2.0);
    dst = P.block<2,1>(0,0) * focal + pp;
}

void
SlidingWindowBA::rectifyImagePoints(const std::vector<cv::Point2f>& src,
                                    std::vector<cv::Point2f>& dst,
                                    double focal) const
{
    dst.resize(src.size());

    cv::Point2f pp(mCamera.parameters().imageWidth() / 2.0f, mCamera.parameters().imageHeight() / 2.0f);
    for (size_t i = 0; i < src.size(); ++i)
    {
        const cv::Point2f& p = src.at(i);

        Eigen::Vector3d P;
        mCamera.liftProjective(p.x, p.y, &P(0), &P(1), &P(2));

        P /= P(2);

        dst.at(i) = cv::Point2f(P(0), P(1)) * focal + pp;
    }
}

void
SlidingWindowBA::triangulatePoints(const Eigen::Quaterniond& q1,
                                   const Eigen::Vector3d& t1,
                                   const std::vector<cv::Point2f>& imagePoints1,
                                   const Eigen::Quaterniond& q2,
                                   const Eigen::Vector3d& t2,
                                   const std::vector<cv::Point2f>& imagePoints2,
                                   std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points3D,
                                   std::vector<size_t>& inliers) const
{
    Eigen::Matrix3d K;
    K << kNominalFocalLength, 0.0, mCamera.parameters().imageWidth() / 2.0,
         0.0, kNominalFocalLength, mCamera.parameters().imageHeight() / 2.0,
         0.0, 0.0, 1.0;

    Eigen::Matrix4d T1 = homogeneousTransform(q1.toRotationMatrix(), t1);
    Eigen::Matrix<double, 3, 4> P1 = K * T1.block<3,4>(0,0);

    Eigen::Matrix4d T2 = homogeneousTransform(q2.toRotationMatrix(), t2);
    Eigen::Matrix<double, 3, 4> P2 = K * T2.block<3,4>(0,0);

    Eigen::Matrix4d H1 = homogeneousTransform(q1.toRotationMatrix(), t1);
    Eigen::Matrix4d H2 = homogeneousTransform(q2.toRotationMatrix(), t2);
    Eigen::Matrix4d H12 = H2 * H1.inverse();

    Eigen::Matrix3d E = skew(Eigen::Vector3d(H12.block<3,1>(0,3))) * H12.block<3,3>(0,0);

    double focal = K(0,0);
    Eigen::Vector2d pp(K(0,2), K(1,2));

    // linear triangulation
    for (size_t i = 0; i < imagePoints1.size(); ++i)
    {
        const cv::Point2f& p1_cv = imagePoints1.at(i);
        const cv::Point2f& p2_cv = imagePoints2.at(i);

        cv::Point2f rect_p1_cv, rect_p2_cv;
        rectifyImagePoint(p1_cv, rect_p1_cv, kNominalFocalLength);
        rectifyImagePoint(p2_cv, rect_p2_cv, kNominalFocalLength);

        Eigen::Matrix4d J;
        J.row(0) = P1.row(2) * rect_p1_cv.x - P1.row(0);
        J.row(1) = P1.row(2) * rect_p1_cv.y - P1.row(1);
        J.row(2) = P2.row(2) * rect_p2_cv.x - P2.row(0);
        J.row(3) = P2.row(2) * rect_p2_cv.y - P2.row(1);

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Vector4d P = svd.matrixV().block<4,1>(0,3);

        P /= P(3);

        // validate scene point
        Eigen::Vector3d p1, p2;
        if (!project3DPoint(q1, t1, P, p1))
        {
            continue;
        }
        if (!project3DPoint(q2, t2, P, p2))
        {
            continue;
        }

        if (mMode == VO)
        {
            if ((p1 - Eigen::Vector3d(p1_cv.x, p1_cv.y, 1.0)).norm() > kTVTReprojErrorThresh)
            {
                continue;
            }

            if ((p2 - Eigen::Vector3d(p2_cv.x, p2_cv.y, 1.0)).norm() > kTVTReprojErrorThresh)
            {
                continue;
            }

            if ((p1 - p2).norm() < kMinDisparity)
            {
                continue;
            }
        }

        points3D.push_back(P.block<3,1>(0,0));
        inliers.push_back(i);
    }
}

void
SlidingWindowBA::tvt(const Eigen::Quaterniond& q1,
                     const Eigen::Vector3d& t1,
                     const std::vector<cv::Point2f>& imagePoints1,
                     const Eigen::Quaterniond& q2,
                     const Eigen::Vector3d& t2,
                     const std::vector<cv::Point2f>& imagePoints2,
                     const Eigen::Quaterniond& q3,
                     const Eigen::Vector3d& t3,
                     const std::vector<cv::Point2f>& imagePoints3,
                     std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points3D,
                     std::vector<size_t>& inliers) const
{
    Eigen::Matrix3d K;
    K << kNominalFocalLength, 0.0, mCamera.parameters().imageWidth() / 2.0,
         0.0, kNominalFocalLength, mCamera.parameters().imageHeight() / 2.0,
         0.0, 0.0, 1.0;

    Eigen::Matrix4d H1 = homogeneousTransform(q1.toRotationMatrix(), t1);
    Eigen::Matrix4d H2 = homogeneousTransform(q2.toRotationMatrix(), t2);
    Eigen::Matrix4d H3 = homogeneousTransform(q3.toRotationMatrix(), t3);

    Eigen::Matrix<double, 3, 4> P1 = K * H1.block<3,4>(0,0);
    Eigen::Matrix<double, 3, 4> P2 = K * H2.block<3,4>(0,0);
    Eigen::Matrix<double, 3, 4> P3 = K * H3.block<3,4>(0,0);

    Eigen::Matrix4d H12 = H2 * H1.inverse();
    Eigen::Matrix4d H23 = H3 * H2.inverse();
    Eigen::Matrix4d H13 = H3 * H1.inverse();

    Eigen::Matrix3d E12 = skew(Eigen::Vector3d(H12.block<3,1>(0,3))) * H12.block<3,3>(0,0);
    Eigen::Matrix3d E23 = skew(Eigen::Vector3d(H23.block<3,1>(0,3))) * H23.block<3,3>(0,0);
    Eigen::Matrix3d E13 = skew(Eigen::Vector3d(H13.block<3,1>(0,3))) * H13.block<3,3>(0,0);

    double focal = K(0,0);
    Eigen::Vector2d pp(K(0,2), K(1,2));

    // linear triangulation
    for (size_t i = 0; i < imagePoints1.size(); ++i)
    {
        const cv::Point2f& p1_cv = imagePoints1.at(i);
        const cv::Point2f& p2_cv = imagePoints2.at(i);
        const cv::Point2f& p3_cv = imagePoints3.at(i);

        cv::Point2f rect_p1_cv, rect_p2_cv, rect_p3_cv;
        rectifyImagePoint(p1_cv, rect_p1_cv, kNominalFocalLength);
        rectifyImagePoint(p2_cv, rect_p2_cv, kNominalFocalLength);
        rectifyImagePoint(p3_cv, rect_p3_cv, kNominalFocalLength);

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
        if (!project3DPoint(q1, t1, scenePoint, p1))
        {
            continue;
        }
        if (!project3DPoint(q2, t2, scenePoint, p2))
        {
            continue;
        }
        if (!project3DPoint(q3, t3, scenePoint, p3))
        {
            continue;
        }

        if (mMode == VO)
        {
            if ((p1 - Eigen::Vector3d(p1_cv.x, p1_cv.y, 1.0)).norm() > kTVTReprojErrorThresh)
            {
                continue;
            }

            if ((p2 - Eigen::Vector3d(p2_cv.x, p2_cv.y, 1.0)).norm() > kTVTReprojErrorThresh)
            {
                continue;
            }

            if ((p3 - Eigen::Vector3d(p3_cv.x, p3_cv.y, 1.0)).norm() > kTVTReprojErrorThresh)
            {
                continue;
            }

            if ((p2 - p3).norm() < kMinDisparity)
            {
                continue;
            }
        }

        points3D.push_back(scenePoint.block<3,1>(0,0));
        inliers.push_back(i);
    }
}

void
SlidingWindowBA::optimize(void)
{
    ceres::Problem problem;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
//    options.gradient_tolerance = 1e-16;
//    options.function_tolerance = 1e-16;
    options.max_num_iterations = 20;

//    ceres::ParameterBlockOrdering* ordering = new ceres::ParameterBlockOrdering;
//    options.linear_solver_ordering = ordering;

    for (std::list<FramePtr>::iterator it = mWindow.begin(); it != mWindow.end(); ++it)
    {
        FramePtr& frame = *it;

        std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

        bool optimizeFrame = false;
        for (size_t i = 0; i < features2D.size(); ++i)
        {
            Point2DFeaturePtr& feature2D = features2D.at(i);

            if (feature2D->feature3D().get() == 0)
            {
                continue;
            }

            ceres::CostFunction* costFunction;
            ceres::LossFunction* lossFunction = new ceres::CauchyLoss(1.0);

            if (mMode == VO)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<CameraReprojectionError, 2, 4, 3, 3>(
                        new CameraReprojectionError(mCamera.parameters(), feature2D->keypoint().pt.x, feature2D->keypoint().pt.y));

                problem.AddResidualBlock(costFunction, lossFunction,
                                         frame->camera()->rotationData(), frame->camera()->translationData(),
                                         feature2D->feature3D()->pointData());
            }
            else
            {
                ceres::CostFunction* costFunction
                    = new ceres::AutoDiffCostFunction<OdometerReprojectionError, 2, 4, 3, 2, 1, 3>(
                        new OdometerReprojectionError(mCamera.parameters(),
                                                      feature2D->keypoint().pt.x, feature2D->keypoint().pt.y));

                problem.AddResidualBlock(costFunction, lossFunction,
                                         m_T_cam_odo.rotationData(),
                                         m_T_cam_odo.translationData(),
                                         frame->odometer()->positionData(),
                                         frame->odometer()->yawData(),
                                         feature2D->feature3D()->pointData());
            }

//            ordering->AddElementToGroup(feature2D->feature3D()->pointData(), 0);

            optimizeFrame = true;
        }

        if (optimizeFrame)
        {
            if (mMode == VO)
            {
                ceres::LocalParameterization* quaternionParameterization =
                    new ceres::QuaternionParameterization;

                problem.SetParameterization(frame->camera()->rotationData(), quaternionParameterization);

                // the points come before the cameras
//                ordering->AddElementToGroup(frame->camera()->rotationData(), 1);
//                ordering->AddElementToGroup(frame->camera()->translationData(), 1);
            }
            else
            {
                // the points come before the extrinsics
//                ordering->AddElementToGroup(frame->odometer()->positionData(), 1);
//                ordering->AddElementToGroup(frame->odometer()->yawData(), 1);

                problem.SetParameterBlockConstant(frame->odometer()->positionData());
                problem.SetParameterBlockConstant(frame->odometer()->yawData());
            }
        }
    }

    if (mMode == ODOMETER)
    {
        ceres::LocalParameterization* quaternionParameterization =
            new ceres::QuaternionParameterization;

        problem.SetParameterization(m_T_cam_odo.rotationData(), quaternionParameterization);

        // the points come before the extrinsics
//        ordering->AddElementToGroup(m_T_cam_odo.rotationData(), 1);
//        ordering->AddElementToGroup(m_T_cam_odo.translationData(), 1);
    }

    if (mWindow.size() > m_N - m_n)
    {
        std::list<FramePtr>::iterator it = mWindow.begin();
        for (int i = 0; i < m_N - m_n; ++i)
        {
            FramePtr& frame = *it;

            if (mMode == VO)
            {
                problem.SetParameterBlockConstant(frame->camera()->rotationData());
                problem.SetParameterBlockConstant(frame->camera()->translationData());
            }

            ++it;
        }

        if (mVerbose)
        {
            std::cout << "# INFO: Setting first " << m_N - m_n << " frames' parameters fixed and optimizing next " << mWindow.size() - m_N + m_n << " frames' parameters." << std::endl;
        }
    }
    else
    {
        std::list<FramePtr>::iterator it = mWindow.begin();
        for (int i = 0; i < 1; ++i)
        {
            FramePtr& frame = *it;

            if (mMode == VO)
            {
                // set constant camera pose corresponding to first frame in the window
                problem.SetParameterBlockConstant(frame->camera()->rotationData());
                problem.SetParameterBlockConstant(frame->camera()->translationData());
            }

            ++it;
        }

        if (mVerbose)
        {
            std::cout << "# INFO: Setting first frame's parameters fixed and optimizing all other parameters." << std::endl;
        }
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (mVerbose)
    {
        std::cout << summary.BriefReport() << std::endl;
    }
}

}
