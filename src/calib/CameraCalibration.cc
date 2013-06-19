#include "camodocal/calib/CameraCalibration.h"

#include <cstdio>
#include <Eigen/Eigen>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "ceres/ceres.h"
#include "../gpl/EigenQuaternionParameterization.h"
#include "../gpl/EigenUtils.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "../camera_models/CostFunctionFactory.h"

namespace camodocal
{

CameraCalibration::CameraCalibration(const Camera::ModelType modelType,
                                     const std::string& cameraName,
                                     const cv::Size& imageSize)
 : mVerbose(false)
{
    mCamera = CameraFactory::instance()->generateCamera(modelType, cameraName, imageSize);
}

void
CameraCalibration::clear(void)
{
    mImagePoints.clear();
}

void
CameraCalibration::addChessboardData(const std::vector<cv::Point2f>& corners)
{
    mImagePoints.push_back(corners);
}

bool
CameraCalibration::calibrate(const cv::Size& boardSize, float squareSize)
{
    int imageCount = mImagePoints.size();

    std::vector< std::vector<cv::Point3f> > objectPoints;
    for (int i = 0; i < imageCount; ++i)
    {
        std::vector<cv::Point3f> objectPointsInView;
        for (int j = 0; j < boardSize.height; ++j)
        {
            for (int k = 0; k < boardSize.width; ++k)
            {
                objectPointsInView.push_back(cv::Point3f(j * squareSize, k * squareSize, 0.0));
            }
        }
        objectPoints.push_back(objectPointsInView);
    }

    // compute intrinsic camera parameters and extrinsic parameters for each of the views
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    bool ret = calibrateHelper(boardSize, objectPoints, mImagePoints,
                               mCamera, rvecs, tvecs);

    mCameraPoses = cv::Mat(imageCount, 6, CV_64F);
    for (int i = 0; i < imageCount; ++i)
    {
        mCameraPoses.at<double>(i,0) = rvecs.at(i).at<double>(0);
        mCameraPoses.at<double>(i,1) = rvecs.at(i).at<double>(1);
        mCameraPoses.at<double>(i,2) = rvecs.at(i).at<double>(2);
        mCameraPoses.at<double>(i,3) = tvecs.at(i).at<double>(0);
        mCameraPoses.at<double>(i,4) = tvecs.at(i).at<double>(1);
        mCameraPoses.at<double>(i,5) = tvecs.at(i).at<double>(2);
    }

    // calculate average reprojection error
    mAvgReprojErr = mCamera->reprojectionError(objectPoints, mImagePoints,
                                               rvecs, tvecs, mReprojErrs);

    return ret;
}

int
CameraCalibration::sampleCount(void) const
{
    return mImagePoints.size();
}

std::vector< std::vector<cv::Point2f> >&
CameraCalibration::imagePoints(void)
{
    return mImagePoints;
}

const std::vector< std::vector<cv::Point2f> >&
CameraCalibration::imagePoints(void) const
{
    return mImagePoints;
}

CameraPtr&
CameraCalibration::camera(void)
{
    return mCamera;
}

const CameraConstPtr
CameraCalibration::camera(void) const
{
    return mCamera;
}

cv::Mat&
CameraCalibration::cameraPoses(void)
{
    return mCameraPoses;
}

const cv::Mat&
CameraCalibration::cameraPoses(void) const
{
    return mCameraPoses;
}

void
CameraCalibration::drawResults(const cv::Size& boardSize, float squareSize,
                               std::vector<cv::Mat>& images) const
{
    std::vector< std::vector<cv::Point3f> > objectPoints;
    for (size_t i = 0; i < images.size(); ++i)
    {
        std::vector<cv::Point3f> objectPointsInView;
        for (int j = 0; j < boardSize.height; ++j)
        {
            for (int k = 0; k < boardSize.width; ++k)
            {
                objectPointsInView.push_back(cv::Point3f(j * squareSize, k * squareSize, 0.0));
            }
        }
        objectPoints.push_back(objectPointsInView);
    }

    std::vector<cv::Mat> rvecs, tvecs;

    for (size_t i = 0; i < images.size(); ++i)
    {
        cv::Mat rvec(3, 1, CV_64F);
        rvec.at<double>(0) = mCameraPoses.at<double>(i,0);
        rvec.at<double>(1) = mCameraPoses.at<double>(i,1);
        rvec.at<double>(2) = mCameraPoses.at<double>(i,2);

        cv::Mat tvec(3, 1, CV_64F);
        tvec.at<double>(0) = mCameraPoses.at<double>(i,3);
        tvec.at<double>(1) = mCameraPoses.at<double>(i,4);
        tvec.at<double>(2) = mCameraPoses.at<double>(i,5);

        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
    }

    int drawShiftBits = 4;
    int drawMultiplier = 1 << drawShiftBits;

    cv::Scalar green(0, 255, 0);
    cv::Scalar red(0, 0, 255);

    for (size_t i = 0; i < images.size(); ++i)
    {
        cv::Mat& image = images.at(i);
        if (image.channels() == 1)
        {
            cv::cvtColor(image, image, CV_GRAY2RGB);
        }

        std::vector<cv::Point2f> estImagePoints;
        mCamera->projectPoints(objectPoints.at(i), rvecs.at(i), tvecs.at(i),
                               estImagePoints);

        float errorSum = 0.0f;
        float errorMax = std::numeric_limits<float>::min();

        for (size_t j = 0; j < mImagePoints.at(i).size(); ++j)
        {
            cv::Point2f pObs = mImagePoints.at(i).at(j);
            cv::Point2f pEst = estImagePoints.at(j);

            cv::circle(image,
                       cv::Point(cvRound(pObs.x * drawMultiplier),
                                 cvRound(pObs.y * drawMultiplier)),
                       5, green, 2, CV_AA, drawShiftBits);

            cv::circle(image,
                       cv::Point(cvRound(pEst.x * drawMultiplier),
                                 cvRound(pEst.y * drawMultiplier)),
                       5, red, 2, CV_AA, drawShiftBits);

            float error = cv::norm(pObs - pEst);

            errorSum += error;
            if (error > errorMax)
            {
                errorMax = error;
            }
        }

        std::ostringstream oss;
        oss << "Reprojection error: avg = " << errorSum / mImagePoints.at(i).size()
            << "   max = " << errorMax;

        cv::putText(image, oss.str(), cv::Point(10, image.rows - 10),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                    1, CV_AA);
    }
}

void
CameraCalibration::writeParams(const std::string& filename) const
{
    mCamera->writeParameters(filename);
}

void
CameraCalibration::setVerbose(bool verbose)
{
    mVerbose = verbose;
}

bool
CameraCalibration::calibrateHelper(const cv::Size& boardSize,
                                   const std::vector< std::vector<cv::Point3f> >& objectPoints,
                                   const std::vector< std::vector<cv::Point2f> >& imagePoints,
                                   CameraPtr& camera,
                                   std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const
{
    rvecs.assign(objectPoints.size(), cv::Mat());
    tvecs.assign(objectPoints.size(), cv::Mat());

    // STEP 1: Estimate intrinsics
    camera->estimateIntrinsics(boardSize, objectPoints, imagePoints);

    // STEP 2: Estimate extrinsics
    for (size_t i = 0; i < objectPoints.size(); ++i)
    {
        camera->estimateExtrinsics(objectPoints.at(i), imagePoints.at(i), rvecs.at(i), tvecs.at(i));
    }

    if (mVerbose)
    {
        std::cout << "[" << camera->cameraName() << "] "
                  << "# INFO: " << "Initial reprojection error: "
                  << std::fixed << std::setprecision(3)
                  << camera->reprojectionError(objectPoints, imagePoints, rvecs, tvecs)
                  << " pixels" << std::endl;
    }

    // STEP 3: optimization using ceres
    optimize(objectPoints, imagePoints, camera, rvecs, tvecs);

    if (mVerbose)
    {
        double err = camera->reprojectionError(objectPoints, imagePoints, rvecs, tvecs);
        std::cout << "[" << camera->cameraName() << "] " << "# INFO: Final reprojection error: "
                  << err << " pixels" << std::endl;
        std::cout << "[" << camera->cameraName() << "] " << "# INFO: "
                  << camera->parametersToString() << std::endl;
    }

    return true;
}

void
CameraCalibration::optimize(const std::vector< std::vector<cv::Point3f> >& objectPoints,
                            const std::vector< std::vector<cv::Point2f> >& imagePoints,
                            CameraPtr& camera,
                            std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const
{
    // Use ceres to do optimization
    ceres::Problem problem;

    double* extrinsicCameraParams[rvecs.size()];
    for (size_t i = 0; i < rvecs.size(); ++i)
    {
        extrinsicCameraParams[i] = new double[7];

        Eigen::Vector3d rvec;
        cv::cv2eigen(rvecs.at(i), rvec);

        AngleAxisToQuaternion(rvec, extrinsicCameraParams[i]);

        extrinsicCameraParams[i][4] = tvecs[i].at<double>(0);
        extrinsicCameraParams[i][5] = tvecs[i].at<double>(1);
        extrinsicCameraParams[i][6] = tvecs[i].at<double>(2);
    }

    std::vector<double> intrinsicCameraParams;
    mCamera->writeParameters(intrinsicCameraParams);

    // create residuals for each observation
    for (size_t i = 0; i < imagePoints.size(); ++i)
    {
        for (size_t j = 0; j < imagePoints.at(i).size(); ++j)
        {
            const cv::Point3f& opt = objectPoints.at(i).at(j);
            const cv::Point2f& ipt = imagePoints.at(i).at(j);

            ceres::CostFunction* costFunction =
                CostFunctionFactory::instance()->generateCostFunction(camera,
                                                                      Eigen::Vector3d(opt.x, opt.y, opt.z),
                                                                      Eigen::Vector2d(ipt.x, ipt.y),
                                                                      CAMERA_INTRINSICS | CAMERA_EXTRINSICS);

            ceres::LossFunction* lossFunction = new ceres::CauchyLoss(1.0);
            problem.AddResidualBlock(costFunction, lossFunction,
                                     intrinsicCameraParams.data(),
                                     extrinsicCameraParams[i],
                                     extrinsicCameraParams[i] + 4);
        }
    }

    for (size_t i = 0; i < imagePoints.size(); ++i)
    {
        ceres::LocalParameterization* quaternionParameterization =
            new EigenQuaternionParameterization;

        problem.SetParameterization(extrinsicCameraParams[i],
                                    quaternionParameterization);
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 1000;
    options.num_threads = 8;

    if (mVerbose)
    {
        options.minimizer_progress_to_stdout = true;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (mVerbose)
    {
        std::cout << summary.FullReport() << "\n";
    }

    camera->readParameters(intrinsicCameraParams);

    for (size_t i = 0; i < rvecs.size(); ++i)
    {
        Eigen::Vector3d rvec;
        QuaternionToAngleAxis(extrinsicCameraParams[i], rvec);
        cv::eigen2cv(rvec, rvecs.at(i));

        cv::Mat& tvec = tvecs.at(i);
        tvec.at<double>(0) = extrinsicCameraParams[i][4];
        tvec.at<double>(1) = extrinsicCameraParams[i][5];
        tvec.at<double>(2) = extrinsicCameraParams[i][6];

        delete [] extrinsicCameraParams[i];
    }
}

}
