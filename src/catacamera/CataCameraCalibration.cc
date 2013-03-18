#include "CataCameraCalibration.h"

#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "../ceres-solver/include/ceres/ceres.h"
#include "../gpl/EigenUtils.h"
#include "CataCamera.h"
#include "CataReprojectionError.h"

#include <cstdio>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <fstream>

namespace camodocal
{

CataCameraCalibration::CataCameraCalibration(const std::string& cameraName,
                                             const cv::Size& imageSize)
 : mVerbose(false)
{
    mCameraParams.cameraName() = cameraName;
    mCameraParams.imageWidth() = imageSize.width;
    mCameraParams.imageHeight() = imageSize.height;
}

void
CataCameraCalibration::clear(void)
{
    mImagePoints.clear();
}

void
CataCameraCalibration::addChessboardData(const std::vector<cv::Point2f>& corners)
{
    mImagePoints.push_back(corners);
}

bool
CataCameraCalibration::calibrate(const cv::Size& boardSize, float squareSize)
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

    double startTime = timeInSeconds();

    // compute intrinsic camera parameters and extrinsic parameters for each of the views
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    bool ret = calibrateHelper(boardSize, objectPoints, mImagePoints,
                               mCameraParams, rvecs, tvecs);

    if (mVerbose)
    {
        std::cout << "# INFO: Calibration took a total time of "
                  << std::fixed << std::setprecision(3) << timeInSeconds() - startTime
                  << " sec.\n";
    }

    mExtrParams = cv::Mat(imageCount, 6, CV_64F);
    for (int i = 0; i < imageCount; ++i)
    {
        mExtrParams.at<double>(i,0) = rvecs.at(i).at<double>(0);
        mExtrParams.at<double>(i,1) = rvecs.at(i).at<double>(1);
        mExtrParams.at<double>(i,2) = rvecs.at(i).at<double>(2);
        mExtrParams.at<double>(i,3) = tvecs.at(i).at<double>(0);
        mExtrParams.at<double>(i,4) = tvecs.at(i).at<double>(1);
        mExtrParams.at<double>(i,5) = tvecs.at(i).at<double>(2);
    }

    // calculate average reprojection error
    mAvgReprojErr = computeReprojectionError(objectPoints, mImagePoints,
                                             mCameraParams,
                                             rvecs, tvecs, mReprojErrs);

    return ret;
}

int
CataCameraCalibration::sampleCount(void) const
{
    return mImagePoints.size();
}

const std::vector< std::vector<cv::Point2f> >&
CataCameraCalibration::imagePoints(void) const
{
    return mImagePoints;
}

CataCamera::Parameters&
CataCameraCalibration::cameraParameters(void)
{
    return mCameraParams;
}

const CataCamera::Parameters&
CataCameraCalibration::cameraParameters(void) const
{
    return mCameraParams;
}

void
CataCameraCalibration::writeParams(const std::string& filename) const
{
    mCameraParams.write(filename);
}

void
CataCameraCalibration::setVerbose(bool verbose)
{
    mVerbose = verbose;
}

bool
CataCameraCalibration::calibrateHelper(const cv::Size& boardSize,
                                       const std::vector< std::vector<cv::Point3f> >& objectPoints,
                                       const std::vector< std::vector<cv::Point2f> >& imagePoints,
                                       CataCamera::Parameters& cameraParams,
                                       std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const
{
    double startTime = timeInSeconds();

    double u0 = mCameraParams.imageWidth() / 2.0;
    double v0 = mCameraParams.imageHeight() / 2.0;

    double gamma0 = 0.0;
    float minReprojErr = std::numeric_limits<float>::max();
    rvecs.assign(objectPoints.size(), cv::Mat());
    tvecs.assign(objectPoints.size(), cv::Mat());

    // STEP 1: Initialize gamma (focal length)
    //         Use non-radial line image and xi = 1
    for (size_t i = 0; i < imagePoints.size(); ++i)
    {
        for (int r = 0; r < boardSize.height; ++r)
        {
            cv::Mat P(boardSize.width, 4, CV_64F);
            for (int c = 0; c < boardSize.width; ++c)
            {
                const cv::Point2f& imagePoint = imagePoints.at(i).at(r * boardSize.width + c);

                double u = imagePoint.x - u0;
                double v = imagePoint.y - v0;

                P.at<double>(c, 0) = u;
                P.at<double>(c, 1) = v;
                P.at<double>(c, 2) = 0.5;
                P.at<double>(c, 3) = -0.5 * (square(u) + square(v));
            }

            cv::Mat C;
            cv::SVD::solveZ(P, C);

            double t = square(C.at<double>(0)) + square(C.at<double>(1)) + C.at<double>(2) * C.at<double>(3);
            if (t < 0)
            {
                continue;
            }

            // check that line image is not radial
            double d = sqrt(1.0 / t);
            double nx = C.at<double>(0) * d;
            double ny = C.at<double>(1) * d;
            if (hypot(nx, ny) > 0.95)
            {
                continue;
            }

            double nz = sqrt(1.0 - square(nx) - square(ny));
            double gamma = fabs(C.at<double>(2) * d / nz);

            CataCamera::Parameters params("",
                                          mCameraParams.imageWidth(), mCameraParams.imageHeight(),
                                          1.0, 0.0, 0.0, 0.0, 0.0,
                                          gamma, gamma, u0, v0);
            for (size_t i = 0; i < objectPoints.size(); ++i)
            {
                estimateExtrinsic(params, objectPoints.at(i), imagePoints.at(i), rvecs.at(i), tvecs.at(i));
            }

            cv::Mat pve;
            float reprojErr = computeReprojectionError(objectPoints, imagePoints, params, rvecs, tvecs, pve);

            if (reprojErr < minReprojErr)
            {
                minReprojErr = reprojErr;
                gamma0 = gamma;
            }
        }
    }

    if (gamma0 <= 0.0 && minReprojErr >= std::numeric_limits<float>::max())
    {
        std::cout << "[" << mCameraParams.cameraName() << "] "
                  << "# INFO: CataCamera model fails with given data. " << std::endl;
        return false;
    }

    // STEP 2: Initialize extrinsic matrices
    //         Use the initial gamma0, u0, v0 and xi = 1
    cameraParams.imageWidth() = mCameraParams.imageWidth();
    cameraParams.imageHeight() = mCameraParams.imageHeight();
    cameraParams.cameraName() = mCameraParams.cameraName();
    cameraParams.xi() = 1.0;
    cameraParams.k1() = 0.0;
    cameraParams.k2() = 0.0;
    cameraParams.p1() = 0.0;
    cameraParams.p2() = 0.0;
    cameraParams.gamma1() = gamma0;
    cameraParams.gamma2() = gamma0;
    cameraParams.u0() = u0;
    cameraParams.v0() = v0;

    for (size_t i = 0; i < objectPoints.size(); ++i)
    {
        estimateExtrinsic(cameraParams, objectPoints.at(i), imagePoints.at(i), rvecs.at(i), tvecs.at(i));
    }

    if (mVerbose)
    {
        std::cout << "# INFO: --- Estimation of initial parameters: "
                  << std::fixed << std::setprecision(3) << timeInSeconds() - startTime << " sec\n";
    }

    cv::Mat pve;
    std::cout << "[" << cameraParams.cameraName() << "] " << "# INFO: " << "Initial reprojection error: "
              << std::fixed << std::setprecision(3)
              << computeReprojectionError(objectPoints, imagePoints, cameraParams, rvecs, tvecs, pve)
              << " pixels" << std::endl;

    startTime = timeInSeconds();

    // STEP 3: optimization using ceres
    optimize(objectPoints, imagePoints, cameraParams, rvecs, tvecs);

    if (mVerbose)
    {
        std::cout << "# INFO: --- Optimization of initial parameters: "
                  << std::fixed << std::setprecision(3) << timeInSeconds() - startTime << " sec\n";

        cv::Mat pve;
        float err = computeReprojectionError(objectPoints, imagePoints, cameraParams, rvecs, tvecs, pve);
        std::cout << "[" << cameraParams.cameraName() << "] " << "# INFO: Final reprojection error with parabola init: "
                  << err << " pixels" << std::endl;
        std::cout << "[" << cameraParams.cameraName() << "] " << "# INFO: Calibration Parameters:" << std::endl
                  << cameraParams << std::endl;

        std::cout << "# INFO: Calibration with parabola model init took "
                  << std::fixed << std::setprecision(3) << timeInSeconds() - startTime << " sec.\n";
    }

    return true;
}

void
CataCameraCalibration::estimateExtrinsic(const CataCamera::Parameters& cameraParams,
                                         const std::vector<cv::Point3f>& objectPoints,
                                         const std::vector<cv::Point2f>& imagePoints,
                                         cv::Mat& rvec, cv::Mat& tvec) const
{
    std::vector<cv::Point2f> Ms(imagePoints.size());
    for (size_t i = 0; i < Ms.size(); ++i)
    {
        double u = (imagePoints.at(i).x - cameraParams.u0()) / cameraParams.gamma1();
        double v = (imagePoints.at(i).y - cameraParams.v0()) / cameraParams.gamma2();
        double len2 = square(u) + square(v);
        double xi = cameraParams.xi();
        double temp = xi + sqrt(1.0 + (1.0 - square(xi)) * len2);
        temp /= len2 + 1.0;
        double x = temp * u;
        double y = temp * v;
        double z = temp - xi;
        Ms.at(i).x = x / z;
        Ms.at(i).y = y / z;
    }

    std::vector<double> distCoeffs;
    distCoeffs.push_back(cameraParams.k1());
    distCoeffs.push_back(cameraParams.k2());
    distCoeffs.push_back(cameraParams.p1());
    distCoeffs.push_back(cameraParams.p2());

    cv::solvePnP(objectPoints, Ms, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);
}

void
CataCameraCalibration::optimize(const std::vector< std::vector<cv::Point3f> >& objectPoints,
                                const std::vector< std::vector<cv::Point2f> >& imagePoints,
                                CataCamera::Parameters& cameraParams,
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

    double intrinsicCameraParams[9];
    intrinsicCameraParams[0] = cameraParams.xi();
    intrinsicCameraParams[1] = cameraParams.k1();
    intrinsicCameraParams[2] = cameraParams.k2();
    intrinsicCameraParams[3] = cameraParams.p1();
    intrinsicCameraParams[4] = cameraParams.p2();
    intrinsicCameraParams[5] = cameraParams.gamma1();
    intrinsicCameraParams[6] = cameraParams.gamma2();
    intrinsicCameraParams[7] = cameraParams.u0();
    intrinsicCameraParams[8] = cameraParams.v0();

    // create residuals for each observation
    for (size_t i = 0; i < imagePoints.size(); ++i)
    {
        for (size_t j = 0; j < imagePoints.at(i).size(); ++j)
        {
            const cv::Point3f& opt = objectPoints.at(i).at(j);
            const cv::Point2f& ipt = imagePoints.at(i).at(j);

            ceres::CostFunction* costFunction =
                new CataReprojectionError(opt.x, opt.y, opt.z, ipt.x, ipt.y);

            ceres::LossFunction* lossFunction = new ceres::CauchyLoss(1.0);
            problem.AddResidualBlock(costFunction, lossFunction,
                                     intrinsicCameraParams,
                                     extrinsicCameraParams[i],
                                     extrinsicCameraParams[i] + 4);
        }
    }

    for (size_t i = 0; i < imagePoints.size(); ++i)
    {
        ceres::LocalParameterization* quaternionParameterization =
            new ceres::QuaternionParameterization;

        problem.SetParameterization(extrinsicCameraParams[i],
                                    quaternionParameterization);
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 100;

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

    cameraParams.xi() = intrinsicCameraParams[0];
    cameraParams.k1() = intrinsicCameraParams[1];
    cameraParams.k2() = intrinsicCameraParams[2];
    cameraParams.p1() = intrinsicCameraParams[3];
    cameraParams.p2() = intrinsicCameraParams[4];
    cameraParams.gamma1() = intrinsicCameraParams[5];
    cameraParams.gamma2() = intrinsicCameraParams[6];
    cameraParams.u0() = intrinsicCameraParams[7];
    cameraParams.v0() = intrinsicCameraParams[8];

    for (size_t i = 0; i < rvecs.size(); ++i)
    {
        Eigen::Vector3d rvec;
        QuaternionToAngleAxis(extrinsicCameraParams[i], rvec);
        cv::eigen2cv(rvec, rvecs.at(i));

        cv::Mat& tvec = tvecs.at(i);
        tvec.at<double>(0) = extrinsicCameraParams[i][4];
        tvec.at<double>(1) = extrinsicCameraParams[i][5];
        tvec.at<double>(2) = extrinsicCameraParams[i][6];
    }
}

void
CataCameraCalibration::projectPoints(const std::vector<cv::Point3f>& objectPoints,
                                     const cv::Mat& rvec,
                                     const cv::Mat& tvec,
                                     const CataCamera::Parameters& cameraParams,
                                     std::vector<cv::Point2f>& imagePoints) const
{
    // project 3D object points to the image plane
    imagePoints.reserve(objectPoints.size());
    double xi = cameraParams.xi();
    double k1 = cameraParams.k1();
    double k2 = cameraParams.k2();
    double p1 = cameraParams.p1(); // i.e. k3
    double p2 = cameraParams.p2(); // i.e. k4
    double gamma1 = cameraParams.gamma1();
    double gamma2 = cameraParams.gamma2();
    double alpha = 0; //cameraParams.alpha();
    double u0 = cameraParams.u0();
    double v0 = cameraParams.v0();

    //double
    cv::Mat R0;
    cv::Rodrigues(rvec, R0);

    Eigen::MatrixXd R(3,3);
    R << R0.at<double>(0,0), R0.at<double>(0,1), R0.at<double>(0,2),
         R0.at<double>(1,0), R0.at<double>(1,1), R0.at<double>(1,2),
         R0.at<double>(2,0), R0.at<double>(2,1), R0.at<double>(2,2);

    Eigen::Vector3d t;
    t << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);

    for (size_t i = 0; i < objectPoints.size(); ++i)
    {
        const cv::Point3f& objectPoint = objectPoints.at(i);

        // Rotate and translate
        Eigen::Vector3d X;
        X << objectPoint.x, objectPoint.y, objectPoint.z;

        X = R * X + t;

        // Transform to model plane
        X.normalize();
        double u = X(0) / (X(2) + xi);
        double v = X(1) / (X(2) + xi);

        double rho_sqr = square(u) + square(v);
        double L = 1.0 + k1 * rho_sqr + k2 * square(rho_sqr);
        double du = 2.0 * p1 * u * v + p2 * (rho_sqr + 2.0 * square(u));
        double dv = p1 * (rho_sqr + 2.0 * square(v)) + 2.0 * p2 * u * v;

        u = L * u + du;
        v = L * v + dv;
        double x = gamma1 * (u + alpha * v) + u0;
        double y = gamma2 * v + v0;

        imagePoints.push_back(cv::Point2f(x, y));
    }
}

float
CataCameraCalibration::computeReprojectionError(const std::vector< std::vector<cv::Point3f> >& objectPoints,
                                                const std::vector< std::vector<cv::Point2f> >& imagePoints,
                                                const CataCamera::Parameters& cameraParams,
                                                const std::vector<cv::Mat>& rvecs,
                                                const std::vector<cv::Mat>& tvecs,
                                                cv::Mat& perViewErrors) const
{
    // Reprojection error changed to L2-norm because I am lazy.
    int imageCount = objectPoints.size();
    size_t pointsSoFar = 0;
    float totalErr = 0.0;

    perViewErrors = cv::Mat(1, imageCount, CV_32F);

    for (int i = 0; i < imageCount; ++i)
    {
        size_t pointCount = imagePoints.at(i).size();

        pointsSoFar += pointCount;

        std::vector<cv::Point2f> estImagePoints;
        projectPoints(objectPoints.at(i), rvecs.at(i), tvecs.at(i),
                      cameraParams, estImagePoints);

        float err = 0.0;
        for (size_t j = 0; j < imagePoints.at(i).size(); ++j)
        {
            err += fabsf(imagePoints.at(i).at(j).x - estImagePoints.at(j).x);
            err += fabsf(imagePoints.at(i).at(j).y - estImagePoints.at(j).y);
        }

        perViewErrors.at<float>(i) = err / pointCount;

        totalErr += err;
    }

    return totalErr / pointsSoFar;
}

}
