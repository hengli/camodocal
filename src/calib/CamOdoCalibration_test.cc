#include <gtest/gtest.h>
#include <iostream>

#include "../gpl/gpl.h"
#include "camodocal/EigenUtils.h"
#include "camodocal/calib/CamOdoCalibration.h"

namespace camodocal
{

TEST(CamOdoCalibration, SetRandomSeed)
{
   // srand(time(0));
    srand(5);
}

TEST(CamOdoCalibration, MultiScale1)
{
    CamOdoCalibration calib;
    calib.setVerbose(false);

    Eigen::Matrix4d H_12_expected = Eigen::Matrix4d::Identity();
    H_12_expected.block<3,3>(0,0) = Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.1, 0.2, 0.3).normalized()).toRotationMatrix();
    H_12_expected.block<3,1>(0,3) << 0.5, 0.6, 0.7;

    std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > > rvecs1, tvecs1, rvecs2, tvecs2;

    int segmentCount = 5;
    rvecs1.resize(segmentCount);
    tvecs1.resize(segmentCount);
    rvecs2.resize(segmentCount);
    tvecs2.resize(segmentCount);

    std::vector<double> scales_expected;
    scales_expected.resize(segmentCount);

    for (int i = 0; i < segmentCount; ++i)
    {
        int motionCount = 2;

        for (int j = 0; j < motionCount; ++j)
        {
            double droll = d2r(random(-10.0, 10.0));
            droll = 0;
            double dpitch =  d2r(random(-10.0, 10.0));
            dpitch = 0;
            double dyaw =  d2r(random(-100.0, 100.0));
            double dx = random(-10.0, 10.0);
            double dy = random(-10.0, 10.0);
            double dz = random(-10.0, 10.0);
            dz = 0;

            Eigen::Matrix3d R;
            R = Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(dpitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(droll, Eigen::Vector3d::UnitX());

            Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
            H.block<3,3>(0,0) = R;
            H.block<3,1>(0,3) << dx, dy, dz;

            Eigen::Matrix4d H_ = H.inverse();
            H = H_;

            Eigen::Vector3d rvec1, tvec1, rvec2, tvec2;

            Eigen::AngleAxisd angleAxis1(H.block<3,3>(0,0));
            rvec1 = angleAxis1.angle() * angleAxis1.axis();

            tvec1 = H.block<3,1>(0,3);

            Eigen::AngleAxisd angleAxis2((H_12_expected.inverse() * H * H_12_expected).block<3,3>(0,0));
            rvec2 = angleAxis2.angle() * angleAxis2.axis();

            tvec2 = (H_12_expected.inverse() * H * H_12_expected).block<3,1>(0,3);

            if (j == 0)
            {
                scales_expected.at(i) = tvec2.norm();
                tvec2.normalize();
            }
            else
            {
                tvec2 /= scales_expected.at(i);
            }

            rvecs1.at(i).push_back(rvec1);
            tvecs1.at(i).push_back(tvec1);
            rvecs2.at(i).push_back(rvec2);
            tvecs2.at(i).push_back(tvec2);
        }
    }

    Eigen::Matrix4d H_12;
    std::vector<double> scales;

    EXPECT_TRUE(calib.estimate(rvecs1, tvecs1, rvecs2, tvecs2, H_12, scales));

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            if (i == 2 && j == 3)
            {
                continue;
            }
            EXPECT_NEAR(H_12_expected(i,j), H_12(i,j), 1e-10) << "Elements differ at (" << i << "," << j << ")";
        }
    }

    for (size_t i = 0; i < scales.size(); ++i)
    {
        EXPECT_NEAR(scales_expected.at(i), scales.at(i), 1e-10);
    }
}

TEST(CamOdoCalibration, MultiScale2)
{
    CamOdoCalibration calib;
    calib.setVerbose(false);

    Eigen::Matrix4d H_12_expected = Eigen::Matrix4d::Identity();
    H_12_expected.block<3,3>(0,0) = Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.1, 0.2, 0.3).normalized()).toRotationMatrix();
    H_12_expected.block<3,1>(0,3) << 0.5, 0.6, 0.7;

    std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > > rvecs1, tvecs1, rvecs2, tvecs2;

    int segmentCount = 5;
    rvecs1.resize(segmentCount);
    tvecs1.resize(segmentCount);
    rvecs2.resize(segmentCount);
    tvecs2.resize(segmentCount);

    std::vector<double> scales_expected;
    scales_expected.resize(segmentCount);

    for (int i = 0; i < segmentCount; ++i)
    {
        int motionCount = random(5, 10);

        for (int j = 0; j < motionCount; ++j)
        {
            double droll = d2r(random(-10.0, 10.0));
            droll = 0;
            double dpitch =  d2r(random(-10.0, 10.0));
            dpitch = 0;
            double dyaw =  d2r(random(-100.0, 100.0));
            double dx = random(-10.0, 10.0);
            double dy = random(-10.0, 10.0);
            double dz = random(-10.0, 10.0);
            dz = 0;

            Eigen::Matrix3d R;
            R = Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(dpitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(droll, Eigen::Vector3d::UnitX());

            Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
            H.block<3,3>(0,0) = R;
            H.block<3,1>(0,3) << dx, dy, dz;

            Eigen::Matrix4d H_ = H.inverse();
            H = H_;

            Eigen::Vector3d rvec1, tvec1, rvec2, tvec2;

            Eigen::AngleAxisd angleAxis1(H.block<3,3>(0,0));
            rvec1 = angleAxis1.angle() * angleAxis1.axis();

            tvec1 = H.block<3,1>(0,3);

            Eigen::AngleAxisd angleAxis2((H_12_expected.inverse() * H * H_12_expected).block<3,3>(0,0));
            rvec2 = angleAxis2.angle() * angleAxis2.axis();

            tvec2 = (H_12_expected.inverse() * H * H_12_expected).block<3,1>(0,3);

            if (j == 0)
            {
                scales_expected.at(i) = tvec2.norm();
                tvec2.normalize();
            }
            else
            {
                tvec2 /= scales_expected.at(i);
            }

            rvecs1.at(i).push_back(rvec1);
            tvecs1.at(i).push_back(tvec1);
            rvecs2.at(i).push_back(rvec2);
            tvecs2.at(i).push_back(tvec2);
        }
    }

    Eigen::Matrix4d H_12;
    std::vector<double> scales;

    EXPECT_TRUE(calib.estimate(rvecs1, tvecs1, rvecs2, tvecs2, H_12, scales));

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            if (i == 2 && j == 3)
            {
                continue;
            }
            EXPECT_NEAR(H_12_expected(i,j), H_12(i,j), 1e-10) << "Elements differ at (" << i << "," << j << ")";
        }
    }

    for (size_t i = 0; i < scales.size(); ++i)
    {
        EXPECT_NEAR(scales_expected.at(i), scales.at(i), 1e-10);
    }
}

TEST(CamOdoCalibration, MultiScaleWithNoise)
{
    CamOdoCalibration calib;
    calib.setVerbose(false);

    Eigen::Matrix4d H_12_expected = Eigen::Matrix4d::Identity();
    H_12_expected.block<3,3>(0,0) = Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.1, 0.2, 0.3).normalized()).toRotationMatrix();
    H_12_expected.block<3,1>(0,3) << 0.5, 0.6, 0.7;

    std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > > rvecs1, tvecs1, rvecs2, tvecs2;

    int segmentCount = 5;
    double sigma = 0.1;
    double tsigma = 0.05;
    cv::RNG rng;

    rvecs1.resize(segmentCount);
    tvecs1.resize(segmentCount);
    rvecs2.resize(segmentCount);
    tvecs2.resize(segmentCount);

    std::vector<double> scales_expected;
    scales_expected.resize(segmentCount);

    for (int i = 0; i < segmentCount; ++i)
    {
        int motionCount = random(10, 20);

        for (int j = 0; j < motionCount; ++j)
        {
            double droll = d2r(random(-10.0, 10.0));
            droll = 0;
            double dpitch =  d2r(random(-10.0, 10.0));
            dpitch = 0;
            double dyaw =  d2r(random(-100.0, 100.0));
            double dx = random(-10.0, 10.0);
            double dy = random(-10.0, 10.0);
            double dz = random(-10.0, 10.0);
            dz = 0;

            Eigen::Matrix3d R;
            R = Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(dpitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(droll, Eigen::Vector3d::UnitX());

            Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
            H.block<3,3>(0,0) = R;
            H.block<3,1>(0,3) << dx, dy, dz;

            Eigen::Matrix4d H_ = H.inverse();
            H = H_;

            Eigen::Vector3d rvec1, tvec1, rvec2, tvec2;

            Eigen::AngleAxisd angleAxis1(H.block<3,3>(0,0));
            rvec1 = angleAxis1.angle() * angleAxis1.axis();

            tvec1 = H.block<3,1>(0,3);

            Eigen::Matrix3d R2 = (H_12_expected.inverse() * H * H_12_expected).block<3,3>(0,0);
            double roll, pitch, yaw;
            mat2RPY(R2, roll, pitch, yaw);

            roll += rng.gaussian(sigma);
            pitch += rng.gaussian(sigma);
            yaw += rng.gaussian(sigma);

            Eigen::AngleAxisd angleAxis2;
            angleAxis2.fromRotationMatrix(RPY2mat(roll, pitch, yaw));
            rvec2 = angleAxis2.angle() * angleAxis2.axis();

            tvec2 = (H_12_expected.inverse() * H * H_12_expected).block<3,1>(0,3);
            tvec2(0) *= (1.0 + rng.gaussian(tsigma));
            tvec2(1) *= (1.0 + rng.gaussian(tsigma));
            tvec2(2) *= (1.0 + rng.gaussian(tsigma));

            if (j == 0)
            {
                scales_expected.at(i) = tvec2.norm();
                tvec2.normalize();
            }
            else
            {
                tvec2 /= scales_expected.at(i);
            }

            rvecs1.at(i).push_back(rvec1);
            tvecs1.at(i).push_back(tvec1);
            rvecs2.at(i).push_back(rvec2);
            tvecs2.at(i).push_back(tvec2);
        }
    }

    Eigen::Matrix4d H_12;
    std::vector<double> scales;

    EXPECT_TRUE(calib.estimate(rvecs1, tvecs1, rvecs2, tvecs2, H_12, scales));

//    for (int i = 0; i < 4; ++i)
//    {
//        for (int j = 0; j < 4; ++j)
//        {
//            if (i == 2 && j == 3)
//            {
//                continue;
//            }
//            EXPECT_NEAR(H_12_expected(i,j), H_12(i,j), 0.0000000001) << "Elements differ at (" << i << "," << j << ")";
//        }
//    }
//
//    for (size_t i = 0; i < scales.size(); ++i)
//    {
//        EXPECT_NEAR(scales_expected.at(i), scales.at(i), 0.0000000001);
//    }

    std::cout << "H_expected = \n" << H_12_expected << std::endl;
    std::cout << "H = \n" << H_12 << std::endl;

    std::cout << "Rotation error = " << r2d(acos(0.5 * ((H_12_expected.block<3,3>(0,0) * H_12.block<3,3>(0,0).transpose()).trace() - 1.0))) << " deg" << std::endl;

    Eigen::Vector2d t_expected = H_12_expected.block<2,1>(0,3);
    Eigen::Vector2d t = H_12.block<2,1>(0,3);

    std::cout << "Translation direction error = " << r2d(acos((t_expected.transpose() * t)(0,0) / (t_expected.norm() * t.norm()))) << " deg" << std::endl;
    std::cout << "Translation error = " << (fabs(t_expected.norm() - t.norm()) / t_expected.norm()) * 100.0 << " %" << std::endl;

    double errScale = 0.0;
    for (size_t i = 0; i < scales.size(); ++i)
    {
        errScale += fabs(scales_expected.at(i) - scales.at(i)) / scales_expected.at(i);
    }
    errScale /= scales.size();
    std::cout << "% scale error = " << errScale * 100 << " %" << std::endl;
    std::cout << std::endl;
}
}
