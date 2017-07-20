#include <boost/math/constants/constants.hpp>
#include <gtest/gtest.h>
#include <iostream>
#include <random>

#include "camodocal/calib/HandEyeCalibration.h"
#include "camodocal/EigenUtils.h"

namespace camodocal
{

TEST(HandEyeCalibration, FullMotion)
{
    HandEyeCalibration::setVerbose(false);

    Eigen::Matrix4d H_12_expected = Eigen::Matrix4d::Identity();
    H_12_expected.block<3,3>(0,0) = Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.1, 0.2, 0.3).normalized()).toRotationMatrix();
    H_12_expected.block<3,1>(0,3) << 0.5, 0.6, 0.7;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > rvecs1, tvecs1, rvecs2, tvecs2;
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> dis(-10.0, 10.0);
    std::uniform_int_distribution<> dis1(-1.0, 1.0);

    int motionCount = 2;
    for (int i = 0; i < motionCount; ++i)
    {
    
        double droll = boost::math::constants::radian<double>()*dis(gen);
        double dpitch = boost::math::constants::radian<double>()*dis(gen);
        double dyaw = boost::math::constants::radian<double>()*dis(gen);
        double dx = dis1(gen);
        double dy = dis1(gen);
        double dz = dis1(gen);

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

        Eigen::AngleAxisd angleAxis1((H_12_expected * H * H_12_expected.inverse()).block<3,3>(0,0));
        rvec1 = angleAxis1.angle() * angleAxis1.axis();

        tvec1 = (H_12_expected * H * H_12_expected.inverse()).block<3,1>(0,3);

        Eigen::AngleAxisd angleAxis2(H.block<3,3>(0,0));
        rvec2 = angleAxis2.angle() * angleAxis2.axis();

        tvec2 = H.block<3,1>(0,3);

        rvecs1.push_back(rvec1);
        tvecs1.push_back(tvec1);
        rvecs2.push_back(rvec2);
        tvecs2.push_back(tvec2);
    }

    Eigen::Matrix4d H_12;
    HandEyeCalibration::estimateHandEyeScrew(rvecs1, tvecs1, rvecs2, tvecs2, H_12);

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(H_12_expected(i,j),H_12(i,j),0.0000000001) << "Elements differ at (" << i << "," << j << ")";
        }
    }
}

TEST(HandEyeCalibration, PlanarMotion)
{
    HandEyeCalibration::setVerbose(false);

    Eigen::Matrix4d H_12_expected = Eigen::Matrix4d::Identity();
    H_12_expected.block<3,3>(0,0) = Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.1, 0.2, 0.3).normalized()).toRotationMatrix();
    H_12_expected.block<3,1>(0,3) << 0.5, 0.6, 0.7;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > rvecs1, tvecs1, rvecs2, tvecs2;
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> dis10(-10.0, 10.0);
    std::uniform_int_distribution<> dis1(-1.0, 1.0);

    int motionCount = 2;
    for (int i = 0; i < motionCount; ++i)
    {
        double droll = boost::math::constants::radian<double>()*dis10(gen);
        droll = 0;
        double dpitch = boost::math::constants::radian<double>()*dis10(gen);
        dpitch = 0;
        double dyaw = boost::math::constants::radian<double>()*dis10(gen);
        double dx = dis1(gen);
        double dy = dis1(gen);
        double dz = dis1(gen);
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

        rvecs1.push_back(rvec1);
        tvecs1.push_back(tvec1);
        rvecs2.push_back(rvec2);
        tvecs2.push_back(tvec2);
    }

    Eigen::Matrix4d H_12;
    HandEyeCalibration::estimateHandEyeScrew(rvecs1, tvecs1, rvecs2, tvecs2, H_12, true);

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            if (i == 2 && j == 3) continue;
            EXPECT_NEAR(H_12_expected(i,j),H_12(i,j),0.0000000001) << "Elements differ at (" << i << "," << j << ")";
        }
    }
}

TEST(HandEyeCalibration, PlanarMotionWithNoise)
{
    HandEyeCalibration::setVerbose(true);

    Eigen::Matrix4d H_12_expected = Eigen::Matrix4d::Identity();
    H_12_expected.block<3,3>(0,0) = Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.1, 0.2, 0.3).normalized()).toRotationMatrix();
    H_12_expected.block<3,1>(0,3) << 0.5, 0.6, 0.7;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > rvecs1, tvecs1, rvecs2, tvecs2;

    double scale = 1.5;
    int motionCount = 200;
    double sigma = 0.0005;

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> dis10(-10.0, 10.0);
    std::uniform_int_distribution<> dis100(-100.0, 100.0);
    std::normal_distribution<> gaussian(0,sigma);
    for (int i = 0; i < motionCount; ++i)
    {
        double droll = boost::math::constants::radian<double>()*dis10(gen);
        droll = 0;
        double dpitch =  boost::math::constants::radian<double>()*dis10(gen);
        dpitch = 0;
        double dyaw =  boost::math::constants::radian<double>()*dis100(gen);
        double dx = dis10(gen);
        double dy = dis10(gen);
        double dz = dis10(gen);
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
        double roll, pitch, yaw;
        mat2RPY(angleAxis2.toRotationMatrix(), roll, pitch, yaw);

        roll += gaussian(gen);
        pitch += gaussian(gen);
        yaw += gaussian(gen);

        angleAxis2.fromRotationMatrix(RPY2mat(roll, pitch, yaw));
        rvec2 = angleAxis2.angle() * angleAxis2.axis();

        tvec2 = (H_12_expected.inverse() * H * H_12_expected).block<3,1>(0,3);

        rvecs1.push_back(rvec1);
        tvecs1.push_back(tvec1);
        rvecs2.push_back(rvec2);
        tvecs2.push_back(tvec2);
    }

    Eigen::Matrix4d H_12;
    HandEyeCalibration::estimateHandEyeScrew(rvecs1, tvecs1, rvecs2, tvecs2, H_12, true);
    Eigen::Matrix4d H1 = H_12_expected;
    Eigen::Matrix4d H2 = H_12;
    std::cout << "# INFO: H_12_expected = " << std::endl;
    std::cout << H_12_expected << std::endl; ;
//    std::cout << H1 << std::endl << H2 << std::endl;

//    for (int i = 0; i < 4; ++i)
//    {
//        for (int j = 0; j < 4; ++j)
//        {
//            if (i == 2 && j == 3) continue;
//            EXPECT_NEAR(H1(i,j),H2(i,j),0.0000000001) << "Elements differ at (" << i << "," << j << ")";
//        }
//    }
}

/*
TEST(HandEyeCalibration, EstimateWithUnitTranslation)
{
    Eigen::Matrix4d H_12_expected = Eigen::Matrix4d::Identity();
//    H_12_expected.block<3,3>(0,0) = Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.0, 0.0, 0.3).normalized()).toRotationMatrix();
//    H_12_expected.block<3,3>(0,0) = Eigen::AngleAxisd(0.4, Eigen::Vector3d(1, 1, 0.3).normalized()).toRotationMatrix();
    H_12_expected.block<3,1>(0,3) << 0.5, 0.6, 0.7;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > rvecs1, tvecs1, rvecs2, tvecs2;

    int motionCount = 2;
    for (int i = 0; i < motionCount; ++i)
    {
        double droll = boost::math::constants::radian<double>()*dis10(gen));
        double dpitch =  boost::math::constants::radian<double>()*dis10(gen));
        double dyaw =  boost::math::constants::radian<double>()*dis10(gen));
        double dx = dis1(gen);
        double dy = dis1(gen);
        double dz = dis1(gen);

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

        Eigen::AngleAxisd angleAxis1((H_12_expected * H * H_12_expected.inverse()).block<3,3>(0,0));
        rvec1 = angleAxis1.angle() * angleAxis1.axis();

        tvec1 = (H_12_expected * H * H_12_expected.inverse()).block<3,1>(0,3);
        tvec1.normalize();

        Eigen::AngleAxisd angleAxis2(H.block<3,3>(0,0));
        rvec2 = angleAxis2.angle() * angleAxis2.axis();

        tvec2 = H.block<3,1>(0,3);

        rvecs1.push_back(rvec1);
        tvecs1.push_back(tvec1);
        rvecs2.push_back(rvec2);
        tvecs2.push_back(tvec2);
    }

    Eigen::Matrix4d H_12;
    HandEyeCalibration::estimateHandEyeScrew(rvecs1, tvecs1, rvecs2, tvecs2, H_12, true);

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(H_12_expected(i,j),H_12(i,j),0.0000000001) << "Elements differ at (" << i << "," << j << ")";
        }
    }
}
*/

}
