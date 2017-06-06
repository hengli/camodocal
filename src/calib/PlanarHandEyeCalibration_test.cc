#include <gtest/gtest.h>
#include <iostream>

#include "../gpl/gpl.h"
#include "camodocal/EigenUtils.h"
#include "camodocal/calib/PlanarHandEyeCalibration.h"

namespace camodocal
{

TEST(PlanarHandEyeCalibration, SetRandomSeed)
{
   // srand(time(0));
    srand(5);
}

TEST(PlanarHandEyeCalibration, Calibrate)
{
    PlanarHandEyeCalibration calib;
    calib.setVerbose(false);

    Eigen::Matrix4d H_12_expected = Eigen::Matrix4d::Identity();
    H_12_expected.block<3,3>(0,0) = Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.1, 0.2, 0.3).normalized()).toRotationMatrix();
    H_12_expected.block<3,1>(0,3) << 0.5, 0.6, 0.7;

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > H1, H2;

    int motionCount = 2;

    for (int j = 0; j < motionCount; ++j)
    {
        double dyaw =  d2r(random(-100.0, 100.0));
        double dx = random(-10.0, 10.0);
        double dy = random(-10.0, 10.0);

        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ());

        Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
        H.block<3,3>(0,0) = R;
        H.block<3,1>(0,3) << dx, dy, 0.0;

        H1.push_back(H_12_expected.inverse() * H * H_12_expected);
        H2.push_back(H);
    }

    ASSERT_TRUE(calib.addMotions(H1, H2));

    Eigen::Matrix4d H_12;
    ASSERT_TRUE(calib.calibrate(H_12));

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
}

}
