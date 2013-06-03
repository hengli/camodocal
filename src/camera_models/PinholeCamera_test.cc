#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <iostream>

#include "camodocal/camera_models/PinholeCamera.h"

namespace camodocal
{

TEST(PinholeCamera, spaceToPlane1)
{
    PinholeCamera camera("camera", 752, 480,
                         -0.473, 0.273, -0.001, 0.001,
                         712.557492, 714.825860, 370.075592, 244.759309);

    Eigen::Vector3d P(0.0, 0.0, 1.0);

    Eigen::Vector2d p_est;
    camera.spaceToPlane(P, p_est);

    EXPECT_NEAR(camera.getParameters().cx(), p_est(0), 1e-10);
    EXPECT_NEAR(camera.getParameters().cy(), p_est(1), 1e-10);
}

TEST(PinholeCamera, spaceToPlane2)
{
    PinholeCamera camera("camera", 752, 480,
                         -0.473, 0.273, -0.001, 0.001,
                         712.557492, 714.825860, 370.075592, 244.759309);

    std::vector<double> params;
    camera.writeParameters(params);
    double q[4] = {0.0, 0.0, 0.0, 1.0};
    double t[3] = {0.0, 0.0, 0.0};

    Eigen::Vector3d P(0.0, 0.0, 1.0);

    Eigen::Vector2d p_est;
    PinholeCamera::spaceToPlane(params.data(), q, t, P, p_est);

    EXPECT_NEAR(camera.getParameters().cx(), p_est(0), 1e-10);
    EXPECT_NEAR(camera.getParameters().cy(), p_est(1), 1e-10);
}


TEST(PinholeCamera, liftProjective)
{
    PinholeCamera camera("camera", 752, 480,
                         -0.473, 0.273, -0.001, 0.001,
                         712.557492, 714.825860, 370.075592, 244.759309);

    Eigen::Vector2d p(camera.getParameters().cx(),
                      camera.getParameters().cy());

    Eigen::Vector3d P_est;
    camera.liftProjective(p, P_est);

    P_est.normalize();

    EXPECT_NEAR(0.0, P_est(0), 1e-10);
    EXPECT_NEAR(0.0, P_est(1), 1e-10);
    EXPECT_NEAR(1.0, P_est(2), 1e-10);
}

TEST(PinholeCamera, consistency)
{
    PinholeCamera camera("camera", 752, 480,
                         -0.473, 0.273, -0.001, 0.001,
                         712.557492, 714.825860, 370.075592, 244.759309);

    Eigen::Vector3d P(1.0, -1.0, 4.0);

    Eigen::Vector2d p_est;
    camera.spaceToPlane(P, p_est);

    Eigen::Vector3d P_est;
    camera.liftProjective(p_est, P_est);

    P.normalize();
    P_est.normalize();

    EXPECT_NEAR(P(0), P_est(0), 1e-8);
    EXPECT_NEAR(P(1), P_est(1), 1e-8);
    EXPECT_NEAR(P(2), P_est(2), 1e-8);
}

}
