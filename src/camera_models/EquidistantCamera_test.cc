#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <iostream>

#include "camodocal/camera_models/EquidistantCamera.h"

namespace camodocal
{

TEST(EquidistantCamera, spaceToPlane1)
{
    EquidistantCamera camera("camera", 1280, 800,
                             -0.01648, -0.00203, 0.00069, -0.00048,
                             419.22826, 420.42160, 655.45487, 389.66377);

    Eigen::Vector3d P(0.0, 0.0, 1.0);

    Eigen::Vector2d p_est;
    camera.spaceToPlane(P, p_est);

    EXPECT_NEAR(camera.getParameters().u0(), p_est(0), 1e-10);
    EXPECT_NEAR(camera.getParameters().v0(), p_est(1), 1e-10);
}

TEST(EquidistantCamera, spaceToPlane2)
{
    EquidistantCamera camera("camera", 1280, 800,
                             -0.01648, -0.00203, 0.00069, -0.00048,
                             419.22826, 420.42160, 655.45487, 389.66377);

    std::vector<double> params;
    camera.writeParameters(params);
    double q[4] = {0.0, 0.0, 0.0, 1.0};
    double t[3] = {0.0, 0.0, 0.0};

    Eigen::Vector3d P(0.0, 0.0, 1.0);

    Eigen::Vector2d p_est;
    EquidistantCamera::spaceToPlane(params.data(), q, t, P, p_est);

    EXPECT_NEAR(camera.getParameters().u0(), p_est(0), 1e-10);
    EXPECT_NEAR(camera.getParameters().v0(), p_est(1), 1e-10);
}


TEST(EquidistantCamera, liftProjective)
{
    EquidistantCamera camera("camera", 1280, 800,
                             -0.01648, -0.00203, 0.00069, -0.00048,
                             419.22826, 420.42160, 655.45487, 389.66377);

    Eigen::Vector2d p(camera.getParameters().u0(),
                      camera.getParameters().v0());

    Eigen::Vector3d P_est;
    camera.liftProjective(p, P_est);

    P_est.normalize();

    EXPECT_NEAR(0.0, P_est(0), 1e-10);
    EXPECT_NEAR(0.0, P_est(1), 1e-10);
    EXPECT_NEAR(1.0, P_est(2), 1e-10);
}

TEST(EquidistantCamera, consistency)
{
    EquidistantCamera camera("camera", 1280, 800,
                             -0.01648, -0.00203, 0.00069, -0.00048,
                             419.22826, 420.42160, 655.45487, 389.66377);

    for (int i = 0; i < 100; ++i)
    {
        Eigen::Vector3d P = Eigen::Vector3d::Random();
        P(2) = fabs(P(2));

        Eigen::Vector2d p_est;
        camera.spaceToPlane(P, p_est);

        Eigen::Vector3d P_est;
        camera.liftProjective(p_est, P_est);

        P.normalize();
        P_est.normalize();

        EXPECT_NEAR(P(0), P_est(0), 1e-10);
        EXPECT_NEAR(P(1), P_est(1), 1e-10);
        EXPECT_NEAR(P(2), P_est(2), 1e-10);
    }
}

}
