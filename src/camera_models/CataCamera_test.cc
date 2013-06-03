#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <iostream>

#include "camodocal/camera_models/CataCamera.h"

namespace camodocal
{

TEST(CataCamera, spaceToPlane1)
{
    CataCamera camera("camera", 1280, 800,
                      0.894975, -0.344504, 0.0984552, -0.00403995, 0.00610364,
                      758.355, 757.615, 646.72, 395.001);

    Eigen::Vector3d P(0.0, 0.0, 1.0);

    Eigen::Vector2d p_est;
    camera.spaceToPlane(P, p_est);

    EXPECT_NEAR(camera.getParameters().u0(), p_est(0), 1e-10);
    EXPECT_NEAR(camera.getParameters().v0(), p_est(1), 1e-10);
}

TEST(CataCamera, spaceToPlane2)
{
    CataCamera camera("camera", 1280, 800,
                      0.894975, -0.344504, 0.0984552, -0.00403995, 0.00610364,
                      758.355, 757.615, 646.72, 395.001);

    std::vector<double> params;
    camera.writeParameters(params);
    double q[4] = {0.0, 0.0, 0.0, 1.0};
    double t[3] = {0.0, 0.0, 0.0};

    Eigen::Vector3d P(0.0, 0.0, 1.0);

    Eigen::Vector2d p_est;
    CataCamera::spaceToPlane(params.data(), q, t, P, p_est);

    EXPECT_NEAR(camera.getParameters().u0(), p_est(0), 1e-10);
    EXPECT_NEAR(camera.getParameters().v0(), p_est(1), 1e-10);
}


TEST(CataCamera, liftProjective)
{
    CataCamera camera("camera", 1280, 800,
                      0.894975, -0.344504, 0.0984552, -0.00403995, 0.00610364,
                      758.355, 757.615, 646.72, 395.001);

    Eigen::Vector2d p(camera.getParameters().u0(),
                      camera.getParameters().v0());

    Eigen::Vector3d P_est;
    camera.liftProjective(p, P_est);

    P_est.normalize();

    EXPECT_NEAR(0.0, P_est(0), 1e-10);
    EXPECT_NEAR(0.0, P_est(1), 1e-10);
    EXPECT_NEAR(1.0, P_est(2), 1e-10);
}

TEST(CataCamera, consistency)
{
    CataCamera camera("camera", 1280, 800,
                      0.894975, -0.344504, 0.0984552, -0.00403995, 0.00610364,
                      758.355, 757.615, 646.72, 395.001);

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

        EXPECT_NEAR(P(0), P_est(0), 1e-2);
        EXPECT_NEAR(P(1), P_est(1), 1e-2);
        EXPECT_NEAR(P(2), P_est(2), 1e-2);
    }
}

}
