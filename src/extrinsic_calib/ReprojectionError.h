#ifndef REPROJECTIONERROR_H
#define REPROJECTIONERROR_H

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "../../library/gpl/EigenUtils.h"

namespace camodocal
{

class CameraReprojectionError
{
public:
    CameraReprojectionError(const CataCameraParameters& intrinsicParams,
                            double observed_x, double observed_y)
     : m_xi(intrinsicParams.xi())
     , m_k1(intrinsicParams.k1())
     , m_k2(intrinsicParams.k2())
     , m_p1(intrinsicParams.p1())
     , m_p2(intrinsicParams.p2())
     , m_gamma1(intrinsicParams.gamma1())
     , m_gamma2(intrinsicParams.gamma2())
     , m_u0(intrinsicParams.u0())
     , m_v0(intrinsicParams.v0())
     , m_observed_x(observed_x)
     , m_observed_y(observed_y)
    {

    }

    template <typename T>
    bool operator()(const T* const cam_q, const T* const cam_t,
                    const T* const point, T* residuals) const
    {
        T q[4] = {cam_q[3], cam_q[0], cam_q[1], cam_q[2]};

        Eigen::Matrix<T, 3, 1> P_cam;
        ceres::QuaternionRotatePoint(q, point, P_cam.data());

        // cartesian coordinates
        P_cam(0) += cam_t[0];
        P_cam(1) += cam_t[1];
        P_cam(2) += cam_t[2];

        // project 3D object point to the image plane
        T xi = T(m_xi);
        T k1 = T(m_k1);
        T k2 = T(m_k2);
        T p1 = T(m_p1);
        T p2 = T(m_p2);
        T gamma1 = T(m_gamma1);
        T gamma2 = T(m_gamma2);
        T alpha = T(0);
        T u0 = T(m_u0);
        T v0 = T(m_v0);

        // Transform to model plane
        P_cam.normalize();

        T u = P_cam(0) / (P_cam(2) + xi);
        T v = P_cam(1) / (P_cam(2) + xi);

        T rho_sqr = square(u) + square(v);
        T L = T(1.0) + k1 * rho_sqr + k2 * square(rho_sqr);
        T du = T(2.0) * p1 * u * v + p2 * (rho_sqr + T(2.0) * square(u));
        T dv = p1 * (rho_sqr + T(2.0) * square(v)) + T(2.0) * p2 * u * v;

        u = L * u + du;
        v = L * v + dv;
        T x = gamma1 * (u + alpha * v) + u0;
        T y = gamma2 * v + v0;

        residuals[0] = x - T(m_observed_x);
        residuals[1] = y - T(m_observed_y);

        return true;
    }

private:
    double m_xi;
    double m_k1, m_k2, m_p1, m_p2;
    double m_gamma1, m_gamma2;
    double m_u0, m_v0;
    double m_observed_x;
    double m_observed_y;
};

class CataOdometerReprojectionError
{
public:
    CataOdometerReprojectionError(double observed_x, double observed_y,
                                  bool optimizeZ = true)
     : m_observed_x(observed_x)
     , m_observed_y(observed_y)
     , m_optimize_z(optimizeZ)
    {

    }

    template <typename T>
    bool operator()(const T* const intrinsic_params,
                    const T* const cam_odo_q, const T* const cam_odo_t,
                    const T* const odo_p, const T* const odo_yaw,
                    const T* const point, T* residuals) const
    {
        T q[4] = {cos(odo_yaw[0] / T(2.0)), T(0.0), T(0.0), -sin(odo_yaw[0] / T(2.0))};

        Eigen::Matrix<T, 3, 1> P;
        ceres::QuaternionRotatePoint(q, point, P.data());

        T t_odo[3] = {odo_p[0], odo_p[1], T(0.0)};
        T t_odo_inv[3];
        ceres::QuaternionRotatePoint(q, t_odo, t_odo_inv);
        t_odo_inv[0] = -t_odo_inv[0]; t_odo_inv[1] = -t_odo_inv[1]; t_odo_inv[2] = -t_odo_inv[2];

        // cartesian coordinates
        P(0) += t_odo_inv[0];
        P(1) += t_odo_inv[1];
        P(2) += t_odo_inv[2];

        q[0] = cam_odo_q[3]; q[1] = -cam_odo_q[0]; q[2] = -cam_odo_q[1]; q[3] = -cam_odo_q[2];

        Eigen::Matrix<T, 3, 1> P_cam;
        ceres::QuaternionRotatePoint(q, P.data(), P_cam.data());

        T t_cam_odo[3];
        t_cam_odo[0] = cam_odo_t[0]; t_cam_odo[1] = cam_odo_t[1];
        if (m_optimize_z)
        {
            t_cam_odo[2] = cam_odo_t[2];
        }
        else
        {
            t_cam_odo[2] = T(0.0);
        }

        T t_odo_cam[3];
        ceres::QuaternionRotatePoint(q, t_cam_odo, t_odo_cam);
        t_odo_cam[0] = -t_odo_cam[0]; t_odo_cam[1] = -t_odo_cam[1]; t_odo_cam[2] = -t_odo_cam[2];

        P_cam(0) += t_odo_cam[0];
        P_cam(1) += t_odo_cam[1];
        P_cam(2) += t_odo_cam[2];

        // project 3D object point to the image plane
        T xi = intrinsic_params[0];
        T k1 = intrinsic_params[1];
        T k2 = intrinsic_params[2];
        T p1 = intrinsic_params[3];
        T p2 = intrinsic_params[4];
        T gamma1 = intrinsic_params[5];
        T gamma2 = intrinsic_params[6];
        T alpha = T(0); //cameraParams.alpha();
        T u0 = intrinsic_params[7];
        T v0 = intrinsic_params[8];

        // Pose to model plane
        T P_cam_norm = P_cam.norm();
        P_cam /= P_cam_norm;

        T u = P_cam(0) / (P_cam(2) + xi);
        T v = P_cam(1) / (P_cam(2) + xi);

        T rho_sqr = square(u) + square(v);
        T L = T(1.0) + k1 * rho_sqr + k2 * square(rho_sqr);
        T du = T(2.0) * p1 * u * v + p2 * (rho_sqr + T(2.0) * square(u));
        T dv = p1 * (rho_sqr + T(2.0) * square(v)) + T(2.0) * p2 * u * v;

        u = L * u + du;
        v = L * v + dv;
        T x = gamma1 * (u + alpha * v) + u0;
        T y = gamma2 * v + v0;

        residuals[0] = x - T(m_observed_x);
        residuals[1] = y - T(m_observed_y);

        return true;
    }

private:
    double m_observed_x;
    double m_observed_y;
    bool m_optimize_z;
};

class OdometerCameraReprojectionError
{
public:
    OdometerCameraReprojectionError(const CataCameraParameters& intrinsicParams,
                                    const Eigen::Vector2d& odo_p,
                                    double odo_yaw,
                                    double observed_x, double observed_y,
                                    bool optimizeZ = true)
     : m_xi(intrinsicParams.xi())
     , m_k1(intrinsicParams.k1())
     , m_k2(intrinsicParams.k2())
     , m_p1(intrinsicParams.p1())
     , m_p2(intrinsicParams.p2())
     , m_gamma1(intrinsicParams.gamma1())
     , m_gamma2(intrinsicParams.gamma2())
     , m_u0(intrinsicParams.u0())
     , m_v0(intrinsicParams.v0())
     , m_odo_p_x(odo_p(0))
     , m_odo_p_y(odo_p(1))
     , m_odo_yaw(odo_yaw)
     , m_observed_x(observed_x)
     , m_observed_y(observed_y)
     , m_optimize_z(optimizeZ)
    {

    }

    template <typename T>
    bool operator()(const T* const cam_odo_q, const T* const cam_odo_t,
                    const T* const point, T* residuals) const
    {
        T q[4] = {cos(T(m_odo_yaw / 2.0)), T(0.0), T(0.0), -sin(T(m_odo_yaw / 2.0))};

        Eigen::Matrix<T, 3, 1> P;
        ceres::QuaternionRotatePoint(q, point, P.data());

        T t_odo[3] = {T(m_odo_p_x), T(m_odo_p_y), T(0.0)};
        T t_odo_inv[3];
        ceres::QuaternionRotatePoint(q, t_odo, t_odo_inv);
        t_odo_inv[0] = -t_odo_inv[0]; t_odo_inv[1] = -t_odo_inv[1]; t_odo_inv[2] = -t_odo_inv[2];

        // cartesian coordinates
        P(0) += t_odo_inv[0];
        P(1) += t_odo_inv[1];
        P(2) += t_odo_inv[2];

        q[0] = cam_odo_q[3]; q[1] = -cam_odo_q[0]; q[2] = -cam_odo_q[1]; q[3] = -cam_odo_q[2];

        Eigen::Matrix<T, 3, 1> P_cam;
        ceres::QuaternionRotatePoint(q, P.data(), P_cam.data());

        T t_cam_odo[3];
        t_cam_odo[0] = cam_odo_t[0]; t_cam_odo[1] = cam_odo_t[1];
        if (m_optimize_z)
        {
            t_cam_odo[2] = cam_odo_t[2];
        }
        else
        {
            t_cam_odo[2] = T(0.0);
        }

        T t_odo_cam[3];
        ceres::QuaternionRotatePoint(q, t_cam_odo, t_odo_cam);
        t_odo_cam[0] = -t_odo_cam[0]; t_odo_cam[1] = -t_odo_cam[1]; t_odo_cam[2] = -t_odo_cam[2];

        P_cam(0) += t_odo_cam[0];
        P_cam(1) += t_odo_cam[1];
        P_cam(2) += t_odo_cam[2];

        // project 3D object point to the image plane
        T xi = T(m_xi);
        T k1 = T(m_k1);
        T k2 = T(m_k2);
        T p1 = T(m_p1);
        T p2 = T(m_p2);
        T gamma1 = T(m_gamma1);
        T gamma2 = T(m_gamma2);
        T alpha = T(0);
        T u0 = T(m_u0);
        T v0 = T(m_v0);

        // Pose to model plane
        P_cam.normalize();

        T u = P_cam(0) / (P_cam(2) + xi);
        T v = P_cam(1) / (P_cam(2) + xi);

        T rho_sqr = square(u) + square(v);
        T L = T(1.0) + k1 * rho_sqr + k2 * square(rho_sqr);
        T du = T(2.0) * p1 * u * v + p2 * (rho_sqr + T(2.0) * square(u));
        T dv = p1 * (rho_sqr + T(2.0) * square(v)) + T(2.0) * p2 * u * v;

        u = L * u + du;
        v = L * v + dv;
        T x = gamma1 * (u + alpha * v) + u0;
        T y = gamma2 * v + v0;

        residuals[0] = x - T(m_observed_x);
        residuals[1] = y - T(m_observed_y);

        return true;
    }

private:
    double m_xi;
    double m_k1, m_k2, m_p1, m_p2;
    double m_gamma1, m_gamma2;
    double m_u0, m_v0;
    double m_odo_p_x;
    double m_odo_p_y;
    double m_odo_yaw;
    double m_observed_x;
    double m_observed_y;
    bool m_optimize_z;
};

class OdometerReprojectionError
{
public:
    OdometerReprojectionError(const CataCameraParameters& intrinsicParams,
                              double observed_x, double observed_y,
                              bool optimizeZ = true, bool scaleError = false)
     : m_xi(intrinsicParams.xi())
     , m_k1(intrinsicParams.k1())
     , m_k2(intrinsicParams.k2())
     , m_p1(intrinsicParams.p1())
     , m_p2(intrinsicParams.p2())
     , m_gamma1(intrinsicParams.gamma1())
     , m_gamma2(intrinsicParams.gamma2())
     , m_u0(intrinsicParams.u0())
     , m_v0(intrinsicParams.v0())
     , m_observed_x(observed_x)
     , m_observed_y(observed_y)
     , m_optimize_z(optimizeZ)
     , m_scale_error(scaleError)
    {

    }

    template <typename T>
    bool operator()(const T* const cam_odo_q, const T* const cam_odo_t,
                    const T* const odo_p, const T* const odo_yaw,
                    const T* const point, T* residuals) const
    {
        T q[4] = {cos(odo_yaw[0] / T(2.0)), T(0.0), T(0.0), -sin(odo_yaw[0] / T(2.0))};

        Eigen::Matrix<T, 3, 1> P;
        ceres::QuaternionRotatePoint(q, point, P.data());

        T t_odo[3] = {odo_p[0], odo_p[1], T(0.0)};
        T t_odo_inv[3];
        ceres::QuaternionRotatePoint(q, t_odo, t_odo_inv);
        t_odo_inv[0] = -t_odo_inv[0]; t_odo_inv[1] = -t_odo_inv[1]; t_odo_inv[2] = -t_odo_inv[2];

        // cartesian coordinates
        P(0) += t_odo_inv[0];
        P(1) += t_odo_inv[1];
        P(2) += t_odo_inv[2];

        q[0] = cam_odo_q[3]; q[1] = -cam_odo_q[0]; q[2] = -cam_odo_q[1]; q[3] = -cam_odo_q[2];

        Eigen::Matrix<T, 3, 1> P_cam;
        ceres::QuaternionRotatePoint(q, P.data(), P_cam.data());

        T t_cam_odo[3];
        t_cam_odo[0] = cam_odo_t[0]; t_cam_odo[1] = cam_odo_t[1];
        if (m_optimize_z)
        {
            t_cam_odo[2] = cam_odo_t[2];
        }
        else
        {
            t_cam_odo[2] = T(0.0);
        }

        T t_odo_cam[3];
        ceres::QuaternionRotatePoint(q, t_cam_odo, t_odo_cam);
        t_odo_cam[0] = -t_odo_cam[0]; t_odo_cam[1] = -t_odo_cam[1]; t_odo_cam[2] = -t_odo_cam[2];

        P_cam(0) += t_odo_cam[0];
        P_cam(1) += t_odo_cam[1];
        P_cam(2) += t_odo_cam[2];

        // project 3D object point to the image plane
        T xi = T(m_xi);
        T k1 = T(m_k1);
        T k2 = T(m_k2);
        T p1 = T(m_p1);
        T p2 = T(m_p2);
        T gamma1 = T(m_gamma1);
        T gamma2 = T(m_gamma2);
        T alpha = T(0);
        T u0 = T(m_u0);
        T v0 = T(m_v0);

        // Pose to model plane
        T P_cam_norm = P_cam.norm();
        P_cam /= P_cam_norm;

        T u = P_cam(0) / (P_cam(2) + xi);
        T v = P_cam(1) / (P_cam(2) + xi);

        T rho_sqr = square(u) + square(v);
        T L = T(1.0) + k1 * rho_sqr + k2 * square(rho_sqr);
        T du = T(2.0) * p1 * u * v + p2 * (rho_sqr + T(2.0) * square(u));
        T dv = p1 * (rho_sqr + T(2.0) * square(v)) + T(2.0) * p2 * u * v;

        u = L * u + du;
        v = L * v + dv;
        T x = gamma1 * (u + alpha * v) + u0;
        T y = gamma2 * v + v0;

        residuals[0] = x - T(m_observed_x);
        residuals[1] = y - T(m_observed_y);

        if (m_scale_error)
        {
            residuals[0] /= P_cam_norm;
            residuals[1] /= P_cam_norm;
        }

        return true;
    }

private:
    double m_xi;
    double m_k1, m_k2, m_p1, m_p2;
    double m_gamma1, m_gamma2;
    double m_u0, m_v0;
    double m_observed_x;
    double m_observed_y;
    bool m_optimize_z;
    bool m_scale_error;
};

}

#endif
