#include "CostFunctionFactory.h"

#include "ceres/ceres.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

namespace camodocal
{

template<typename T>
void
worldToCameraTransform(const T* const q_cam_odo, const T* const t_cam_odo,
                       const T* const p_odo, const T* const yaw_odo,
                       bool optimizeZ,
                       T* q, T* t)
{
    T q_odo[4] = {cos(yaw_odo[0] / T(2.0)), T(0.0), T(0.0), -sin(yaw_odo[0] / T(2.0))};

    T q_odo_cam[4] = {q_cam_odo[3], -q_cam_odo[0], -q_cam_odo[1], -q_cam_odo[2]};

    T q0[4];
    ceres::QuaternionProduct(q_odo_cam, q_odo, q0);

    T t0[3];
    T t_odo[3] = {p_odo[0], p_odo[1], T(0.0)};
    ceres::QuaternionRotatePoint(q_odo, t_odo, t0);

    t0[0] += t_cam_odo[0];
    t0[1] += t_cam_odo[1];

    if (optimizeZ)
    {
        t0[2] += t_cam_odo[2];
    }

    ceres::QuaternionRotatePoint(q_odo_cam, t0, t);
    t[0] = -t[0]; t[1] = -t[1]; t[2] = -t[2]; t[3] = -t[3];

    // Convert quaternion from Ceres convention (w, x, y, z)
    // to Eigen convention (x, y, z, w)
    q[0] = q0[1]; q[1] = q0[2]; q[2] = q0[3]; q[3] = q0[0];
}

// variables: camera intrinsics and camera extrinsics
template<class CameraT>
class ReprojectionError1
{
public:
    ReprojectionError1(const Eigen::Vector3d& observed_P,
                       const Eigen::Vector2d& observed_p)
     : m_observed_P(observed_P), m_observed_p(observed_p) {}

    template <typename T>
    bool operator()(const T* const intrinsic_params,
                    const T* const q,
                    const T* const t,
                    T* residuals) const
    {
        Eigen::Matrix<T,3,1> P;
        P(0) = T(m_observed_P(0));
        P(1) = T(m_observed_P(1));
        P(2) = T(m_observed_P(2));

        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params, q, t, P, predicted_p);

        residuals[0] = predicted_p(0) - T(m_observed_p(0));
        residuals[1] = predicted_p(1) - T(m_observed_p(1));

        return true;
    }

private:
    // observed 3D point
    Eigen::Vector3d m_observed_P;

    // observed 2D point
    Eigen::Vector2d m_observed_p;
};

// variables: camera extrinsics, 3D point
template<class CameraT>
class ReprojectionError2
{
public:
    ReprojectionError2(const std::vector<double>& intrinsic_params,
                       const Eigen::Vector2d& observed_p)
     : m_intrinsic_params(intrinsic_params), m_observed_p(observed_p) {}

    template <typename T>
    bool operator()(const T* const q, const T* const t,
                    const T* const point, T* residuals) const
    {
        Eigen::Matrix<T,3,1> P;
        P(0) = T(point[0]);
        P(1) = T(point[1]);
        P(2) = T(point[2]);

        std::vector<T> intrinsic_params(m_intrinsic_params.begin(), m_intrinsic_params.end());

        // project 3D object point to the image plane
        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params.data(), q, t, P, predicted_p);

        residuals[0] = predicted_p(0) - T(m_observed_p(0));
        residuals[1] = predicted_p(1) - T(m_observed_p(1));

        return true;
    }

private:
    // camera intrinsics
    std::vector<double> m_intrinsic_params;

    // observed 2D point
    Eigen::Vector2d m_observed_p;
};

template<class CameraT>
class ReprojectionError3
{
public:
    ReprojectionError3(const Eigen::Vector2d& observed_p, bool optimizeZ)
     : m_observed_p(observed_p), m_optimize_z(optimizeZ) {}

    ReprojectionError3(const std::vector<double>& intrinsic_params,
                       const Eigen::Vector2d& observed_p, bool optimizeZ)
     : m_intrinsic_params(intrinsic_params), m_observed_p(observed_p)
     , m_optimize_z(optimizeZ) {}

    ReprojectionError3(const std::vector<double>& intrinsic_params,
                       const Eigen::Vector2d& odo_pos, double odo_yaw,
                       const Eigen::Vector2d& observed_p, bool optimizeZ)
     : m_intrinsic_params(intrinsic_params)
     , m_odo_pos(odo_pos), m_odo_yaw(odo_yaw)
     , m_observed_p(observed_p)
     , m_optimize_z(optimizeZ) {}

    // variables: camera intrinsics, camera-to-odometry transform,
    //            odometry extrinsics, 3D point
    template <typename T>
    bool operator()(const T* const intrinsic_params,
                    const T* const q_cam_odo, const T* const t_cam_odo,
                    const T* const p_odo, const T* const yaw_odo,
                    const T* const point, T* residuals) const
    {
        T q[4], t[3];
        worldToCameraTransform(q_cam_odo, t_cam_odo, p_odo, yaw_odo, m_optimize_z, q, t);

        Eigen::Matrix<T,3,1> P(point[0], point[1], point[2]);

        // project 3D object point to the image plane
        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params, q, t, P, predicted_p);

        residuals[0] = predicted_p(0) - T(m_observed_p(0));
        residuals[1] = predicted_p(1) - T(m_observed_p(1));

        return true;
    }

    // variables: camera-to-odometry transform, 3D point
    template <typename T>
    bool operator()(const T* const q_cam_odo, const T* const t_cam_odo,
                    const T* const point, T* residuals) const
    {
        T p_odo[2] = {T(m_odo_pos(0)), T(m_odo_pos(1))};
        T yaw_odo = T(m_odo_yaw);
        T q[4], t[3];

        worldToCameraTransform(q_cam_odo, t_cam_odo, p_odo, &yaw_odo, m_optimize_z, q, t);

        std::vector<T> intrinsic_params(m_intrinsic_params.begin(), m_intrinsic_params.end());
        Eigen::Matrix<T,3,1> P(point[0], point[1], point[2]);

        // project 3D object point to the image plane
        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params.data(), q, t, P, predicted_p);

        residuals[0] = predicted_p(0) - T(m_observed_p(0));
        residuals[1] = predicted_p(1) - T(m_observed_p(1));

        return true;
    }

    // variables: camera-to-odometry transform, odometry extrinsics, 3D point
    template <typename T>
    bool operator()(const T* const q_cam_odo, const T* const t_cam_odo,
                    const T* const p_odo, const T* const yaw_odo,
                    const T* const point, T* residuals) const
    {
        T q[4], t[3];
        worldToCameraTransform(q_cam_odo, t_cam_odo, p_odo, yaw_odo, m_optimize_z, q, t);

        std::vector<T> intrinsic_params(m_intrinsic_params.begin(), m_intrinsic_params.end());
        Eigen::Matrix<T,3,1> P(point[0], point[1], point[2]);

        // project 3D object point to the image plane
        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params.data(), q, t, P, predicted_p);

        residuals[0] = predicted_p(0) - T(m_observed_p(0));
        residuals[1] = predicted_p(1) - T(m_observed_p(1));

        return true;
    }

private:
    // camera intrinsics
    std::vector<double> m_intrinsic_params;

    // observed odometry
    Eigen::Vector2d m_odo_pos;
    double m_odo_yaw;

    // observed 2D point
    Eigen::Vector2d m_observed_p;

    bool m_optimize_z;
};

boost::shared_ptr<CostFunctionFactory> CostFunctionFactory::m_instance;

CostFunctionFactory::CostFunctionFactory()
{

}

boost::shared_ptr<CostFunctionFactory>
CostFunctionFactory::instance(void)
{
    if (m_instance.get() == 0)
    {
        m_instance.reset(new CostFunctionFactory);
    }

    return m_instance;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const CameraConstPtr& camera,
                                          const Eigen::Vector3d& observed_P,
                                          const Eigen::Vector2d& observed_p,
                                          int flags) const
{
    ceres::CostFunction* costFunction = 0;

    switch (flags)
    {
    case CAMERA_INTRINSICS | CAMERA_EXTRINSICS:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<EquidistantCamera>, 2, 8, 4, 3>(
                    new ReprojectionError1<EquidistantCamera>(observed_P, observed_p));
            break;
        case Camera::PINHOLE:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<PinholeCamera>, 2, 8, 4, 3>(
                    new ReprojectionError1<PinholeCamera>(observed_P, observed_p));
            break;
        case Camera::MEI:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<CataCamera>, 2, 9, 4, 3>(
                    new ReprojectionError1<CataCamera>(observed_P, observed_p));
            break;
        }
        break;
    }

   return costFunction;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const CameraConstPtr& camera,
                                          const Eigen::Vector2d& observed_p,
                                          int flags, bool optimize_cam_odo_z) const
{
    ceres::CostFunction* costFunction = 0;

    std::vector<double> intrinsic_params;
    camera->writeParameters(intrinsic_params);

    switch (flags)
    {
    case CAMERA_EXTRINSICS | POINT_3D:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError2<EquidistantCamera>, 2, 4, 3, 3>(
                    new ReprojectionError2<EquidistantCamera>(intrinsic_params, observed_p));
            break;
        case Camera::PINHOLE:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError2<PinholeCamera>, 2, 4, 3, 3>(
                    new ReprojectionError2<PinholeCamera>(intrinsic_params, observed_p));
            break;
        case Camera::MEI:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError2<CataCamera>, 2, 4, 3, 3>(
                    new ReprojectionError2<CataCamera>(intrinsic_params, observed_p));
            break;
        }
        break;
    case CAMERA_ODOMETRY_EXTRINSICS | ODOMETRY_EXTRINSICS | POINT_3D:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<EquidistantCamera>(intrinsic_params, observed_p, optimize_cam_odo_z));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p, optimize_cam_odo_z));
            }
            break;
        case Camera::PINHOLE:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p, optimize_cam_odo_z));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p, optimize_cam_odo_z));
            }
            break;
        case Camera::MEI:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, observed_p, optimize_cam_odo_z));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, observed_p, optimize_cam_odo_z));
            }
            break;
        }
        break;
    case CAMERA_INTRINSICS | CAMERA_ODOMETRY_EXTRINSICS | ODOMETRY_EXTRINSICS | POINT_3D:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 8, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<EquidistantCamera>(observed_p, optimize_cam_odo_z));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p, optimize_cam_odo_z));
            }
            break;
        case Camera::PINHOLE:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p, optimize_cam_odo_z));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p, optimize_cam_odo_z));
            }
            break;
        case Camera::MEI:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 9, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<CataCamera>(observed_p, optimize_cam_odo_z));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 9, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<CataCamera>(observed_p, optimize_cam_odo_z));
            }
            break;
        }
        break;
    }

    return costFunction;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const CameraConstPtr& camera,
                                          const Eigen::Vector2d& odo_pos,
                                          double odo_yaw,
                                          const Eigen::Vector2d& observed_p,
                                          int flags, bool optimize_cam_odo_z) const
{
    ceres::CostFunction* costFunction = 0;

    std::vector<double> intrinsic_params;
    camera->writeParameters(intrinsic_params);

    switch (flags)
    {
    case CAMERA_ODOMETRY_EXTRINSICS | POINT_3D:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 4, 3, 3>(
                        new ReprojectionError3<EquidistantCamera>(intrinsic_params, odo_pos, odo_yaw, observed_p, optimize_cam_odo_z));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 4, 2, 3>(
                        new ReprojectionError3<EquidistantCamera>(intrinsic_params, odo_pos, odo_yaw, observed_p, optimize_cam_odo_z));
            }
            break;
        case Camera::PINHOLE:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, odo_pos, odo_yaw, observed_p, optimize_cam_odo_z));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 2, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, odo_pos, odo_yaw, observed_p, optimize_cam_odo_z));
            }
            break;
        case Camera::MEI:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 3, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, odo_pos, odo_yaw, observed_p, optimize_cam_odo_z));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 2, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, odo_pos, odo_yaw, observed_p, optimize_cam_odo_z));
            }
            break;
        }
        break;
    }

    return costFunction;
}

}

