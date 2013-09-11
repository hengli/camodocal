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
                       const T* const p_odo, const T* const att_odo,
                       int flags,
                       T* q, T* t)
{
    Eigen::Quaternion<T> q_z_inv(cos(att_odo[0] / T(2)), T(0), T(0), -sin(att_odo[0] / T(2)));
    Eigen::Quaternion<T> q_y_inv(T(1), T(0), T(0), T(0));
    Eigen::Quaternion<T> q_x_inv(T(1), T(0), T(0), T(0));

    if (flags & OPTIMIZE_ODOMETRY_6D)
    {
        q_y_inv = Eigen::Quaternion<T>(cos(att_odo[1] / T(2)), T(0), -sin(att_odo[1] / T(2)), T(0));
        q_x_inv = Eigen::Quaternion<T>(cos(att_odo[2] / T(2)), -sin(att_odo[2] / T(2)), T(0), T(0));
    }

    Eigen::Quaternion<T> q_zyx_inv = q_x_inv * q_y_inv * q_z_inv;

    T q_odo[4] = {q_zyx_inv.w(), q_zyx_inv.x(), q_zyx_inv.y(), q_zyx_inv.z()};

    T q_odo_cam[4] = {q_cam_odo[3], -q_cam_odo[0], -q_cam_odo[1], -q_cam_odo[2]};

    T q0[4];
    ceres::QuaternionProduct(q_odo_cam, q_odo, q0);

    T t0[3];
    T t_odo[3] = {p_odo[0], p_odo[1], T(0)};

    if (flags & OPTIMIZE_ODOMETRY_6D)
    {
        t_odo[2] = p_odo[2];
    }

    ceres::QuaternionRotatePoint(q_odo, t_odo, t0);

    t0[0] += t_cam_odo[0];
    t0[1] += t_cam_odo[1];

    if (flags & OPTIMIZE_CAMERA_ODOMETRY_Z)
    {
        t0[2] += t_cam_odo[2];
    }

    ceres::QuaternionRotatePoint(q_odo_cam, t0, t);
    t[0] = -t[0]; t[1] = -t[1]; t[2] = -t[2];

    // Convert quaternion from Ceres convention (w, x, y, z)
    // to Eigen convention (x, y, z, w)
    q[0] = q0[1]; q[1] = q0[2]; q[2] = q0[3]; q[3] = q0[0];
}

template<class CameraT>
class ReprojectionError1
{
public:
    ReprojectionError1(const Eigen::Vector3d& observed_P,
                       const Eigen::Vector2d& observed_p)
     : m_observed_P(observed_P), m_observed_p(observed_p) {}

    ReprojectionError1(const std::vector<double>& intrinsic_params,
                       const Eigen::Vector3d& observed_P,
                       const Eigen::Vector2d& observed_p)
     : m_intrinsic_params(intrinsic_params)
     , m_observed_P(observed_P), m_observed_p(observed_p) {}

    // variables: camera intrinsics and camera extrinsics
    template <typename T>
    bool operator()(const T* const intrinsic_params,
                    const T* const q,
                    const T* const t,
                    T* residuals) const
    {
        Eigen::Matrix<T,3,1> P = m_observed_P.cast<T>();

        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params, q, t, P, predicted_p);

        residuals[0] = predicted_p(0) - T(m_observed_p(0));
        residuals[1] = predicted_p(1) - T(m_observed_p(1));

        return true;
    }

    // variables: camera intrinsics and camera extrinsics
    template <typename T>
    bool operator()(const T* const q_cam_odo, const T* const t_cam_odo,
                    const T* const p_odo, const T* const att_odo,
                    T* residuals) const
    {
        T q[4], t[3];
        worldToCameraTransform(q_cam_odo, t_cam_odo, p_odo, att_odo, OPTIMIZE_ODOMETRY_6D | OPTIMIZE_CAMERA_ODOMETRY_Z, q, t);

        Eigen::Matrix<T,3,1> P = m_observed_P.cast<T>();

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
    ReprojectionError3(const Eigen::Vector2d& observed_p, int optimize_flags)
     : m_observed_p(observed_p), m_optimize_flags(optimize_flags) {}

    ReprojectionError3(const std::vector<double>& intrinsic_params,
                       const Eigen::Vector2d& observed_p, int optimize_flags)
     : m_intrinsic_params(intrinsic_params), m_observed_p(observed_p)
     , m_optimize_flags(optimize_flags) {}

    ReprojectionError3(const std::vector<double>& intrinsic_params,
                       const Eigen::Vector3d& odo_pos,
                       const Eigen::Vector3d& odo_att,
                       const Eigen::Vector2d& observed_p, int optimize_flags)
     : m_intrinsic_params(intrinsic_params)
     , m_odo_pos(odo_pos), m_odo_att(odo_att)
     , m_observed_p(observed_p)
     , m_optimize_flags(optimize_flags) {}

    ReprojectionError3(const std::vector<double>& intrinsic_params,
                       const Eigen::Quaterniond& cam_odo_q,
                       const Eigen::Vector3d& cam_odo_t,
                       const Eigen::Vector3d& odo_pos,
                       const Eigen::Vector3d& odo_att,
                       const Eigen::Vector2d& observed_p, int optimize_flags)
     : m_intrinsic_params(intrinsic_params)
     , m_cam_odo_q(cam_odo_q), m_cam_odo_t(cam_odo_t)
     , m_odo_pos(odo_pos), m_odo_att(odo_att)
     , m_observed_p(observed_p)
     , m_optimize_flags(optimize_flags) {}

    // variables: camera intrinsics, camera-to-odometry transform,
    //            odometry extrinsics, 3D point
    template <typename T>
    bool operator()(const T* const intrinsic_params,
                    const T* const q_cam_odo, const T* const t_cam_odo,
                    const T* const p_odo, const T* const att_odo,
                    const T* const point, T* residuals) const
    {
        T q[4], t[3];
        worldToCameraTransform(q_cam_odo, t_cam_odo, p_odo, att_odo, m_optimize_flags, q, t);

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
        T p_odo[3] = {T(m_odo_pos(0)), T(m_odo_pos(1)), T(m_odo_pos(2))};
        T att_odo[3] = {T(m_odo_att(0)), T(m_odo_att(1)), T(m_odo_att(2))};
        T q[4], t[3];

        worldToCameraTransform(q_cam_odo, t_cam_odo, p_odo, att_odo, m_optimize_flags, q, t);

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
                    const T* const p_odo, const T* const att_odo,
                    const T* const point, T* residuals) const
    {
        T q[4], t[3];
        worldToCameraTransform(q_cam_odo, t_cam_odo, p_odo, att_odo, m_optimize_flags, q, t);

        std::vector<T> intrinsic_params(m_intrinsic_params.begin(), m_intrinsic_params.end());
        Eigen::Matrix<T,3,1> P(point[0], point[1], point[2]);

        // project 3D object point to the image plane
        Eigen::Matrix<T,2,1> predicted_p;
        CameraT::spaceToPlane(intrinsic_params.data(), q, t, P, predicted_p);

        residuals[0] = predicted_p(0) - T(m_observed_p(0));
        residuals[1] = predicted_p(1) - T(m_observed_p(1));

        return true;
    }

    // variables: 3D point
    template <typename T>
    bool operator()(const T* const point, T* residuals) const
    {
        T q_cam_odo[4] = {T(m_cam_odo_q.coeffs()(0)), T(m_cam_odo_q.coeffs()(1)), T(m_cam_odo_q.coeffs()(2)), T(m_cam_odo_q.coeffs()(3))};
        T t_cam_odo[3] = {T(m_cam_odo_t(0)), T(m_cam_odo_t(1)), T(m_cam_odo_t(2))};
        T p_odo[3] = {T(m_odo_pos(0)), T(m_odo_pos(1)), T(m_odo_pos(2))};
        T att_odo[3] = {T(m_odo_att(0)), T(m_odo_att(1)), T(m_odo_att(2))};
        T q[4], t[3];

        worldToCameraTransform(q_cam_odo, t_cam_odo, p_odo, att_odo, m_optimize_flags, q, t);

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

    // observed camera-odometry transform
    Eigen::Quaterniond m_cam_odo_q;
    Eigen::Vector3d m_cam_odo_t;

    // observed odometry
    Eigen::Vector3d m_odo_pos;
    Eigen::Vector3d m_odo_att;

    // observed 2D point
    Eigen::Vector2d m_observed_p;

    int m_optimize_flags;
};

// variables: camera intrinsics and camera extrinsics
template<class CameraT>
class StereoReprojectionError
{
public:
    StereoReprojectionError(const Eigen::Vector3d& observed_P,
                            const Eigen::Vector2d& observed_p_l,
                            const Eigen::Vector2d& observed_p_r)
     : m_observed_P(observed_P)
     , m_observed_p_l(observed_p_l)
     , m_observed_p_r(observed_p_r)
    {

    }

    template <typename T>
    bool operator()(const T* const intrinsic_params_l,
                    const T* const intrinsic_params_r,
                    const T* const q_l,
                    const T* const t_l,
                    const T* const q_l_r,
                    const T* const t_l_r,
                    T* residuals) const
    {
        Eigen::Matrix<T,3,1> P;
        P(0) = T(m_observed_P(0));
        P(1) = T(m_observed_P(1));
        P(2) = T(m_observed_P(2));

        Eigen::Matrix<T,2,1> predicted_p_l;
        CameraT::spaceToPlane(intrinsic_params_l, q_l, t_l, P, predicted_p_l);

        Eigen::Quaternion<T> q_r = Eigen::Quaternion<T>(q_l_r) * Eigen::Quaternion<T>(q_l);

        Eigen::Matrix<T,3,1> t_r;
        t_r(0) = t_l[0];
        t_r(1) = t_l[1];
        t_r(2) = t_l[2];

        t_r = Eigen::Quaternion<T>(q_l_r) * t_r;
        t_r(0) += t_l_r[0];
        t_r(1) += t_l_r[1];
        t_r(2) += t_l_r[2];

        Eigen::Matrix<T,2,1> predicted_p_r;
        CameraT::spaceToPlane(intrinsic_params_r, q_r.coeffs().data(), t_r.data(), P, predicted_p_r);

        residuals[0] = predicted_p_l(0) - T(m_observed_p_l(0));
        residuals[1] = predicted_p_l(1) - T(m_observed_p_l(1));
        residuals[2] = predicted_p_r(0) - T(m_observed_p_r(0));
        residuals[3] = predicted_p_r(1) - T(m_observed_p_r(1));

        return true;
    }

private:
    // observed 3D point
    Eigen::Vector3d m_observed_P;

    // observed 2D point
    Eigen::Vector2d m_observed_p_l;
    Eigen::Vector2d m_observed_p_r;
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

    std::vector<double> intrinsic_params;
    camera->writeParameters(intrinsic_params);

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
    case CAMERA_ODOMETRY_EXTRINSICS | ODOMETRY_6D_EXTRINSICS:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<EquidistantCamera>, 2, 4, 3, 3, 3>(
                    new ReprojectionError1<EquidistantCamera>(intrinsic_params, observed_P, observed_p));
            break;
        case Camera::PINHOLE:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<PinholeCamera>, 2, 4, 3, 3, 3>(
                    new ReprojectionError1<PinholeCamera>(intrinsic_params, observed_P, observed_p));
            break;
        case Camera::MEI:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError1<CataCamera>, 2, 4, 3, 3, 3>(
                    new ReprojectionError1<CataCamera>(intrinsic_params, observed_P, observed_p));
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

    int optimize_flags = 0;
    if (optimize_cam_odo_z)
    {
        optimize_flags = OPTIMIZE_CAMERA_ODOMETRY_Z;
    }

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
    case CAMERA_ODOMETRY_EXTRINSICS | ODOMETRY_3D_EXTRINSICS | POINT_3D:
        optimize_flags |= OPTIMIZE_ODOMETRY_3D;
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<EquidistantCamera>(intrinsic_params, observed_p, optimize_flags));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p, optimize_flags));
            }
            break;
        case Camera::PINHOLE:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p, optimize_flags));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p, optimize_flags));
            }
            break;
        case Camera::MEI:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, observed_p, optimize_flags));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, observed_p, optimize_flags));
            }
            break;
        }
        break;
    case CAMERA_ODOMETRY_EXTRINSICS | ODOMETRY_6D_EXTRINSICS | POINT_3D:
        optimize_flags |= OPTIMIZE_ODOMETRY_6D;
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<EquidistantCamera>(intrinsic_params, observed_p, optimize_flags));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p, optimize_flags));
            }
            break;
        case Camera::PINHOLE:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p, optimize_flags));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, observed_p, optimize_flags));
            }
            break;
        case Camera::MEI:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, observed_p, optimize_flags));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, observed_p, optimize_flags));
            }
            break;
        }
        break;
    case CAMERA_INTRINSICS | CAMERA_ODOMETRY_EXTRINSICS | ODOMETRY_3D_EXTRINSICS | POINT_3D:
        optimize_flags |= OPTIMIZE_ODOMETRY_3D;
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 8, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<EquidistantCamera>(observed_p, optimize_flags));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p, optimize_flags));
            }
            break;
        case Camera::PINHOLE:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p, optimize_flags));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p, optimize_flags));
            }
            break;
        case Camera::MEI:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 9, 4, 3, 2, 1, 3>(
                        new ReprojectionError3<CataCamera>(observed_p, optimize_flags));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 9, 4, 2, 2, 1, 3>(
                        new ReprojectionError3<CataCamera>(observed_p, optimize_flags));
            }
            break;
        }
        break;
    case CAMERA_INTRINSICS | CAMERA_ODOMETRY_EXTRINSICS | ODOMETRY_6D_EXTRINSICS | POINT_3D:
        optimize_flags |= OPTIMIZE_ODOMETRY_6D;
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 8, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<EquidistantCamera>(observed_p, optimize_flags));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p, optimize_flags));
            }
            break;
        case Camera::PINHOLE:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p, optimize_flags));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 8, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(observed_p, optimize_flags));
            }
            break;
        case Camera::MEI:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 9, 4, 3, 3, 3, 3>(
                        new ReprojectionError3<CataCamera>(observed_p, optimize_flags));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 9, 4, 2, 3, 3, 3>(
                        new ReprojectionError3<CataCamera>(observed_p, optimize_flags));
            }
            break;
        }
        break;
    }

    return costFunction;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const CameraConstPtr& camera,
                                          const Eigen::Vector3d& odo_pos,
                                          const Eigen::Vector3d& odo_att,
                                          const Eigen::Vector2d& observed_p,
                                          int flags, bool optimize_cam_odo_z) const
{
    ceres::CostFunction* costFunction = 0;

    std::vector<double> intrinsic_params;
    camera->writeParameters(intrinsic_params);

    int optimize_flags = 0;
    if (optimize_cam_odo_z)
    {
        optimize_flags = OPTIMIZE_CAMERA_ODOMETRY_Z;
    }

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
                        new ReprojectionError3<EquidistantCamera>(intrinsic_params, odo_pos, odo_att, observed_p, optimize_flags));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 4, 2, 3>(
                        new ReprojectionError3<EquidistantCamera>(intrinsic_params, odo_pos, odo_att, observed_p, optimize_flags));
            }
            break;
        case Camera::PINHOLE:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 3, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, odo_pos, odo_att, observed_p, optimize_flags));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 4, 2, 3>(
                        new ReprojectionError3<PinholeCamera>(intrinsic_params, odo_pos, odo_att, observed_p, optimize_flags));
            }
            break;
        case Camera::MEI:
            if (optimize_cam_odo_z)
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 3, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, odo_pos, odo_att, observed_p, optimize_flags));
            }
            else
            {
                costFunction =
                    new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 4, 2, 3>(
                        new ReprojectionError3<CataCamera>(intrinsic_params, odo_pos, odo_att, observed_p, optimize_flags));
            }
            break;
        }
        break;
    }

    return costFunction;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const CameraConstPtr& camera,
                                          const Eigen::Quaterniond& cam_odo_q,
                                          const Eigen::Vector3d& cam_odo_t,
                                          const Eigen::Vector3d& odo_pos,
                                          const Eigen::Vector3d& odo_att,
                                          const Eigen::Vector2d& observed_p,
                                          int flags) const
{
    ceres::CostFunction* costFunction = 0;

    std::vector<double> intrinsic_params;
    camera->writeParameters(intrinsic_params);

    int optimize_flags = OPTIMIZE_CAMERA_ODOMETRY_Z | OPTIMIZE_ODOMETRY_6D;

    switch (flags)
    {
    case POINT_3D:
        switch (camera->modelType())
        {
        case Camera::KANNALA_BRANDT:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError3<EquidistantCamera>, 2, 3>(
                    new ReprojectionError3<EquidistantCamera>(intrinsic_params, cam_odo_q, cam_odo_t, odo_pos, odo_att, observed_p, optimize_flags));
            break;
        case Camera::PINHOLE:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError3<PinholeCamera>, 2, 3>(
                    new ReprojectionError3<PinholeCamera>(intrinsic_params, cam_odo_q, cam_odo_t, odo_pos, odo_att, observed_p, optimize_flags));
            break;
        case Camera::MEI:
            costFunction =
                new ceres::AutoDiffCostFunction<ReprojectionError3<CataCamera>, 2, 3>(
                    new ReprojectionError3<CataCamera>(intrinsic_params, cam_odo_q, cam_odo_t, odo_pos, odo_att, observed_p, optimize_flags));
            break;
        }
        break;
    }

    return costFunction;
}

ceres::CostFunction*
CostFunctionFactory::generateCostFunction(const CameraConstPtr& cameraL,
                                          const CameraConstPtr& cameraR,
                                          const Eigen::Vector3d& observed_P,
                                          const Eigen::Vector2d& observed_p_l,
                                          const Eigen::Vector2d& observed_p_r) const
{
    ceres::CostFunction* costFunction = 0;

    if (cameraL->modelType() != cameraR->modelType())
    {
        return costFunction;
    }

    switch (cameraL->modelType())
    {
    case Camera::KANNALA_BRANDT:
        costFunction =
            new ceres::AutoDiffCostFunction<StereoReprojectionError<EquidistantCamera>, 4, 8, 8, 4, 3, 4, 3>(
                new StereoReprojectionError<EquidistantCamera>(observed_P, observed_p_l, observed_p_r));
        break;
    case Camera::PINHOLE:
        costFunction =
            new ceres::AutoDiffCostFunction<StereoReprojectionError<PinholeCamera>, 4, 8, 8, 4, 3, 4, 3>(
                new StereoReprojectionError<PinholeCamera>(observed_P, observed_p_l, observed_p_r));
        break;
    case Camera::MEI:
        costFunction =
            new ceres::AutoDiffCostFunction<StereoReprojectionError<CataCamera>, 4, 9, 9, 4, 3, 4, 3>(
                new StereoReprojectionError<CataCamera>(observed_P, observed_p_l, observed_p_r));
        break;
    }

    return costFunction;
}

}

