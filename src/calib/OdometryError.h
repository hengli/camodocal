#ifndef ODOMETRYERROR_H
#define ODOMETRYERROR_H

#include "camodocal/EigenUtils.h"

namespace camodocal
{

class OdometryError
{
public:
    OdometryError(const Eigen::Matrix4d& meas_H_01)
     : m_meas_H_01(meas_H_01)
     , m_sqrtPrecisionMat(Eigen::Matrix3d::Identity())
    {

    }

    OdometryError(const Eigen::Matrix4d& meas_H_01,
                  const Eigen::Matrix3d& sqrtPrecisionMat)
     : m_meas_H_01(meas_H_01)
     , m_sqrtPrecisionMat(sqrtPrecisionMat)
    {

    }

    template <typename T>
    bool operator()(const T* const t0, const T* const r0,
                    const T* const t1, const T* const r1,
                    T* residuals) const
    {
        Eigen::Matrix<T,4,4> H0 = Eigen::Matrix<T,4,4>::Identity();
        H0.block(0,0,3,3) = RPY2mat(r0[2], r0[1], r0[0]);
        H0(0,3) = t0[0];
        H0(1,3) = t0[1];
        H0(2,3) = t0[2];

        Eigen::Matrix<T,4,4> H1 = Eigen::Matrix<T,4,4>::Identity();
        H1.block(0,0,3,3) = RPY2mat(r1[2], r1[1], r1[0]);
        H1(0,3) = t1[0];
        H1(1,3) = t1[1];
        H1(2,3) = t1[2];

        Eigen::Matrix<T,4,4> pred_H_01 = H1.inverse() * H0;

        Eigen::Matrix<T,4,4> err_H = m_meas_H_01.cast<T>() * pred_H_01.inverse();
        Eigen::Matrix<T,3,3> err_R = err_H.block(0,0,3,3);

        T roll, pitch, yaw;
        mat2RPY(err_R, roll, pitch, yaw);

        Eigen::Matrix<T,3,1> err;
        err << err_H(0,3), err_H(1,3), yaw;

        Eigen::Matrix<T,3,1> err_weighted = m_sqrtPrecisionMat.cast<T>() * err;

        residuals[0] = err_weighted(0);
        residuals[1] = err_weighted(1);
        residuals[2] = err_weighted(2);

        return true;
    }

private:
    Eigen::Matrix4d m_meas_H_01;
    Eigen::Matrix3d m_sqrtPrecisionMat;
};

}

#endif
