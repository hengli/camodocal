#ifndef CATAREPROJECTIONERROR_H
#define CATAREPROJECTIONERROR_H

#include "ceres/rotation.h"

namespace camodocal
{

class CataReprojectionError: public ceres::SizedCostFunction<2, 9, 4, 3>
{
public:
    CataReprojectionError(double point_x, double point_y, double point_z,
                          double observed_x, double observed_y)
        : m_point_x(point_x), m_point_y(point_y), m_point_z(point_z),
          m_observed_x(observed_x), m_observed_y(observed_y) {}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const
    {
        const double* const intrinsicCameraParams = parameters[0];
        const double* const extrinsicCameraQuat = parameters[1];
        const double* const extrinsicCameraTrans = parameters[2];

        double point[3];
        point[0] = double(m_point_x);
        point[1] = double(m_point_y);
        point[2] = double(m_point_z);

        double q[4];
        memcpy(q, extrinsicCameraQuat, 4 * sizeof(double));
        double q_len2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];

        double P[3];
        ceres::QuaternionRotatePoint(extrinsicCameraQuat, point, P);

        double t[3];
        memcpy(t, extrinsicCameraTrans, 3 * sizeof(double));

        P[0] += t[0];
        P[1] += t[1];
        P[2] += t[2];

        // project 3D object point to the image plane
        double xi = intrinsicCameraParams[0];
        double k1 = intrinsicCameraParams[1];
        double k2 = intrinsicCameraParams[2];
        double p1 = intrinsicCameraParams[3];
        double p2 = intrinsicCameraParams[4];
        double gamma1 = intrinsicCameraParams[5];
        double gamma2 = intrinsicCameraParams[6];
        double alpha = 0.0; //cameraParams.alpha();
        double u0 = intrinsicCameraParams[7];
        double v0 = intrinsicCameraParams[8];

        // transform to model plane
        double p[3];
        double p_len2 = square(P[0]) + square(P[1]) + square(P[2]);
        double p_len = sqrt(p_len2);
        p[0] = P[0] / p_len;
        p[1] = P[1] / p_len;
        p[2] = P[2] / p_len;

        double u = p[0] / (p[2] + xi);
        double v = p[1] / (p[2] + xi);

        double rho_sqr = square(u) + square(v);
        double L = 1.0 + k1 * rho_sqr + k2 * square(rho_sqr);
        double du = 2.0 * p1 * u * v + p2 * (rho_sqr + 2.0 * square(u));
        double dv = p1 * (rho_sqr + 2.0 * square(v)) + 2.0 * p2 * u * v;

        double u1 = L * u + du;
        double v1 = L * v + dv;
        double x = gamma1 * (u1 + alpha * v1) + u0;
        double y = gamma2 * v1 + v0;

        residuals[0] = x - m_observed_x;
        residuals[1] = y - m_observed_y;

        if (jacobians != 0 && jacobians[0] != 0 && jacobians[1] != 0 && jacobians[2] != 0)
        {
            // camera intrinsics

            // d(P)/d(xi)
            jacobians[0][0] = gamma1*p[0]*(-4.0*p2*u-4.0*p1*v-L)/square(p[2]+xi);
            jacobians[0][9] = gamma2*p[1]*(-4.0*p2*u-4.0*p1*v-L)/square(p[2]+xi);

            // d(P)/d(k1)
            jacobians[0][1] = gamma1*u*rho_sqr;
            jacobians[0][10] = gamma2*v*rho_sqr;

            // d(P)/d(k2)
            jacobians[0][2] = gamma1*u*square(rho_sqr);
            jacobians[0][11] = gamma2*v*square(rho_sqr);

            // d(P)/d(p1)
            jacobians[0][3] = 2.0*gamma1*u*v;
            jacobians[0][12] = gamma2*(2.0*square(v)+rho_sqr);

            // d(P)/d(p2)
            jacobians[0][4] = gamma1*(2.0*square(u)+rho_sqr);
            jacobians[0][13] = 2.0*gamma2*u*v;

            // d(P)/d(gamma1)
            jacobians[0][5] = u1;
            jacobians[0][14] = 0.0;

            // d(P)/d(gamma2)
            jacobians[0][6] = 0.0;
            jacobians[0][15] = v1;

            // d(P)/d(u0)
            jacobians[0][7] = 1.0;
            jacobians[0][16] = 0.0;

            // d(P)/d(v0)
            jacobians[0][8] = 0.0;
            jacobians[0][17] = 1.0;

            // camera extrinsics

            double J_H_rw[3];
            J_H_rw[0] = (2.0*(2.0*point[0]*q[0]*(square(q[2]) + square(q[3])) +
                        point[2]*(-(square(q[0])*q[2]) - 2.0*q[0]*q[1]*q[3] +
                        q[2]*(1 - square(q[0]))) +
                        point[1]*(-2.0*q[0]*q[1]*q[2] + square(q[0])*q[3] -
                        q[3]*(1 - square(q[0]))))) / q_len2;

            J_H_rw[1] = (2.0*(2.0*point[1]*q[0]*(square(q[1]) + square(q[3])) +
                        point[2]*(square(q[0])*q[1] - 2.0*q[0]*q[2]*q[3] -
                        q[1]*(1 - square(q[0]))) +
                        point[0]*(-2.0*q[0]*q[1]*q[2] - square(q[0])*q[3] +
                        q[3]*(1 - square(q[0]))))) / q_len2;

            J_H_rw[2] = (2.0*(2.0*point[2]*q[0]*(square(q[1]) + square(q[2])) +
                        point[1]*(-(square(q[0])*q[1]) - 2.0*q[0]*q[2]*q[3] +
                        q[1]*(1 - square(q[0]))) +
                        point[0]*(square(q[0])*q[2] - 2.0*q[0]*q[1]*q[3] -
                        q[2]*(1 - square(q[0]))))) / q_len2;

            double J_H_rx[3];
            J_H_rx[0] = (2.0*(2.0*point[0]*q[1]*(square(q[2]) + square(q[3])) +
                        point[2]*(-2.0*q[0]*q[1]*q[2] + (square(q[0]) - square(q[1]) + square(q[2]))*q[3] +
                        cube(q[3])) + point[1]*
                        (square(q[0])*q[2] + 2.0*q[0]*q[1]*q[3] +
                        q[2]*(-square(q[1]) + square(q[2]) + square(q[3]))))) / q_len2;

            J_H_rx[1] = (2.0*(-2.0*q[1]*(-(point[2]*q[0]*q[1]) + point[0]*q[1]*q[2] +
                           point[1]*(square(q[0]) + square(q[2])) + point[0]*q[0]*q[3] + point[2]*q[2]*q[3])\
                           - (point[2]*q[0] - point[0]*q[2])*
                           (square(q[0]) + square(q[1]) + square(q[2]) + square(q[3])))) / q_len2;

            J_H_rx[2] = (2.0*(-2.0*point[2]*q[1]*(square(q[0]) + square(q[3])) +
                        point[0]*(2.0*q[0]*q[1]*q[2] + (square(q[0]) - square(q[1]) + square(q[2]))*q[3] +
                        cube(q[3])) + point[1]*
                        (cube(q[0]) - 2.0*q[1]*q[2]*q[3] +
                        q[0]*(-square(q[1]) + square(q[2]) + square(q[3]))))) / q_len2;

            double J_H_ry[3];
            J_H_ry[0] = (2.0*(-2.0*point[0]*(square(q[0]) + square(q[1]))*q[2] +
                        point[1]*(square(q[0])*q[1] + cube(q[1]) - q[1]*square(q[2]) + 2.0*q[0]*q[2]*q[3] +
                        q[1]*square(q[3])) + point[2]*
                        (cube(q[0]) - 2.0*q[1]*q[2]*q[3] +
                        q[0]*(square(q[1]) - square(q[2]) + square(q[3]))))) / q_len2;

            J_H_ry[1] = (2.0*(2.0*point[1]*q[2]*(square(q[1]) + square(q[3])) +
                        point[2]*(2.0*q[0]*q[1]*q[2] + (square(q[0]) + square(q[1]) - square(q[2]))*q[3] +
                        cube(q[3])) + point[0]*
                        (square(q[0])*q[1] - 2.0*q[0]*q[2]*q[3] +
                        q[1]*(square(q[1]) - square(q[2]) + square(q[3]))))) / q_len2;

            J_H_ry[2] = (-4.0*point[2]*q[2]*(square(q[0]) + square(q[3])) +
                        2.0*point[1]*(-2.0*q[0]*q[1]*q[2] + (square(q[0]) + square(q[1]) - square(q[2]))*q[3] +
                        cube(q[3])) - 2.0*point[0]*
                        (cube(q[0]) + 2.0*q[1]*q[2]*q[3] +
                        q[0]*(square(q[1]) - square(q[2]) + square(q[3])))) / q_len2;

            double J_H_rz[3];
            J_H_rz[0] = (-2.0*(point[1]*q[0] - point[2]*q[1])*(1-square(q[3])) -
                        4.0*(point[0]*(square(q[0]) + square(q[1])) + (point[2]*q[0] + point[1]*q[1])*q[2])*
                        q[3] + 2.0*(point[1]*q[0] - point[2]*q[1])*square(q[3])) / q_len2;

            J_H_rz[1] = (2.0*(point[0]*q[0] + point[2]*q[2])*(1-square(q[3])) -
                        4.0*(-(point[2]*q[0]*q[1]) + point[0]*q[1]*q[2] +
                        point[1]*(square(q[0]) + square(q[2])))*q[3] -
                        2.0*(point[0]*q[0] + point[2]*q[2])*square(q[3])) / q_len2;

            J_H_rz[2] = (2.0*(point[0]*q[1] + point[1]*q[2])*(1-square(q[3])) +
                        4.0*(-(point[1]*q[0]*q[1]) + point[0]*q[0]*q[2] +
                        point[2]*(square(q[1]) + square(q[2])))*q[3] -
                        2.0*(point[0]*q[1] + point[1]*q[2])*square(q[3])) / q_len2;

            double J_P_x[2];
            J_P_x[0] = gamma1/p_len*(4.0*p1*square(u)*v*p[2]-4.0*p1*square(u)*p[1]+2.0*p1*v/(p[2]+xi)+p2*(4.0*cube(u)*p[2]-4.0*p[0]*square(u)+4.0*u/(p[2]+xi))+square(u)*p[2]*L-p[0]*u*L+L/(p[2]+xi));
            J_P_x[1] = gamma2/p_len*(4.0*p2*square(u)*v*p[2]-4.0*p2*square(u)*p[1]+2.0*p2*v/(p[2]+xi)+p1*(4.0*u*square(v)*p[2]-4.0*p[0]*square(v))+u*v*p[2]*L-p[0]*v*L);

            double J_P_y[2];
            J_P_y[0] = gamma1/p_len*(4.0*p1*u*square(v)*p[2]-4.0*p1*p[0]*square(v)+2.0*p1*u/(p[2]+xi)+p2*(4.0*square(u)*v*p[2]-4.0*square(u)*p[1])+u*v*p[2]*L-u*p[1]*L);
            J_P_y[1] = gamma2/p_len*(4.0*p2*u*square(v)*p[2]-4.0*p2*square(v)*p[0]+2.0*p2*u/(p[2]+xi)+p1*(4.0*cube(v)*p[2]-4.0*p[1]*square(v)+4.0*v/(p[2]+xi))+square(v)*p[2]*L-p[1]*v*L+L/(p[2]+xi));

            double J_P_z[2];
            J_P_z[0] = -gamma1*1.0/pow(P[2]+xi*sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2]),6.0)*(P[2]*xi+sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2]))*1.0/sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*(P[0]*(P[2]*P[2]*P[2]*P[2])+(P[0]*P[0]*P[0]*P[0]*P[0])*k2*5.0+(P[0]*P[0]*P[0])*(P[1]*P[1])*k2*1.0E1+(P[0]*P[0]*P[0])*(P[2]*P[2])*k1*3.0+(P[0]*P[0])*(P[2]*P[2]*P[2])*p2*6.0+(P[1]*P[1])*(P[2]*P[2]*P[2])*p2*2.0+P[0]*(xi*xi*xi*xi)*pow(P[0]*P[0]+P[1]*P[1]+P[2]*P[2],2.0)+P[0]*(P[1]*P[1]*P[1]*P[1])*k2*5.0+P[0]*P[1]*(P[2]*P[2]*P[2])*p1*4.0+(P[0]*P[0])*p2*(xi*xi*xi)*pow(P[0]*P[0]+P[1]*P[1]+P[2]*P[2],3.0/2.0)*6.0+(P[1]*P[1])*p2*(xi*xi*xi)*pow(P[0]*P[0]+P[1]*P[1]+P[2]*P[2],3.0/2.0)*2.0+P[0]*(P[1]*P[1])*(P[2]*P[2])*k1*3.0+P[0]*(P[2]*P[2])*(xi*xi)*(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*6.0+P[0]*(P[2]*P[2]*P[2])*xi*sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*4.0+P[0]*P[2]*(xi*xi*xi)*pow(P[0]*P[0]+P[1]*P[1]+P[2]*P[2],3.0/2.0)*4.0+(P[0]*P[0]*P[0])*k1*(xi*xi)*(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*3.0+P[0]*(P[1]*P[1])*k1*(xi*xi)*(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*3.0+(P[0]*P[0]*P[0])*P[2]*k1*xi*sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*6.0+(P[0]*P[0])*P[2]*p2*(xi*xi)*(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*1.8E1+(P[1]*P[1])*P[2]*p2*(xi*xi)*(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*6.0+P[0]*P[1]*p1*(xi*xi*xi)*pow(P[0]*P[0]+P[1]*P[1]+P[2]*P[2],3.0/2.0)*4.0+(P[0]*P[0])*(P[2]*P[2])*p2*xi*sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*1.8E1+(P[1]*P[1])*(P[2]*P[2])*p2*xi*sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*6.0+P[0]*P[1]*P[2]*p1*(xi*xi)*(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*1.2E1+P[0]*(P[1]*P[1])*P[2]*k1*xi*sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*6.0+P[0]*P[1]*(P[2]*P[2])*p1*xi*sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*1.2E1);
            J_P_z[1] = -gamma2*1.0/pow(P[2]+xi*sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2]),6.0)*(P[2]*xi+sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2]))*1.0/sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*(P[1]*(P[2]*P[2]*P[2]*P[2])+(P[1]*P[1]*P[1]*P[1]*P[1])*k2*5.0+(P[0]*P[0])*(P[1]*P[1]*P[1])*k2*1.0E1+(P[1]*P[1]*P[1])*(P[2]*P[2])*k1*3.0+(P[0]*P[0])*(P[2]*P[2]*P[2])*p1*2.0+(P[1]*P[1])*(P[2]*P[2]*P[2])*p1*6.0+P[1]*(xi*xi*xi*xi)*pow(P[0]*P[0]+P[1]*P[1]+P[2]*P[2],2.0)+(P[0]*P[0]*P[0]*P[0])*P[1]*k2*5.0+P[0]*P[1]*(P[2]*P[2]*P[2])*p2*4.0+(P[0]*P[0])*p1*(xi*xi*xi)*pow(P[0]*P[0]+P[1]*P[1]+P[2]*P[2],3.0/2.0)*2.0+(P[1]*P[1])*p1*(xi*xi*xi)*pow(P[0]*P[0]+P[1]*P[1]+P[2]*P[2],3.0/2.0)*6.0+(P[0]*P[0])*P[1]*(P[2]*P[2])*k1*3.0+P[1]*(P[2]*P[2])*(xi*xi)*(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*6.0+P[1]*(P[2]*P[2]*P[2])*xi*sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*4.0+P[1]*P[2]*(xi*xi*xi)*pow(P[0]*P[0]+P[1]*P[1]+P[2]*P[2],3.0/2.0)*4.0+(P[1]*P[1]*P[1])*k1*(xi*xi)*(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*3.0+(P[0]*P[0])*P[1]*k1*(xi*xi)*(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*3.0+(P[1]*P[1]*P[1])*P[2]*k1*xi*sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*6.0+(P[0]*P[0])*P[2]*p1*(xi*xi)*(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*6.0+(P[1]*P[1])*P[2]*p1*(xi*xi)*(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*1.8E1+P[0]*P[1]*p2*(xi*xi*xi)*pow(P[0]*P[0]+P[1]*P[1]+P[2]*P[2],3.0/2.0)*4.0+(P[0]*P[0])*(P[2]*P[2])*p1*xi*sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*6.0+(P[1]*P[1])*(P[2]*P[2])*p1*xi*sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*1.8E1+P[0]*P[1]*P[2]*p2*(xi*xi)*(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*1.2E1+(P[0]*P[0])*P[1]*P[2]*k1*xi*sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*6.0+P[0]*P[1]*(P[2]*P[2])*p2*xi*sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2])*1.2E1);

            // d(P)/d(q(0))
            jacobians[1][0] = J_P_x[0]*J_H_rw[0] + J_P_y[0]*J_H_rw[1] + J_P_z[0]*J_H_rw[2];
            jacobians[1][4] = J_P_x[1]*J_H_rw[0] + J_P_y[1]*J_H_rw[1] + J_P_z[1]*J_H_rw[2];

            // d(P)/d(q(1))
            jacobians[1][1] = J_P_x[0]*J_H_rx[0] + J_P_y[0]*J_H_rx[1] + J_P_z[0]*J_H_rx[2];
            jacobians[1][5] = J_P_x[1]*J_H_rx[0] + J_P_y[1]*J_H_rx[1] + J_P_z[1]*J_H_rx[2];

            // d(P)/d(q(2))
            jacobians[1][2] = J_P_x[0]*J_H_ry[0] + J_P_y[0]*J_H_ry[1] + J_P_z[0]*J_H_ry[2];
            jacobians[1][6] = J_P_x[1]*J_H_ry[0] + J_P_y[1]*J_H_ry[1] + J_P_z[1]*J_H_ry[2];

            // d(P)/d(q(3))
            jacobians[1][3] = J_P_x[0]*J_H_rz[0] + J_P_y[0]*J_H_rz[1] + J_P_z[0]*J_H_rz[2];
            jacobians[1][7] = J_P_x[1]*J_H_rz[0] + J_P_y[1]*J_H_rz[1] + J_P_z[1]*J_H_rz[2];

            // d(P)/d(t(1))
            jacobians[2][0] = J_P_x[0];
            jacobians[2][3] = J_P_x[1];

            // d(P)/d(t(2))
            jacobians[2][1] = J_P_y[0];
            jacobians[2][4] = J_P_y[1];

            // d(P)/d(t(3))
            jacobians[2][2] = J_P_z[0];
            jacobians[2][5] = J_P_z[1];
        }

        return true;
    }

    template <typename T>
    bool operator()(const T* const intrinsicCameraParams,
                    const T* const extrinsicCameraQuat,
                    const T* const extrinsicCameraTrans,
                    T* residuals) const
    {
        T point[3];
        point[0] = T(m_point_x);
        point[1] = T(m_point_y);
        point[2] = T(m_point_z);

        T p[3];
        ceres::QuaternionRotatePoint(extrinsicCameraQuat, point, p);

        p[0] += extrinsicCameraTrans[4];
        p[1] += extrinsicCameraTrans[5];
        p[2] += extrinsicCameraTrans[6];

        // project 3D object point to the image plane
        T xi = intrinsicCameraParams[0];
        T k1 = intrinsicCameraParams[1];
        T k2 = intrinsicCameraParams[2];
        T p1 = intrinsicCameraParams[3];
        T p2 = intrinsicCameraParams[4];
        T gamma1 = intrinsicCameraParams[5];
        T gamma2 = intrinsicCameraParams[6];
        T alpha = T(0); //cameraParams.alpha();
        T u0 = intrinsicCameraParams[7];
        T v0 = intrinsicCameraParams[8];

        // Transform to model plane
        T len = sqrt(square(p[0]) + square(p[1]) + square(p[2]));
        p[0] /= len;
        p[1] /= len;
        p[2] /= len;

        T u = p[0] / (p[2] + xi);
        T v = p[1] / (p[2] + xi);

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
    double m_point_x;
    double m_point_y;
    double m_point_z;
    double m_observed_x;
    double m_observed_y;
};

}

#endif
