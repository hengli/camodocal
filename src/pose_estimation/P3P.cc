/*
 * Copyright (c) 2011, Laurent Kneip, ETH Zurich
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * P3P.cpp
 *
 *  Created on: Nov 2, 2010
 *      Author: Laurent Kneip
 * Description: Compute the absolute pose of a camera using three 3D-to-2D correspondences
 *   Reference: A Novel Parametrization of the P3P-Problem for a Direct Computation of
 *              Absolute Camera Position and Orientation
 *
 *       Input: featureVectors: vector of 3D UNITARY feature vectors
 *              worldPoints: vector of 3D world points
 *              solutions: vector of homogeneous transforms
 *                         the obtained transforms are defined as transforming points from the cam to the world frame
 *      Output: bool: true if correct execution
 *                    false if world points aligned
 */

#include "P3P.h"

#include <cmath>
#include <complex>
#include <iostream>
#include <cmath>

namespace Eigen
{
    typedef Matrix<double,5,1> Vector5d;
}

namespace camodocal
{

bool
solveQuartic(const Eigen::Vector5d& factors, Eigen::Vector4d& realRoots)
{
    double A = factors(0);
    double B = factors(1);
    double C = factors(2);
    double D = factors(3);
    double E = factors(4);

    double A_pw2 = A * A;
    double B_pw2 = B * B;
    double A_pw3 = A_pw2 * A;
    double B_pw3 = B_pw2 * B;
    double A_pw4 = A_pw3 * A;
    double B_pw4 = B_pw3 * B;

    double alpha = -3.0 * B_pw2 / (8.0 * A_pw2) + C / A;
    double beta = B_pw3 / (8.0 * A_pw3) - B * C / (2.0 * A_pw2) + D / A;
    double gamma = -3.0 * B_pw4 / (256.0 * A_pw4) + B_pw2 * C / (16.0 * A_pw3) - B * D / (4.0 * A_pw2) + E / A;

    double alpha_pw2 = alpha * alpha;
    double alpha_pw3 = alpha_pw2 * alpha;

    std::complex<double> P (-alpha_pw2 / 12.0 - gamma, 0);
    std::complex<double> Q (-alpha_pw3 / 108.0 + alpha * gamma / 3.0 - beta * beta / 8.0, 0);
    std::complex<double> R = -Q / 2.0 + sqrt(Q * Q / 4.0 + P * P * P / 27.0);

    std::complex<double> U = pow(R, 1.0 / 3.0);
    std::complex<double> y;

    if (U.real() == 0.0)
    {
        y = -5.0 * alpha / 6.0 - pow(Q, 1.0 / 3.0);
    }
    else
    {
        y = -5.0 * alpha / 6.0 - P / (3.0 * U) + U;
    }

    std::complex<double> w = sqrt(alpha + 2.0 * y);

    std::complex<double> temp;

    temp = -B / (4.0 * A) + 0.5 * (w + sqrt(-(3.0 * alpha + 2.0 * y + 2.0 * beta / w)));
    realRoots(0) = temp.real();
    temp = -B / (4.0 * A) + 0.5 * (w - sqrt(-(3.0 * alpha +2.0 * y + 2.0 * beta / w)));
    realRoots(1) = temp.real();
    temp = -B / (4.0 * A) + 0.5 * (-w + sqrt(-(3.0 * alpha + 2.0 * y - 2.0 * beta / w)));
    realRoots(2) = temp.real();
    temp = -B / (4.0 * A) + 0.5 * (-w - sqrt(-(3.0 * alpha + 2.0 * y - 2.0 * beta / w)));
    realRoots(3) = temp.real();

    return true;
}

bool
solveP3P(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& featureVectors,
         const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& worldPoints,
         std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& solutions)
{
    solutions.clear();
    
    // Extraction of world points

    Eigen::Vector3d P1 = worldPoints.at(0);
    Eigen::Vector3d P2 = worldPoints.at(1);
    Eigen::Vector3d P3 = worldPoints.at(2);

    // Verification that world points are not colinear

    Eigen::Vector3d temp1 = P2 - P1;
    Eigen::Vector3d temp2 = P3 - P1;

    if ((temp1.cross(temp2)).norm() == 0.0)
    {
        return false;
    }

    // Extraction of feature vectors

    Eigen::Vector3d f1 = featureVectors.at(0);
    Eigen::Vector3d f2 = featureVectors.at(1);
    Eigen::Vector3d f3 = featureVectors.at(2);

    // Creation of intermediate camera frame

    Eigen::Vector3d e1 = f1;
    Eigen::Vector3d e3 = f1.cross(f2);
    e3.normalize();
    Eigen::Vector3d e2 = e3.cross(e1);

    Eigen::Matrix3d T;
    T.row(0) = e1;
    T.row(1) = e2;
    T.row(2) = e3;

    f3 = T * f3;

    // Reinforce that f3(2) > 0 for having theta in [0;pi]

    if (f3(2) > 0.0)
    {
        f1 = featureVectors.at(1);
        f2 = featureVectors.at(0);
        f3 = featureVectors.at(2);

        e1 = f1;
        e3 = f1.cross(f2);
        e3.normalize();
        e2 = e3.cross(e1);

        T.row(0) = e1;
        T.row(1) = e2;
        T.row(2) = e3;

        f3 = T * f3;

        P1 = worldPoints.at(1);
        P2 = worldPoints.at(0);
        P3 = worldPoints.at(2);
    }

    // Creation of intermediate world frame

    Eigen::Vector3d n1 = P2 - P1;
    n1.normalize();
    Eigen::Vector3d n3 = n1.cross(P3 - P1);
    n3.normalize();
    Eigen::Vector3d n2 = n3.cross(n1);

    Eigen::Matrix3d N;
    N.row(0) = n1;
    N.row(1) = n2;
    N.row(2) = n3;

    // Extraction of known parameters

    P3 = N * (P3 - P1);

    double d_12 = (P2 - P1).norm();
    double f_1 = f3(0) / f3(2);
    double f_2 = f3(1) / f3(2);
    double p_1 = P3(0);
    double p_2 = P3(1);

    double cos_beta = f1.dot(f2);
    double b = 1.0 / (1.0 - cos_beta * cos_beta) - 1.0;

    if (cos_beta < 0.0)
    {
        b = -sqrt(b);
    }
    else
    {
        b = sqrt(b);
    }

    // Definition of temporary variables for avoiding multiple computation

    double f_1_pw2 = f_1 * f_1;
    double f_2_pw2 = f_2 * f_2;
    double p_1_pw2 = p_1 * p_1;
    double p_1_pw3 = p_1_pw2 * p_1;
    double p_1_pw4 = p_1_pw3 * p_1;
    double p_2_pw2 = p_2 * p_2;
    double p_2_pw3 = p_2_pw2 * p_2;
    double p_2_pw4 = p_2_pw3 * p_2;
    double d_12_pw2 = d_12 * d_12;
    double b_pw2 = b * b;

    // Computation of factors of 4th degree polynomial

    Eigen::Vector5d factors;

    factors(0) = - f_2_pw2 * p_2_pw4
                 - p_2_pw4 * f_1_pw2
                 - p_2_pw4;

    factors(1) = 2.0 * p_2_pw3 * d_12 * b
                 + 2.0 * f_2_pw2 * p_2_pw3 * d_12 * b
                 - 2.0 * f_2 * p_2_pw3 * f_1 * d_12;

    factors(2) = - f_2_pw2 * p_2_pw2 * p_1_pw2
                 - f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2
                 - f_2_pw2 * p_2_pw2 * d_12_pw2
                 + f_2_pw2 * p_2_pw4
                 + p_2_pw4 * f_1_pw2
                 + 2.0 * p_1 * p_2_pw2 * d_12
                 + 2.0 * f_1 * f_2 * p_1 * p_2_pw2 * d_12 * b
                 - p_2_pw2 * p_1_pw2 * f_1_pw2
                 + 2.0 * p_1 * p_2_pw2 * f_2_pw2 * d_12
                 - p_2_pw2 * d_12_pw2 * b_pw2
                 - 2.0 * p_1_pw2 * p_2_pw2;

    factors(3) = 2.0 * p_1_pw2 * p_2 * d_12 * b
                 + 2.0 * f_2 * p_2_pw3 * f_1 * d_12
                 - 2.0 * f_2_pw2 * p_2_pw3 * d_12 * b
                 - 2.0 * p_1 * p_2 * d_12_pw2 * b;

    factors(4) = - 2.0 * f_2 * p_2_pw2 * f_1 * p_1 * d_12 * b
                 + f_2_pw2 * p_2_pw2 * d_12_pw2
                 + 2.0 * p_1_pw3 * d_12
                 - p_1_pw2 * d_12_pw2
                 + f_2_pw2 * p_2_pw2 * p_1_pw2
                 - p_1_pw4
                 - 2.0 * f_2_pw2 * p_2_pw2 * p_1 * d_12
                 + p_2_pw2 * f_1_pw2 * p_1_pw2
                 + f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2;

    // Computation of roots

    Eigen::Vector4d realRoots;

    solveQuartic(factors, realRoots);

    // Backsubstitution of each solution

    for (int i = 0; i < 4; ++i)
    {
        if (fabs(realRoots(i)) > 1.0)
        {
            continue;
        }

        double cot_alpha = (-f_1 * p_1 / f_2 - realRoots(i) * p_2 + d_12 * b ) / (- f_1 * realRoots(i) * p_2 / f_2 + p_1 - d_12);

        double cos_theta = realRoots(i);
        double sin_theta = sqrt(1.0 - realRoots(i) * realRoots(i));
        double sin_alpha = sqrt(1.0 / (cot_alpha * cot_alpha + 1.0));
        double cos_alpha = sqrt(1.0 - sin_alpha * sin_alpha);

        if (cot_alpha < 0.0)
        {
            cos_alpha = -cos_alpha;
        }

        Eigen::Vector3d C;
        C << d_12 * cos_alpha * (sin_alpha * b + cos_alpha),
             cos_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha),
             sin_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha);

        Eigen::Matrix3d R;
        R <<  -cos_alpha,  -sin_alpha * cos_theta,  -sin_alpha * sin_theta,
               sin_alpha,  -cos_alpha * cos_theta,  -cos_alpha * sin_theta,
               0.0,        -sin_theta,               cos_theta;

        Eigen::Matrix4d H;
        H.setIdentity();
        H.block<3,3>(0,0) = N.transpose() * R.transpose() * T;
        H.block<3,1>(0,3) = P1 + N.transpose() * C;

        // TODO: Find out why nan element occurs (rare occurrence).
        bool valid = true;
        for (int j = 0; j < 3; ++j)
        {
            for (int k = 0; k < 4; ++k)
            {
                if (std::isnan(H(j,k)))
                {
                    valid = false;

                    break;
                }
            }

            if (!valid)
            {
                break;
            }
        }

        if (valid)
        {
            solutions.push_back(H);
        }
    }

    return true;
}

}
