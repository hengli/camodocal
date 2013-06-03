#include "CubicSpline.h"

#include <cmath>

namespace camodocal
{

CubicSpline::CubicSpline()
{
    for (int i = 0; i < 4; ++i)
    {
        m_p[i].setZero();
    }
}

Eigen::Vector2d
CubicSpline::derivEval(double t) const
{
    return m_p[1] + 2.0 * m_p[2] * t + 3.0 * m_p[3] * t * t;
}

Eigen::Vector2d
CubicSpline::deriv2Eval(double t) const
{
    return 2.0 * m_p[2] + 6.0 * m_p[3] * t;
}

Eigen::Vector2d
CubicSpline::eval(double t) const
{
    Eigen::Vector2d point(0.0, 0.0);
    double tpow = 1.0;

    for (int i = 0; i < 4; ++i)
    {
        point += m_p[i] * tpow;
        tpow *= t;
    }

    return point;
}

void
CubicSpline::fit(const Eigen::Vector2d& p1, double theta1,
                 const Eigen::Vector2d& p2, double theta2)
{
    Eigen::Vector2d dp = p2 - p1;

    double k = dp.norm();

    double kc1 = k * cos(theta1);
    double ks1 = k * sin(theta1);
    double kc2 = k * cos(theta2);
    double ks2 = k * sin(theta2);

    m_p[0] = p1;
    m_p[1] << kc1, ks1;
    m_p[2] << 3.0 * dp(0) - 2.0 * kc1 - kc2,
              3.0 * dp(1) - 2.0 * ks1 - ks2;
    m_p[3] << -2.0 * dp(0) + kc1 + kc2,
              -2.0 * dp(1) + ks1 + ks2;
}

double
CubicSpline::length(double t1, double t2)
{
    Eigen::Vector2d last_p(0.0, 0.0);
    double d = 0;

    for (int i = 0; i < 100; ++i)
    {
        Eigen::Vector2d p = eval(t1 + i / 100.0 * (t2 - t1));

        if (i > 0)
        {
            d += (p - last_p).norm();
        }

        last_p = p;
    }

    return d;
}

Eigen::Vector2d
CubicSpline::p(int i) const
{
    return m_p[i];
}

}
