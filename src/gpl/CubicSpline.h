#ifndef CUBICSPLINE_H
#define CUBICSPLINE_H

#include <Eigen/Eigen>

namespace camodocal
{

/**
 * This class interpolates between two points using a cubic spline.
 */
class CubicSpline
{
public:
    /**
     * Constructor
     */
    CubicSpline();

    /**
     * This function returns the first derivative at distance t.
     * @param t the percentage distance from point 1 to point 2
     */
    Eigen::Vector2d derivEval(double t) const;

    /**
     * This function returns the second derivative at distance t.
     * @param t the percentage distance from point 1 to point 2
     */
    Eigen::Vector2d deriv2Eval(double t) const;

    /**
     * This function returns the point given the distance t.
     * @param t the percentage distance from point 1 to point 2
     */
    Eigen::Vector2d eval(double t) const;

    /**
     * Interpolate between point 1 and point 2
     * @param x1          x-coordinate of point 1
     * @param y1          y-coordinate of point 1
     * @param theta1      heading of point 1
     * @param x2          x-coordinate of point 2
     * @param y2          y-coordinate of point 2
     * @param theta2      heading of point 2
     */
    void fit(const Eigen::Vector2d& p1, double theta1,
             const Eigen::Vector2d& p2, double theta2);

    /**
     * Calculate the distance between a segment.
     * @param t1          start segment from point 1
     * @param t2          end segment from point 1
     */
    double length(double t1, double t2);

    /**
     * Return the ith coefficient.
     * @param i           coefficient index
     * @return            ith coefficient
     */
    Eigen::Vector2d p(int i) const;

private:
    Eigen::Vector2d m_p[4];
};

}

#endif
