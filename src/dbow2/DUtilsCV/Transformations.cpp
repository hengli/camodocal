/**
 * File: Transformations.cpp
 * Project: DUtilsCV library
 * Author: Dorian Galvez-Lopez
 * Date: October, 2010
 * Description: Functions to operate with transformation matrices
 *
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "Transformations.h"
#include "Types.h"
#include <opencv/cv.h>

#ifdef HAVE_OPENCV3
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#endif

using namespace DUtilsCV;

// ----------------------------------------------------------------------------

cv::Mat Transformations::rotx(double theta, double X, double Y, double Z)
{
  double c = cos(theta);
  double s = sin(theta);
  
  return (cv::Mat_<double>(4,4) <<
    1,  0,  0,  X,
    0,  c, -s,  Y,
    0,  s,  c,  Z,
    0,  0,  0,  1);
}

// ----------------------------------------------------------------------------

cv::Mat Transformations::roty(double theta, double X, double Y, double Z)
{
  double c = cos(theta);
  double s = sin(theta);
  
  return (cv::Mat_<double>(4,4) <<
    c,  0,  s,  X,
    0,  1,  0,  Y,
   -s,  0,  c,  Z,
    0,  0,  0,  1);
}

// ----------------------------------------------------------------------------

cv::Mat Transformations::rotz(double theta, double X, double Y, double Z)
{
  double c = cos(theta);
  double s = sin(theta);
  
  return (cv::Mat_<double>(4,4) <<
    c, -s,  0,  X,
    s,  c,  0,  Y,
    0,  0,  1,  Z,
    0,  0,  0,  1);
}

// ----------------------------------------------------------------------------

cv::Mat Transformations::transl(double X, double Y, double Z)
{
  return (cv::Mat_<double>(4,4) <<
    1, 0, 0, X,
    0, 1, 0, Y,
    0, 0, 1, Z,
    0, 0, 0, 1);
}

// ----------------------------------------------------------------------------

cv::Mat Transformations::rotvec(const cv::Mat &axis, double theta)
{
  double norm = cv::norm(axis);
  const cv::Mat ax = axis/norm;
  
  const double c = cos(theta);
  const double s = sin(theta);
  const double v = 1 - c;
    
  double r[3];
  Types::vectorize(ax, r);
  
  /*
  if(ax.type() == CV_32F)
  {
    if(ax.rows > 1)
    {
      for(int i = 0; i < 3; ++i) r[i] = ax.at<float>(i,0);
    }else
    {
      for(int i = 0; i < 3; ++i) r[i] = ax.at<float>(0,i);
    }
  }
  else if(ax.type() == CV_64F)
  {
    if(ax.rows > 1)
    {
      for(int i = 0; i < 3; ++i) r[i] = ax.at<double>(i,0);
    }else
    {
      for(int i = 0; i < 3; ++i) r[i] = ax.at<double>(0,i);
    }
  }
  else
  {
    // copy and convert to double
    Types::vectorize(ax, r);
  }
  */
  
  const double xxv = r[0]*r[0]*v;
  const double xyv = r[0]*r[1]*v;
  const double xzv = r[0]*r[2]*v;
  const double yzv = r[1]*r[2]*v;
  
  const double yyv = r[1]*r[1]*v;
  const double zzv = r[2]*r[2]*v;
  
  const double xs = r[0]*s;
  const double ys = r[1]*s;
  const double zs = r[2]*s;
  
  return (cv::Mat_<double>(4,4) <<
    xxv+c,    xyv-zs,   xzv+ys,   0,
    xyv+zs,   yyv+c,    yzv-xs,   0,
    xzv-ys,   yzv+xs,   zzv+c,    0,
    0,        0,        0,        1);
}

// ----------------------------------------------------------------------------

/*
// Needs checking
cv::Mat Transformations::euler(double zphi, double ytheta, double zpsi,
  double tx, double ty, double tz)
{
  
  const double cph = cos(zphi);
  const double sph = cos(zphi);
  const double ct = cos(ytheta);
  const double st = cos(ytheta);
  const double cps = cos(zpsi);
  const double sps = cos(zpsi);
  
  return (cv::Mat_<double>(4,4) <<
    cph*cps*ct-sph*sps,   -cps*sph-cph*ct*sps,  cph*st,   tx,
    cph*sps+cps*ct*sph,    cph*cps-ct*sph*sps,  sph*st,   ty,
               -cps*st,                sps*st,      ct,   tz,
                     0,                     0,       0,     1);
}
*/

// ----------------------------------------------------------------------------

cv::Mat Transformations::inv(const cv::Mat &aTb)
{
  // inv(T) = [R^t | -R^t p]
  const cv::Mat R = aTb.rowRange(0,3).colRange(0,3);
  const cv::Mat t = aTb.rowRange(0,3).colRange(3,4);
 
  cv::Mat Rt = R.t();
  cv::Mat t2 = -Rt*t;
  
  cv::Mat ret;
  if(aTb.type() == CV_32F)
  {
    ret = (cv::Mat_<float>(4,4) <<
      Rt.at<float>(0,0), Rt.at<float>(0,1), Rt.at<float>(0,2), t2.at<float>(0,0),
      Rt.at<float>(1,0), Rt.at<float>(1,1), Rt.at<float>(1,2), t2.at<float>(1,0),
      Rt.at<float>(2,0), Rt.at<float>(2,1), Rt.at<float>(2,2), t2.at<float>(2,0),
      0, 0, 0, 1);
  
  }else
  {
    ret = (cv::Mat_<double>(4,4) <<
      Rt.at<double>(0,0), Rt.at<double>(0,1), Rt.at<double>(0,2), t2.at<double>(0,0),
      Rt.at<double>(1,0), Rt.at<double>(1,1), Rt.at<double>(1,2), t2.at<double>(1,0),
      Rt.at<double>(2,0), Rt.at<double>(2,1), Rt.at<double>(2,2), t2.at<double>(2,0),
      0, 0, 0, 1);
  }

  return ret;
}

// ----------------------------------------------------------------------------

cv::Mat Transformations::composeRt(const cv::Mat &r, const cv::Mat &t)
{
  assert(r.type() == t.type());
  
  cv::Mat R;
  if(r.rows != 3 || r.cols != 3)
    cv::Rodrigues(r, R);
  else
    R = r;

  // normalization factor
  double vd = 1;
  if(t.rows > 3)
  {
    if(t.type() == CV_32F) vd = t.at<float>(3,0);
    else vd = t.at<double>(3,0);
  }
  else if(t.cols > 3)
  {
    if(t.type() == CV_32F) vd = t.at<float>(0,3);
    else vd = t.at<double>(0,3);
  }

  if(R.type() == CV_32F)
  {
    if(t.rows > 1)
    {    
      return (cv::Mat_<float>(4,4) << 
        R.at<float>(0,0), R.at<float>(0,1), R.at<float>(0,2), t.at<float>(0,0) / vd,
        R.at<float>(1,0), R.at<float>(1,1), R.at<float>(1,2), t.at<float>(1,0) / vd,
        R.at<float>(2,0), R.at<float>(2,1), R.at<float>(2,2), t.at<float>(2,0) / vd,
        0, 0, 0, 1
      );
    }
    else
    {
      return (cv::Mat_<float>(4,4) << 
        R.at<float>(0,0), R.at<float>(0,1), R.at<float>(0,2), t.at<float>(0,0) / vd,
        R.at<float>(1,0), R.at<float>(1,1), R.at<float>(1,2), t.at<float>(0,1) / vd,
        R.at<float>(2,0), R.at<float>(2,1), R.at<float>(2,2), t.at<float>(0,2) / vd,
        0, 0, 0, 1
      );
    }
  }
  else
  {
    if(t.rows > 1)
    {
      return (cv::Mat_<double>(4,4) << 
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0) / vd,
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0) / vd,
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0) / vd,
        0, 0, 0, 1
      );
    }
    else
    {
      return (cv::Mat_<double>(4,4) << 
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(0,1),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(0,2),
        0, 0, 0, 1
      );
    }
  }
}

// ----------------------------------------------------------------------------

void Transformations::decomposeRt(const cv::Mat &T, cv::Mat &R, cv::Mat &t)
{
  R = T( cv::Range(0,3), cv::Range(0,3) );
  
  if(t.rows == 3 && t.cols == 1)
    t = T( cv::Range(0,3), cv::Range(3,4) );
  else
    t = T.col(3);
}

// ----------------------------------------------------------------------------

