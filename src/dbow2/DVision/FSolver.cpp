/**
 * File: FSolver.cpp
 * Project: DVision library
 * Author: Dorian Galvez-Lopez
 * Date: November 17, 2011
 * Description: Computes fundamental matrices
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
 
#include <vector>
#include <cmath>
#include <opencv/cv.h>
#include "FSolver.h"

#include "DUtils.h"
#include "DUtilsCV.h"

using namespace std;
using namespace DUtils;
using namespace DUtilsCV;

namespace DVision {

// ----------------------------------------------------------------------------

FSolver::FSolver()
{
  setImageSize(1,1);
}

// ----------------------------------------------------------------------------

FSolver::FSolver(int cols, int rows)
{
  setImageSize(cols, rows);
}

// ----------------------------------------------------------------------------

void FSolver::setImageSize(int cols, int rows)
{
  const double c = (double)cols;
  const double r = (double)rows;
  
  m_N = (cv::Mat_<double>(3,3) <<
    1./c, 0, -0.5,
    0, 1./r, -0.5,
    0, 0, 1);
     
  m_N_t = m_N.t();
}

// ----------------------------------------------------------------------------

bool FSolver::checkFundamentalMat(const cv::Mat &P1, const cv::Mat &P2,
    double reprojection_error, int min_points, double p, int max_its) const
{
  cv::Mat F = findFundamentalMat(P1, P2, reprojection_error, min_points,
    NULL, false, p, max_its);
  return !F.empty();
}
    
// ----------------------------------------------------------------------------

cv::Mat FSolver::findFundamentalMat(const cv::Mat &P1, const cv::Mat &P2,
  double reprojection_error, int min_points, 
  std::vector<uchar>* _status, bool compute_F, double p, int MAX_ITS) const
{
  const int M = 8; // model points
  const double log1p = log(1-p);
  int N = (P1.rows > P1.cols ? P1.rows : P1.cols); // candidate points
  
  if(N < min_points) return cv::Mat();
  
  if(min_points < M) min_points = M;
  
  int max_its;
  if(N == min_points)
    max_its = 1;
  else
  {
    max_its = int(log1p / log(1 - pow(double(min_points)/double(N), min_points)));
    if(max_its > MAX_ITS || max_its <= 0) max_its = MAX_ITS;
    // < 0 in case of wrapping around because of overflow
    // this is to avoid numerical problems
    if(max_its == std::numeric_limits<int>::max()) max_its--;
  }
  
  if(_status)
  {
    _status->resize(N);
    std::fill(_status->begin(), _status->end(), 0);
  }
  
  cv::Mat Q1, Q2;   // Qi = Pi, 3xN CV_64F, homogeneous, image coords
  cv::Mat Qc1, Qc2; // Qci = Qi, central coords
  
  normalizePoints(P1, Q1); // image coords
  normalizePoints(P2, Q2);
  
  Qc1 = m_N * Q1; // central coords
  Qc2 = m_N * Q2;
  
  int best_ninliers = 0;
  cv::Mat best_status;
  
  DUtils::Random::SeedRandOnce();
  
  vector<unsigned int> all_i_available;
  vector<unsigned int> i_available;
  all_i_available.reserve(N);
  i_available.reserve(N);
  
  vector<unsigned int> i_model;
  i_model.resize(M);
  
  for(int i = 0; i < N; ++i) 
    all_i_available.push_back(i);
  
  for(int it = 0; it < max_its; ++it)
  {
    // get model points
    i_available = all_i_available;
    
    for(short i = 0; i < M; ++i)
    {
      int idx = DUtils::Random::RandomInt(0, i_available.size()-1);
      i_model[i] = i_available[idx];
      i_available[idx] = i_available.back();
      i_available.pop_back();
    }

    // get model
    cv::Mat Fc = _computeF(Qc1, Qc2, i_model); // central coordinates
    // Fc12 s.t. lc1 = Fc12 * xc2
    
    if(!Fc.empty())
    {
      // check error
      cv::Mat l1 = (m_N_t * Fc) * Qc2; // Qc2
      // l1 is in image coordinates
        
      // normalize lines
      // [a b c]' -> [a b c]'/n, n = sqrt(a^2 + b^2)
      cv::Mat sq_ab, norms;
      cv::multiply(l1.rowRange(0,2), l1.rowRange(0,2), sq_ab);
      
      cv::reduce(sq_ab, norms, 0, CV_REDUCE_SUM); // 0 = single row
      cv::sqrt(norms, norms); // norms is Nx2
      
      cv::Mat thresholds = norms * reprojection_error; // Nx1
      // previous line assumes that x1(3,:) = 1
      // if not, it would add: thresholds = thresholds .* x1(3,:)
      
      // distances (w/o normalization)
      // d(x, l) = dot(x*, l*), * means normalized
      cv::Mat prod, dot;
      cv::multiply(l1, Q1, prod); // l1 against Q1 (homogeneous in image coords)
      cv::reduce(prod, dot, 0, CV_REDUCE_SUM); // dot is Nx1
      
      // error w/o sign
      dot = abs(dot);
      
      // get inliers
      cv::Mat status;
      cv::compare(dot, thresholds, status, cv::CMP_LE); 
      
      int ninliers = cv::countNonZero(status);
          
      if(ninliers > best_ninliers && ninliers >= min_points)
      {
        best_ninliers = ninliers;
        best_status = status;
        if(!compute_F)
        {
          // success, we can finish
          it = max_its; // this stops the loop
        }
        else
        {
          int its = int(log1p / log(1 - pow(double(ninliers)/double(N), M)));
          if(0 <= its && its < max_its) max_its = its;
        } // if computeF
      } // if ninliers > best_ninliers && ...
    } // if !Fc.empty
  } // ransac loop
  
  if(!best_status.empty())
  {
    // copy status
    if(_status)
    {
      unsigned char *c = best_status.ptr<unsigned char>();
      unsigned char *s = &((*_status)[0]);
      for(unsigned int i = 0; i < _status->size(); ++i, ++c, ++s)
      {
        if(*c != 0) *s = 1;
      }
    }
    
    // get final F
    if(compute_F)
    {
      vector<unsigned int> best_i;
      best_i.reserve(best_ninliers);
      
      unsigned char *c = best_status.ptr<unsigned char>();
      for(int i = 0; i < best_status.cols; ++i, ++c)
      {
        if(*c) best_i.push_back(i);
      }
      
      cv::Mat Fc12 = _computeF(Qc1, Qc2, best_i);
      
      // denormalize
      if(!Fc12.empty()) return m_N_t * Fc12 * m_N;
    }
    else
    {
      // final F is not necessary, give any value
      return cv::Mat::eye(3, 3, CV_64F);
    }
  }
  
  return cv::Mat();
}

// ----------------------------------------------------------------------------

void FSolver::normalizePoints(const cv::Mat &P, cv::Mat &Q) const
{
  // turn P into homogeneous coords first
  
  if(P.rows == 3) // P is 3xN
  {
    if(P.type() == CV_64F)
      Q = P.clone();
    else
      P.convertTo(Q, CV_64F);
    
    cv::Mat aux = Q.row(0);
    cv::divide(aux, Q.row(2), aux);
    
    aux = Q.row(1);
    cv::divide(aux, Q.row(2), aux);
    
  }
  else if(P.rows == 2) // P is 2xN
  {
    const int N = P.cols;
    
    Q.create(3, N, CV_64F);
    Q.row(2) = 1;
    if(P.type() == CV_64F)
    {
      Q.rowRange(0,2) = P * 1;
    }
    else
    {
      cv::Mat aux = Q.rowRange(0,2);
      P.convertTo(aux, CV_64F);
    }
  }
  else if(P.cols == 3) // P is Nx3
  {
    if(P.type() == CV_64F)
      Q = P.t();
    else
    {
      P.convertTo(Q, CV_64F);
      Q = Q.t();
    } 
    
    cv::Mat aux = Q.row(0);
    cv::divide(aux, Q.row(2), aux);
    
    aux = Q.row(1);
    cv::divide(aux, Q.row(2), aux);
  }
  else if(P.cols == 2) // P is Nx2
  {
    const int N = P.rows;
    
    Q.create(N, 3, CV_64F);
    Q.col(2) = 1;
    cv::Mat aux;
    if(P.type() == CV_64F)
    {
      aux = Q.rowRange(0,2);
      P.rowRange(0,2).copyTo(aux);
    }
    else
    {
      aux = Q.colRange(0,2);
      P.convertTo(aux, CV_64F);
    }
    
    Q = Q.t();
  }  
}

// ----------------------------------------------------------------------------

cv::Mat FSolver::_computeF(const cv::Mat &Qc1, const cv::Mat &Qc2, 
    const std::vector<unsigned int> &i_cols) const
{
  const unsigned int N = i_cols.size();
  if(N < 8) return cv::Mat(); 

  // compute F12
  const double min_inv_cond_number = 1e-16;
  const static cv::Mat d = (cv::Mat_<double>(3,3) <<
      1, 0, 0,     0, 1, 0,    0, 0, 0);

  cv::Mat M(N, 9, CV_64F);
  
  double *p = M.ptr<double>(0);
  
  for(unsigned int i = 0; i < N; ++i)
  {
    unsigned int c = i_cols[i];
    
    // M12[i, :] = [ p1[0]*p2', p1[1]*p2', p2' ]
    const double p10 = Qc1.at<double>(0, c);
    const double p11 = Qc1.at<double>(1, c);
    const double p20 = Qc2.at<double>(0, c);
    const double p21 = Qc2.at<double>(1, c);
    // p12 and p22 are 1
    
    *p++ = p10 * p20;
    *p++ = p10 * p21;
    *p++ = p10;
    
    *p++ = p11 * p20;
    *p++ = p11 * p21;
    *p++ = p11;
    
    *p++ = p20;
    *p++ = p21;
    *p++ = 1;
  }
  
  cv::SVD svd(M, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
  
  //ratio of the smallest singular value to the largest singular value
  double inv_cond = svd.w.ptr<double>()[ svd.w.rows-1 ] / svd.w.ptr<double>()[0];
  
  if(inv_cond < min_inv_cond_number)
    return cv::Mat();
  else
  {
    cv::Mat Fm = svd.vt.row(8).reshape(0, 3); // F12
    // force det = 0
    
    svd(Fm, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    
    return svd.u * d * svd.vt;
  }
}

// ----------------------------------------------------------------------------

 } // namespace FSolver
