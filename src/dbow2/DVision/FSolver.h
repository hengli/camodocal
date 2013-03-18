/**
 * File: FSolver.h
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

#ifndef __D_F_SOLVER__
#define __D_F_SOLVER__

#include <opencv/cv.h>
#include <vector>

namespace DVision {

/// Computes fundamental matrices
class FSolver
{
public:

  /**
   * Creates the solver without setting the image dimensions
   */
  FSolver();
  
  /**
   * Creates the solver and set the image dimensions
   * @param cols width of images
   * @param rows height of images
   */
  FSolver(int cols, int rows);
  
  /**
   * Destructor
   */
  virtual ~FSolver(){}
  
  /**
   * Sets image size
   * @param cols
   * @param rows
   */
  virtual void setImageSize(int cols, int rows);
  
  /**
   * Finds a fundamental matrix from the given correspondences by running
   * RANSAC
   * @param P1 2xN, 3xN, Nx2, Nx3, correspondences of image 1 in image coordinates
   * @param P2 2xN, 3xN, Nx2, Nx3, correspondences of image 2 in image coordinates
   * @param reprojection_error max reprojection error for getting inliers
   * @param min_points min number of required inliers
   * @param status (out) vector s.t. status[i] == 1 if i-th point 
   *   is an inlier, 0 otherwise
   * @param computeF (default: true) if false, the final F is not computed and
   *   an arbitrary non-empty 3x3 matrix is returned (it saves a svd operation)
   * @param probability RANSAC success probability
   * @param max_its maximum number of RANSAC iterations 
   * @return F s.t. P1' * F * P2 == 0, or empty
   */
  cv::Mat findFundamentalMat(const cv::Mat &P1, const cv::Mat &P2,
    double reprojection_error, int min_points = 9, 
    std::vector<uchar>* status = NULL,
    bool computeF = true, double probability = 0.99, int max_its = 500) const;

  /**
   * Checks if a consistent fundamental matrix can be computed from the given
   * points. It is not computed, though.
   * @param P1 2xN, 3xN, Nx2, Nx3, correspondences of image 1 in image coordinates
   * @param P2 2xN, 3xN, Nx2, Nx3, correspondences of image 2 in image coordinates
   * @param reprojection_error max reprojection error for getting inliers
   * @param min_points min number of required inliers
   * @param probability RANSAC success probability
   * @param max_its maximum number of RANSAC iterations 
   * @return true iff some fundamental matrix is found
   */
  bool checkFundamentalMat(const cv::Mat &P1, const cv::Mat &P2,
    double reprojection_error, int min_points = 9,
    double probability = 0.99, int max_its = 500) const;

protected:

  /**
   * Normalize points
   * @param P 2xN, 3xN, Nx2, Nx3, float or double points 
   * @param Q (out) 3xN (double) homogeneous coordinates of points
   */
  void normalizePoints(const cv::Mat &P, cv::Mat &Q) const;
  
  /**
   * Computes F from correspondences Q1(:,i_cols), Q2(:,i_cols)
   * @param Qc1 3xN normalized 
   * @param Qc2 3xN normalized
   * @param i_cols # >= 9
   * @return F12 3x3 or empty
   */
  cv::Mat _computeF(const cv::Mat &Qc1, const cv::Mat &Qc2, 
    const std::vector<unsigned int> &i_cols) const;
  
  /**
   * Get inliers in the opencv manner
   * @param Q1 3xN homogeneous points
   * @param Q2 3xN homogeneous points
   * @param F12 fundamental matrix
   * @param err max reprojection error
   * @param status (out) 1xN CV_8U vector with 1 in the positions of the inliers
   */  
  //void getInliers(const cv::Mat &Q1, const cv::Mat &Q2, 
  //  const cv::Mat &F12, double err, cv::Mat &status) const;

protected:
  
  /// Normalization matrix
  cv::Mat m_N;
  
  /// Traspose of normalization matrix
  cv::Mat m_N_t;

};

} // namespace DVision

#endif
