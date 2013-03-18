/**
 * File: HSolver.h
 * Project: DVision library
 * Author: Dorian Galvez-Lopez
 * Date: March 15, 2012
 * Description: Computes homographies
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

#ifndef __D_H_SOLVER__
#define __D_H_SOLVER__

#include <opencv/cv.h>
#include <vector>
#include "FSolver.h"

namespace DVision {

/// Computes fundamental matrices
class HSolver: protected FSolver
{
public:

  /**
   * Creates the solver without setting the image dimensions
   */
  HSolver();
  
  /**
   * Creates the solver and set the image dimensions
   * @param cols width of images
   * @param rows height of images
   */
  HSolver(int cols, int rows);
  
  /**
   * Destructor
   */
  virtual ~HSolver(){}

  /**
   * Sets image size
   * @param cols
   * @param rows
   * @use FSolver::setImageSize
   */
  virtual void setImageSize(int cols, int rows);  
  
  /**
   * Finds an homography matrix from the given correspondences by running
   * RANSAC
   * @param P1 2xN, 3xN, Nx2, Nx3, correspondences of image 1 in image coordinates
   * @param P2 2xN, 3xN, Nx2, Nx3, correspondences of image 2 in image coordinates
   * @param reprojection_error max reprojection error for getting inliers
   * @param min_points min number of required inliers
   * @param status (out) vector s.t. status[i] == 1 if i-th point 
   *   is an inlier, 0 otherwise
   * @param computeH (default: true) if false, the final H is not computed and
   *   an arbitrary non-empty 3x3 matrix is returned (it saves a svd operation)
   * @param probability RANSAC success probability
   * @param max_its maximum number of RANSAC iterations 
   * @return H s.t. P1' * H * P2 == 0, or empty
   */
  cv::Mat findHomography(const cv::Mat &P1, const cv::Mat &P2,
    double reprojection_error, int min_points = 5, 
    std::vector<uchar>* status = NULL,
    bool computeH = true, double probability = 0.99, int max_its = 500) const;

  /**
   * Checks if a consistent homography matrix can be computed from the given
   * points. It is not computed, though.
   * @param P1 2xN, 3xN, Nx2, Nx3, correspondences of image 1 in image coordinates
   * @param P2 2xN, 3xN, Nx2, Nx3, correspondences of image 2 in image coordinates
   * @param reprojection_error max reprojection error for getting inliers
   * @param min_points min number of required inliers
   * @param probability RANSAC success probability
   * @param max_its maximum number of RANSAC iterations 
   * @return true iff some fundamental matrix is found
   */
  bool checkHomography(const cv::Mat &P1, const cv::Mat &P2,
    double reprojection_error, int min_points = 5,
    double probability = 0.99, int max_its = 500) const;

protected:

  /**
   * Computes H from correspondences Q1(:,i_cols), Q2(:,i_cols)
   * @param Qc1 3xN normalized 
   * @param Qc2 3xN normalized
   * @param i_cols # >= 4
   * @return H12 3x3 or empty
   */
  cv::Mat _computeH(const cv::Mat &Qc1, const cv::Mat &Qc2, 
    const std::vector<unsigned int> &i_cols) const;

};

} // namespace DVision

#endif
