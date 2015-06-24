/**
 * File: ImageFunctions.cpp
 * Project: DVision library
 * Author: Dorian Galvez-Lopez
 * Date: February 22, 2012
 * Description: Several functions for images
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

#include <opencv/cv.h>
#ifdef HAVE_OPENCV3
#include <opencv2/imgproc/imgproc.hpp>
#endif // HAVE_OPENCV3
#include "ImageFunctions.h"
#include "DUtils.h"

using namespace DVision;

// ----------------------------------------------------------------------------

cv::Mat ImageFunctions::getPatch(const cv::Mat &im, const cv::KeyPoint &kp, 
  int final_size, bool rectifyOrientation, bool useCartesianAngle)
{
  if(im.empty()) return cv::Mat();
  
  cv::Mat p1;
  if (rectifyOrientation && kp.angle >= 0)
  {
    // there is rotation
    float rotation_margin = kp.size / 2.f * 0.414213562373095; 
      // 0.414... = (2-sqrt(2))/sqrt(2)
    
    int psize = kp.size + rotation_margin*2; // bigger patch
    cv::Mat patch = ImageFunctions::getPatch(im, kp.pt, psize);
    
    double angle; // angle of keypoint in cartesian reference in [-180,180]
    
    if(useCartesianAngle)
      angle = kp.angle;
    else
      angle = 360 - kp.angle;
    
    angle = DUtils::Math::signedAngle(angle); // degrees, [-180, +180]
    
    // this function uses cartesian reference, with angle in degrees in [-180,180]
    cv::Mat rot = cv::getRotationMatrix2D(
      cv::Point2f(patch.cols/2.f, patch.rows/2.f), -angle, 1);
    
    cv::Mat r_patch;
    cv::warpAffine(patch, r_patch, rot, cv::Size(patch.rows, patch.cols),
      cv::INTER_AREA);
    
    // remove the margin
    // this is not accurate if the keypoint is very close to the border
    // of the image
    int c0 = rotation_margin;
    int cf = r_patch.cols - rotation_margin;
    int r0 = rotation_margin;
    int rf = r_patch.rows - rotation_margin;
    
    p1 = r_patch( cv::Range(r0, rf), cv::Range(c0, cf) ).clone();
  }
  else
  {
    p1 = ImageFunctions::getPatch(im, kp.pt, kp.size);
  }
  
  // scale if necessary
  cv::Mat ret;
  if(final_size >= 0)
    cv::resize(p1, ret, cv::Size(final_size, final_size), cv::INTER_AREA);
  else
    ret = p1;
  
  return ret;
}

// ----------------------------------------------------------------------------

cv::Mat ImageFunctions::getPatch(const cv::Mat &im, const cv::Point2f &pt,
  unsigned int patch_size)
{
  if(im.empty()) return cv::Mat();
  
  cv::Mat ret;
  
  int c0 = cvRound(pt.x) - patch_size/2;
  int r0 = cvRound(pt.y) - patch_size/2;
  int cf = cvRound(pt.x) + patch_size/2 - 1 + (patch_size % 2);
  int rf = cvRound(pt.y) + patch_size/2 - 1 + (patch_size % 2);
  
  if( c0 < im.cols && r0 < im.rows )
  {
    if(c0 < 0) c0 = 0;
    if(r0 < 0) r0 = 0;
    if(cf >= im.cols) cf = im.cols-1;
    if(rf >= im.rows) rf = im.rows-1;
    
    ret = im( cv::Range(r0, rf), cv::Range(c0, cf) ).clone();
  }
  
  return ret;
}

// ----------------------------------------------------------------------------


