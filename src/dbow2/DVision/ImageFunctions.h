/**
 * File: ImageFunctions.h
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

#ifndef __D_IMAGE_FUNCTIONS__
#define __D_IMAGE_FUNCTIONS__

#include <opencv/cv.h>

namespace DVision {

class ImageFunctions
{
public:

  /**
   * Extracts the patch described by a keypoint from an image. The patch is
   * cropped if it goes beyond the image size.
   * @param im image to extract the patch from
   * @param kp keypoint with the coordinates, the size and the angle of the
   *   patch
   * @param final_size if given, the patch is resized to this size. If not
   *   given, the size of the keypoint is used as a diameter.
   * @param rectifyOrientation iif true, the patch is rotated according to 
   *   the angle of the keypoint (if it is not -1)
   * @param useCartesianAngle if true, the angle of the keypoints is assumed
   *   to be in Cartesian axis. If false, the vision reference is used instead
   *   (X points right, Y points up)
   * @note OpenCV SURF keypoints use Cartesian coordinates
   * @note OpenCV ORB keypoints use vision coordinates 
   */
  static cv::Mat getPatch(const cv::Mat &im, const cv::KeyPoint &kp, 
    int final_size = -1, bool rectifyOrientation = true,
    bool useCartesianAngle = false);

  /**
   * Extracts a patch from an image. The patch is cropped if it goes beyond
   * the image size
   * @param im image to extract the patch from
   * @param pt center of the patch in im
   * @param patch_size number of rows (and cols) of the patch extracted
   */
  static cv::Mat getPatch(const cv::Mat &im, const cv::Point2f &pt,
    unsigned int patch_size);

};

} // namespace DVision

#endif


