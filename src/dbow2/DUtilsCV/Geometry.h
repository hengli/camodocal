/**
 * File: Geometry.h
 * Project: DUtilsCV library
 * Author: Dorian Galvez-Lopez
 * Date: October 11, 2010
 * Description: OpenCV-related geometry functions
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

#ifndef __D_CV_GEOMETRY__
#define __D_CV_GEOMETRY__

#include <opencv/cv.h>

namespace DUtilsCV
{

/// Geometry functions
class Geometry
{
public:

  /**
	 * Calculates the squared Euclidean distance between two OpenCV vectors
	 * @param a row/col vector of N components
	 * @param b row/col vector of N components
	 * @return squared Euclidean distance
	 * @deprecated not efficient
	 */
	static double sqDistance(const cv::Mat &a, const cv::Mat &b);

};

}

#endif

