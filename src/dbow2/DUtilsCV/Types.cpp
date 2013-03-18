/**
 * File: Types.cpp
 * Project: DUtilsCV library
 * Author: Dorian Galvez-Lopez
 * Date: October 20, 2011
 * Description: OpenCV-related data types functions
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
#include <string>
#include "Types.h"

using namespace DUtilsCV;

// ----------------------------------------------------------------------------

std::string Types::type(const cv::Mat &m)
{
  std::string ret;

  switch(m.type())
  {
    case CV_8U: ret = "CV_8U"; break;
    case CV_8S: ret = "CV_8S"; break;
    case CV_16U: ret = "CV_16U"; break;
    case CV_16S: ret = "CV_16S"; break;
    case CV_32S: ret = "CV_32S"; break;
    case CV_32F: ret = "CV_32F"; break;
    case CV_64F: ret = "CV_64F"; break;
    default: ret = ""; break;
  }
  
  return ret;
}

// ----------------------------------------------------------------------------


