/**
 * File: CvVersion.h
 * Project: 
 * Author: Dorian Galvez-Lopez
 * Date: February 3, 2013
 * Description: Checks OpenCV version
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

#if !CV22 && !CV23 && !CV24
  #if CV_MAJOR_VERSION == 2
    #if CV_MINOR_VERSION == 2
      #define CV22 1
    #else
      #if CV_MINOR_VERSION == 3
        #define CV23 1
      #else
        #define CV24 1
      #endif
    #endif
  #else
    #define CV24 1
  #endif
#endif


