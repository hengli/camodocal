//
//    cvWrapper - header file to define CvPoint in case openCV is not installed
//
//    Copyright (C) 2010  Elmar Mair
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef CVWRAPPER_H_
#define CVWRAPPER_H_

#define HAVE_OPENCV  //we normally do have opencv

#ifdef HAVE_OPENCV
	#include <opencv2/opencv.hpp>
#else
	typedef struct CvPoint
	{
		int x;
		int y;
	}
	CvPoint;
#endif

#endif /* CVWRAPPER_H_ */
