//
//    AstDetector - the interface class for the AGAST corner detector
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

#ifndef ASTDETECTOR_H
#define ASTDETECTOR_H

#include <vector>
#include <iostream>

struct CvPoint;

namespace agast{

	class AstDetector
	{
		public:
			AstDetector():xsize(0),ysize(0),b(-1) {}
			AstDetector(int width, int height, int thr):xsize(width),ysize(height),b(thr) {}
			virtual ~AstDetector(){;}
			virtual void detect(const unsigned char* im, std::vector<CvPoint>& corners_all)=0;
			virtual int get_borderWidth()=0;
			void nms(const unsigned char* im,
					const std::vector<CvPoint>& corners_all, std::vector<CvPoint>& corners_nms);
			void processImage(const unsigned char* im,
					std::vector<CvPoint>& keypoints_nms) {
				std::vector<CvPoint> keypoints;
				detect(im,keypoints);
				nms(im,keypoints,keypoints_nms);}
			void set_threshold(int b_){b=b_;}
			void set_imageSize(int xsize_, int ysize_){xsize=xsize_; ysize=ysize_; init_pattern();}
			virtual int cornerScore(const unsigned char* p)=0;

		protected:
			virtual void init_pattern()=0;
			void score(const unsigned char* i, const std::vector<CvPoint>& corners_all);
			void nonMaximumSuppression(const std::vector<CvPoint>& corners_all,
					std::vector<CvPoint>& corners_nms);
			std::vector<int> scores;
			std::vector<int> nmsFlags;
			int xsize, ysize;
			int b;
	};

}

#endif /* AGASTDETECTOR_H */
