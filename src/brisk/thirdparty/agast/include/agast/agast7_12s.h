//
//    agast7s - AGAST, an adaptive and generic corner detector based on the
//              accelerated segment test for a 12 pixel mask in square format
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

#ifndef AGAST7_12S_H
#define AGAST7_12S_H

#include <stdint.h>
#include "AstDetector.h"

struct CvPoint;

namespace agast{

	class AgastDetector7_12s : public AstDetector
	{
		public:
			AgastDetector7_12s():AstDetector(){;}
			AgastDetector7_12s(int width, int height, int thr):AstDetector(width, height, thr){init_pattern();};
			~AgastDetector7_12s(){}
			void detect(const unsigned char* im,
					std::vector<CvPoint>& keypoints);
			void nms(const unsigned char* im,
					const std::vector<CvPoint>& keypoints, std::vector<CvPoint>& keypoints_nms);
			int get_borderWidth(){return borderWidth;}
			int cornerScore(const unsigned char* p);

		private:
			static const int borderWidth=2;
			int_fast16_t s_offset0;
			int_fast16_t s_offset1;
			int_fast16_t s_offset2;
			int_fast16_t s_offset3;
			int_fast16_t s_offset4;
			int_fast16_t s_offset5;
			int_fast16_t s_offset6;
			int_fast16_t s_offset7;
			int_fast16_t s_offset8;
			int_fast16_t s_offset9;
			int_fast16_t s_offset10;
			int_fast16_t s_offset11;

			void init_pattern()
			{
				s_offset0=(-2)+(0)*xsize;
				s_offset1=(-2)+(-1)*xsize;
				s_offset2=(-1)+(-2)*xsize;
				s_offset3=(0)+(-2)*xsize;
				s_offset4=(1)+(-2)*xsize;
				s_offset5=(2)+(-1)*xsize;
				s_offset6=(2)+(0)*xsize;
				s_offset7=(2)+(1)*xsize;
				s_offset8=(1)+(2)*xsize;
				s_offset9=(0)+(2)*xsize;
				s_offset10=(-1)+(2)*xsize;
				s_offset11=(-2)+(1)*xsize;
			}
	};

}

#endif /* AGAST7_12S_H */
