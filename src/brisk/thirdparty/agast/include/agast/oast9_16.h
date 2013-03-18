//
//    agast9 - OAST, an optimal corner detector based on the
//              accelerated segment test for a 16 pixel mask
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

#ifndef OAST9_16_H
#define OAST9_16_H

#include <stdint.h>
#include "AstDetector.h"

struct CvPoint;

namespace agast{

	class OastDetector9_16 : public AstDetector
	{
		public:
			OastDetector9_16():AstDetector(){;}
			OastDetector9_16(int width, int height, int thr):AstDetector(width, height, thr){init_pattern();};
			~OastDetector9_16(){}
			void detect(const unsigned char* im,
					std::vector<CvPoint>& keypoints);
			void nms(const unsigned char* im,
					const std::vector<CvPoint>& keypoints, std::vector<CvPoint>& keypoints_nms);
			int get_borderWidth(){return borderWidth;}
			int cornerScore(const unsigned char* p);

		private:
			static const int borderWidth=3;
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
			int_fast16_t s_offset12;
			int_fast16_t s_offset13;
			int_fast16_t s_offset14;
			int_fast16_t s_offset15;

			void init_pattern()
			{
				s_offset0=(-3)+(0)*xsize;
				s_offset1=(-3)+(-1)*xsize;
				s_offset2=(-2)+(-2)*xsize;
				s_offset3=(-1)+(-3)*xsize;
				s_offset4=(0)+(-3)*xsize;
				s_offset5=(1)+(-3)*xsize;
				s_offset6=(2)+(-2)*xsize;
				s_offset7=(3)+(-1)*xsize;
				s_offset8=(3)+(0)*xsize;
				s_offset9=(3)+(1)*xsize;
				s_offset10=(2)+(2)*xsize;
				s_offset11=(1)+(3)*xsize;
				s_offset12=(0)+(3)*xsize;
				s_offset13=(-1)+(3)*xsize;
				s_offset14=(-2)+(2)*xsize;
				s_offset15=(-3)+(1)*xsize;
			}
	};

}

#endif /* OAST9_16_H */
