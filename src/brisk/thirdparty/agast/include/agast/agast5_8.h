//
//    agast5 - AGAST, an adaptive and generic corner detector based on the
//              accelerated segment test for a 8 pixel mask
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

#ifndef AGAST5_8_H
#define AGAST5_8_H

#include <stdint.h>
#include "AstDetector.h"

struct CvPoint;

namespace agast{

class AgastDetector5_8 : public AstDetector
{
	public:
		AgastDetector5_8():AstDetector(){;}
		AgastDetector5_8(int width, int height, int thr):AstDetector(width, height, thr){init_pattern();};
		~AgastDetector5_8(){}
		void detect(const unsigned char* im,
				std::vector<CvPoint>& keypoints);
		void nms(const unsigned char* im,
				const std::vector<CvPoint>& keypoints, std::vector<CvPoint>& keypoints_nms);
		int get_borderWidth(){return borderWidth;}
		int cornerScore(const unsigned char* p);

	private:
		static const int borderWidth=1;
		int_fast16_t s_offset0;
		int_fast16_t s_offset1;
		int_fast16_t s_offset2;
		int_fast16_t s_offset3;
		int_fast16_t s_offset4;
		int_fast16_t s_offset5;
		int_fast16_t s_offset6;
		int_fast16_t s_offset7;

		void init_pattern()
		{
			s_offset0=(-1)+(0)*xsize;
			s_offset1=(-1)+(-1)*xsize;
			s_offset2=(0)+(-1)*xsize;
			s_offset3=(1)+(-1)*xsize;
			s_offset4=(1)+(0)*xsize;
			s_offset5=(1)+(1)*xsize;
			s_offset6=(0)+(1)*xsize;
			s_offset7=(-1)+(1)*xsize;
		}
};

}

#endif /* AGAST5_8_H */
