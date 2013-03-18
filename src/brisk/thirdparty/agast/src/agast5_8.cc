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

//machine generated code
//probability of an equal pixel on the Bresenham's circle: 0.33 and 0.1
//number of equal pixels to switch: 1
//number of unequal pixels to switch: 6
//memory costs: cache=0.2
//              same line=1
//              memory=4

#include <stdint.h>																			
#include <stdlib.h>
#include "cvWrapper.h"
#include "agast5_8.h"

using namespace std;
using namespace agast;

void AgastDetector5_8::detect(const unsigned char* im, std::vector<CvPoint>& corners_all)
{
	int total=0;
	int nExpectedCorners=corners_all.capacity();
	CvPoint h;
	register int x, y;
	register int xsizeB=xsize - 2;
	register int ysizeB=ysize - 1;
	register int_fast16_t offset0, offset1, offset2, offset3, offset4, offset5, offset6, offset7;
	register int width;

	corners_all.resize(0);

	offset0=s_offset0;
	offset1=s_offset1;
	offset2=s_offset2;
	offset3=s_offset3;
	offset4=s_offset4;
	offset5=s_offset5;
	offset6=s_offset6;
	offset7=s_offset7;
	width=xsize;

	for(y=1; y < ysizeB; y++)
	{										
		x=0;
		while(1)							
		{									
homogeneous:
{
			x++;			
			if(x>xsizeB)	
				break;
			else
			{
				register const unsigned char* const p = im + y*width + x;
				register const int cb = *p + b;
				register const int c_b = *p - b;
				if(p[offset0] > cb)
				  if(p[offset2] > cb)
					if(p[offset3] > cb)
					  if(p[offset5] > cb)
						if(p[offset1] > cb)
						  if(p[offset4] > cb)
							goto success_structured;
						  else
							if(p[offset7] > cb)
							  goto success_structured;
							else
							  goto homogeneous;
						else
						  if(p[offset4] > cb)
							if(p[offset6] > cb)
							  goto success_structured;
							else
							  goto homogeneous;
						  else
							goto homogeneous;
					  else
						if(p[offset1] > cb)
						  if(p[offset4] > cb)
							goto success_homogeneous;
						  else
							if(p[offset7] > cb)
							  goto success_homogeneous;
							else
							  goto homogeneous;
						else
						  goto homogeneous;
					else
					  if(p[offset7] > cb)
						if(p[offset6] > cb)
						  if(p[offset5] > cb)
							if(p[offset1] > cb)
							  goto success_structured;
							else
							  if(p[offset4] > cb)
								goto success_structured;
							  else
								goto homogeneous;
						  else
							if(p[offset1] > cb)
							  goto success_homogeneous;
							else
							  goto homogeneous;
						else
						  goto homogeneous;
					  else
						if(p[offset5] < c_b)
						  if(p[offset3] < c_b)
							if(p[offset7] < c_b)
							  if(p[offset4] < c_b)
								if(p[offset6] < c_b)
								  goto success_structured;
								else
								  goto structured;
							  else
								goto homogeneous;
							else
							  goto homogeneous;
						  else
							goto homogeneous;
						else
						  goto homogeneous;
				  else
					if(p[offset5] > cb)
					  if(p[offset7] > cb)
						if(p[offset6] > cb)
						  if(p[offset1] > cb)
							goto success_homogeneous;
						  else
							if(p[offset4] > cb)
							  goto success_homogeneous;
							else
							  goto homogeneous;
						else
						  goto homogeneous;
					  else
						goto homogeneous;
					else
					  if(p[offset5] < c_b)
						if(p[offset3] < c_b)
						  if(p[offset2] < c_b)
							if(p[offset1] < c_b)
							  if(p[offset4] < c_b)
								goto success_structured;
							  else
								goto homogeneous;
							else
							  if(p[offset4] < c_b)
								if(p[offset6] < c_b)
								  goto success_structured;
								else
								  goto homogeneous;
							  else
								goto homogeneous;
						  else
							if(p[offset7] < c_b)
							  if(p[offset4] < c_b)
								if(p[offset6] < c_b)
								  goto success_structured;
								else
								  goto homogeneous;
							  else
								goto homogeneous;
							else
							  goto homogeneous;
						else
						  goto homogeneous;
					  else
						goto homogeneous;
				else if(p[offset0] < c_b)
				  if(p[offset2] < c_b)
					if(p[offset7] > cb)
					  if(p[offset3] < c_b)
						if(p[offset5] < c_b)
						  if(p[offset1] < c_b)
							if(p[offset4] < c_b)
							  goto success_structured;
							else
							  goto structured;
						  else
							if(p[offset4] < c_b)
							  if(p[offset6] < c_b)
								goto success_structured;
							  else
								goto structured;
							else
							  goto homogeneous;
						else
						  if(p[offset1] < c_b)
							if(p[offset4] < c_b)
							  goto success_structured;
							else
							  goto homogeneous;
						  else
							goto homogeneous;
					  else
						if(p[offset5] > cb)
						  if(p[offset3] > cb)
							if(p[offset4] > cb)
							  if(p[offset6] > cb)
								goto success_structured;
							  else
								goto structured;
							else
							  goto homogeneous;
						  else
							goto homogeneous;
						else
						  goto homogeneous;
					else
					  if(p[offset7] < c_b)
						if(p[offset3] < c_b)
						  if(p[offset5] < c_b)
							if(p[offset1] < c_b)
							  goto success_structured;
							else
							  if(p[offset4] < c_b)
								if(p[offset6] < c_b)
								  goto success_structured;
								else
								  goto structured;
							  else
								goto homogeneous;
						  else
							if(p[offset1] < c_b)
							  goto success_homogeneous;
							else
							  goto homogeneous;
						else
						  if(p[offset6] < c_b)
							if(p[offset5] < c_b)
							  if(p[offset1] < c_b)
								goto success_structured;
							  else
								if(p[offset4] < c_b)
								  goto success_structured;
								else
								  goto homogeneous;
							else
							  if(p[offset1] < c_b)
								goto success_homogeneous;
							  else
								goto homogeneous;
						  else
							goto homogeneous;
					  else
						if(p[offset3] < c_b)
						  if(p[offset5] < c_b)
							if(p[offset1] < c_b)
							  if(p[offset4] < c_b)
								goto success_structured;
							  else
								goto homogeneous;
							else
							  if(p[offset4] < c_b)
								if(p[offset6] < c_b)
								  goto success_structured;
								else
								  goto homogeneous;
							  else
								goto homogeneous;
						  else
							if(p[offset1] < c_b)
							  if(p[offset4] < c_b)
								goto success_homogeneous;
							  else
								goto homogeneous;
							else
							  goto homogeneous;
						else
						  goto homogeneous;
				  else
					if(p[offset5] > cb)
					  if(p[offset3] > cb)
						if(p[offset2] > cb)
						  if(p[offset1] > cb)
							if(p[offset4] > cb)
							  goto success_structured;
							else
							  goto homogeneous;
						  else
							if(p[offset4] > cb)
							  if(p[offset6] > cb)
								goto success_structured;
							  else
								goto homogeneous;
							else
							  goto homogeneous;
						else
						  if(p[offset7] > cb)
							if(p[offset4] > cb)
							  if(p[offset6] > cb)
								goto success_structured;
							  else
								goto homogeneous;
							else
							  goto homogeneous;
						  else
							goto homogeneous;
					  else
						goto homogeneous;
					else
					  if(p[offset5] < c_b)
						if(p[offset7] < c_b)
						  if(p[offset6] < c_b)
							if(p[offset1] < c_b)
							  goto success_homogeneous;
							else
							  if(p[offset4] < c_b)
								goto success_homogeneous;
							  else
								goto homogeneous;
						  else
							goto homogeneous;
						else
						  goto homogeneous;
					  else
						goto homogeneous;
				else
				  if(p[offset3] > cb)
					if(p[offset5] > cb)
					  if(p[offset2] > cb)
						if(p[offset1] > cb)
						  if(p[offset4] > cb)
							goto success_homogeneous;
						  else
							goto homogeneous;
						else
						  if(p[offset4] > cb)
							if(p[offset6] > cb)
							  goto success_homogeneous;
							else
							  goto homogeneous;
						  else
							goto homogeneous;
					  else
						if(p[offset7] > cb)
						  if(p[offset4] > cb)
							if(p[offset6] > cb)
							  goto success_homogeneous;
							else
							  goto homogeneous;
						  else
							goto homogeneous;
						else
						  goto homogeneous;
					else
					  goto homogeneous;
				  else
					if(p[offset3] < c_b)
					  if(p[offset5] < c_b)
						if(p[offset2] < c_b)
						  if(p[offset1] < c_b)
							if(p[offset4] < c_b)
							  goto success_homogeneous;
							else
							  goto homogeneous;
						  else
							if(p[offset4] < c_b)
							  if(p[offset6] < c_b)
								goto success_homogeneous;
							  else
								goto homogeneous;
							else
							  goto homogeneous;
						else
						  if(p[offset7] < c_b)
							if(p[offset4] < c_b)
							  if(p[offset6] < c_b)
								goto success_homogeneous;
							  else
								goto homogeneous;
							else
							  goto homogeneous;
						  else
							goto homogeneous;
					  else
						goto homogeneous;
					else
					  goto homogeneous;
			}
}
structured:
{
			x++;			
			if(x>xsizeB)	
				break;
			else
			{
				register const unsigned char* const p = im + y*width + x;
				register const int cb = *p + b;
				register const int c_b = *p - b;
				if(p[offset0] > cb)
				  if(p[offset2] > cb)
					if(p[offset3] > cb)
					  if(p[offset5] > cb)
						if(p[offset7] > cb)
						  if(p[offset1] > cb)
							goto success_structured;
						  else
							if(p[offset4] > cb)
							  if(p[offset6] > cb)
								goto success_structured;
							  else
								goto structured;
							else
							  goto structured;
						else
						  if(p[offset1] > cb)
							if(p[offset4] > cb)
							  goto success_structured;
							else
							  goto structured;
						  else
							if(p[offset4] > cb)
							  if(p[offset6] > cb)
								goto success_structured;
							  else
								goto structured;
							else
							  goto structured;
					  else
						if(p[offset7] > cb)
						  if(p[offset1] > cb)
							goto success_structured;
						  else
							goto structured;
						else
						  if(p[offset1] > cb)
							if(p[offset4] > cb)
							  goto success_structured;
							else
							  goto structured;
						  else
							goto structured;
					else
					  if(p[offset7] > cb)
						if(p[offset6] > cb)
						  if(p[offset5] > cb)
							if(p[offset1] > cb)
							  goto success_structured;
							else
							  if(p[offset4] > cb)
								goto success_structured;
							  else
								goto structured;
						  else
							if(p[offset1] > cb)
							  goto success_structured;
							else
							  goto structured;
						else
						  goto structured;
					  else
						if(p[offset5] < c_b)
						  if(p[offset3] < c_b)
							if(p[offset7] < c_b)
							  if(p[offset4] < c_b)
								if(p[offset6] < c_b)
								  goto success_structured;
								else
								  goto structured;
							  else
								goto structured;
							else
							  goto homogeneous;
						  else
							goto homogeneous;
						else
						  goto structured;
				  else
					if(p[offset5] > cb)
					  if(p[offset7] > cb)
						if(p[offset6] > cb)
						  if(p[offset1] > cb)
							goto success_structured;
						  else
							if(p[offset4] > cb)
							  goto success_structured;
							else
							  goto structured;
						else
						  goto structured;
					  else
						goto structured;
					else
					  if(p[offset5] < c_b)
						if(p[offset3] < c_b)
						  if(p[offset2] < c_b)
							if(p[offset1] < c_b)
							  if(p[offset4] < c_b)
								goto success_structured;
							  else
								goto structured;
							else
							  if(p[offset4] < c_b)
								if(p[offset6] < c_b)
								  goto success_structured;
								else
								  goto structured;
							  else
								goto structured;
						  else
							if(p[offset7] < c_b)
							  if(p[offset4] < c_b)
								if(p[offset6] < c_b)
								  goto success_homogeneous;
								else
								  goto homogeneous;
							  else
								goto homogeneous;
							else
							  goto homogeneous;
						else
						  goto structured;
					  else
						goto homogeneous;
				else if(p[offset0] < c_b)
				  if(p[offset2] < c_b)
					if(p[offset7] > cb)
					  if(p[offset3] < c_b)
						if(p[offset5] < c_b)
						  if(p[offset1] < c_b)
							if(p[offset4] < c_b)
							  goto success_structured;
							else
							  goto structured;
						  else
							if(p[offset4] < c_b)
							  if(p[offset6] < c_b)
								goto success_structured;
							  else
								goto structured;
							else
							  goto structured;
						else
						  if(p[offset1] < c_b)
							if(p[offset4] < c_b)
							  goto success_structured;
							else
							  goto structured;
						  else
							goto structured;
					  else
						if(p[offset5] > cb)
						  if(p[offset3] > cb)
							if(p[offset4] > cb)
							  if(p[offset6] > cb)
								goto success_structured;
							  else
								goto structured;
							else
							  goto structured;
						  else
							goto homogeneous;
						else
						  goto structured;
					else
					  if(p[offset7] < c_b)
						if(p[offset3] < c_b)
						  if(p[offset5] < c_b)
							if(p[offset1] < c_b)
							  goto success_structured;
							else
							  if(p[offset4] < c_b)
								if(p[offset6] < c_b)
								  goto success_structured;
								else
								  goto structured;
							  else
								goto structured;
						  else
							if(p[offset1] < c_b)
							  goto success_structured;
							else
							  goto structured;
						else
						  if(p[offset6] < c_b)
							if(p[offset5] < c_b)
							  if(p[offset1] < c_b)
								goto success_structured;
							  else
								if(p[offset4] < c_b)
								  goto success_structured;
								else
								  goto structured;
							else
							  if(p[offset1] < c_b)
								goto success_structured;
							  else
								goto structured;
						  else
							goto structured;
					  else
						if(p[offset3] < c_b)
						  if(p[offset5] < c_b)
							if(p[offset1] < c_b)
							  if(p[offset4] < c_b)
								goto success_homogeneous;
							  else
								goto homogeneous;
							else
							  if(p[offset4] < c_b)
								if(p[offset6] < c_b)
								  goto success_homogeneous;
								else
								  goto homogeneous;
							  else
								goto homogeneous;
						  else
							if(p[offset1] < c_b)
							  if(p[offset4] < c_b)
								goto success_homogeneous;
							  else
								goto homogeneous;
							else
							  goto homogeneous;
						else
						  goto homogeneous;
				  else
					if(p[offset5] > cb)
					  if(p[offset3] > cb)
						if(p[offset2] > cb)
						  if(p[offset1] > cb)
							if(p[offset4] > cb)
							  goto success_structured;
							else
							  goto structured;
						  else
							if(p[offset4] > cb)
							  if(p[offset6] > cb)
								goto success_structured;
							  else
								goto structured;
							else
							  goto structured;
						else
						  if(p[offset7] > cb)
							if(p[offset4] > cb)
							  if(p[offset6] > cb)
								goto success_homogeneous;
							  else
								goto homogeneous;
							else
							  goto homogeneous;
						  else
							goto homogeneous;
					  else
						goto structured;
					else
					  if(p[offset5] < c_b)
						if(p[offset7] < c_b)
						  if(p[offset6] < c_b)
							if(p[offset1] < c_b)
							  goto success_structured;
							else
							  if(p[offset4] < c_b)
								goto success_structured;
							  else
								goto structured;
						  else
							goto structured;
						else
						  goto structured;
					  else
						goto homogeneous;
				else
				  if(p[offset3] > cb)
					if(p[offset5] > cb)
					  if(p[offset2] > cb)
						if(p[offset1] > cb)
						  if(p[offset4] > cb)
							goto success_homogeneous;
						  else
							goto homogeneous;
						else
						  if(p[offset4] > cb)
							if(p[offset6] > cb)
							  goto success_homogeneous;
							else
							  goto homogeneous;
						  else
							goto homogeneous;
					  else
						if(p[offset7] > cb)
						  if(p[offset4] > cb)
							if(p[offset6] > cb)
							  goto success_homogeneous;
							else
							  goto homogeneous;
						  else
							goto homogeneous;
						else
						  goto homogeneous;
					else
					  goto homogeneous;
				  else
					if(p[offset3] < c_b)
					  if(p[offset5] < c_b)
						if(p[offset2] < c_b)
						  if(p[offset1] < c_b)
							if(p[offset4] < c_b)
							  goto success_homogeneous;
							else
							  goto homogeneous;
						  else
							if(p[offset4] < c_b)
							  if(p[offset6] < c_b)
								goto success_homogeneous;
							  else
								goto homogeneous;
							else
							  goto homogeneous;
						else
						  if(p[offset7] < c_b)
							if(p[offset4] < c_b)
							  if(p[offset6] < c_b)
								goto success_homogeneous;
							  else
								goto homogeneous;
							else
							  goto homogeneous;
						  else
							goto homogeneous;
					  else
						goto homogeneous;
					else
					  goto homogeneous;
			}
}
success_homogeneous:
			if(total == nExpectedCorners) 	
			{								
				if(nExpectedCorners==0)
				{
					nExpectedCorners=512;
					corners_all.reserve(nExpectedCorners);
				}
				else
				{
					nExpectedCorners *=2;
					corners_all.reserve(nExpectedCorners);
				}
			}
			h.x=x;
			h.y=y;
			corners_all.push_back(h);
			total++;						
			goto homogeneous;				
success_structured:
			if(total == nExpectedCorners) 	
			{								
				if(nExpectedCorners==0)
				{
					nExpectedCorners=512;
					corners_all.reserve(nExpectedCorners);
				}
				else
				{
					nExpectedCorners *=2;
					corners_all.reserve(nExpectedCorners);
				}
			}
			h.x=x;
			h.y=y;
			corners_all.push_back(h);
			total++;						
			goto structured;				
		}									
	}										
}

//end of file
