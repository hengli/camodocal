//
//    oast9 - OAST, an optimal corner detector based on the
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

//machine generated code
//probability of an equal pixel on the Bresenham's circle: 0.33
//memory costs: cache=0.2
//              same line=1
//              memory=4


#include <stdint.h>																			
#include <stdlib.h>
#include "cvWrapper.h"
#include "oast9_16.h"

using namespace std;
using namespace agast;

void OastDetector9_16::detect(const unsigned char* im, vector<CvPoint>& corners_all)
{
	int total=0;
	int nExpectedCorners=corners_all.capacity();
	CvPoint h;
	register int x, y;
	register int xsizeB=xsize - 4;
	register int ysizeB=ysize - 3;
    register int_fast16_t offset0, offset1, offset2, offset3, offset4, offset5, offset6, offset7, offset8, offset9, offset10, offset11, offset12, offset13, offset14, offset15;
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
    offset8=s_offset8;
    offset9=s_offset9;
    offset10=s_offset10;
    offset11=s_offset11;
    offset12=s_offset12;
    offset13=s_offset13;
    offset14=s_offset14;
    offset15=s_offset15;
    width=xsize;

	for(y=3; y < ysizeB; y++)			
	{										
		x=2;								
		while(1)							
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
					if(p[offset4] > cb)
					  if(p[offset5] > cb)
						if(p[offset7] > cb)
						  if(p[offset3] > cb)
							if(p[offset1] > cb)
							  if(p[offset6] > cb)
								if(p[offset8] > cb)
								  {}
								else
								  if(p[offset15] > cb)
									{}
								  else
									continue;
							  else
								if(p[offset13] > cb)
								  if(p[offset14] > cb)
									if(p[offset15] > cb)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  if(p[offset8] > cb)
								if(p[offset9] > cb)
								  if(p[offset10] > cb)
									if(p[offset6] > cb)
									  {}
									else
									  if(p[offset11] > cb)
										if(p[offset12] > cb)
										  if(p[offset13] > cb)
											if(p[offset14] > cb)
											  if(p[offset15] > cb)
												{}
											  else
												continue;
											else
											  continue;
										  else
											continue;
										else
										  continue;
									  else
										continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
						  else
							if(p[offset10] > cb)
							  if(p[offset11] > cb)
								if(p[offset12] > cb)
								  if(p[offset8] > cb)
									if(p[offset9] > cb)
									  if(p[offset6] > cb)
										{}
									  else
										if(p[offset13] > cb)
										  if(p[offset14] > cb)
											if(p[offset15] > cb)
											  {}
											else
											  continue;
										  else
											continue;
										else
										  continue;
									else
									  if(p[offset1] > cb)
										if(p[offset13] > cb)
										  if(p[offset14] > cb)
											if(p[offset15] > cb)
											  {}
											else
											  continue;
										  else
											continue;
										else
										  continue;
									  else
										continue;
								  else
									if(p[offset1] > cb)
									  if(p[offset13] > cb)
										if(p[offset14] > cb)
										  if(p[offset15] > cb)
											{}
										  else
											continue;
										else
										  continue;
									  else
										continue;
									else
									  continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						else if(p[offset7] < c_b)
						  if(p[offset14] > cb)
							if(p[offset15] > cb)
							  if(p[offset1] > cb)
								if(p[offset3] > cb)
								  if(p[offset6] > cb)
									{}
								  else
									if(p[offset13] > cb)
									  {}
									else
									  continue;
								else
								  if(p[offset10] > cb)
									if(p[offset11] > cb)
									  if(p[offset12] > cb)
										if(p[offset13] > cb)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset8] > cb)
								  if(p[offset9] > cb)
									if(p[offset10] > cb)
									  if(p[offset11] > cb)
										if(p[offset12] > cb)
										  if(p[offset13] > cb)
											{}
										  else
											continue;
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  continue;
						  else if(p[offset14] < c_b)
							if(p[offset8] < c_b)
							  if(p[offset9] < c_b)
								if(p[offset10] < c_b)
								  if(p[offset11] < c_b)
									if(p[offset12] < c_b)
									  if(p[offset13] < c_b)
										if(p[offset6] < c_b)
										  {}
										else
										  if(p[offset15] < c_b)
											{}
										  else
											continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						  else
							continue;
						else
						  if(p[offset14] > cb)
							if(p[offset15] > cb)
							  if(p[offset1] > cb)
								if(p[offset3] > cb)
								  if(p[offset6] > cb)
									{}
								  else
									if(p[offset13] > cb)
									  {}
									else
									  continue;
								else
								  if(p[offset10] > cb)
									if(p[offset11] > cb)
									  if(p[offset12] > cb)
										if(p[offset13] > cb)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset8] > cb)
								  if(p[offset9] > cb)
									if(p[offset10] > cb)
									  if(p[offset11] > cb)
										if(p[offset12] > cb)
										  if(p[offset13] > cb)
											{}
										  else
											continue;
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  continue;
						  else
							continue;
					  else if(p[offset5] < c_b)
						if(p[offset12] > cb)
						  if(p[offset13] > cb)
							if(p[offset14] > cb)
							  if(p[offset15] > cb)
								if(p[offset1] > cb)
								  if(p[offset3] > cb)
									{}
								  else
									if(p[offset10] > cb)
									  if(p[offset11] > cb)
										{}
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset8] > cb)
									if(p[offset9] > cb)
									  if(p[offset10] > cb)
										if(p[offset11] > cb)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset6] > cb)
								  if(p[offset7] > cb)
									if(p[offset8] > cb)
									  if(p[offset9] > cb)
										if(p[offset10] > cb)
										  if(p[offset11] > cb)
											{}
										  else
											continue;
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  continue;
						  else
							continue;
						else if(p[offset12] < c_b)
						  if(p[offset7] < c_b)
							if(p[offset8] < c_b)
							  if(p[offset9] < c_b)
								if(p[offset10] < c_b)
								  if(p[offset11] < c_b)
									if(p[offset13] < c_b)
									  if(p[offset6] < c_b)
										{}
									  else
										if(p[offset14] < c_b)
										  if(p[offset15] < c_b)
											{}
										  else
											continue;
										else
										  continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						  else
							continue;
						else
						  continue;
					  else
						if(p[offset12] > cb)
						  if(p[offset13] > cb)
							if(p[offset14] > cb)
							  if(p[offset15] > cb)
								if(p[offset1] > cb)
								  if(p[offset3] > cb)
									{}
								  else
									if(p[offset10] > cb)
									  if(p[offset11] > cb)
										{}
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset8] > cb)
									if(p[offset9] > cb)
									  if(p[offset10] > cb)
										if(p[offset11] > cb)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset6] > cb)
								  if(p[offset7] > cb)
									if(p[offset8] > cb)
									  if(p[offset9] > cb)
										if(p[offset10] > cb)
										  if(p[offset11] > cb)
											{}
										  else
											continue;
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  continue;
						  else
							continue;
						else if(p[offset12] < c_b)
						  if(p[offset7] < c_b)
							if(p[offset8] < c_b)
							  if(p[offset9] < c_b)
								if(p[offset10] < c_b)
								  if(p[offset11] < c_b)
									if(p[offset13] < c_b)
									  if(p[offset14] < c_b)
										if(p[offset6] < c_b)
										  {}
										else
										  if(p[offset15] < c_b)
											{}
										  else
											continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						  else
							continue;
						else
						  continue;
					else if(p[offset4] < c_b)
					  if(p[offset11] > cb)
						if(p[offset12] > cb)
						  if(p[offset13] > cb)
							if(p[offset10] > cb)
							  if(p[offset14] > cb)
								if(p[offset15] > cb)
								  if(p[offset1] > cb)
									{}
								  else
									if(p[offset8] > cb)
									  if(p[offset9] > cb)
										{}
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset6] > cb)
									if(p[offset7] > cb)
									  if(p[offset8] > cb)
										if(p[offset9] > cb)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset5] > cb)
								  if(p[offset6] > cb)
									if(p[offset7] > cb)
									  if(p[offset8] > cb)
										if(p[offset9] > cb)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  if(p[offset1] > cb)
								if(p[offset3] > cb)
								  if(p[offset14] > cb)
									if(p[offset15] > cb)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
						  else
							continue;
						else
						  continue;
					  else if(p[offset11] < c_b)
						if(p[offset7] < c_b)
						  if(p[offset8] < c_b)
							if(p[offset9] < c_b)
							  if(p[offset10] < c_b)
								if(p[offset6] < c_b)
								  if(p[offset5] < c_b)
									if(p[offset3] < c_b)
									  {}
									else
									  if(p[offset12] < c_b)
										{}
									  else
										continue;
								  else
									if(p[offset12] < c_b)
									  if(p[offset13] < c_b)
										if(p[offset14] < c_b)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset12] < c_b)
									if(p[offset13] < c_b)
									  if(p[offset14] < c_b)
										if(p[offset15] < c_b)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								continue;
							else
							  continue;
						  else
							continue;
						else
						  continue;
					  else
						continue;
					else
					  if(p[offset11] > cb)
						if(p[offset12] > cb)
						  if(p[offset13] > cb)
							if(p[offset10] > cb)
							  if(p[offset14] > cb)
								if(p[offset15] > cb)
								  if(p[offset1] > cb)
									{}
								  else
									if(p[offset8] > cb)
									  if(p[offset9] > cb)
										{}
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset6] > cb)
									if(p[offset7] > cb)
									  if(p[offset8] > cb)
										if(p[offset9] > cb)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset5] > cb)
								  if(p[offset6] > cb)
									if(p[offset7] > cb)
									  if(p[offset8] > cb)
										if(p[offset9] > cb)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  if(p[offset1] > cb)
								if(p[offset3] > cb)
								  if(p[offset14] > cb)
									if(p[offset15] > cb)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
						  else
							continue;
						else
						  continue;
					  else if(p[offset11] < c_b)
						if(p[offset7] < c_b)
						  if(p[offset8] < c_b)
							if(p[offset9] < c_b)
							  if(p[offset10] < c_b)
								if(p[offset12] < c_b)
								  if(p[offset13] < c_b)
									if(p[offset6] < c_b)
									  if(p[offset5] < c_b)
										{}
									  else
										if(p[offset14] < c_b)
										  {}
										else
										  continue;
									else
									  if(p[offset14] < c_b)
										if(p[offset15] < c_b)
										  {}
										else
										  continue;
									  else
										continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						  else
							continue;
						else
						  continue;
					  else
						continue;
				  else if(p[offset2] < c_b)
					if(p[offset9] > cb)
					  if(p[offset10] > cb)
						if(p[offset11] > cb)
						  if(p[offset8] > cb)
							if(p[offset12] > cb)
							  if(p[offset13] > cb)
								if(p[offset14] > cb)
								  if(p[offset15] > cb)
									{}
								  else
									if(p[offset6] > cb)
									  if(p[offset7] > cb)
										{}
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset5] > cb)
									if(p[offset6] > cb)
									  if(p[offset7] > cb)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset4] > cb)
								  if(p[offset5] > cb)
									if(p[offset6] > cb)
									  if(p[offset7] > cb)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  if(p[offset3] > cb)
								if(p[offset4] > cb)
								  if(p[offset5] > cb)
									if(p[offset6] > cb)
									  if(p[offset7] > cb)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
						  else
							if(p[offset1] > cb)
							  if(p[offset12] > cb)
								if(p[offset13] > cb)
								  if(p[offset14] > cb)
									if(p[offset15] > cb)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						else
						  continue;
					  else
						continue;
					else if(p[offset9] < c_b)
					  if(p[offset7] < c_b)
						if(p[offset8] < c_b)
						  if(p[offset6] < c_b)
							if(p[offset5] < c_b)
							  if(p[offset4] < c_b)
								if(p[offset3] < c_b)
								  if(p[offset1] < c_b)
									{}
								  else
									if(p[offset10] < c_b)
									  {}
									else
									  continue;
								else
								  if(p[offset10] < c_b)
									if(p[offset11] < c_b)
									  if(p[offset12] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset10] < c_b)
								  if(p[offset11] < c_b)
									if(p[offset12] < c_b)
									  if(p[offset13] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  if(p[offset10] < c_b)
								if(p[offset11] < c_b)
								  if(p[offset12] < c_b)
									if(p[offset13] < c_b)
									  if(p[offset14] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
						  else
							if(p[offset10] < c_b)
							  if(p[offset11] < c_b)
								if(p[offset12] < c_b)
								  if(p[offset13] < c_b)
									if(p[offset14] < c_b)
									  if(p[offset15] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						else
						  continue;
					  else
						continue;
					else
					  continue;
				  else
					if(p[offset9] > cb)
					  if(p[offset10] > cb)
						if(p[offset11] > cb)
						  if(p[offset8] > cb)
							if(p[offset12] > cb)
							  if(p[offset13] > cb)
								if(p[offset14] > cb)
								  if(p[offset15] > cb)
									{}
								  else
									if(p[offset6] > cb)
									  if(p[offset7] > cb)
										{}
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset5] > cb)
									if(p[offset6] > cb)
									  if(p[offset7] > cb)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset4] > cb)
								  if(p[offset5] > cb)
									if(p[offset6] > cb)
									  if(p[offset7] > cb)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  if(p[offset3] > cb)
								if(p[offset4] > cb)
								  if(p[offset5] > cb)
									if(p[offset6] > cb)
									  if(p[offset7] > cb)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
						  else
							if(p[offset1] > cb)
							  if(p[offset12] > cb)
								if(p[offset13] > cb)
								  if(p[offset14] > cb)
									if(p[offset15] > cb)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						else
						  continue;
					  else
						continue;
					else if(p[offset9] < c_b)
					  if(p[offset7] < c_b)
						if(p[offset8] < c_b)
						  if(p[offset10] < c_b)
							if(p[offset11] < c_b)
							  if(p[offset6] < c_b)
								if(p[offset5] < c_b)
								  if(p[offset4] < c_b)
									if(p[offset3] < c_b)
									  {}
									else
									  if(p[offset12] < c_b)
										{}
									  else
										continue;
								  else
									if(p[offset12] < c_b)
									  if(p[offset13] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset12] < c_b)
									if(p[offset13] < c_b)
									  if(p[offset14] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset12] < c_b)
								  if(p[offset13] < c_b)
									if(p[offset14] < c_b)
									  if(p[offset15] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  continue;
						  else
							continue;
						else
						  continue;
					  else
						continue;
					else
					  continue;
				else if(p[offset0] < c_b)
				  if(p[offset2] > cb)
					if(p[offset9] > cb)
					  if(p[offset7] > cb)
						if(p[offset8] > cb)
						  if(p[offset6] > cb)
							if(p[offset5] > cb)
							  if(p[offset4] > cb)
								if(p[offset3] > cb)
								  if(p[offset1] > cb)
									{}
								  else
									if(p[offset10] > cb)
									  {}
									else
									  continue;
								else
								  if(p[offset10] > cb)
									if(p[offset11] > cb)
									  if(p[offset12] > cb)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset10] > cb)
								  if(p[offset11] > cb)
									if(p[offset12] > cb)
									  if(p[offset13] > cb)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  if(p[offset10] > cb)
								if(p[offset11] > cb)
								  if(p[offset12] > cb)
									if(p[offset13] > cb)
									  if(p[offset14] > cb)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
						  else
							if(p[offset10] > cb)
							  if(p[offset11] > cb)
								if(p[offset12] > cb)
								  if(p[offset13] > cb)
									if(p[offset14] > cb)
									  if(p[offset15] > cb)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						else
						  continue;
					  else
						continue;
					else if(p[offset9] < c_b)
					  if(p[offset10] < c_b)
						if(p[offset11] < c_b)
						  if(p[offset8] < c_b)
							if(p[offset12] < c_b)
							  if(p[offset13] < c_b)
								if(p[offset14] < c_b)
								  if(p[offset15] < c_b)
									{}
								  else
									if(p[offset6] < c_b)
									  if(p[offset7] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset5] < c_b)
									if(p[offset6] < c_b)
									  if(p[offset7] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset4] < c_b)
								  if(p[offset5] < c_b)
									if(p[offset6] < c_b)
									  if(p[offset7] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  if(p[offset3] < c_b)
								if(p[offset4] < c_b)
								  if(p[offset5] < c_b)
									if(p[offset6] < c_b)
									  if(p[offset7] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
						  else
							if(p[offset1] < c_b)
							  if(p[offset12] < c_b)
								if(p[offset13] < c_b)
								  if(p[offset14] < c_b)
									if(p[offset15] < c_b)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						else
						  continue;
					  else
						continue;
					else
					  continue;
				  else if(p[offset2] < c_b)
					if(p[offset4] > cb)
					  if(p[offset11] > cb)
						if(p[offset7] > cb)
						  if(p[offset8] > cb)
							if(p[offset9] > cb)
							  if(p[offset10] > cb)
								if(p[offset6] > cb)
								  if(p[offset5] > cb)
									if(p[offset3] > cb)
									  {}
									else
									  if(p[offset12] > cb)
										{}
									  else
										continue;
								  else
									if(p[offset12] > cb)
									  if(p[offset13] > cb)
										if(p[offset14] > cb)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset12] > cb)
									if(p[offset13] > cb)
									  if(p[offset14] > cb)
										if(p[offset15] > cb)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								continue;
							else
							  continue;
						  else
							continue;
						else
						  continue;
					  else if(p[offset11] < c_b)
						if(p[offset12] < c_b)
						  if(p[offset13] < c_b)
							if(p[offset10] < c_b)
							  if(p[offset14] < c_b)
								if(p[offset15] < c_b)
								  if(p[offset1] < c_b)
									{}
								  else
									if(p[offset8] < c_b)
									  if(p[offset9] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset6] < c_b)
									if(p[offset7] < c_b)
									  if(p[offset8] < c_b)
										if(p[offset9] < c_b)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset5] < c_b)
								  if(p[offset6] < c_b)
									if(p[offset7] < c_b)
									  if(p[offset8] < c_b)
										if(p[offset9] < c_b)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  if(p[offset1] < c_b)
								if(p[offset3] < c_b)
								  if(p[offset14] < c_b)
									if(p[offset15] < c_b)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
						  else
							continue;
						else
						  continue;
					  else
						continue;
					else if(p[offset4] < c_b)
					  if(p[offset5] > cb)
						if(p[offset12] > cb)
						  if(p[offset7] > cb)
							if(p[offset8] > cb)
							  if(p[offset9] > cb)
								if(p[offset10] > cb)
								  if(p[offset11] > cb)
									if(p[offset13] > cb)
									  if(p[offset6] > cb)
										{}
									  else
										if(p[offset14] > cb)
										  if(p[offset15] > cb)
											{}
										  else
											continue;
										else
										  continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						  else
							continue;
						else if(p[offset12] < c_b)
						  if(p[offset13] < c_b)
							if(p[offset14] < c_b)
							  if(p[offset15] < c_b)
								if(p[offset1] < c_b)
								  if(p[offset3] < c_b)
									{}
								  else
									if(p[offset10] < c_b)
									  if(p[offset11] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset8] < c_b)
									if(p[offset9] < c_b)
									  if(p[offset10] < c_b)
										if(p[offset11] < c_b)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset6] < c_b)
								  if(p[offset7] < c_b)
									if(p[offset8] < c_b)
									  if(p[offset9] < c_b)
										if(p[offset10] < c_b)
										  if(p[offset11] < c_b)
											{}
										  else
											continue;
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  continue;
						  else
							continue;
						else
						  continue;
					  else if(p[offset5] < c_b)
						if(p[offset7] > cb)
						  if(p[offset14] > cb)
							if(p[offset8] > cb)
							  if(p[offset9] > cb)
								if(p[offset10] > cb)
								  if(p[offset11] > cb)
									if(p[offset12] > cb)
									  if(p[offset13] > cb)
										if(p[offset6] > cb)
										  {}
										else
										  if(p[offset15] > cb)
											{}
										  else
											continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						  else if(p[offset14] < c_b)
							if(p[offset15] < c_b)
							  if(p[offset1] < c_b)
								if(p[offset3] < c_b)
								  if(p[offset6] < c_b)
									{}
								  else
									if(p[offset13] < c_b)
									  {}
									else
									  continue;
								else
								  if(p[offset10] < c_b)
									if(p[offset11] < c_b)
									  if(p[offset12] < c_b)
										if(p[offset13] < c_b)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset8] < c_b)
								  if(p[offset9] < c_b)
									if(p[offset10] < c_b)
									  if(p[offset11] < c_b)
										if(p[offset12] < c_b)
										  if(p[offset13] < c_b)
											{}
										  else
											continue;
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  continue;
						  else
							continue;
						else if(p[offset7] < c_b)
						  if(p[offset3] < c_b)
							if(p[offset1] < c_b)
							  if(p[offset6] < c_b)
								if(p[offset8] < c_b)
								  {}
								else
								  if(p[offset15] < c_b)
									{}
								  else
									continue;
							  else
								if(p[offset13] < c_b)
								  if(p[offset14] < c_b)
									if(p[offset15] < c_b)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  if(p[offset8] < c_b)
								if(p[offset9] < c_b)
								  if(p[offset10] < c_b)
									if(p[offset6] < c_b)
									  {}
									else
									  if(p[offset11] < c_b)
										if(p[offset12] < c_b)
										  if(p[offset13] < c_b)
											if(p[offset14] < c_b)
											  if(p[offset15] < c_b)
												{}
											  else
												continue;
											else
											  continue;
										  else
											continue;
										else
										  continue;
									  else
										continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
						  else
							if(p[offset10] < c_b)
							  if(p[offset11] < c_b)
								if(p[offset12] < c_b)
								  if(p[offset8] < c_b)
									if(p[offset9] < c_b)
									  if(p[offset6] < c_b)
										{}
									  else
										if(p[offset13] < c_b)
										  if(p[offset14] < c_b)
											if(p[offset15] < c_b)
											  {}
											else
											  continue;
										  else
											continue;
										else
										  continue;
									else
									  if(p[offset1] < c_b)
										if(p[offset13] < c_b)
										  if(p[offset14] < c_b)
											if(p[offset15] < c_b)
											  {}
											else
											  continue;
										  else
											continue;
										else
										  continue;
									  else
										continue;
								  else
									if(p[offset1] < c_b)
									  if(p[offset13] < c_b)
										if(p[offset14] < c_b)
										  if(p[offset15] < c_b)
											{}
										  else
											continue;
										else
										  continue;
									  else
										continue;
									else
									  continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						else
						  if(p[offset14] < c_b)
							if(p[offset15] < c_b)
							  if(p[offset1] < c_b)
								if(p[offset3] < c_b)
								  if(p[offset6] < c_b)
									{}
								  else
									if(p[offset13] < c_b)
									  {}
									else
									  continue;
								else
								  if(p[offset10] < c_b)
									if(p[offset11] < c_b)
									  if(p[offset12] < c_b)
										if(p[offset13] < c_b)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset8] < c_b)
								  if(p[offset9] < c_b)
									if(p[offset10] < c_b)
									  if(p[offset11] < c_b)
										if(p[offset12] < c_b)
										  if(p[offset13] < c_b)
											{}
										  else
											continue;
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  continue;
						  else
							continue;
					  else
						if(p[offset12] > cb)
						  if(p[offset7] > cb)
							if(p[offset8] > cb)
							  if(p[offset9] > cb)
								if(p[offset10] > cb)
								  if(p[offset11] > cb)
									if(p[offset13] > cb)
									  if(p[offset14] > cb)
										if(p[offset6] > cb)
										  {}
										else
										  if(p[offset15] > cb)
											{}
										  else
											continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						  else
							continue;
						else if(p[offset12] < c_b)
						  if(p[offset13] < c_b)
							if(p[offset14] < c_b)
							  if(p[offset15] < c_b)
								if(p[offset1] < c_b)
								  if(p[offset3] < c_b)
									{}
								  else
									if(p[offset10] < c_b)
									  if(p[offset11] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset8] < c_b)
									if(p[offset9] < c_b)
									  if(p[offset10] < c_b)
										if(p[offset11] < c_b)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset6] < c_b)
								  if(p[offset7] < c_b)
									if(p[offset8] < c_b)
									  if(p[offset9] < c_b)
										if(p[offset10] < c_b)
										  if(p[offset11] < c_b)
											{}
										  else
											continue;
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  continue;
						  else
							continue;
						else
						  continue;
					else
					  if(p[offset11] > cb)
						if(p[offset7] > cb)
						  if(p[offset8] > cb)
							if(p[offset9] > cb)
							  if(p[offset10] > cb)
								if(p[offset12] > cb)
								  if(p[offset13] > cb)
									if(p[offset6] > cb)
									  if(p[offset5] > cb)
										{}
									  else
										if(p[offset14] > cb)
										  {}
										else
										  continue;
									else
									  if(p[offset14] > cb)
										if(p[offset15] > cb)
										  {}
										else
										  continue;
									  else
										continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						  else
							continue;
						else
						  continue;
					  else if(p[offset11] < c_b)
						if(p[offset12] < c_b)
						  if(p[offset13] < c_b)
							if(p[offset10] < c_b)
							  if(p[offset14] < c_b)
								if(p[offset15] < c_b)
								  if(p[offset1] < c_b)
									{}
								  else
									if(p[offset8] < c_b)
									  if(p[offset9] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset6] < c_b)
									if(p[offset7] < c_b)
									  if(p[offset8] < c_b)
										if(p[offset9] < c_b)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset5] < c_b)
								  if(p[offset6] < c_b)
									if(p[offset7] < c_b)
									  if(p[offset8] < c_b)
										if(p[offset9] < c_b)
										  {}
										else
										  continue;
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  if(p[offset1] < c_b)
								if(p[offset3] < c_b)
								  if(p[offset14] < c_b)
									if(p[offset15] < c_b)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
						  else
							continue;
						else
						  continue;
					  else
						continue;
				  else
					if(p[offset9] > cb)
					  if(p[offset7] > cb)
						if(p[offset8] > cb)
						  if(p[offset10] > cb)
							if(p[offset11] > cb)
							  if(p[offset6] > cb)
								if(p[offset5] > cb)
								  if(p[offset4] > cb)
									if(p[offset3] > cb)
									  {}
									else
									  if(p[offset12] > cb)
										{}
									  else
										continue;
								  else
									if(p[offset12] > cb)
									  if(p[offset13] > cb)
										{}
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset12] > cb)
									if(p[offset13] > cb)
									  if(p[offset14] > cb)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset12] > cb)
								  if(p[offset13] > cb)
									if(p[offset14] > cb)
									  if(p[offset15] > cb)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  continue;
						  else
							continue;
						else
						  continue;
					  else
						continue;
					else if(p[offset9] < c_b)
					  if(p[offset10] < c_b)
						if(p[offset11] < c_b)
						  if(p[offset8] < c_b)
							if(p[offset12] < c_b)
							  if(p[offset13] < c_b)
								if(p[offset14] < c_b)
								  if(p[offset15] < c_b)
									{}
								  else
									if(p[offset6] < c_b)
									  if(p[offset7] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								else
								  if(p[offset5] < c_b)
									if(p[offset6] < c_b)
									  if(p[offset7] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset4] < c_b)
								  if(p[offset5] < c_b)
									if(p[offset6] < c_b)
									  if(p[offset7] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  if(p[offset3] < c_b)
								if(p[offset4] < c_b)
								  if(p[offset5] < c_b)
									if(p[offset6] < c_b)
									  if(p[offset7] < c_b)
										{}
									  else
										continue;
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
						  else
							if(p[offset1] < c_b)
							  if(p[offset12] < c_b)
								if(p[offset13] < c_b)
								  if(p[offset14] < c_b)
									if(p[offset15] < c_b)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						else
						  continue;
					  else
						continue;
					else
					  continue;
				else
				  if(p[offset7] > cb)
					if(p[offset8] > cb)
					  if(p[offset9] > cb)
						if(p[offset6] > cb)
						  if(p[offset5] > cb)
							if(p[offset4] > cb)
							  if(p[offset3] > cb)
								if(p[offset2] > cb)
								  if(p[offset1] > cb)
									{}
								  else
									if(p[offset10] > cb)
									  {}
									else
									  continue;
								else
								  if(p[offset10] > cb)
									if(p[offset11] > cb)
									  {}
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset10] > cb)
								  if(p[offset11] > cb)
									if(p[offset12] > cb)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  if(p[offset10] > cb)
								if(p[offset11] > cb)
								  if(p[offset12] > cb)
									if(p[offset13] > cb)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
						  else
							if(p[offset10] > cb)
							  if(p[offset11] > cb)
								if(p[offset12] > cb)
								  if(p[offset13] > cb)
									if(p[offset14] > cb)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						else
						  if(p[offset10] > cb)
							if(p[offset11] > cb)
							  if(p[offset12] > cb)
								if(p[offset13] > cb)
								  if(p[offset14] > cb)
									if(p[offset15] > cb)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						  else
							continue;
					  else
						continue;
					else
					  continue;
				  else if(p[offset7] < c_b)
					if(p[offset8] < c_b)
					  if(p[offset9] < c_b)
						if(p[offset6] < c_b)
						  if(p[offset5] < c_b)
							if(p[offset4] < c_b)
							  if(p[offset3] < c_b)
								if(p[offset2] < c_b)
								  if(p[offset1] < c_b)
									{}
								  else
									if(p[offset10] < c_b)
									  {}
									else
									  continue;
								else
								  if(p[offset10] < c_b)
									if(p[offset11] < c_b)
									  {}
									else
									  continue;
								  else
									continue;
							  else
								if(p[offset10] < c_b)
								  if(p[offset11] < c_b)
									if(p[offset12] < c_b)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							else
							  if(p[offset10] < c_b)
								if(p[offset11] < c_b)
								  if(p[offset12] < c_b)
									if(p[offset13] < c_b)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
						  else
							if(p[offset10] < c_b)
							  if(p[offset11] < c_b)
								if(p[offset12] < c_b)
								  if(p[offset13] < c_b)
									if(p[offset14] < c_b)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						else
						  if(p[offset10] < c_b)
							if(p[offset11] < c_b)
							  if(p[offset12] < c_b)
								if(p[offset13] < c_b)
								  if(p[offset14] < c_b)
									if(p[offset15] < c_b)
									  {}
									else
									  continue;
								  else
									continue;
								else
								  continue;
							  else
								continue;
							else
							  continue;
						  else
							continue;
					  else
						continue;
					else
					  continue;
				  else
					continue;
			}
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
		}									
	}										
}

//end of file
