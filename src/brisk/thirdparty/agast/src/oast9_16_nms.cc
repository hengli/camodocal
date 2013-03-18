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

#include <stdint.h>
#include <stdlib.h>
#include "cvWrapper.h"
#include "oast9_16.h"

using namespace std;
using namespace agast;

//using also bisection as propsed by Edward Rosten in FAST,
//but it is based on the OAST
int OastDetector9_16::cornerScore(const unsigned char* p)
{
    int bmin = b;
    //std::cout <<int(b)<<":";
    int bmax = 255;
    int b_test = (bmax + bmin)/2;

	register int_fast16_t offset0=s_offset0;
	register int_fast16_t offset1=s_offset1;
	register int_fast16_t offset2=s_offset2;
	register int_fast16_t offset3=s_offset3;
	register int_fast16_t offset4=s_offset4;
	register int_fast16_t offset5=s_offset5;
	register int_fast16_t offset6=s_offset6;
	register int_fast16_t offset7=s_offset7;
	register int_fast16_t offset8=s_offset8;
	register int_fast16_t offset9=s_offset9;
	register int_fast16_t offset10=s_offset10;
	register int_fast16_t offset11=s_offset11;
	register int_fast16_t offset12=s_offset12;
	register int_fast16_t offset13=s_offset13;
	register int_fast16_t offset14=s_offset14;
	register int_fast16_t offset15=s_offset15;

	while(1)
	{
		register const int cb = *p + b_test;
		//std::cout << offset0 << ".";
		register const int c_b = *p - b_test;
		if(p[offset0] > cb)
		  if(p[offset2] > cb)
		    if(p[offset4] > cb)
		      if(p[offset5] > cb)
		        if(p[offset7] > cb)
		          if(p[offset3] > cb)
		            if(p[offset1] > cb)
		              if(p[offset6] > cb)
		                if(p[offset8] > cb)
		                  goto is_a_corner;
		                else
		                  if(p[offset15] > cb)
		                    goto is_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset13] > cb)
		                  if(p[offset14] > cb)
		                    if(p[offset15] > cb)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              if(p[offset8] > cb)
		                if(p[offset9] > cb)
		                  if(p[offset10] > cb)
		                    if(p[offset6] > cb)
		                      goto is_a_corner;
		                    else
		                      if(p[offset11] > cb)
		                        if(p[offset12] > cb)
		                          if(p[offset13] > cb)
		                            if(p[offset14] > cb)
		                              if(p[offset15] > cb)
		                                goto is_a_corner;
		                              else
		                                goto is_not_a_corner;
		                            else
		                              goto is_not_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            if(p[offset10] > cb)
		              if(p[offset11] > cb)
		                if(p[offset12] > cb)
		                  if(p[offset8] > cb)
		                    if(p[offset9] > cb)
		                      if(p[offset6] > cb)
		                        goto is_a_corner;
		                      else
		                        if(p[offset13] > cb)
		                          if(p[offset14] > cb)
		                            if(p[offset15] > cb)
		                              goto is_a_corner;
		                            else
		                              goto is_not_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                    else
		                      if(p[offset1] > cb)
		                        if(p[offset13] > cb)
		                          if(p[offset14] > cb)
		                            if(p[offset15] > cb)
		                              goto is_a_corner;
		                            else
		                              goto is_not_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                  else
		                    if(p[offset1] > cb)
		                      if(p[offset13] > cb)
		                        if(p[offset14] > cb)
		                          if(p[offset15] > cb)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		        else if(p[offset7] < c_b)
		          if(p[offset14] > cb)
		            if(p[offset15] > cb)
		              if(p[offset1] > cb)
		                if(p[offset3] > cb)
		                  if(p[offset6] > cb)
		                    goto is_a_corner;
		                  else
		                    if(p[offset13] > cb)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset10] > cb)
		                    if(p[offset11] > cb)
		                      if(p[offset12] > cb)
		                        if(p[offset13] > cb)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset8] > cb)
		                  if(p[offset9] > cb)
		                    if(p[offset10] > cb)
		                      if(p[offset11] > cb)
		                        if(p[offset12] > cb)
		                          if(p[offset13] > cb)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else if(p[offset14] < c_b)
		            if(p[offset8] < c_b)
		              if(p[offset9] < c_b)
		                if(p[offset10] < c_b)
		                  if(p[offset11] < c_b)
		                    if(p[offset12] < c_b)
		                      if(p[offset13] < c_b)
		                        if(p[offset6] < c_b)
		                          goto is_a_corner;
		                        else
		                          if(p[offset15] < c_b)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          if(p[offset14] > cb)
		            if(p[offset15] > cb)
		              if(p[offset1] > cb)
		                if(p[offset3] > cb)
		                  if(p[offset6] > cb)
		                    goto is_a_corner;
		                  else
		                    if(p[offset13] > cb)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset10] > cb)
		                    if(p[offset11] > cb)
		                      if(p[offset12] > cb)
		                        if(p[offset13] > cb)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset8] > cb)
		                  if(p[offset9] > cb)
		                    if(p[offset10] > cb)
		                      if(p[offset11] > cb)
		                        if(p[offset12] > cb)
		                          if(p[offset13] > cb)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		      else if(p[offset5] < c_b)
		        if(p[offset12] > cb)
		          if(p[offset13] > cb)
		            if(p[offset14] > cb)
		              if(p[offset15] > cb)
		                if(p[offset1] > cb)
		                  if(p[offset3] > cb)
		                    goto is_a_corner;
		                  else
		                    if(p[offset10] > cb)
		                      if(p[offset11] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset8] > cb)
		                    if(p[offset9] > cb)
		                      if(p[offset10] > cb)
		                        if(p[offset11] > cb)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset6] > cb)
		                  if(p[offset7] > cb)
		                    if(p[offset8] > cb)
		                      if(p[offset9] > cb)
		                        if(p[offset10] > cb)
		                          if(p[offset11] > cb)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else if(p[offset12] < c_b)
		          if(p[offset7] < c_b)
		            if(p[offset8] < c_b)
		              if(p[offset9] < c_b)
		                if(p[offset10] < c_b)
		                  if(p[offset11] < c_b)
		                    if(p[offset13] < c_b)
		                      if(p[offset6] < c_b)
		                        goto is_a_corner;
		                      else
		                        if(p[offset14] < c_b)
		                          if(p[offset15] < c_b)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        if(p[offset12] > cb)
		          if(p[offset13] > cb)
		            if(p[offset14] > cb)
		              if(p[offset15] > cb)
		                if(p[offset1] > cb)
		                  if(p[offset3] > cb)
		                    goto is_a_corner;
		                  else
		                    if(p[offset10] > cb)
		                      if(p[offset11] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset8] > cb)
		                    if(p[offset9] > cb)
		                      if(p[offset10] > cb)
		                        if(p[offset11] > cb)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset6] > cb)
		                  if(p[offset7] > cb)
		                    if(p[offset8] > cb)
		                      if(p[offset9] > cb)
		                        if(p[offset10] > cb)
		                          if(p[offset11] > cb)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else if(p[offset12] < c_b)
		          if(p[offset7] < c_b)
		            if(p[offset8] < c_b)
		              if(p[offset9] < c_b)
		                if(p[offset10] < c_b)
		                  if(p[offset11] < c_b)
		                    if(p[offset13] < c_b)
		                      if(p[offset14] < c_b)
		                        if(p[offset6] < c_b)
		                          goto is_a_corner;
		                        else
		                          if(p[offset15] < c_b)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		    else if(p[offset4] < c_b)
		      if(p[offset11] > cb)
		        if(p[offset12] > cb)
		          if(p[offset13] > cb)
		            if(p[offset10] > cb)
		              if(p[offset14] > cb)
		                if(p[offset15] > cb)
		                  if(p[offset1] > cb)
		                    goto is_a_corner;
		                  else
		                    if(p[offset8] > cb)
		                      if(p[offset9] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset6] > cb)
		                    if(p[offset7] > cb)
		                      if(p[offset8] > cb)
		                        if(p[offset9] > cb)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset5] > cb)
		                  if(p[offset6] > cb)
		                    if(p[offset7] > cb)
		                      if(p[offset8] > cb)
		                        if(p[offset9] > cb)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              if(p[offset1] > cb)
		                if(p[offset3] > cb)
		                  if(p[offset14] > cb)
		                    if(p[offset15] > cb)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else if(p[offset11] < c_b)
		        if(p[offset7] < c_b)
		          if(p[offset8] < c_b)
		            if(p[offset9] < c_b)
		              if(p[offset10] < c_b)
		                if(p[offset6] < c_b)
		                  if(p[offset5] < c_b)
		                    if(p[offset3] < c_b)
		                      goto is_a_corner;
		                    else
		                      if(p[offset12] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                  else
		                    if(p[offset12] < c_b)
		                      if(p[offset13] < c_b)
		                        if(p[offset14] < c_b)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset12] < c_b)
		                    if(p[offset13] < c_b)
		                      if(p[offset14] < c_b)
		                        if(p[offset15] < c_b)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		    else
		      if(p[offset11] > cb)
		        if(p[offset12] > cb)
		          if(p[offset13] > cb)
		            if(p[offset10] > cb)
		              if(p[offset14] > cb)
		                if(p[offset15] > cb)
		                  if(p[offset1] > cb)
		                    goto is_a_corner;
		                  else
		                    if(p[offset8] > cb)
		                      if(p[offset9] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset6] > cb)
		                    if(p[offset7] > cb)
		                      if(p[offset8] > cb)
		                        if(p[offset9] > cb)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset5] > cb)
		                  if(p[offset6] > cb)
		                    if(p[offset7] > cb)
		                      if(p[offset8] > cb)
		                        if(p[offset9] > cb)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              if(p[offset1] > cb)
		                if(p[offset3] > cb)
		                  if(p[offset14] > cb)
		                    if(p[offset15] > cb)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else if(p[offset11] < c_b)
		        if(p[offset7] < c_b)
		          if(p[offset8] < c_b)
		            if(p[offset9] < c_b)
		              if(p[offset10] < c_b)
		                if(p[offset12] < c_b)
		                  if(p[offset13] < c_b)
		                    if(p[offset6] < c_b)
		                      if(p[offset5] < c_b)
		                        goto is_a_corner;
		                      else
		                        if(p[offset14] < c_b)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                    else
		                      if(p[offset14] < c_b)
		                        if(p[offset15] < c_b)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		  else if(p[offset2] < c_b)
		    if(p[offset9] > cb)
		      if(p[offset10] > cb)
		        if(p[offset11] > cb)
		          if(p[offset8] > cb)
		            if(p[offset12] > cb)
		              if(p[offset13] > cb)
		                if(p[offset14] > cb)
		                  if(p[offset15] > cb)
		                    goto is_a_corner;
		                  else
		                    if(p[offset6] > cb)
		                      if(p[offset7] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset5] > cb)
		                    if(p[offset6] > cb)
		                      if(p[offset7] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset4] > cb)
		                  if(p[offset5] > cb)
		                    if(p[offset6] > cb)
		                      if(p[offset7] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              if(p[offset3] > cb)
		                if(p[offset4] > cb)
		                  if(p[offset5] > cb)
		                    if(p[offset6] > cb)
		                      if(p[offset7] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            if(p[offset1] > cb)
		              if(p[offset12] > cb)
		                if(p[offset13] > cb)
		                  if(p[offset14] > cb)
		                    if(p[offset15] > cb)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		    else if(p[offset9] < c_b)
		      if(p[offset7] < c_b)
		        if(p[offset8] < c_b)
		          if(p[offset6] < c_b)
		            if(p[offset5] < c_b)
		              if(p[offset4] < c_b)
		                if(p[offset3] < c_b)
		                  if(p[offset1] < c_b)
		                    goto is_a_corner;
		                  else
		                    if(p[offset10] < c_b)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset10] < c_b)
		                    if(p[offset11] < c_b)
		                      if(p[offset12] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset10] < c_b)
		                  if(p[offset11] < c_b)
		                    if(p[offset12] < c_b)
		                      if(p[offset13] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              if(p[offset10] < c_b)
		                if(p[offset11] < c_b)
		                  if(p[offset12] < c_b)
		                    if(p[offset13] < c_b)
		                      if(p[offset14] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            if(p[offset10] < c_b)
		              if(p[offset11] < c_b)
		                if(p[offset12] < c_b)
		                  if(p[offset13] < c_b)
		                    if(p[offset14] < c_b)
		                      if(p[offset15] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    if(p[offset9] > cb)
		      if(p[offset10] > cb)
		        if(p[offset11] > cb)
		          if(p[offset8] > cb)
		            if(p[offset12] > cb)
		              if(p[offset13] > cb)
		                if(p[offset14] > cb)
		                  if(p[offset15] > cb)
		                    goto is_a_corner;
		                  else
		                    if(p[offset6] > cb)
		                      if(p[offset7] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset5] > cb)
		                    if(p[offset6] > cb)
		                      if(p[offset7] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset4] > cb)
		                  if(p[offset5] > cb)
		                    if(p[offset6] > cb)
		                      if(p[offset7] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              if(p[offset3] > cb)
		                if(p[offset4] > cb)
		                  if(p[offset5] > cb)
		                    if(p[offset6] > cb)
		                      if(p[offset7] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            if(p[offset1] > cb)
		              if(p[offset12] > cb)
		                if(p[offset13] > cb)
		                  if(p[offset14] > cb)
		                    if(p[offset15] > cb)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		    else if(p[offset9] < c_b)
		      if(p[offset7] < c_b)
		        if(p[offset8] < c_b)
		          if(p[offset10] < c_b)
		            if(p[offset11] < c_b)
		              if(p[offset6] < c_b)
		                if(p[offset5] < c_b)
		                  if(p[offset4] < c_b)
		                    if(p[offset3] < c_b)
		                      goto is_a_corner;
		                    else
		                      if(p[offset12] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                  else
		                    if(p[offset12] < c_b)
		                      if(p[offset13] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset12] < c_b)
		                    if(p[offset13] < c_b)
		                      if(p[offset14] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset12] < c_b)
		                  if(p[offset13] < c_b)
		                    if(p[offset14] < c_b)
		                      if(p[offset15] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
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
		                    goto is_a_corner;
		                  else
		                    if(p[offset10] > cb)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset10] > cb)
		                    if(p[offset11] > cb)
		                      if(p[offset12] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset10] > cb)
		                  if(p[offset11] > cb)
		                    if(p[offset12] > cb)
		                      if(p[offset13] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              if(p[offset10] > cb)
		                if(p[offset11] > cb)
		                  if(p[offset12] > cb)
		                    if(p[offset13] > cb)
		                      if(p[offset14] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            if(p[offset10] > cb)
		              if(p[offset11] > cb)
		                if(p[offset12] > cb)
		                  if(p[offset13] > cb)
		                    if(p[offset14] > cb)
		                      if(p[offset15] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		    else if(p[offset9] < c_b)
		      if(p[offset10] < c_b)
		        if(p[offset11] < c_b)
		          if(p[offset8] < c_b)
		            if(p[offset12] < c_b)
		              if(p[offset13] < c_b)
		                if(p[offset14] < c_b)
		                  if(p[offset15] < c_b)
		                    goto is_a_corner;
		                  else
		                    if(p[offset6] < c_b)
		                      if(p[offset7] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset5] < c_b)
		                    if(p[offset6] < c_b)
		                      if(p[offset7] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset4] < c_b)
		                  if(p[offset5] < c_b)
		                    if(p[offset6] < c_b)
		                      if(p[offset7] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              if(p[offset3] < c_b)
		                if(p[offset4] < c_b)
		                  if(p[offset5] < c_b)
		                    if(p[offset6] < c_b)
		                      if(p[offset7] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            if(p[offset1] < c_b)
		              if(p[offset12] < c_b)
		                if(p[offset13] < c_b)
		                  if(p[offset14] < c_b)
		                    if(p[offset15] < c_b)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
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
		                      goto is_a_corner;
		                    else
		                      if(p[offset12] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                  else
		                    if(p[offset12] > cb)
		                      if(p[offset13] > cb)
		                        if(p[offset14] > cb)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset12] > cb)
		                    if(p[offset13] > cb)
		                      if(p[offset14] > cb)
		                        if(p[offset15] > cb)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else if(p[offset11] < c_b)
		        if(p[offset12] < c_b)
		          if(p[offset13] < c_b)
		            if(p[offset10] < c_b)
		              if(p[offset14] < c_b)
		                if(p[offset15] < c_b)
		                  if(p[offset1] < c_b)
		                    goto is_a_corner;
		                  else
		                    if(p[offset8] < c_b)
		                      if(p[offset9] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset6] < c_b)
		                    if(p[offset7] < c_b)
		                      if(p[offset8] < c_b)
		                        if(p[offset9] < c_b)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset5] < c_b)
		                  if(p[offset6] < c_b)
		                    if(p[offset7] < c_b)
		                      if(p[offset8] < c_b)
		                        if(p[offset9] < c_b)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              if(p[offset1] < c_b)
		                if(p[offset3] < c_b)
		                  if(p[offset14] < c_b)
		                    if(p[offset15] < c_b)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
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
		                        goto is_a_corner;
		                      else
		                        if(p[offset14] > cb)
		                          if(p[offset15] > cb)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else if(p[offset12] < c_b)
		          if(p[offset13] < c_b)
		            if(p[offset14] < c_b)
		              if(p[offset15] < c_b)
		                if(p[offset1] < c_b)
		                  if(p[offset3] < c_b)
		                    goto is_a_corner;
		                  else
		                    if(p[offset10] < c_b)
		                      if(p[offset11] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset8] < c_b)
		                    if(p[offset9] < c_b)
		                      if(p[offset10] < c_b)
		                        if(p[offset11] < c_b)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset6] < c_b)
		                  if(p[offset7] < c_b)
		                    if(p[offset8] < c_b)
		                      if(p[offset9] < c_b)
		                        if(p[offset10] < c_b)
		                          if(p[offset11] < c_b)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
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
		                          goto is_a_corner;
		                        else
		                          if(p[offset15] > cb)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else if(p[offset14] < c_b)
		            if(p[offset15] < c_b)
		              if(p[offset1] < c_b)
		                if(p[offset3] < c_b)
		                  if(p[offset6] < c_b)
		                    goto is_a_corner;
		                  else
		                    if(p[offset13] < c_b)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset10] < c_b)
		                    if(p[offset11] < c_b)
		                      if(p[offset12] < c_b)
		                        if(p[offset13] < c_b)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset8] < c_b)
		                  if(p[offset9] < c_b)
		                    if(p[offset10] < c_b)
		                      if(p[offset11] < c_b)
		                        if(p[offset12] < c_b)
		                          if(p[offset13] < c_b)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else if(p[offset7] < c_b)
		          if(p[offset3] < c_b)
		            if(p[offset1] < c_b)
		              if(p[offset6] < c_b)
		                if(p[offset8] < c_b)
		                  goto is_a_corner;
		                else
		                  if(p[offset15] < c_b)
		                    goto is_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset13] < c_b)
		                  if(p[offset14] < c_b)
		                    if(p[offset15] < c_b)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              if(p[offset8] < c_b)
		                if(p[offset9] < c_b)
		                  if(p[offset10] < c_b)
		                    if(p[offset6] < c_b)
		                      goto is_a_corner;
		                    else
		                      if(p[offset11] < c_b)
		                        if(p[offset12] < c_b)
		                          if(p[offset13] < c_b)
		                            if(p[offset14] < c_b)
		                              if(p[offset15] < c_b)
		                                goto is_a_corner;
		                              else
		                                goto is_not_a_corner;
		                            else
		                              goto is_not_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            if(p[offset10] < c_b)
		              if(p[offset11] < c_b)
		                if(p[offset12] < c_b)
		                  if(p[offset8] < c_b)
		                    if(p[offset9] < c_b)
		                      if(p[offset6] < c_b)
		                        goto is_a_corner;
		                      else
		                        if(p[offset13] < c_b)
		                          if(p[offset14] < c_b)
		                            if(p[offset15] < c_b)
		                              goto is_a_corner;
		                            else
		                              goto is_not_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                    else
		                      if(p[offset1] < c_b)
		                        if(p[offset13] < c_b)
		                          if(p[offset14] < c_b)
		                            if(p[offset15] < c_b)
		                              goto is_a_corner;
		                            else
		                              goto is_not_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                  else
		                    if(p[offset1] < c_b)
		                      if(p[offset13] < c_b)
		                        if(p[offset14] < c_b)
		                          if(p[offset15] < c_b)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          if(p[offset14] < c_b)
		            if(p[offset15] < c_b)
		              if(p[offset1] < c_b)
		                if(p[offset3] < c_b)
		                  if(p[offset6] < c_b)
		                    goto is_a_corner;
		                  else
		                    if(p[offset13] < c_b)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset10] < c_b)
		                    if(p[offset11] < c_b)
		                      if(p[offset12] < c_b)
		                        if(p[offset13] < c_b)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset8] < c_b)
		                  if(p[offset9] < c_b)
		                    if(p[offset10] < c_b)
		                      if(p[offset11] < c_b)
		                        if(p[offset12] < c_b)
		                          if(p[offset13] < c_b)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
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
		                          goto is_a_corner;
		                        else
		                          if(p[offset15] > cb)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else if(p[offset12] < c_b)
		          if(p[offset13] < c_b)
		            if(p[offset14] < c_b)
		              if(p[offset15] < c_b)
		                if(p[offset1] < c_b)
		                  if(p[offset3] < c_b)
		                    goto is_a_corner;
		                  else
		                    if(p[offset10] < c_b)
		                      if(p[offset11] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset8] < c_b)
		                    if(p[offset9] < c_b)
		                      if(p[offset10] < c_b)
		                        if(p[offset11] < c_b)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset6] < c_b)
		                  if(p[offset7] < c_b)
		                    if(p[offset8] < c_b)
		                      if(p[offset9] < c_b)
		                        if(p[offset10] < c_b)
		                          if(p[offset11] < c_b)
		                            goto is_a_corner;
		                          else
		                            goto is_not_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
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
		                        goto is_a_corner;
		                      else
		                        if(p[offset14] > cb)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                    else
		                      if(p[offset14] > cb)
		                        if(p[offset15] > cb)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else if(p[offset11] < c_b)
		        if(p[offset12] < c_b)
		          if(p[offset13] < c_b)
		            if(p[offset10] < c_b)
		              if(p[offset14] < c_b)
		                if(p[offset15] < c_b)
		                  if(p[offset1] < c_b)
		                    goto is_a_corner;
		                  else
		                    if(p[offset8] < c_b)
		                      if(p[offset9] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset6] < c_b)
		                    if(p[offset7] < c_b)
		                      if(p[offset8] < c_b)
		                        if(p[offset9] < c_b)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset5] < c_b)
		                  if(p[offset6] < c_b)
		                    if(p[offset7] < c_b)
		                      if(p[offset8] < c_b)
		                        if(p[offset9] < c_b)
		                          goto is_a_corner;
		                        else
		                          goto is_not_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              if(p[offset1] < c_b)
		                if(p[offset3] < c_b)
		                  if(p[offset14] < c_b)
		                    if(p[offset15] < c_b)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
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
		                      goto is_a_corner;
		                    else
		                      if(p[offset12] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                  else
		                    if(p[offset12] > cb)
		                      if(p[offset13] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset12] > cb)
		                    if(p[offset13] > cb)
		                      if(p[offset14] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset12] > cb)
		                  if(p[offset13] > cb)
		                    if(p[offset14] > cb)
		                      if(p[offset15] > cb)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		    else if(p[offset9] < c_b)
		      if(p[offset10] < c_b)
		        if(p[offset11] < c_b)
		          if(p[offset8] < c_b)
		            if(p[offset12] < c_b)
		              if(p[offset13] < c_b)
		                if(p[offset14] < c_b)
		                  if(p[offset15] < c_b)
		                    goto is_a_corner;
		                  else
		                    if(p[offset6] < c_b)
		                      if(p[offset7] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset5] < c_b)
		                    if(p[offset6] < c_b)
		                      if(p[offset7] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset4] < c_b)
		                  if(p[offset5] < c_b)
		                    if(p[offset6] < c_b)
		                      if(p[offset7] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              if(p[offset3] < c_b)
		                if(p[offset4] < c_b)
		                  if(p[offset5] < c_b)
		                    if(p[offset6] < c_b)
		                      if(p[offset7] < c_b)
		                        goto is_a_corner;
		                      else
		                        goto is_not_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            if(p[offset1] < c_b)
		              if(p[offset12] < c_b)
		                if(p[offset13] < c_b)
		                  if(p[offset14] < c_b)
		                    if(p[offset15] < c_b)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
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
		                    goto is_a_corner;
		                  else
		                    if(p[offset10] > cb)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset10] > cb)
		                    if(p[offset11] > cb)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset10] > cb)
		                  if(p[offset11] > cb)
		                    if(p[offset12] > cb)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              if(p[offset10] > cb)
		                if(p[offset11] > cb)
		                  if(p[offset12] > cb)
		                    if(p[offset13] > cb)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            if(p[offset10] > cb)
		              if(p[offset11] > cb)
		                if(p[offset12] > cb)
		                  if(p[offset13] > cb)
		                    if(p[offset14] > cb)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          if(p[offset10] > cb)
		            if(p[offset11] > cb)
		              if(p[offset12] > cb)
		                if(p[offset13] > cb)
		                  if(p[offset14] > cb)
		                    if(p[offset15] > cb)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else if(p[offset7] < c_b)
		    if(p[offset8] < c_b)
		      if(p[offset9] < c_b)
		        if(p[offset6] < c_b)
		          if(p[offset5] < c_b)
		            if(p[offset4] < c_b)
		              if(p[offset3] < c_b)
		                if(p[offset2] < c_b)
		                  if(p[offset1] < c_b)
		                    goto is_a_corner;
		                  else
		                    if(p[offset10] < c_b)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                else
		                  if(p[offset10] < c_b)
		                    if(p[offset11] < c_b)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		              else
		                if(p[offset10] < c_b)
		                  if(p[offset11] < c_b)
		                    if(p[offset12] < c_b)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              if(p[offset10] < c_b)
		                if(p[offset11] < c_b)
		                  if(p[offset12] < c_b)
		                    if(p[offset13] < c_b)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            if(p[offset10] < c_b)
		              if(p[offset11] < c_b)
		                if(p[offset12] < c_b)
		                  if(p[offset13] < c_b)
		                    if(p[offset14] < c_b)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          if(p[offset10] < c_b)
		            if(p[offset11] < c_b)
		              if(p[offset12] < c_b)
		                if(p[offset13] < c_b)
		                  if(p[offset14] < c_b)
		                    if(p[offset15] < c_b)
		                      goto is_a_corner;
		                    else
		                      goto is_not_a_corner;
		                  else
		                    goto is_not_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    goto is_not_a_corner;

		is_a_corner:
			bmin=b_test;
			goto end;

		is_not_a_corner:
			bmax=b_test;
			goto end;

		end:

		if(bmin == bmax - 1 || bmin == bmax)
			return bmin;
		b_test = (bmin + bmax) / 2;
	}
}

