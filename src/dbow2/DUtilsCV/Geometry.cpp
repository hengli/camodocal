/**
 * File: Geometry.cpp
 * Project: DUtilsCV library
 * Author: Dorian Galvez-Lopez
 * Date: October 11, 2010
 * Description: OpenCV-related geometry functions
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
#include "Geometry.h"
#include "Types.h"

using namespace DUtilsCV;

// ----------------------------------------------------------------------------

// This function is deprecated
double Geometry::sqDistance(const cv::Mat &a, const cv::Mat &b)
{
  assert( (a.rows == 1 || a.cols == 1) && (b.rows == 1 || b.cols == 1) );
  assert( a.rows*a.cols == b.rows*b.cols );
  
  int N = (a.rows > 1 ? a.rows : a.cols);
  
  // ToDo: avoid copy
  
  double *adata = new double[N];
  double *bdata = new double[N];
  
  Types::vectorize<double>(a, adata);
  Types::vectorize<double>(b, bdata);
  
  double ret = 0.0;
  
  if( N % 4 == 0)
  {
    for(int i = 0; i < N; i += 4)
    {
      ret += (adata[ i ]-bdata[ i ]) * (adata[ i ]-bdata[ i ]);
      ret += (adata[i+1]-bdata[i+1]) * (adata[i+1]-bdata[i+1]);
      ret += (adata[i+2]-bdata[i+2]) * (adata[i+2]-bdata[i+2]);
      ret += (adata[i+3]-bdata[i+3]) * (adata[i+3]-bdata[i+3]);
    }
  }
  else
  {
    for(int i = 0; i < N; ++i)
      ret += (adata[i]-bdata[i]) * (adata[i]-bdata[i]);
  }
  
  delete [] adata;
  delete [] bdata;
  
  return ret;

}

// ----------------------------------------------------------------------------

