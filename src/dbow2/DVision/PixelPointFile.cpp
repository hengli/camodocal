/**
 * File: PixelPointFile.cpp
 * Project: DVision library
 * Author: Dorian Galvez-Lopez
 * Date: October 8, 2010
 * Description: manages structures of pixels + 3d
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

#include <vector>
#include <fstream>
#include "PixelPointFile.h"
#include "DUtils.h"

using namespace std;
using namespace DVision;

// ---------------------------------------------------------------------------

void PixelPointFile::saveFile(const std::string &filename,
    const std::vector<PixelPoint> &points)
{
  // Format:
  // N
  // u v x y z idx
  // ...
  
  fstream f(filename.c_str(), ios::out);
  if(!f.is_open()) throw DUtils::DException("Cannot open file " + filename);
  
  f << points.size() << endl;
  
  f.setf(ios::fixed, ios::floatfield);
  f.precision(4);
  
  std::vector<PixelPoint>::const_iterator pit;
  for(pit = points.begin(); pit != points.end(); ++pit)
  {
    f << pit->u << " "
      << pit->v << " "
      << pit->x << " "
      << pit->y << " "
      << pit->z << " "
      << pit->idx << endl;
  }

  f.close();
}

// ---------------------------------------------------------------------------

void PixelPointFile::readFile(const std::string &filename,
    std::vector<PixelPoint> &points)
{
  points.clear();
  
  fstream f(filename.c_str(), ios::in);
  if(!f.is_open()) throw DUtils::DException("Cannot open file " + filename);
  
  int N;
  f >> N;
  
  points.reserve(N);
  
  for(int i = 0; i < N; ++i)
  {
    float u, v, x, y, z;
    int idx;
    f >> u >> v >> x >> y >> z >> idx;
    
    if(!f.fail())
    {
      points.push_back(PixelPoint(u, v, x, y, z, idx));
    }else
    {
      break;
    }
  }
  
  f.close();
}

// ---------------------------------------------------------------------------



