/**
 * File: PLYFile.cpp
 * Project: DVision library
 * Author: Dorian Galvez-Lopez
 * Date: October 8, 2010
 * Description: Manager of simple PLY files
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

#include "PLYFile.h"
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

using namespace std;
using namespace DVision::PMVS;

// ---------------------------------------------------------------------------

void PLYFile::readFile(const std::string &filename, 
  std::vector<PLYPoint>& points)
{
  // Format:
  // ply
  // ... headers (ignored) ...
  // end_header
  // Points
  // 
  // Each point:
  // x y z nx ny nz r g b
  
  points.clear();
  points.reserve(5000);
  
  fstream f(filename.c_str(), ios::in);
  if(!f.is_open()) throw string("PLYFile: cannot read file ") + filename;
  
  string line;
  bool header = true;
  while(!f.eof())
  {
    getline(f, line);
    if(!f.eof() && !line.empty())
    {
      if(header){
        header = (line.compare("end_header") != 0);
      }else{
        // point
        points.push_back(PLYPoint());
        PLYPoint &p = points.back();
        
        stringstream ss(line);
        ss >> p.x >> p.y >> p.z >> p.nx >> p.ny >> p.nz >> p.r >> p.g >> p.b;
      }
    }
  }
  
  f.close();
  
}

// ---------------------------------------------------------------------------

void PLYFile::saveFile(const std::string &filename, 
  const std::vector<PLYPoint>& points)
{
  fstream f(filename.c_str(), ios::out);
  if(!f.is_open()) throw string("PLYFile: cannot open file ") + filename;
  
  // standard header
  f << "ply" << endl;
  f << "format ascii 1.0" << endl;
  f << "element vertex " << points.size() << endl;
  f << "property float x" << endl;
  f << "property float y" << endl;
  f << "property float z" << endl;
  f << "property float nx" << endl;
  f << "property float ny" << endl;
  f << "property float nz" << endl;
  f << "property uchar diffuse_red" << endl;
  f << "property uchar diffuse_green" << endl;
  f << "property uchar diffuse_blue" << endl;
  f << "end_header" << endl;
  
  f.precision(6);
  
  vector<PLYPoint>::const_iterator pit;
  for(pit = points.begin(); pit != points.end(); ++pit)
  {
    f << pit->x << " "
      << pit->y << " "
      << pit->z << " "
      << pit->nx << " "
      << pit->ny << " "
      << pit->nz << " "
      << pit->r << " "
      << pit->g << " "
      << pit->b << endl;
  }
  
  f.close();
}

// ---------------------------------------------------------------------------

int PLYFile::getNumberOfPoints(const std::string &filename)
{
  fstream f(filename.c_str(), ios::in);
  if(!f.is_open()) throw string("PLYFile: cannot open file ") + filename;
  
  // standard header
  string line;
  getline(f, line); // ply
  getline(f, line); // format ascii 1.0
  getline(f, line); // element vertex N
  
  int N;
  if(0 == sscanf(line.c_str(), "element vertex %d", &N))
    throw string("PLYFile: format not supported in ") + filename;
  
  return N;
}

// ---------------------------------------------------------------------------

