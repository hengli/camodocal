/**
 * File: PatchFile.cpp
 * Project: DVision library
 * Author: Dorian Galvez-Lopez
 * Date: October 4, 2010
 * Description: Manager of simple PVMS .patch files
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

#include "PatchFile.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

using namespace std;
using namespace DVision::PMVS;

// ----------------------------------------------------------------------------

void PatchFile::readFile(const std::string &filename, 
  std::vector<Patch> &patches)
{
  fstream f(filename.c_str(), ios::in);
  if(!f.is_open()) throw string("PatchFile: cannot read file ") + filename;
  
  string line;
  getline(f, line); // PATCHES keyword
  getline(f, line); // number of patches
  const int N = PatchFile::parseInt(line);

  patches.resize(N);
  
  for(int i = 0; i < N; ++i)
  {
    Patch& patch = patches[i];
    
    getline(f, line); // PATCHS keyword
    getline(f, line); // 3d coords
        
    sscanf(line.c_str(), "%lf %lf %lf %lf", 
      &patch.x, &patch.y, &patch.z, &patch.s);
        
    getline(f, line); // normal vector
    sscanf(line.c_str(), "%lf %lf %lf %lf", 
      &patch.nx, &patch.ny, &patch.nz, &patch.ns);
            
    getline(f, line); // photometric consistency
    sscanf(line.c_str(), "%lf %lf %lf", 
      &patch.consistency, &patch.dbg1, &patch.dbg2);
          
    getline(f, line); // strong list length
    int n = PatchFile::parseInt(line);
    patch.strong_visibility_list.resize(n);
    getline(f, line); // strong list
    stringstream ss(line);
    for(int j = 0; j < n; ++j)
    {
      ss >> patch.strong_visibility_list[j];
    }
    
    getline(f, line); // weak list length
    n = PatchFile::parseInt(line);
    patch.weak_visibility_list.resize(n);
    getline(f, line); // weak list
    stringstream ss2(line);
    for(int j = 0; j < n; ++j)
    {
      ss2 >> patch.weak_visibility_list[j];
    }
    
    getline(f, line); // blank
  }

  f.close();
}

// ----------------------------------------------------------------------------

void PatchFile::saveFile(const std::string &filename,
    const std::vector<Patch> &patches)
{
  fstream f(filename.c_str(), ios::out);
  if(!f.is_open()) throw string("PatchFile: cannot write in ") + filename;
  
  f.precision(6);
  
  f << "PATCHES" << endl;
  f << patches.size() << endl;

  for(unsigned int i = 0; i < patches.size(); ++i)
  {
    const Patch& patch = patches[i];
    
    f << "PATCHS" << endl;
    f << patch.x << " " << patch.y << " " << patch.z << " " << patch.s << endl;
    f << patch.nx << " " << patch.ny << " " << patch.nz << " " << patch.ns << endl;
    f << patch.consistency << " " << patch.dbg1 << " " << patch.dbg2 << endl;
    
    f << patch.strong_visibility_list.size() << endl;
    for(unsigned int j = 0; j < patch.strong_visibility_list.size(); ++j)
    {
      f << patch.strong_visibility_list[j] << " ";
    }
    f << endl;
    
    f << patch.weak_visibility_list.size() << endl;
    for(unsigned int j = 0; j < patch.weak_visibility_list.size(); ++j)
    {
      f << patch.weak_visibility_list[j] << " ";
    }
    f << endl;
    
    f << endl; // blank line 
  }
  
  f.close();
}

// ----------------------------------------------------------------------------

void PatchFile::readFile(const std::string &filename, 
    std::vector<std::vector<int> > &visibility, bool use_weak_list)
{
  visibility.resize(0);
  
  fstream f(filename.c_str(), ios::in);
  if(!f.is_open()) throw string("PatchFile: cannot read file ") + filename;
  
  string line;
  
  getline(f, line); // header "PATCHES"
  getline(f, line); // number of patches
  
  int N;
  sscanf(line.c_str(), "%d", &N);
  
  visibility.reserve(N);
  
  for(int pt_idx = 0; pt_idx < N; ++pt_idx)
  {
    getline(f, line); // header "PATCHS"
    getline(f, line); // point in homogeneous coordinates
    getline(f, line); // surface normal
    getline(f, line); // photometric consistency info

    // number of images with visibility and consistency
    PatchFile::readVisibilityIndices(f, visibility, pt_idx);

    if(use_weak_list)
    {
      // use also the visibility w/o consistency list
      PatchFile::readVisibilityIndices(f, visibility, pt_idx);
    }else{
      // ignore those indices
      getline(f, line); // number of images with visibility but w/o consistency
      getline(f, line); // list of indices
    }

    getline(f, line); // blank
  }
    
  f.close();
}

// ----------------------------------------------------------------------------

void PatchFile::readVisibilityIndices(std::fstream &f, 
    std::vector<std::vector<int> > &visibility, int pt_idx)
{
  std::string line;
  getline(f, line); // number of indices
  
  int n;
  sscanf(line.c_str(), "%d", &n);
    
  getline(f, line); // list of indices
  stringstream ss(line);
  
  for(int j = 0; j < n; ++j)
  {
    int img_idx;
    ss >> img_idx;
    
    if((int)visibility.size() <= img_idx)
    {
      visibility.resize(img_idx+1);
    }
    
    visibility[img_idx].push_back(pt_idx);
  }
}

// ----------------------------------------------------------------------------

int PatchFile::parseInt(const std::string &s)
{
  stringstream ss(s);
  int n;
  ss >> n;
  return n;
}

// ----------------------------------------------------------------------------

int PatchFile::getNumberOfPoints(const std::string &filename)
{
  fstream f(filename.c_str(), ios::in);
  if(!f.is_open()) throw string("PatchFile: cannot read file ") + filename;
  
  string line;
  
  getline(f, line); // header "PATCHES"
  getline(f, line); // number of patches
  
  int N;
  sscanf(line.c_str(), "%d", &N);
  
  f.close();
  
  return N;
}

// ----------------------------------------------------------------------------

