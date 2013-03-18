/**
 * File: BundleCamera.cpp
 * Project: DVision library
 * Author: Dorian Galvez-Lopez
 * Date: November 2, 2010
 * Description: Class to read camera info from Bundle output files
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
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <opencv/cv.h>
#include "BundleCamera.h"

using namespace DVision::Bundle;
using namespace std;

// ---------------------------------------------------------------------------

void CameraFile::Camera::save(const std::string &filename, 
      const std::string &comment) const
{
  fstream f(filename.c_str(), ios::out);
  if(!f.is_open()) throw string("BundleCamera: cannot open ") + filename;

  if(comment.empty())
    f << "# Single camera (opencv reference)" << endl;
  else
  {
    string cm = comment;
    replace(cm.begin(), cm.end(), '\n', ' ');
    f << "# " << cm << endl;
  }
  f << "1 0" << endl; // 1 camera, 0 points

  this->save(f);
  
  f.close();
}

// ---------------------------------------------------------------------------

void CameraFile::Camera::load(const std::string &filename)
{
  fstream f(filename.c_str(), ios::in);
  if(!f.is_open()) throw string("BundleCamera: cannot open ") + filename;
  
  // Format: 
  // # Comment
  // Number_of_cameras Number_of_points_(we_can_ignore)
  // 
  // Each camera:
  // f k1 k2
  // R11 R12 R13
  // R21 R22 R23
  // R31 R32 R33
  // t1 t2 t3
  
  int ncameras = -1;
  string line;
  while(!f.eof() && ncameras < 0){
    getline(f, line);
    
    if(!f.eof())
    {
      if(line[0] != '#')
      {
        stringstream ss(line);
        ss >> ncameras;
      }
    }
  }
  
  load(f);
  
  f.close();
}

// ---------------------------------------------------------------------------

void CameraFile::Camera::save(std::fstream &f) const
{
  f.setf(ios::fixed, ios::floatfield);
  f.precision(6);
  f << this->f << " " << this->k1 << " " << this->k2 << endl;
    
  for(int r = 0; r < 3; ++r)
  {
    for(int c = 0; c < 3; ++c)
    {
      f << setprecision(12) << this->R.at<double>(r,c) << " ";      
    }
    f << endl;
  }
  
  for(int r = 0; r < 3; ++r)
    f << this->t.at<double>(r,0) << " ";
  f << endl;
}

// ---------------------------------------------------------------------------

void CameraFile::Camera::load(std::fstream &f)
{
  R.create(3, 3, CV_64F);
  t.create(3, 1, CV_64F);
  
  string line;
  {
    getline(f, line); // f k1 k2
    stringstream ss(line);
    ss >> this->f >> k1 >> k2;
  }
  
  {
    getline(f, line); // R11 R12 R13
    stringstream ss(line);

    ss >> R.at<double>(0,0)
      >> R.at<double>(0,1)
      >> R.at<double>(0,2);
  }
  
  {
    getline(f, line); // R21 R22 R23
    stringstream ss(line);

    ss >> R.at<double>(1,0)
      >> R.at<double>(1,1)
      >> R.at<double>(1,2);
  }
    
  {
    getline(f, line); // R31 R32 R33
    stringstream ss(line);

    ss >> R.at<double>(2,0)
      >> R.at<double>(2,1)
      >> R.at<double>(2,2);
  }
  
  {
    getline(f, line); // t1 t2 t3
    stringstream ss(line);
    
    ss >> t.at<double>(0,0)
      >> t.at<double>(1,0)
      >> t.at<double>(2,0);
  }
}

// ---------------------------------------------------------------------------

void CameraFile::readFile(const std::string &filename, 
    std::vector<Camera> &cameras)
{
  // Format: 
  // # Comment
  // Number_of_cameras Number_of_points_(we_can_ignore)
  // 
  // Each camera:
  // f k1 k2
  // R11 R12 R13
  // R21 R22 R23
  // R31 R32 R33
  // t1 t2 t3
  
  fstream f(filename.c_str(), ios::in);
  if(!f.is_open()) throw string("BundleCamera: cannot open ") + filename;
  
  string line;
  
  int ncameras = -1;
  
  while(!f.eof() && ncameras < 0){
    getline(f, line);
    
    if(!f.eof())
    {
      if(line[0] != '#')
      {
        stringstream ss(line);
        ss >> ncameras;
      }
    }
  }
  
  readFromStream(f, ncameras, cameras);
  
  f.close();
}

// ---------------------------------------------------------------------------

void CameraFile::readFromStream(std::fstream &f, int N,
  std::vector<Camera> &cameras)
{
  cameras.resize(N);
  for(int i = 0; i < N; ++i)
  {
    cameras[i].load(f);
  }
}

// ---------------------------------------------------------------------------
      
void CameraFile::saveFile(const std::string &filename, 
    const std::vector<Camera> &cameras)
{
  fstream f(filename.c_str(), ios::out);
  if(!f.is_open()) throw string("BundleCamera: cannot open ") + filename;
  
  f << "# A contraption file with the bundle.out format in opencv reference system" << endl;
  f << cameras.size() << " 0" << endl;
  
  saveToStream(f, cameras);
  
  f.close();
}

// ---------------------------------------------------------------------------

void CameraFile::saveToStream(std::fstream &f, 
  const std::vector<Camera> &cameras)
{
  vector<Camera>::const_iterator cit;
  for(cit = cameras.begin(); cit != cameras.end(); ++cit)
  {
    cit->save(f);
  }
}

// ---------------------------------------------------------------------------

