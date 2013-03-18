/**
 * File: IO.cpp
 * Project: DUtilsCV library
 * Author: Dorian Galvez-Lopez
 * Date: October 11, 2010
 * Description: OpenCV-related IO functions
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

#include <iostream>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string>
#include "IO.h"
#include "Types.h"

using namespace std;
using namespace DUtilsCV;

// ----------------------------------------------------------------------------

void IO::saveKeyPoints(const std::string &filename,
    const std::vector<cv::KeyPoint> &keys,
    const std::string &nodename)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  cv::write(fs, nodename, keys);
}

// ----------------------------------------------------------------------------
  
void IO::loadKeyPoints(const std::string &filename,
    std::vector<cv::KeyPoint> &keys,
    const std::string &nodename)
{
  keys.resize(0);
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  cv::read(fs[nodename], keys);
}

// ----------------------------------------------------------------------------

void IO::print(const cv::Mat &m, const std::string &name, std::ostream &f)
{
  switch(m.type())
  {
    case CV_8U: IO::print<unsigned char>(m, name, f); break;
    case CV_8S: IO::print<char>(m, name, f); break;
    case CV_16U: IO::print<unsigned short>(m, name, f); break;
    case CV_16S: IO::print<short>(m, name, f); break;
    case CV_32S: IO::print<int>(m, name, f); break;
    case CV_32F: IO::print<float>(m, name, f); break;
    case CV_64F: IO::print<double>(m, name, f); break;
  }
}

// ----------------------------------------------------------------------------

void IO::printSize(const cv::Mat &m, const std::string &name, std::ostream &f)
{
  if(!name.empty()) f << name << ": ";
  if(m.channels() == 1)
    f << m.rows << " x " << m.cols;
  else
    f << m.rows << " x " << m.cols << " x " << m.channels();
  f << std::endl;
}

// ----------------------------------------------------------------------------

void IO::printType(const cv::Mat &m, const std::string &name, std::ostream &f)
{
  if(!name.empty()) f << name << ": ";
  f << Types::type(m) << std::endl;
}

// ----------------------------------------------------------------------------

