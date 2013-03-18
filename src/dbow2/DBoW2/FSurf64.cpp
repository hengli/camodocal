/**
 * File: FSurf64.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: functions for Surf64 descriptors
 *
 * This file is licensed under a Creative Commons 
 * Attribution-NonCommercial-ShareAlike 3.0 license. 
 * This file can be freely used and users can use, download and edit this file 
 * provided that credit is attributed to the original author. No users are 
 * permitted to use this file for commercial purposes unless explicit permission
 * is given by the original author. Derivative works must be licensed using the
 * same or similar license.
 * Check http://creativecommons.org/licenses/by-nc-sa/3.0/ to obtain further
 * details.
 *
 */
 
#include <vector>
#include <string>
#include <sstream>

#include "FClass.h"
#include "FSurf64.h"

using namespace std;

namespace DBoW2 {

// --------------------------------------------------------------------------

void FSurf64::meanValue(const std::vector<FSurf64::pDescriptor> &descriptors, 
  FSurf64::TDescriptor &mean)
{
  mean.resize(0);
  mean.resize(FSurf64::L, 0);
  
  float s = descriptors.size();
  
  vector<FSurf64::pDescriptor>::const_iterator it;
  for(it = descriptors.begin(); it != descriptors.end(); ++it)
  {
    const FSurf64::TDescriptor &desc = **it;
    for(int i = 0; i < FSurf64::L; i += 4)
    {
      mean[i  ] += desc[i  ] / s;
      mean[i+1] += desc[i+1] / s;
      mean[i+2] += desc[i+2] / s;
      mean[i+3] += desc[i+3] / s;
    }
  }
}

// --------------------------------------------------------------------------
  
double FSurf64::distance(const FSurf64::TDescriptor &a, const FSurf64::TDescriptor &b)
{
  double sqd = 0.;
  for(int i = 0; i < FSurf64::L; i += 4)
  {
    sqd += (a[i  ] - b[i  ])*(a[i  ] - b[i  ]);
    sqd += (a[i+1] - b[i+1])*(a[i+1] - b[i+1]);
    sqd += (a[i+2] - b[i+2])*(a[i+2] - b[i+2]);
    sqd += (a[i+3] - b[i+3])*(a[i+3] - b[i+3]);
  }
  return sqd;
}

// --------------------------------------------------------------------------

std::string FSurf64::toString(const FSurf64::TDescriptor &a)
{
  stringstream ss;
  for(int i = 0; i < FSurf64::L; ++i)
  {
    ss << a[i] << " ";
  }
  return ss.str();
}

// --------------------------------------------------------------------------
  
void FSurf64::fromString(FSurf64::TDescriptor &a, const std::string &s)
{
  a.resize(FSurf64::L);
  
  stringstream ss(s);
  for(int i = 0; i < FSurf64::L; ++i)
  {
    ss >> a[i];
  }
}

// --------------------------------------------------------------------------

void FSurf64::toMat32F(const std::vector<TDescriptor> &descriptors, 
    cv::Mat &mat)
{
  if(descriptors.empty())
  {
    mat.release();
    return;
  }
  
  const int N = descriptors.size();
  const int L = FSurf64::L;
  
  mat.create(N, L, CV_32F);
  
  for(int i = 0; i < N; ++i)
  {
    const TDescriptor& desc = descriptors[i];
    float *p = mat.ptr<float>(i);
    for(int j = 0; j < L; ++j, ++p)
    {
      *p = desc[j];
    }
  } 
}

// --------------------------------------------------------------------------

} // namespace DBoW2

