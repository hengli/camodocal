/**
 * File: FSurf64.h
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
 
#ifndef __D_T_F_SURF_64__
#define __D_T_F_SURF_64__

#include <opencv/cv.h>
#include <vector>
#include <string>

#include "FClass.h"

namespace DBoW2 {

/// Functions to manipulate SURF64 descriptors
class FSurf64: protected FClass
{
public:

  /// Descriptor type
  typedef std::vector<float> TDescriptor;
  /// Pointer to a single descriptor
  typedef const TDescriptor *pDescriptor;
  /// Descriptor length
  static const int L = 64; 

  /**
   * Returns the number of dimensions of the descriptor space
   * @return dimensions
   */
  inline static int dimensions()
  {
    return L;
  }

  /**
   * Calculates the mean value of a set of descriptors
   * @param descriptors vector of pointers to descriptors
   * @param mean mean descriptor
   */
  static void meanValue(const std::vector<pDescriptor> &descriptors, 
    TDescriptor &mean);
  
  /**
   * Calculates the (squared) distance between two descriptors
   * @param a
   * @param b
   * @return (squared) distance
   */
  static double distance(const TDescriptor &a, const TDescriptor &b);
  
  /**
   * Returns a string version of the descriptor
   * @param a descriptor
   * @return string version
   */
  static std::string toString(const TDescriptor &a);
  
  /**
   * Returns a descriptor from a string
   * @param a descriptor
   * @param s string version
   */
  static void fromString(TDescriptor &a, const std::string &s);

  /**
   * Returns a mat with the descriptors in float format
   * @param descriptors
   * @param mat (out) NxL 32F matrix
   */
  static void toMat32F(const std::vector<TDescriptor> &descriptors, 
    cv::Mat &mat);

};

} // namespace DBoW2

#endif
