/*	
 * File: Math.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: April 2010
 * Modified: December 2010
 * Description: some math functions
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

#pragma once
#ifndef __D_MATH__
#define __D_MATH__

#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>

namespace DUtils {

/// Math functions
class Math {

public:

  /**
   * Returns the mean of a population
   * @param v population vector
   */
  template <class T>
  static double Mean(const std::vector<T> &v)
  {
    if(v.empty())
      return 0;
    else{
      double sum = 0;
      typename std::vector<T>::const_iterator it;
      for(it = v.begin(); it != v.end(); it++){
        sum += *it;
      }
      return sum/double(v.size());
    }
  }
  
  // ------------------------------------------------------------------------

  /**
   * Returns the standard deviation of a population
   * @param v population vector
   */
  template <class T>
  static double Stdev(const std::vector<T> &v)
  {
    return Math::Stdev<T>(v, Math::Mean<T>(v));
  }

  // ------------------------------------------------------------------------

  /**
   * Returns the standard deviation of a population
   * @param v population vector
   * @param mean the mean of the population
   */
  template <class T>
  static double Stdev(const std::vector<T> &v, double mean)
  {
    if(v.size() <= 1)
      return 0;
    else{
      // stdev = sqrt( Sum{ (x_i - mean)^2 } / (N-1) )
      double sum = 0;
      typename std::vector<T>::const_iterator it;
      for(it = v.begin(); it != v.end(); it++){
        sum += pow(*it - mean, 2);
      }
      return sqrt(sum/double(v.size()-1));
    }
  }

  // ------------------------------------------------------------------------

  /** 
   * Returns the median of a population
   * @param v
   */
  template <class T>
  static double Median(const std::vector<T> &v)
  {
    if(v.empty())
      return 0;
    else{
      std::vector<T> w = v;
      sort(w.begin(), w.end());
      int i = w.size() / 2;
      if(w.size() % 2 == 1)
      {
        return (double)w[i];
      }
      else
      {
        return ((double)w[i-1] + (double)w[i]) / 2.;
      }
    }
  }

  // ------------------------------------------------------------------------

  /**
   * Returns the minimum value of v
   * @param v
   * @return minimum value
   */
  template <class T>
  static T Min(const std::vector<T> &v)
  {
    return Math::MinMax(v, true, 
      bool2type<std::numeric_limits<T>::is_specialized>());
  }
  
  /**
   * Returns the maximum value of v
   * @param v
   * @return minimum value
   */
  template <class T>
  static T Max(const std::vector<T> &v)
  {
    return Math::MinMax(v, false, 
      bool2type<std::numeric_limits<T>::is_specialized>());
  }

  // ------------------------------------------------------------------------
  
  /**
   * Converts an angle in [0..360) into an angle in (-180..180]
   * @param angle angle in degrees
   * @return signed angle in degrees
   */
  template<class T>
  inline static T signedAngle(T angle)
  {
    return ( angle <= 180 ? angle : angle - 360 );
  }
  
  /**
   * Converts an angle in (-180..180] into an angle in [0..360)
   * @param angle angle in degrees
   * @return absolute angle in degrees
   */
  template<class T>
  inline static T absoluteAngle(T angle)
  {
    return ( angle >= 0 ? angle : angle + 360 );
  }
  
  // ------------------------------------------------------------------------

protected:
  // to test numeric types in compilation-time
  template<bool> struct bool2type { };

  /**
   * Returns the minimum or maximum for numeric types
   * @param v
   * @param minormax true for calculating the min, false otherwise
   */
  template<class T>
  static T MinMax(const std::vector<T> &v, bool minormax, bool2type<true>);

  /**
   * Returns the minimum or maximum for non-numeric types
   * @param v
   * @param minormax true for calculating the min, false otherwise
   */
  template<class T>
  static T MinMax(const std::vector<T> &v, bool minormax, bool2type<false>);

};

// ----------------------------------------------------------------------------

template<class T>
T Math::MinMax(const std::vector<T> &v, bool minormax, bool2type<true>)
{
  if(v.empty())
    return std::numeric_limits<T>::quiet_NaN( );
  else if(minormax)
    return *std::min_element(v.begin(), v.end());
  else
    return *std::max_element(v.begin(), v.end());
}

// ----------------------------------------------------------------------------

template<class T>
T Math::MinMax(const std::vector<T> &v, bool minormax, bool2type<false>)
{
  if(v.empty())
    return T();
  else if(minormax)
    return *std::min_element(v.begin(), v.end());
  else
    return *std::max_element(v.begin(), v.end());
}

// ----------------------------------------------------------------------------

}

#endif

