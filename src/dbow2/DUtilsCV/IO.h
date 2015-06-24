/**
 * File: IO.h
 * Project: DUtilsCV library
 * Author: Dorian Galvez
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

#ifndef __D_CV_IO__
#define __D_CV_IO__

#include <iostream>
#include <vector>
#include <opencv/cv.h>
#ifndef HAVE_OPENCV3
#include <opencv2/legacy/legacy.hpp>
#endif // HAVE_OPENCV3
#include <string>

namespace DUtilsCV
{

/// Input/Output functions for storage and printing
class IO
{
public:

  /** 
   * Saves a set of keypoints in the opencv-format
   * @param filename
   * @param keys
   * @param nodename name of the node of the set of keypoints to store
   *  then in xml/yaml format
   */
  static void saveKeyPoints(const std::string &filename,
    const std::vector<cv::KeyPoint> &keys,
    const std::string &nodename = "keys");
  
  /**
   * Loads a set of keypoints
   * @param filename
   * @param keys
   * @param nodename
   */
  static void loadKeyPoints(const std::string &filename,
    std::vector<cv::KeyPoint> &keys,
    const std::string &nodename = "keys");
  
  /**
   * Prints a mat of scalar values
   * @param m
   * @param name optional name given when printing
   * @param f stream where m is printed out
   */
  static void print(const cv::Mat &m,
    const std::string &name = "", std::ostream &f = std::cout);
  
  /**
   * Prints any mat of the type given
   * @param m
   * @param name optional name given when printing
   * @param f stream where m is printed out
   */
  template<class T>
  static void print(const cv::Mat &m, 
    const std::string &name = "", std::ostream &f = std::cout);
  
  /** 
   * Prints the size of the given matrix
   * @param m
   * @param name optional name given when printing
   * @param f stream
   */
  static void printSize(const cv::Mat &m,
    const std::string &name = "", std::ostream &f = std::cout);
  
  /**
   * Prints the data type of the given matrix
   * @param m 
   * @param name optional name given when printing
   * @param f stream
   */
  static void printType(const cv::Mat &m,
    const std::string &name = "", std::ostream &f = std::cout);
  
  /**
   * Saves any type of data that supports the "write" function
   * @param filename
   * @param c the data structure to save
   * @param nodename
   */
  template<class T>
  static void save(const std::string &filename, const T& c,
    const std::string &nodename = "data");

  /**
   * Loads any type of data that supports the "read" function
   * @param filename
   * @param c the data structure to load
   * @param nodename
   */
  template<class T>
  static void load(const std::string &filename, T& c,
    const std::string &nodename = "data");

#ifndef HAVE_OPENCV3
  // Save and Load functions to make calling easier

  /**
   * Saves a fern classifier
   * @param filename
   * @param c
   * @param nodename
   */
  inline static void save(const std::string &filename, 
    const cv::FernClassifier &c,
    const std::string &nodename = "fern_classifier");

  /**
   * Loads a fern classifier
   * @param filename
   * @param c
   * @param nodename
   */
  inline static void load(const std::string &filename, 
    cv::FernClassifier &c,
    const std::string &nodename = "fern_classifier");
#endif // HAVE_OPENCV3

};

// ---------------------------------------------------------------------------

template<class T>
void IO::print(const cv::Mat &m, const std::string &name, std::ostream &f)
{
  if(!name.empty()) f << name << " = ";
  f << "[ " << std::endl;
  for(int r = 0; r < m.rows; ++r)
  {
    for(int c = 0; c < m.cols; ++c)
    {
      f << m.at<T>(r, c) << " ";
    }
    f << std::endl;
  }
  f << "]" << std::endl;
}

// ---------------------------------------------------------------------------

template<class T>
void IO::save(const std::string &filename, const T& c,
  const std::string &nodename)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  c.write(fs, nodename);
}

// ---------------------------------------------------------------------------

template<class T>
void IO::load(const std::string &filename, T& c,
  const std::string &nodename)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  c.read(fs[nodename]);
}

// ---------------------------------------------------------------------------

#ifndef HAVE_OPENCV3
inline void IO::save(const std::string &filename, 
    const cv::FernClassifier &c,
    const std::string &nodename)
{
  IO::save<cv::FernClassifier>(filename, c, nodename);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

inline void IO::load(const std::string &filename, 
    cv::FernClassifier &c,
    const std::string &nodename)
{
  IO::load<cv::FernClassifier>(filename, c, nodename);
}
#endif
// ---------------------------------------------------------------------------

}

#endif

