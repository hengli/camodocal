/**
 * File: BundleCamera.h
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

#ifndef __BUNDLE_CAMERA__
#define __BUNDLE_CAMERA__

#include <vector>
#include <opencv/cv.h>
#include <string>
#include <fstream>
 
namespace DVision {

/// Manages data from Bundle software
namespace Bundle {

/// Manages Bundle camera files
class CameraFile
{
public:

  /// A camera from a camera file
  class Camera
  {
  public:
    /// Focal length and distortion parameters
    float f, k1, k2;
    /// Camera rotation
    cv::Mat R;
    /// Camera translation
    cv::Mat t;

  public:
    /**
     * Creates an empty camera
     */
    Camera(){}
    
    /**
     * Loads the camera from a file
     * @param filename
     */
    Camera(const std::string &filename)
    {
      load(filename);
    }
    
    /**
     * Saves the camera in a file with the bund.out format
     * @param filename
     * @param comment comment for the first line
     */
    void save(const std::string &filename, 
      const std::string &comment = "") const;
      
    /**
     * Reads the first camera of filename
     * @param filename
     */
    void load(const std::string &filename);
      
  protected:
    friend class CameraFile;
    
    /**
     * Saves the camera info, w/o any other mata information, in the
     * given stream
     * @param f
     */
    void save(std::fstream &f) const;
    
    /**
     * Loads the camera info, w/o any other mata information, from the
     * given stream
     * @param f
     */
    void load(std::fstream &f);
    
  };

public:
  /** 
   * Returns the camera information from the given file
   * @param filename
   * @param cameras
   */
  static void readFile(const std::string &filename, 
    std::vector<Camera> &cameras);
      
  /** 
   * Saves the camera into the given filename
   * @param filename
   * @param cameras
   */
  static void saveFile(const std::string &filename, 
    const std::vector<Camera> &cameras);

protected:

  /** 
   * Returns the camera information from the given stream
   * @param f
   * @param N number of cameras
   * @param returned cameras
   */
  static void readFromStream(std::fstream &f, int N,
    std::vector<Camera> &cameras);
  
  /**
   * Writes the camera info to the stream
   * @param f
   * @note it does not write the number of cameras
   */
  static void saveToStream(std::fstream &f, 
    const std::vector<Camera> &cameras);

};

}
}

#endif

