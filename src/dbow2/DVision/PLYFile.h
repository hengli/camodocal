/**
 * File: PLYFile.h
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

#ifndef __PLY_FILE__
#define __PLY_FILE__

#include <string>
#include <vector>

namespace DVision {
namespace PMVS {

/// Manages a subset of PLY files
class PLYFile
{
public:
  /// Single PLY point
  struct PLYPoint
  {
    double x, y, z;
    double nx, ny, nz;
    int r, g, b;  
  };
  
public:
  
  /** 
   * Reads a simply PLY file and returns its 3D points and colors
   * @param filename
   * @param points
   */
  static void readFile(const std::string &filename, std::vector<PLYPoint>& points);
  
  /** 
   * Reads a simply PLY file and returns its 3D points and colors
   * @param filename
   * @param points
   */
  static void saveFile(const std::string &filename, const std::vector<PLYPoint>& points);
  
  /**
   * Reads the header of the given filename and returns the number of points
   * it contains
   * @param filename
   */
  static int getNumberOfPoints(const std::string &filename);
  
};

}
}

#endif
