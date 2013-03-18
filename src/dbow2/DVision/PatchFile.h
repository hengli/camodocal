/**
 * File: PatchFile.h
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

#ifndef __PATCH_FILE__
#define __PATCH_FILE__

#include <string>
#include <vector>

namespace DVision {

/// Manages data from PMVS software
namespace PMVS {

/// Manages patch files
class PatchFile
{
public:
  /// A single patch
  struct Patch
  {
    double x, y, z, s; // s: scale (homogenous coordinates)
    double nx, ny, nz, ns; // ns: idem
    
    /// Extra and debugging information
    double consistency, dbg1, dbg2;
    /// Indexes of covisible points
    std::vector<int> strong_visibility_list;
    /// Indexes of points that may be covisible with this patch
    std::vector<int> weak_visibility_list;
  };
public:
  /** 
   * Reads a patch file and returns the visibility info of each file
   * @param filename
   * @param visibility visibility[i]= list of point indices seen in the i-th img
   * @param use_weak_list if true, those points which are likely to be visible
   *        but do not have texture consistency are also considered as visible
   */
  static void readFile(const std::string &filename, 
    std::vector<std::vector<int> > &visibility, bool use_weak_list = false);

  /**
   * Reads the patchs of a patch file
   * @param filename
   * @param patches
   */
  static void readFile(const std::string &filename, 
    std::vector<Patch> &patches);

  /**
   * Saves the patches into filename
   * @param filename
   * @param patches
   */
  static void saveFile(const std::string &filename,
    const std::vector<Patch> &patches);

protected:

  /**
   * Reads a list of indices from f and updates the visibility vector accordingly
   * @param f fstream in a line with the number of indices in the next line
   * @param visibility
   * @param pt_idx index of the current 3d point
   */
  static void readVisibilityIndices(std::fstream &f, 
    std::vector<std::vector<int> > &visibility, int pt_idx);
  
  /** 
   * Returns the integer represented by the string s
   * @param s
   * @return int
   */
  static int parseInt(const std::string &s);

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
