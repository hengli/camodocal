/**
 * File: Mat.h
 * Project: DUtilsCV library
 * Author: Dorian Galvez-Lopez
 * Date: February 21, 2012
 * Description: functions for matrices
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

#ifndef __D_CV_MAT__
#define __D_CV_MAT__

#include <opencv/cv.h>
#include <vector>

namespace DUtilsCV
{

class Mat
{
public:

  /**
   * Removes rows from a matrix
   * @param m matrix to modify
   * @param rows indices of rows to remove. This vector is modified
   */
  static void removeRows(cv::Mat &m, std::vector<unsigned int> &rows);

  /**
   * Removes rows from a matrix
   * @param m matrix to modify
   * @param rows indices of rows to remove
   */
  static void removeRows(cv::Mat &m, const std::vector<unsigned int> &rows);

public:

  /**
   * Removes rows from a matrix
   * @param m matrix to modify
   * @param rows indices of rows to remove. This vector must contain valid
   *   indices, in [0, m.rows), in ascending order and without repeated items
   */
  static void _removeRows(cv::Mat &m, const std::vector<unsigned int> &rows);

protected:

  /**
   * Removes rows from a matrix with continuous data type T
   * @param m matrix to modify. isContinuous must hold
   * @param rows indices of rows to remove. This vector must contain valid
   *   indices, in [0, m.rows), in ascending order and without repeated items
   */
  template<class T>
  static void _removeRowsContinuous(cv::Mat &m, 
    const std::vector<unsigned int> &rows);

  /**
   * Removes rows from a matrix with data type T
   * @param m matrix to modify
   * @param rows indices of rows to remove. This vector must contain valid
   *   indices, in [0, m.rows), in ascending order and without repeated items
   */
  static void _removeRowsNonContinuous(cv::Mat &m, 
    const std::vector<unsigned int> &rows);

};

// --------------------------------------------------------------------------

template<class T>
void Mat::_removeRowsContinuous(cv::Mat &m, 
  const std::vector<unsigned int> &rows)
{
  // always preserve the order of the rest of rows
  
  // remove rows in descending order, grouping when possible
  int end_row = m.rows;
  int i_idx = (int)rows.size() - 1;
  while(i_idx >= 0)
  {
    int j_idx = i_idx - 1;
    while(j_idx >= 0 && ((int)(rows[i_idx] - rows[j_idx]) == i_idx - j_idx))
    {
      j_idx--;
    }
    //data.erase(data.begin() + indices[j_idx + 1], 
    //  data.begin() + indices[i_idx] + 1);
    
    std::copy( m.ptr<T>(rows[i_idx]+1), m.ptr<T>(end_row),
      m.ptr<T>(rows[j_idx + 1]) );
    
    end_row -= rows[i_idx] - rows[j_idx+1] + 1;
    
    i_idx = j_idx;
  }
  
  // remove last rows
  m.resize(end_row);
}

// --------------------------------------------------------------------------

} // namespace DUtilsCV

#endif

