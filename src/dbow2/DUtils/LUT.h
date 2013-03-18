/*
 * File: LUT.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: June 2012
 * Description: some LUTs for binary tasks
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

#ifndef __D_LUT__
#define __D_LUT__

namespace DUtils
{

class LUT
{
public:

  /// Number of bits set in 1 byte
  static int ones8bits[256]; 

};

} // namespace DUtils

#endif

