/*
 * File: StringFunctions.h
 * Author: Dorian Galvez-Lopez
 * Date: December 2010
 * Description: string functions
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

#ifndef __D_STRING__
#define __D_STRING__

#include <string>
#include <vector>
#include <string>
#include <sstream>

namespace DUtils {

/// Functions to manipulate strings
class StringFunctions
{
public:

  /**
   * Splits the given string into single tokens
   * @param s string
   * @param tokens returned tokens (no empty tokens are returned)
   * @param delims delimitation characters
   * @param max_splits maximum number of splits. If -1, all the possible splits
   *   are done. Otherwise, those delimiters found after the max number of
   *   splits has been done are ignored
   */
  static void split(const std::string &s, std::vector<std::string> &tokens,
    const std::string &delims = " \t\n", int max_splits = -1);

  /** 
   * Removes blank spaces, tabs and new lines from the beginning and the 
   * end of the string
   * @param s
   */
  static void trim(std::string &s);

  /**
   * Removes from a string all the chars after finding the first char given
   * (this included)
   * @param s string to modify
   * @param c character to find and to start the removal from
   * @param escape if given, the char c found in the string is ignored if it 
   *   is the same as escape. This string must include the character c
   * @example removeFrom(s, '#', "\#")
   */
  static void removeFrom(std::string &s, const char c, 
    const std::string &escape = "");
  
  /** 
   * Replaces each occurrence of one of several strings in s by another string
   * @param s the original string whose substring will be replaced
   * @param map a vector of pairs of string where the first one is the substring
   *   to search for, and the second one, the replacing text
   * @note the entries are searched for as they appear in the map vector. An
   *   entry can replace the text already put by previous entries
   */
  static void replace(std::string &s, 
    const std::vector<std::pair<std::string, std::string> > &map);
  
  /** 
   * The same as above, but only with one "pair"
   * @param s
   * @param search
   * @param rep
   */
  static void replace(std::string &s, const std::string &search,
    const std::string &rep);

  /**
   * Converts a piece of data into a string
   * @param data
   */
  template<class T>
  static std::string toString(const T& data);
  
  /**
   * Returns a data type from a string representation
   * @param s
   */  
  template<class T>
  static T fromString(const std::string &s);
  
};

// --------------------------------------------------------------------------

template<class T>
std::string StringFunctions::toString(const T& data)
{
  std::stringstream ss;
  ss << data;
  return ss.str();
}

// --------------------------------------------------------------------------

template<>
inline std::string StringFunctions::fromString(const std::string &s)
{ 
  return s;
}

template<class T>
T StringFunctions::fromString(const std::string &s)
{ 
  if(s.empty())
    return T();
  else
  {
    std::stringstream ss(s);
    T ret;
    ss >> ret;
    return ret;
  }
}

// --------------------------------------------------------------------------

}

#endif

