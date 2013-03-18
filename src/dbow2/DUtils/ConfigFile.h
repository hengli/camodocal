/*	
 * File: ConfigFile.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: February 15, 2011
 * Description: simple text file for human-machine storage
 *
 * Short description of configuration text files:
 * - The text files managed are simple files with one entry per line.
 * - Entries have the form "item = value".
 * - Blank spaces and tabs are ignored in the beginning and the end of the
 *   value part.
 * - The value can be given between quotation marks ("value") to void ignoring
 *   the blank spaces.
 * - The equal symbol (=) can safely apper in the value part. The first equal
 *   symbol is used to tell item and value apart only.
 * - The number symbol (#) can be used for comments. It can be escaped with
 *   the sequence \# 
 * - The name of the items can be used as variables in the value part with
 *   the notation $(item). 
 * - Anonymous values can appear. It is, entries with no "item = ". These are
 *   automatically given the entry with name "?X", where X is an integer which
 *   starts at 0. These values can be retrieved as strings with getAnonymous.
 * 
 * Example:
 *   name = Albert Einstein
 *   age = 76
 *   location = "Ulm, Germany" # quotation marks are not necessary here
 *   eq = E = mc^2
 *   description = $(name) was $(age) and was born in $(location). $(eq)!
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

#ifndef __D_CONFIG_FILE__
#define __D_CONFIG_FILE__

#include "DException.h"
#include "FileModes.h"
#include "LineFile.h"
#include "StringFunctions.h"
#include <map>
#include <set>
#include <fstream>

namespace DUtils {

/// Manages simple text-based configuration files
class ConfigFile
{
public:
  ConfigFile();
  ~ConfigFile();
  
  /** 
   * Creates a config file by opening a file
   * @param filename
   * @param mode: READ or WRITE
   * @throws DException if cannot open the file
   */
  ConfigFile(const char *filename, const FILE_MODES mode);
  
  /** 
   * Creates a config file by opening a file
   * @param filename
   * @param mode: READ or WRITE
   * @throws DException if cannot open the file
   */
  ConfigFile(const std::string &filename, const FILE_MODES mode);
  
  /**
   * Opens a file for reading. It closes any other opened file
   * @param filename
   * @throws DException if cannot create the file
   */
  void OpenForReading(const char *filename);
  
  /**
   * Opens a file for reading. It closes any other opened file
   * @param filename
   * @throws DException if cannot create the file
   */
  inline void OpenForReading(const std::string &filename)
  {
    OpenForReading(filename.c_str());
  }

  /** 
   * Opens a file for writing. It closes any other opened file
   * @param filename
   * @throws DException if cannot create the file
   */
  void OpenForWriting(const char *filename);
  
  /** 
   * Opens a file for writing. It closes any other opened file
   * @param filename
   * @throws DException if cannot create the file
   */
  inline void OpenForWriting(const std::string &filename)
  {
    OpenForWriting(filename.c_str());
  }

  /**
   * Opens a file for writing at the end. It closes any other opened file
   * @param filename
   * @throws DException if cannot open the file
   */
  void OpenForAppending(const char *filename);
  
  /**
   * Opens a file for writing at the end. It closes any other opened file
   * @param filename
   * @throws DException if cannot open the file
   */
  inline void OpenForAppending(const std::string &filename)
  {
    OpenForAppending(filename.c_str());
  }

  /**
   * Closes any opened file. It is not necessary to call this function
	 * explicitly
	 */
	void Close();
	
	/**
	 * Gets data from the file. Returns 0 or similar if it does not exist.
	 * @param name
	 */
	template<class T>
	T get(const std::string &name) const;
	
	/**
	 * Gets data from the file. Returns 0 or similar if it does not exist
	 * @param name
	 */
	template<class T>
	T get(const char *name) const;
	
	/**
	 * Returns the number of anonymous entries when reading a file
	 * @return n
	 */
	inline int sizeAnonymous() const;
	
	/** 
	 * Gets anonymous data from the file. Returns 0 or similar if it does not
	 * exist
	 * @param n index of anonymous entry
	 */
	template<class T>
	T getAnonymous(int n) const;
	
	/**
	 * Writes or overwrites a piece of data with the given name
	 * @param name
	 * @param data
	 */
	template<class T>
	void put(const std::string &name, const T &data);
	
	/**
	 * Writes or overwrites a piece of data with the given name
	 * @param name
	 * @param data
	 */
	template<class T>
	void put(const char *name, const T &data);

protected:

  /**
   * Initiates the file
   */
  void Init(const char *filename, const FILE_MODES mode);
  
  /**
   * Reads the content of the m_file already opened
   * and stores it in m_data.
   * This operation also resolves the possible name deferencing when using
   * variables
   */
  void readContent();
  
  /**
   * Writes the items into the file
   */
  void writeContent();
  
  /**
   * Resolves the value of the variables used as right-hand expressions
   */
  void resolveVariables();
  
  /** 
   * Resolves the value of a single variable
   * @param value the string value to modify
   * @param used the names of the tokens that cannot be replaced because of
   *   circular dependencies
   */
  void resolveVar(std::string &value, const std::set<string> &used);
	
protected:

  /// Data read from the file
  std::map<std::string, std::string> m_data; // <key, value>
  /// File
  LineFile m_file;
  /// Number of unknowns entries read
  int m_unknowns;

};

// ----------------------------------------------------------------------------

template<class T>
T ConfigFile::getAnonymous(int n) const
{
  const char U = '?';
  stringstream ss;
  ss << U << n;
  return get<T>(ss.str().c_str());
}

template<class T>
T ConfigFile::get(const std::string &name) const
{
  return get<T>(name.c_str());
}

template<class T>
T ConfigFile::get(const char *name) const
{
  std::map<std::string, std::string>::const_iterator it =
    m_data.find(name);
  
  if(it != m_data.end())
    return StringFunctions::fromString<T>(it->second);
  else
    return T();
}

// ----------------------------------------------------------------------------

template<class T>
void ConfigFile::put(const std::string &name, const T &data)
{
  put<T>(name.c_str(), data);
}

template<class T>
void ConfigFile::put(const char *name, const T &data)
{
  pair<std::map<std::string, std::string>::iterator, bool> status;
  
  std::string value = StringFunctions::toString<T>(data);
  
  status = m_data.insert(make_pair(name, value));
  
  if(!status.second)
  {
    status.first->second = value;
  }
}

// ----------------------------------------------------------------------------

int ConfigFile::sizeAnonymous() const
{
  return m_unknowns;
}

// ----------------------------------------------------------------------------

}

#endif
