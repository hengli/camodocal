/*	
 * File: LineFile.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: October 6, 2009
 * Description: reads and writes text line files
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
#ifndef __D_LINE_FILE__
#define __D_LINE_FILE__

#include "DException.h"
#include "FileModes.h"
#include <vector>
#include <fstream>
using namespace std;

namespace DUtils {

/// Reads and writes a file by string lines
class LineFile
{
public:
	/**
	 * Creates a linefile with no file
	 */
	LineFile(void);

	/**
	 * Closes any opened file
	 */
	~LineFile(void);

	/**
	 * Creates a linefile by opening a file
	 * @param filename
	 * @param mode: READ or WRITE
	 * @throws DException if cannot open the file
	 */
	LineFile(const char *filename, const FILE_MODES mode);
	
	/** 
	 * Creates a linefile by opening a file
	 * @param filename
	 * @param mode: READ or WRITE
	 * @throws DException if cannot open the file
	 */
	LineFile(const string &filename, const FILE_MODES mode);

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
	inline void OpenForReading(const string &filename)
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
	inline void OpenForWriting(const string &filename)
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
	inline void OpenForAppending(const string &filename)
	{
		OpenForAppending(filename.c_str());
	}

	/**
	 * Says whether the end of the file has been reached. It is not necessary
	 * to read a last empty line to reach eof
	 * @return true iff there is nothing else to read
	 * @throws DException if wrong access mode
	 */
	bool Eof();

	/**
	 * Closes any opened file. It is not necessary to call this function
	 * explicitly
	 */
	void Close();

	/**
	 * Writes a line
	 * @throws DException if wrong access mode
	 */
	LineFile& operator<< (const char *s);
	
	/**
	 * Writes a line
	 * @throws DException if wrong access mode
	 */
	inline LineFile& operator<< (const string &s)
	{
		return this->operator <<(s.c_str()); 
	}

	/**
	 * Reads a line
	 * @param s: string to write on
	 * @throws DException if wrong access mode
	 */
	LineFile& operator>> (string &s);

  /**
   * Reads all the remaining lines in the file
   * @param v vector to store the lines in
   * @throws DException if wrong access mode
   */
  LineFile& operator>> (vector<string> &v);

	/**
	 * Writes several lines at a time
	 * @param v: vector of line strings
	 * @throws DException if wrong access mode
	 */
	void Dump(const vector<string> &v);
	
	/**
	 * Writes several lines at a time
	 * @param v: vector of line strings
	 * @throws DException if wrong access mode
	 */
	inline LineFile& operator<< (const vector<string> &v)
	{
		Dump(v);
		return *this;
	}

	/**
	 * In reading mode, reads and throws away the next line
	 */
	void DiscardLine();
	
	/**
	 * Returns the opening mode
	 */
	inline FILE_MODES GetOpenMode() const { return m_mode; }

protected:

	/**
	 * Initializes the object by opening a file
	 * @param filename file to open
	 * @param mode opening mode
	 * @throws DException if cannot open the file
	 */
	void Init(const char *filename, const FILE_MODES mode);

protected:
  /// Opening mode
	FILE_MODES m_mode;		// opening mode
	/// File stream
	fstream m_f;			// fstream
	/// Next line to read
	string m_next_line;	// next line to read
};

}

#endif

