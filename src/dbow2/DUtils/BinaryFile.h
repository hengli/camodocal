/*	
 * File: BinaryFile.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: April 2010
 * Description: reads and writes binary files in network byte order.
 *    Manages endianness and data size automatically.
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
#ifndef __D_BINARY_FILE__
#define __D_BINARY_FILE__

#include "DException.h"
#include "FileModes.h"
#include <fstream>
using namespace std;

namespace DUtils {

/// Manages files in binary format
class BinaryFile
{
public:
	
	/**
	 * Creates a binary file with no file
	 */
	BinaryFile(void);

	/**
	 * Closes any opened file
	 */
	~BinaryFile(void);

	/**
	 * Creates a binary file by opening a file
	 * @param filename
	 * @param mode: READ or WRITE
	 * @throws DException if cannot open the file
	 */
	BinaryFile(const char *filename, const FILE_MODES mode);
	
	/**
	 * Creates a binary file by opening a file
	 * @param filename
	 * @param mode: READ or WRITE
	 * @throws DException if cannot open the file
	 */
	BinaryFile(const string &filename, const FILE_MODES mode);

	/**
	 * Opens a file for reading. It closes any other opened file
	 * @param filename
	 * @throws DException if cannot open the file
	 */
	void OpenForReading(const char *filename);
	
	/**
	 * Opens a file for reading. It closes any other opened file
	 * @param filename
	 * @throws DException if cannot open the file
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
	 * Says whether the end of the file has been reached. Requires to
	 * read the end of the file to return true
	 * @return true iff the end of the file has been already read
	 * @throws DException if wrong access mode
	 */
	inline bool Eof();

	/**
	 * Closes any opened file. It is not necessary to call this function
	 * explicitly
	 */
	void Close();

	/**
	 * Reads the next byte and throws it away
	 * @throws DException if wrong access mode
	 */
	inline void DiscardNextByte(){
		DiscardBytes(1);
	}

	/**
	 * Reads n bytes and discards them
	 * @param count number of bytes to discard
	 * @throws DException if wrong access mode
	 */
	void DiscardBytes(int count);

	/**
	 * Returns the number of bytes read in reading mode
	 * @return number of bytes read
	 */
	unsigned int BytesRead();

	/**
	 * Writes a byte char
	 * @throws DException if wrong access mode
	 */
	BinaryFile& operator<< (char v);

	/**
	 * Writes a 4 byte integer value
	 * @throws DException if wrong access mode
	 */
	BinaryFile& operator<< (int v);

	/**
	 * Writes a 4 byte float value
	 * @throws DException if wrong access mode
	 */
	BinaryFile& operator<< (float v);

	/**
	 * Writes a 8 byte float value
	 * @throws DException if wrong access mode
	 */
	BinaryFile& operator<< (double v);

	/**
	 * Reads a byte char
	 * @throws DException if wrong access mode
	 */
	BinaryFile& operator>>(char &v);

	/**
	 * Reads a 4 byte integer value
	 * @throws DException if wrong access mode
	 */
	BinaryFile& operator>>(int &v);

	/**
	 * Reads a 4 byte float value
	 * @throws DException if wrong access mode
	 */
	BinaryFile& operator>>(float &v);

	/**
	 * Reads a 8 byte float value
	 * @throws DException if wrong access mode
	 */
	BinaryFile& operator>>(double &v);

protected:

	/**
	 * Initializes the object by opening a file
	 * @param filename file to open
	 * @param mode opening mode
	 * @throws DException if cannot open the file
	 */
	void Init(const char *filename, const FILE_MODES mode);

	/** 
	 * Checks the endianness of this machine
	 */
	void setEndianness();

	/**
	 * Converts a float into 4 bytes in network order
	 * @param v float value
	 * @param buf (out) byte buffer output. Only buf[0..3] is used
	 */
	void hton_f(float v, char buf[8]) const;

	/**
	 * Converts a double into 8 bytes in network order
	 * @param d double value
	 * @param buf (out) byte buffer output
	 */
	void hton_d(double d, char buf[8]) const;

	/**
	 * Converts an array of bytes in network order into a 4 byte float
	 * @param buf byte array. only buf[0..3] is used
	 * @return float value
	 */
	float ntoh_f(char buf[8]) const;

	/**
	 * Converts an array of bytes in network order into a 8 byte double
	 * @param buf byte array
	 * @return double value
	 */
	double ntoh_d(char buf[8]) const;

	/** 
	 * Returns if this machine uses little endian
	 * @return true iff little endian
	 */
	inline bool isLittleEndian() const
	{
		return (m_is_little_endian == 1 ? true : false);
	}

protected:
  /// Opening mode
	FILE_MODES m_mode;		// opening mode
	/// File stream
	fstream m_f;			// fstream
	/// Auxiliar buffer
	char m_aux[8];	// auxiliar buffer

	/// Current machine endianness
	int m_is_little_endian; // 1: little endian, 0: big endian, -1: not set

};

}

#endif
