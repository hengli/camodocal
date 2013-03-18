/*
 * File: FileFunctions.h
 * Author: Dorian Galvez-Lopez
 * Date: June 2009
 * Description: file system functions
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

#ifndef __D_FILE_FUNCTIONS__
#define __D_FILE_FUNCTIONS__

#pragma once

#include <vector>
#include <string>

namespace DUtils {

/// General functions to manipulate directories and files
class FileFunctions
{
public:
	
	/**
	 * Creates the directory 'path'. The parent directory must exist
	 * @param path
	 */
	static void MkDir(const char *path);

	/**
	 * Removes a directory and its content
	 * @param path
	 */
	static void RmDir(const char *path);

	/**
	 * Removes a file 
	 * @param path
	 */
	static void RmFile(const char *path);

	/**
	 * Checks the existence of a folder
	 * @return true iff the directory exists
	 */
	static bool DirExists(const char *path);

	/**
	 * Checks the existence of a file
	 * @returns true iff the file exists
	 */
	static bool FileExists(const char *filename);

	/**
	 * Returns the relative path of the files located in the path given and 
	 * whose right path of the name matches 'right'
	 * @param path: path to directory
	 * @param right: string like "_L.png"
	 * @param sorted: if true, the files are sorted alphabetically
	 * @return path list
	 */
	static std::vector<std::string> Dir(const char *path, const char *right,
	  bool sorted = false);

	/**
	 * Extracts the filename of the given path
	 * @param filepath path
	 * @return file name
	 */
	static std::string FileName(const std::string filepath);

	/**
	 * Extracts the path, file name and extension of the given path
	 * @param filepath
	 * @param path (out): path to file
	 * @param filename (out): filename without extension or dot
	 * @param ext (out): extension without dot
	 */
	static void FileParts(const std::string filepath, std::string &path, 
		std::string &filename, std::string &ext);
	
};

}

#endif
