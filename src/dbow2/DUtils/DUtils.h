/*
 * File: DUtils.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: October 6, 2009
 * Description: include file for including all the library functionalities
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

/*! \mainpage DUtils Library
 *
 * DUtils library for C++:
 * Collection of classes with general utilities for C++ applications.
 *
 * Written by Dorian Galvez-Lopez,
 * University of Zaragoza
 * 
 * Check my website to obtain updates: http://webdiis.unizar.es/~dorian
 *
 * \section license License
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL) as 
 * published by the Free Software Foundation, either version 3 of the License, 
 * or any later version.
 *
 */


#pragma once

#ifndef __D_UTILS__
#define __D_UTILS__

/// Several utilities for C++ programs
namespace DUtils
{
}

// Exception
#include "DException.h"

// Files
#include "FileModes.h"
#include "LineFile.h"
#include "BinaryFile.h"
#include "FileFunctions.h"
#include "ConfigFile.h"

// Timestamp
#include "Timestamp.h"
#include "TimeManager.h"
#include "Profiler.h"

// Random numbers
#include "Random.h"

// Math
#include "Math.hpp"

// STL
#include "STL.h"

// Strings
#include "StringFunctions.h"

// LUTs
#include "LUT.h"

// Debug
#include "DebugFunctions.h"

#endif
