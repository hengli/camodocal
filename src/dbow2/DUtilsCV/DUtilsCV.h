/*
 * File: DUtilsCV.h
 * Project: DUtilsCV library
 * Author: Dorian Galvez-Lopez
 * Date: September 23, 2010
 * Description: several OpenCV-related functions for solving common tasks
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

/*! \mainpage DUtilsCV Library
 *
 * DUtilsCV library for C++ and OpenCV:
 * Collection of classes with general utilities for C++ and OpenCV applications.
 *
 * Written by Dorian Galvez-Lopez,
 * University of Zaragoza
 * 
 * Check my website to obtain updates: http://webdiis.unizar.es/~dorian
 *
 * \section requirements Requirements
 * This library requires the DUtils library and the OpenCV library.
 *
 * \section license License
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL) as 
 * published by the Free Software Foundation, either version 3 of the License, 
 * or any later version.
 *
 */

#ifndef __D_UTILS_CV__
#define __D_UTILS_CV__

/// Several utilities for C++ and OpenCV programs
namespace DUtilsCV
{
}

// Drawing functions
#include "Drawing.h"

// GUI
#include "GUI.h"

// IO
#include "IO.h"

// Mat functions
#include "Mat.h"

// Data types
#include "Types.h"

// Geometry
#include "Transformations.h"
#include "Geometry.h"

// CV version
#include "CvVersion.h"

#endif

