/*
 * File: DVision.h
 * Project: DVision library
 * Author: Dorian Galvez-Lopez
 * Date: October 4, 2010
 * Description: several functions for computer vision
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

/*! \mainpage DVision Library
 *
 * DVision library for C++ and OpenCV:
 * Collection of classes with computer vision functionality
 *
 * Written by Dorian Galvez-Lopez,
 * University of Zaragoza
 * 
 * Check my website to obtain updates: http://webdiis.unizar.es/~dorian
 *
 * \section requirements Requirements
 * This library requires the DUtils and DUtilsCV libraries and the OpenCV library.
 *
 * \section license License
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL) as 
 * published by the Free Software Foundation, either version 3 of the License, 
 * or any later version.
 *
 */

#ifndef __D_VISION__
#define __D_VISION__


/// Computer vision functions
namespace DVision
{
}

// Features and descriptors
#include "SurfSet.h"
#include "BRIEF.h"
#include "ORB.h"

// Image functions
#include "ImageFunctions.h"

// Matches
#include "Matches.h"

// Pixels and 3d
#include "PixelPointFile.h"

// Bundle interface
#include "BundleCamera.h"

// PMVS interface
#include "PMVSCamera.h"
#include "PatchFile.h"
#include "PLYFile.h"

// Epipolar geometry
#include "FSolver.h"
#include "HSolver.h"


#endif
