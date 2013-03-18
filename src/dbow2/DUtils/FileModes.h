/*	
 * File: FileModes.h
 * Project: DUtils library
 * Author: Dorian Galvez
 * Date: April 2010
 * Description: types used with file managers
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
#ifndef __D_FILE_MODES__
#define __D_FILE_MODES__

namespace DUtils {

/// Opening file modes
enum FILE_MODES {
	READ = 1,
	WRITE = 2,
	APPEND = 4,
	WRITE_APPEND = 6 // == WRITE & APPEND
};

}

#endif

