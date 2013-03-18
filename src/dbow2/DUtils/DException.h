/*	
 * File: DException.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: October 6, 2009
 * Description: general exception of the library
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

#ifndef __D_EXCEPTION__
#define __D_EXCEPTION__

#include <stdexcept>
#include <string>
using namespace std;

namespace DUtils {

/// General exception
class DException :
	public exception
{
public:
	/**
	 * Creates an exception with a general error message
	 */
	DException(void) throw(): m_message("DUtils exception"){}

	/**
	 * Creates an exception with a custom error message
	 * @param msg: message
	 */
	DException(const char *msg) throw(): m_message(msg){}
	
	/**
	 * Creates an exception with a custom error message
	 * @param msg: message
	 */
	DException(const string &msg) throw(): m_message(msg){}

  /**
	 * Destructor
	 */
	virtual ~DException(void) throw(){}

	/**
	 * Returns the exception message
	 */
	virtual const char* what() const throw()
	{
		return m_message.c_str();
	}

protected:
  /// Error message
	string m_message;
};

}

#endif

