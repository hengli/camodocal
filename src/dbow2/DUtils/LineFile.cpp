/*	
 * File: LineFile.cpp
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

#include "LineFile.h"
#include "DException.h"
#include "FileModes.h"
#include <vector>
#include <string>
using namespace std;

using namespace DUtils;

LineFile::LineFile(void): m_next_line("")
{
}

LineFile::~LineFile(void)
{
	Close();
}

LineFile::LineFile(const char *filename, const FILE_MODES mode)
{
	Init(filename, mode);
}

LineFile::LineFile(const string &filename, const FILE_MODES mode)
{
	Init(filename.c_str(), mode);
}

void LineFile::Init(const char *filename, const FILE_MODES mode)
{
	m_next_line = "";
	
	if(mode & READ){
		OpenForReading(filename);
	}else if((mode & WRITE) && (mode & APPEND)){
		OpenForAppending(filename);
	}else if(mode & WRITE){
		OpenForWriting(filename);
	}else{
		throw DException("Wrong access mode");
	}
}

void LineFile::OpenForReading(const char *filename)
{
	m_f.open(filename, ios::in);
	if(!m_f.is_open()){
		throw DException(string("Cannot open ") + filename + " for reading");
	}else{
		m_mode = READ;
	}
}

void LineFile::OpenForWriting(const char *filename)
{
	m_f.open(filename, ios::out);
	if(!m_f.is_open()){
		throw DException(string("Cannot open ") + filename + " for writing");
	}else{
		m_mode = WRITE;
	}
}

void LineFile::OpenForAppending(const char *filename)
{
	m_f.open(filename, ios::out | ios::app);
	if(!m_f.is_open()){
		throw DException(string("Cannot open ") + filename + " for writing");
	}else{
		m_mode = DUtils::FILE_MODES(WRITE | APPEND);
	}
}

void LineFile::Close()
{
	if(m_f.is_open()) m_f.close();
}

bool LineFile::Eof()
{
	if(!m_f.is_open()) return true;

	if(m_mode & READ){
		if(m_f.eof())
			return true;
		else if(!m_next_line.empty())
			return false;
		else{
			getline(m_f, m_next_line);
			return m_f.eof();
		}
	}else
		throw DException("Wrong access mode");

}

LineFile& LineFile::operator<< (const char *s)
{
	if(!m_f.is_open()) throw DException("File is not open");

	if(m_mode & WRITE)
		m_f << s << endl;
	else
		throw DException("Wrong access mode");
	
	return *this;
}

LineFile& LineFile::operator>> (string &s)
{
	if(!m_f.is_open()) throw DException("File is not open");

	if(m_mode & READ){
		if(m_f.eof()){
			s.clear();
		}else if(!m_next_line.empty()){
			s = m_next_line;
			m_next_line.clear();
		}else{
			getline(m_f, s);
			if(m_f.eof()){
				s.clear();
			}
		}
		
	}else
		throw DException("Wrong access mode");
	
	return *this;
}

LineFile& LineFile::operator>> (vector<string> &v)
{
	if(!m_f.is_open()) throw DException("File is not open");

  v.clear();

	if(m_mode & READ){
	  if(!m_next_line.empty()){
	    v.push_back(m_next_line);
		  m_next_line.clear();
		}
		
		string s;
		while(!m_f.eof())
		{
		  getline(m_f, s);
		  if(!m_f.eof()) v.push_back(s);
		}
		
	}else
		throw DException("Wrong access mode");
	
	return *this;
}

void LineFile::Dump(const vector<string> &v)
{
	if(!m_f.is_open()) throw DException("File is not open");
		
	if(m_mode & WRITE){
		vector<string>::const_iterator it;
		for(it = v.begin(); it != v.end(); it++){
			m_f << *it << endl;
		}
	}else
		throw DException("Wrong access mode");
}

void LineFile::DiscardLine()
{
	string nul;
	*this >> nul;
}


