/*	
 * File: ConfigFile.cpp
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: February 15, 2011
 * Description: simple text file for human-machine storage
 *
 * Note: see ConfigFile.h for examples of configuration text files.
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

#include "DException.h"
#include "FileModes.h"
#include "LineFile.h"
#include "ConfigFile.h"
#include "StringFunctions.h"
#include <vector>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
using namespace std;
using namespace DUtils;

// ----------------------------------------------------------------------------

ConfigFile::ConfigFile()
{
}

// ----------------------------------------------------------------------------

ConfigFile::~ConfigFile()
{
  Close();
}

// ----------------------------------------------------------------------------

ConfigFile::ConfigFile(const char *filename, const FILE_MODES mode)
{
  Init(filename, mode);
}

// ----------------------------------------------------------------------------

ConfigFile::ConfigFile(const string &filename, const FILE_MODES mode)
{
  Init(filename.c_str(), mode);
}

// ----------------------------------------------------------------------------

void ConfigFile::Init(const char *filename, const FILE_MODES mode)
{
  m_unknowns = 0;
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

// ----------------------------------------------------------------------------

void ConfigFile::OpenForReading(const char *filename)
{
  m_file.OpenForReading(filename);
  readContent();
}

// ----------------------------------------------------------------------------

void ConfigFile::OpenForWriting(const char *filename)
{
  m_file.OpenForWriting(filename);
}

// ----------------------------------------------------------------------------

void ConfigFile::OpenForAppending(const char *filename)
{
  m_file.OpenForAppending(filename);
}

// ----------------------------------------------------------------------------

void ConfigFile::Close()
{
  if(m_file.GetOpenMode() & APPEND || m_file.GetOpenMode() & WRITE)
  {
    writeContent();
  }

  m_file.Close();
}

// ----------------------------------------------------------------------------

void ConfigFile::writeContent()
{
  std::map<std::string, std::string>::const_iterator mit;
  
  for(mit = m_data.begin(); mit != m_data.end(); ++mit)
  {
    string s = mit->first + " = " + mit->second;
    m_file << s;
  }
}

// ----------------------------------------------------------------------------

void ConfigFile::readContent()
{
  m_data.clear();
  
  m_unknowns = 0;
  const string U = "?"; // prefix of anonymous tokens
  
  string s;
  
  while(!m_file.Eof())
  {
    m_file >> s;
    
    // remove comments, but escape \#
    StringFunctions::removeFrom(s, '#', "\\#");
    StringFunctions::trim(s);
    
    vector<std::string> tokens;
    StringFunctions::split(s, tokens, "=", 1); // split only once
    
    if(tokens.size() >= 2)
    {
      StringFunctions::trim(tokens[0]);
      StringFunctions::trim(tokens[1]);
      
      // if value is between quotes, remove them because it is a literal string
      if(tokens[1].size() > 1)
      {
        if(tokens[1][0] == '"' && tokens[1][tokens[1].size()-1] == '"')
        {
          tokens[1].erase(tokens[1].begin() + tokens[1].size() - 1);
          tokens[1].erase(tokens[1].begin());
        }
      }
      
      put(tokens[0], tokens[1]);
    }
    else if(tokens.size() == 1)
    {
      // anonymous token
      stringstream ss;
      ss << U << m_unknowns++;
      put(ss.str(), tokens[0]);
    }
  }
  
  // resolve replace variable names in the right hand side tokens now
  resolveVariables();
}

// ----------------------------------------------------------------------------

void ConfigFile::resolveVariables()
{
  std::map<std::string, std::string>::iterator mit;
  std::set<string> used;
  
  for(mit = m_data.begin(); mit != m_data.end(); ++mit)
  {
    used.clear();
    used.insert(mit->first); // avoid circular dependencies
    resolveVar(mit->second, used);
  }
}

// ----------------------------------------------------------------------------

void ConfigFile::resolveVar(std::string &value, const std::set<string> &used)
{
  vector<pair<string, string> > replacements;
  map<string, string>::iterator mit;
  
  const char D = '$'; // variable char 
  
  for(string::size_type p = value.find(D); p != string::npos; 
    p = value.find(D, p+1) )
  {
    if(p != string::npos)
    {
      // check if there is a string in () after p
      if(p+1 < value.length() && value[p+1] == '(')
      {
        string::size_type pf = value.find(')', p+2);
        if(pf != string::npos)
        {
          string token = value.substr(p + 2, pf - p - 2);
          
          if(used.find(token) != used.end())
          {
            cout << "Warning: ConfigFile: circular dependency found: \"" 
              << token << "\"" << endl;
          }
          else
          {
            // check if token is valid
            mit = m_data.find(token);
            
            if(mit == m_data.end())
            {
              cout << "Warning: ConfigFile: token unknown: \"" 
                << token << "\"" << endl;
            }
            else
            {
              // resolve 
              const string extoken = string(1, D) + "(" + token + ")";
              set<string> used2 = used;
              used2.insert(token);
              resolveVar(mit->second, used2);
              replacements.push_back(make_pair(extoken, mit->second));
            } 
          } // if circular dependency
          
        } // if final part of token found
      } // if initial part of token found
    } // if $ found
  } // for
  
  // do the replacements
  StringFunctions::replace(value, replacements);
}

// ----------------------------------------------------------------------------


