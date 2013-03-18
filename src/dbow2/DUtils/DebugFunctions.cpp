/*
 * File: DebugFunctions.cpp
 * Author: Dorian Galvez-Lopez
 * Date: April 2012
 * Description: debug functions
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

#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>

#include "DebugFunctions.h"

using namespace std;
using namespace DUtils;

#ifdef _WIN32
#ifndef WIN32
#define WIN32
#endif
#endif

#ifndef WIN32
extern "C" 
{
  #include <sys/types.h>
  #include <unistd.h>
}
#endif

// ----------------------------------------------------------------------------

unsigned long DebugFunctions::getMemoryUsage()
{
#ifdef WIN32
  return 0;
#else

  pid_t pid = getpid();
  
  char buf[64];
  if(64 == snprintf(buf, 64, "/proc/%d/statm", pid)) return 0;

  fstream f(buf, ios::in);
  
  if(!f.is_open()) return 0;
  
  // http://www.kernel.org/doc/man-pages/online/pages/man5/proc.5.html
  // format (measured in pages):
  //  size       total program size
  //             (same as VmSize in /proc/[pid]/status)
  //  resident   resident set size
  //             (same as VmRSS in /proc/[pid]/status)
  //  share      shared pages (from shared mappings)
  //  text       text (code)
  //  lib        library (unused in Linux 2.6)
  //  data       data + stack
  //  dt         dirty pages (unused in Linux 2.6)

  unsigned long sz;
  f >> sz; // pages
  
  f.close();
  
  long pagesize = sysconf(_SC_PAGE_SIZE); // bytes
  
  if(pagesize == -1)
    return 0;
  else
    return sz * pagesize;

#endif
}


// ----------------------------------------------------------------------------

std::string DebugFunctions::formatBytes(unsigned long bytes, 
  unsigned long factor)
{
  stringstream ss;
  
  if(bytes < factor)
    ss << bytes << " B";
  else
  {
    const int N = 3;
    std::string suffix[N] = { " KB", " MB", " GB" };

    double f = (double)factor;
    double v = (double)bytes / f;
    int i = 0;
    for(; i < N-1 && v >= f; ++i) v /= f;
    
    ss << v << setprecision(2) << suffix[i];
  }
  
  return ss.str();
}

// ----------------------------------------------------------------------------


