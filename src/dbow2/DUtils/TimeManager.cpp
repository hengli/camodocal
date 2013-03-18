/*
 * File: TimeManager.cpp
 * Author: Dorian Galvez-Lopez
 * Date: February 2011
 * Description: allows to sort a collection of timestamps and get them 
 *   at a desired frequency
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

#include <vector>
#include <algorithm>
#include "Timestamp.h"
#include "TimeManager.h"

using namespace std;
using namespace DUtils;

// ---------------------------------------------------------------------------

TimeManager::TimeManager()
{
}

// ---------------------------------------------------------------------------

TimeManager::~TimeManager()
{
}

// ---------------------------------------------------------------------------  

void TimeManager::add(const Timestamp &t)
{
  if(m_entries.empty())
  {
    m_is_sorted = true;
  }
  else if(m_entries.back().timestamp > t)
  {
    m_is_sorted = false;
  }
  
  m_entries.push_back(tEntry(t, m_entries.size()) );
}

// ---------------------------------------------------------------------------  

Timestamp TimeManager::operator[](unsigned int idx)
{
  sort();
  return m_entries[idx].timestamp;
}

// ---------------------------------------------------------------------------  

void TimeManager::clear()
{
  m_entries.clear();
  m_is_sorted = true;
}

// ---------------------------------------------------------------------------  

bool TimeManager::le(const tEntry &a, const tEntry &b)
{
  return a.timestamp < b.timestamp;
}

// ---------------------------------------------------------------------------  

void TimeManager::remove(const Timestamp &t, bool decrease_indexes)
{
  vector<tEntry>::iterator it;
  int removed_id = -1;
  
  tEntry entry(t);
  
  if(m_is_sorted)
  {
    it = lower_bound(m_entries.begin(), m_entries.end(), entry, TimeManager::le);
    
    if(it != m_entries.end() && it->timestamp == t)
    {
      removed_id = it->index;
      m_entries.erase(it);
    }
  }
  else
  {
    for(it = m_entries.begin(); it != m_entries.end(); ++it)
    {
      if(it->timestamp == t) break;
    }
    if(it != m_entries.end())
    {
      removed_id = it->index;
      m_entries[ it - m_entries.begin() ] = m_entries.back();
      m_entries.pop_back();
    }
  }
  
  // decreases all the indices after the removed one
  if(removed_id != -1 && decrease_indexes)
  {
    for(it = m_entries.begin(); it != m_entries.end(); ++it)
    {
      if((int)it->index > removed_id) it->index--;
    }
  }
  
  if(!m_is_sorted) 
    m_is_sorted = (m_entries.size() <= 1);
}
  
// ---------------------------------------------------------------------------

TimeManager::iterator TimeManager::begin(float frequency)
{
  sort(); // make sure timestamps are in order
  
  TimeManager::iterator ret;
  ret.m_frequency = frequency;
  ret.m_tm = this;
  
  if(m_entries.empty())
  {
    ret.index = -1;
  }
  else
  {
    ret.index = m_entries[0].index;
    ret.timestamp = m_entries[0].timestamp;
  }
  
  return ret;
}

// ---------------------------------------------------------------------------

TimeManager::iterator TimeManager::beginAt(const Timestamp &t, float frequency)
{
  sort();
  
  TimeManager::iterator ret;
  ret.m_frequency = frequency;
  ret.m_tm = this;
  ret.index = -1;
  
  if(!m_entries.empty())
  {
    ret.set(t);
  }
  
  return ret;
}

// ---------------------------------------------------------------------------

TimeManager::iterator TimeManager::beginAfter(double seconds, float frequency)
{
  sort();
  return beginAt(m_entries[0].timestamp + seconds, frequency);
}

// ---------------------------------------------------------------------------

Timestamp TimeManager::getFirstTimestamp()
{
  sort();
  return m_entries[0].timestamp;
}

// ---------------------------------------------------------------------------

Timestamp TimeManager::getLastTimestamp()
{
  sort();
  return m_entries.back().timestamp;
}

// ---------------------------------------------------------------------------

void TimeManager::sort()
{
  if(!m_is_sorted)
  {
    std::sort(m_entries.begin(), m_entries.end(), TimeManager::le);
    m_is_sorted = true;
  }
}

// ---------------------------------------------------------------------------

void TimeManager::iterator::operator++()
{
  Timestamp desired_time;
  if(m_frequency > 0)
  {
    desired_time = this->timestamp + 1.f/m_frequency;
  }
  else
  {
    desired_time = this->timestamp.plus(0, 1); // next instant
  }
  
  set(desired_time, false);
}

// ---------------------------------------------------------------------------

void TimeManager::iterator::step(double secs)
{
  Timestamp desired_time = this->timestamp + secs;
  set(desired_time);
}

// ---------------------------------------------------------------------------

void TimeManager::iterator::operator--()
{
  Timestamp desired_time;
  if(m_frequency > 0)
  {
    desired_time = this->timestamp - 1.f/m_frequency;
    set(desired_time, true);
  }
  else
  {
    desired_time = this->timestamp.minus(0, 1); // previous instant
    set(desired_time, true);
  }

}

// ---------------------------------------------------------------------------
    
void TimeManager::iterator::operator+=(int n)
{
  Timestamp desired_time;
  if(m_frequency > 0)
  {
    desired_time = this->timestamp + n*1.f/m_frequency;
    set(desired_time);
  }
  else
  {
    for(int i = 0; i < n; ++i)
    {
      desired_time = this->timestamp.plus(0, 1); // next instant
      set(desired_time);
      
      if(this->index == -1) break; // end of seq
    }
  }
}

// ---------------------------------------------------------------------------

void TimeManager::iterator::set(const Timestamp &desired_time, bool moving_backwards)
//bool geq_order)
{  
  // assume the time manager was not altered while the existence of this
  // iterator, so that it is still sorted
  
  if(m_tm->m_entries.empty())
  {
    this->index = -1;
    return;
  }
  
  vector<TimeManager::tEntry>::const_iterator it =
    lower_bound(m_tm->m_entries.begin(), m_tm->m_entries.end(), 
      desired_time, TimeManager::le);  
 
  // it >= desired_time
  // select the closer timestamps from [it-1, it] which is different
  // from the current one
  
  if(it == m_tm->m_entries.end())
  {
    // nothing
    /*
    if(this->index != (int)m_tm->m_entries.back().index)
    {
      it = m_tm->m_entries.begin() + m_tm->m_entries.size() - 1;
    }
    */
  }
  else if(it == m_tm->m_entries.begin())
  {
    if(this->index == (int)m_tm->m_entries[0].index)
    {
      if(moving_backwards)
        it = m_tm->m_entries.end();
      else
      {
        it = m_tm->m_entries.begin() + 1; // second element or end
      } 
    }
  }
  else
  {
    double d1 = it->timestamp - desired_time;
    double d2 = desired_time - (it-1)->timestamp;
        
    if(d2 < d1)
    {
      it = it-1;
    }
    
    if((int)it->index == this->index)
    {
      if(moving_backwards) it--;
      else it++;
    }
  }
  
  if(it != m_tm->m_entries.end())
  {
    this->index = it->index;
    this->timestamp = it->timestamp;
  }
  else
  {
    this->index = -1;
  }
  
  /*
  if(geq_order)
  {
    // search for the first timestamp in m_tm >= to desired_time
    
    if(it == m_tm->m_entries.end())
    {
      // end of sequence
      this->index = -1;
    }
    else
    {
      this->index = it->index;
      this->timestamp = it->timestamp;
    }
    
  } // if geq_order
  else
  {
    // search for the last timestamp in m_tm < to desired_time
    if(it == m_tm->m_entries.begin())
    {
      // before the sequence
      this->index = -1;
    }
    else
    {
      it = it - 1;
      
      this->index = it->index;
      this->timestamp = it->timestamp;
    }
    
  } // if !geq_order
  */
}

// ---------------------------------------------------------------------------

