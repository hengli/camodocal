/*
 * File: TimeManager.h
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

#ifndef __D_TIME_MANAGER__
#define __D_TIME_MANAGER__

#include <vector>
#include "Timestamp.h"

namespace DUtils
{

/// Manages collections of timestamps
class TimeManager
{
public:

  /// Iterator of managed data (these data are copied)
  class iterator
  {
  public:

    /// Index of the current timestamp as it was inserted into the TimeManager 
    /// or -1 if the index is not valid (all the timestamp collection was 
    /// traversed)
    int index; 
    
    /// Current timestamp (if the iterator is valid)
    Timestamp timestamp;

  public:
    
    /** 
     * Returns whether this iterator is valid
     * (if not, the sequences has been iterated until the end)
     * @return true if the iterator is valid
     */
    inline bool good() const { return index >= 0; }
    
    /** 
     * Moves the iterator to the next timestamp according to the frequency
     * The timestamp of the returned iterator is the smallest one that is
     * greater or equal to the current time + frequency^-1 (or to the next
     * timestamps if frequency is -1)
     */
    void operator++();
    
    /** 
     * Moves the iterator to the previous timestamp according to the frequency
     * The timestamp of the returned iterator is the smallest one that is
     * greater or equal to the current time - frequency^-1 (or to the previous
     * timestamps if frequency is -1)
     */
    void operator--();
    
    /** 
     * Sum n steps of "frequency" size to the current iterator
     * @param n
     */
    void operator+=(int n);
    
    /**
     * Moves the iterator secs seconds forward
     * @param secs
     */
    void step(double secs);
    
    /**
     * Sets a new frequency for this iterator
     * @param frequency
     */
    inline void setFrequency(float frequency)
    {
      m_frequency = frequency;
    }
  
  protected:
    
    /**
     * Tries to set the iterator pointing to the desired time (other than the
     * current one)
     * @param desired_time
     * @param moving_backwards says iif the time is moving backwards
     */
    void set(const Timestamp &desired_time, bool moving_backwards = false);
    
    /**
     * Tries to set the iterator pointing to the desired time
     * @param desired_time
     * @param geq_order if true, the timestamp set is the first one that
     *   is >= desired_time. If false, the timestamp set is the last one that
     *   is < desired_time
     */
    //void set(const Timestamp &desired_time, bool geq_order = true);
  
  protected:
    friend class TimeManager;
    
    /// Frequency at which the iterator moves
    float m_frequency; // can be -1 to mean all the timestamps
    /// TimeManager associated to this iterator
    const TimeManager *m_tm;
  };

public:

  TimeManager();
  ~TimeManager();
  
  /**
   * Returns the idx-th timestamp after sorting all the timestamps
   * @param idx 0 <= idx < size
   */
  Timestamp operator[](unsigned int idx);
  
  /**
   * Adds a timestamp to the collection. 
   * This action invalidate the created iterators
   * @param t
   */
  void add(const Timestamp &t);
  
  /**
   * Removes the given timestamps from the collection. This can also decrease the
   * insertion index of the timestamps added after the removed one.
   * This action invalidates the created iterators
   * @param t
   * @param decrease_indexes if true, indexes of next timestamps are decreased
   */
  void remove(const Timestamp &t, bool decrease_indexes);
  
  /**
   * Removes all the timestamps.
   * This action invalidates the created iterators
   */
  void clear();
  
  /**
   * Says if the collection is empty
   * @return true iff empty
   */
  inline bool empty() const
  {
    return m_entries.empty();
  }
  
  /**
   * Returns the number of timestamps in the collection
   * @return number of entries
   */
  inline unsigned int size() const
  {
    return m_entries.size();
  }
  
  /**
   * Returns an iterator to the first item of the sequence. When the interator
   * is incremented, it will according with the desired frequency here
   * @param frequency frequency to get the timestamps. It can be -1 to mean
   *   all the timestamps
   */
  iterator begin(float frequency = -1);
  
  /**
   * Returns an interator to the lower timestamp that is greater or equal to
   * the first timetamp + seconds (first timestamp = the oldest one)
   * @param seconds
   * @param frequency frequency of iterator. -1 by default (single step frequency)
   */
  iterator beginAfter(double seconds, float frequency = -1);
  
  /**
   * Returns an interator to the closest timestamp to the given one
   * @param t
   * @param frequency frequency of iterator. -1 by default (single step)
   */
  iterator beginAt(const Timestamp &t, float frequency = -1);
  
  /**
   * Returns the first timestamp (in order) of the collection
   * @return first timestamp
   */
  Timestamp getFirstTimestamp();
  
  /**
   * Returns the last timestmap (in order) of the collection
   * @return last timestamp
   */
  Timestamp getLastTimestamp();
    
protected:

  /**
   * Makes the stored timestamps to be sorted in ascending order
   */
  void sort();

protected:
  friend class iterator;

  /// Single entry of the collection
  struct tEntry
  {
    /// Timestamp
    Timestamp timestamp;
    /// Index of the timestamp according to the moment it was added
    unsigned int index; // index got when the timestamp was added
    
    /**
     * Uninitialized entry
     */
    tEntry(){}
    
    /**
     * Initializes entry with a timestamp
     * @param t timestamp
     */
    tEntry(const Timestamp &t): timestamp(t), index(-1){}
    
    /**
     * Initializes entry with a timestamp and an index
     * @param t timestamp
     * @param i index
     */
    tEntry(const Timestamp &t, unsigned int i): timestamp(t), index(i){}
  };

  /// All the entries of the timestamp collection
  std::vector<tEntry> m_entries;
  /// Flag to tell when the entries are sorted
  bool m_is_sorted;
  
protected:

  /**
   * Checks if timestamp of a < timestamp of b
   * @param a
   * @param b
   * @return true iif a < b
   */
  static bool le(const tEntry &a, const tEntry &b);
 

};

}

#endif
