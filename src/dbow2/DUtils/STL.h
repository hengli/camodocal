/*	
 * File: STL.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: November 2010
 * Description: STL-related functions
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
#ifndef __D_STL__
#define __D_STL__

#include <vector>
#include <algorithm>
#include <iostream>
#include <string>

namespace DUtils {

/// Functions to use with STL containers
class STL {
public:

  /**
   * Removes from data the items of given indices
   * @param data (in/out) vector to remove items from
   * @param indices indices of items to remove
   * @param preserve_order if true, the items kept in data are in the same
   *    order as given. If false, the order may be different; this makes 
   *    this operation faster
   */
  template<class T>
  static void removeIndices(std::vector<T> &data, 
    const std::vector<unsigned int> &indices, bool preserve_order = true);
  
  /**
   * Removes from data the items of given indices, but can modify the index vector
   * @param data (in/out) vector to remove items from
   * @param indices indices of items to remove
   * @param preserve_order if true, the items kept in data are in the same
   *    order as given. If false, the order may be different; this makes 
   *    this operation faster
   */
  template<class T>
  static void removeIndices(std::vector<T> &data, 
    std::vector<unsigned int> &indices, bool preserve_order = true);

  /** 
   * Removes from data the items whose status is 0
   * @param data (in/out) vector to remove items from
   * @param status status of items in the vector.Those with status 0 are removed
   * @param preserve_order if true, the items kept in data are in the same
   *    order as given. If false, the order may be different; this makes 
   *    this operation faster
   */
  template<class T>
  static void removeIndices(std::vector<T> &data,
    const std::vector<unsigned char> &status, bool preserve_order = true);

  /**
   * Prints the content of the vector in f
   * Format: <name> = [ v1 v2 ... ]\n
   * @param v
   * @param name (optional) name given to the printed variable
   * @param f stream
   */
  template<class T>
  static void print(const std::vector<T> &v, const std::string &name,
   std::ostream &f = std::cout);

  /**
   * Returns the indexes of the sorted input elements in the range [first, last),
   * without moving the input data
   * @param first 
   * @param last 
   * @param sorted_indexes (out) indexes in the order 
   */
  template< class RandomIt >
  inline static void indexSort(RandomIt first, RandomIt last, 
    std::vector<unsigned int>& sorted_indexes);
  
  /**
   * Returns the indexes of the sorted input elements in the range [first, last),
   * without moving the input data
   * @param first 
   * @param last 
   * @param sorted_indexes (out) indexes in the order 
   * @param fun comparison function
   */
  template< class RandomIt, class Compare >
  static void indexSort(RandomIt first, RandomIt last, 
    std::vector<unsigned int>& sorted_indexes, Compare fun);

  /**
   * Arranges items of a container in the given order. This function is
   * useful after an indexSort
   * @param first first element of ranged data: [first, last)
   * @param last last+1 element of ranged data: [first, last)
   * @param indices indexes of the items in their final place
   *   indices must have length == last-first, with no repeated and valid items
   */
  template<class RandomIt>
  inline static void arrange(RandomIt first, RandomIt last,
    const std::vector<unsigned int> &indices);

public:

  /** 
   * Removes the items of given indices from data 
   * @param data (in/out) vector to remove items from
   * @param indices indices of items to remove. It must be in ascending order,
   *    with valid and no repeated indexes. If they are not, use
   *    the ::removeIndices functions instead.
   * @param preserve_order if true, the items kept in data are in the same
   *    order as given. If false, the order may be different; this makes 
   *    this operation faster
   */
  template<class T>
  static void _removeIndices(std::vector<T> &data, 
    const std::vector<unsigned int> &indices, bool preserve_order);

protected:

  /**
   * Auxiliar functor used as comparison function when performing index sorting
   * @param RandomIt iterator class
   * @param Compare comparison function for the type of the iterator 
   */
  template<class RandomIt, class Compare> 
  struct index_cmp 
  {
    index_cmp(const RandomIt& _first, const Compare &_fun)
      : m_first(_first), m_fun(_fun) {}
    
    // unsigned int is the type of the vector of indexes
    bool operator()(const unsigned int a, const unsigned int b) const
    {
      return m_fun( *(m_first + a), *(m_first + b) );
    }
    
  private:
    const RandomIt& m_first;
    const Compare& m_fun;
  };
  

};

// ---------------------------------------------------------------------------

template<class T>
void STL::removeIndices(std::vector<T> &data,
  const std::vector<unsigned char> &status, bool preserve_order)
{
  assert(data.size() == status.size());
  
  std::vector<unsigned int> indices;
  for(unsigned int i = 0; i < status.size(); ++i)
    if (status[i] == 0) indices.push_back(i);
  
  STL::_removeIndices(data, indices, preserve_order);
}

// ---------------------------------------------------------------------------

template<class T>
void STL::removeIndices(std::vector<T> &data, 
  const std::vector<unsigned int> &indices, bool preserve_order)
{
  if(indices.empty()) return;
  
  std::vector<unsigned int> copied_indices = indices;
  STL::removeIndices(data, copied_indices, preserve_order);
}

// ---------------------------------------------------------------------------

template<class T>
void STL::removeIndices(std::vector<T> &data, 
  std::vector<unsigned int> &indices, bool preserve_order)
{
  if(indices.empty()) return;
  
  // sort the index entries    
  std::sort(indices.begin(), indices.end()); // ascending order
  
  // remove those indices that exceed the data vector length
  {
    int i_idx = (int)indices.size() - 1;
    while( indices[i_idx] >= data.size() ) i_idx--;
    indices.resize(i_idx+1);
  }
    
  // make sure there are no repeated indices
  {
    const std::vector<unsigned int>::iterator last =
      std::unique(indices.begin(), indices.end());
    indices.erase(last, indices.end());
  }
  
  STL::_removeIndices(data, indices, preserve_order);
}

// ---------------------------------------------------------------------------

template<class T>
void STL::_removeIndices(std::vector<T> &data, 
  const std::vector<unsigned int> &indices, bool preserve_order)
{
  // go
  if(preserve_order)
  {
    // remove indices in descending order, grouping when possible
    int i_idx = (int)indices.size() - 1;
    while(i_idx >= 0)
    {
      int j_idx = i_idx - 1;
      while(j_idx >= 0 && ((int)(indices[i_idx] - indices[j_idx]) == i_idx - j_idx))
      {
        j_idx--;
      }
      data.erase(data.begin() + indices[j_idx + 1], 
        data.begin() + indices[i_idx] + 1);
      i_idx = j_idx;
    }
    
  }
  else
  { 
    // swap with the last items
    int nremoved = 0;
    
    const typename std::vector<T>::iterator first = data.begin();
    const typename std::vector<T>::iterator last = data.end()-1;
  
    int i_idx = (int)indices.size() - 1;
    
    // exception case: removing items are at the end of the vector
    while(i_idx >= 0 && 
      (indices.size() - i_idx == data.size() - indices[i_idx]))
    {
      i_idx--;
      nremoved++;
    }
        
    while(i_idx >= 0)
    {
      int j_idx = i_idx - 1;
      while(j_idx >= 0 && ((int)(indices[i_idx] - indices[j_idx]) == i_idx - j_idx))
      {
        j_idx--;
      }
      
      int nremoving = i_idx - j_idx;
      
      const typename std::vector<T>::iterator cpy_end = last - nremoved + 1;
      const typename std::vector<T>::iterator cpy_src = cpy_end - 
        std::min( nremoving, (int)data.size()-1 - nremoved - (int)indices[i_idx] );
      const typename std::vector<T>::iterator trg = first + indices[j_idx + 1];
            
      std::copy( cpy_src, cpy_end, trg );
      
      nremoved += nremoving;
      i_idx = j_idx;
    }
    
    data.resize(data.size() - nremoved);

    // v2, presumedly slower
#if 0
    std::vector<unsigned int>::reverse_iterator rit;
    for(rit = indices.rbegin(); rit != indices.rend(); ++rit)
    {
      if(*rit < data.size())
      {
        *(first + *rit) = *(last - nremoved);
        nremoved++;
      }
    }
    data.resize(data.size() - nremoved);
#endif
  }
  
}

// ---------------------------------------------------------------------------

template<class T>
void STL::print(const std::vector<T> &v, const std::string &name,
  std::ostream &f)
{
  if(!name.empty())
  {
    f << name << " = ";
  }
  f << "[ ";
  
  typename std::vector<T>::const_iterator vit;
  for(vit = v.begin(); vit != v.end(); ++vit)
  {
    f << *vit << " ";
  }
  f << "]";
  f << endl;
}

// ---------------------------------------------------------------------------

template< class RandomIt >
void STL::indexSort(RandomIt first, RandomIt last, 
  std::vector<unsigned int>& sorted_indexes)
{
  STL::indexSort(first, last, sorted_indexes, 
    std::less<typename RandomIt::value_type>() );
}

// ---------------------------------------------------------------------------

template< class RandomIt, class Compare >
void STL::indexSort(RandomIt first, RandomIt last, 
  std::vector<unsigned int>& sorted_indexes, Compare fun)
{
  if(last == first)
  {
    sorted_indexes.clear();
    return;
  }
  
  sorted_indexes.clear();
  sorted_indexes.reserve(last - first);
  
  RandomIt it = first;
  for(unsigned int i = 0; it != last; ++it, ++i)
    sorted_indexes.push_back(i);
  
  std::sort(sorted_indexes.begin(), sorted_indexes.end(), 
    index_cmp<RandomIt, Compare>(first, fun));
}

// ---------------------------------------------------------------------------

template<class RandomIt>
void STL::arrange(RandomIt first, RandomIt last,
  const std::vector<unsigned int> &indices)
{
  for(size_t i = 0; i < indices.size(); ++i)
  {
    unsigned int idx = indices[i];
    while(idx < i)
    {
      idx = indices[idx];
    }
    
    if(idx > i)
    {
      iter_swap(first + idx, first + i);
    }
  }
}

// ---------------------------------------------------------------------------

}

#endif

