/**
 * File: FeatureVector.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: feature vector
 *
 * This file is licensed under a Creative Commons 
 * Attribution-NonCommercial-ShareAlike 3.0 license. 
 * This file can be freely used and users can use, download and edit this file 
 * provided that credit is attributed to the original author. No users are 
 * permitted to use this file for commercial purposes unless explicit permission
 * is given by the original author. Derivative works must be licensed using the
 * same or similar license.
 * Check http://creativecommons.org/licenses/by-nc-sa/3.0/ to obtain further
 * details.
 *
 */

#include "FeatureVector.h"
#include <map>
#include <vector>
#include <iostream>

namespace DBoW2 {

// ---------------------------------------------------------------------------

FeatureVector::FeatureVector(void)
{
}

// ---------------------------------------------------------------------------

FeatureVector::~FeatureVector(void)
{
}

// ---------------------------------------------------------------------------

void FeatureVector::addFeature(NodeId id, unsigned int i_feature)
{
  FeatureVector::iterator vit = this->lower_bound(id);
  
  if(vit != this->end() && vit->first == id)
  {
    vit->second.push_back(i_feature);
  }
  else
  {
    vit = this->insert(vit, FeatureVector::value_type(id, 
      std::vector<unsigned int>() ));
    vit->second.push_back(i_feature);
  }
}

// ---------------------------------------------------------------------------

std::ostream& operator<<(std::ostream &out, 
  const FeatureVector &v)
{
  if(!v.empty())
  {
    FeatureVector::const_iterator vit = v.begin();
    
    const std::vector<unsigned int>* f = &vit->second;

    out << "<" << vit->first << ": [";
    if(!f->empty()) out << (*f)[0];
    for(unsigned int i = 1; i < f->size(); ++i)
    {
      out << ", " << (*f)[i];
    }
    out << "]>";
    
    for(++vit; vit != v.end(); ++vit)
    {
      f = &vit->second;
      
      out << ", <" << vit->first << ": [";
      if(!f->empty()) out << (*f)[0];
      for(unsigned int i = 1; i < f->size(); ++i)
      {
        out << ", " << (*f)[i];
      }
      out << "]>";
    }
  }
  
  return out;  
}

// ---------------------------------------------------------------------------

} // namespace DBoW2
