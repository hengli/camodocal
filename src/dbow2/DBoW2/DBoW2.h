/*
 * File: DBoW2.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: Generic include file for the DBoW2 classes and
 *   the specialized vocabularies and databases
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

/*! \mainpage DBoW2 Library
 *
 * DBoW2 library for C++:
 * Bag-of-word image database for image retrieval.
 *
 * Written by Dorian Galvez-Lopez,
 * University of Zaragoza
 * 
 * Check my website to obtain updates: http://webdiis.unizar.es/~dorian
 *
 * \section requirements Requirements
 * This library requires the DUtils, DUtilsCV, DVision and OpenCV libraries,
 * as well as the boost::dynamic_bitset class.
 *
 * \section citation Citation
 * If you use this software in academic works, please cite:
 <pre>
   @@INPROCEEDINGS{GalvezIROS11,
    author={Galvez-Lopez, Dorian and Tardos, Juan D.},
    booktitle={Intelligent Robots and Systems (IROS), 2011 IEEE/RSJ International Conference on},
    title={Real-time loop detection with bags of binary words},
    year={2011},
    month={sept.},
    volume={},
    number={},
    pages={51 -58},
    keywords={},
    doi={10.1109/IROS.2011.6094885},
    ISSN={2153-0858}
  }
 </pre>
 *
 * \section license License
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

#ifndef __D_T_DBOW2__
#define __D_T_DBOW2__

/// Includes all the data structures to manage vocabularies and image databases
namespace DBoW2
{
}

#include "TemplatedVocabulary.h"
#include "TemplatedDatabase.h"
#include "BowVector.h"
#include "FeatureVector.h"
#include "QueryResults.h"
#include "FSurf64.h"
#include "FBrief.h"
#include "FOrb.h"

/// SURF64 Vocabulary
typedef DBoW2::TemplatedVocabulary<DBoW2::FSurf64::TDescriptor, DBoW2::FSurf64> 
  Surf64Vocabulary;

/// SURF64 Database
typedef DBoW2::TemplatedDatabase<DBoW2::FSurf64::TDescriptor, DBoW2::FSurf64> 
  Surf64Database;
  
/// BRIEF Vocabulary
typedef DBoW2::TemplatedVocabulary<DBoW2::FBrief::TDescriptor, DBoW2::FBrief> 
  BriefVocabulary;

/// BRIEF Database
typedef DBoW2::TemplatedDatabase<DBoW2::FBrief::TDescriptor, DBoW2::FBrief> 
  BriefDatabase;

/// ORB Vocabulary
typedef DBoW2::TemplatedVocabulary<DBoW2::FOrb::TDescriptor, DBoW2::FOrb>
  OrbVocabulary;

/// ORB Database
typedef DBoW2::TemplatedDatabase<DBoW2::FOrb::TDescriptor, DBoW2::FOrb>
  OrbDatabase;

#endif

