/**
 * File: Demo.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez / Lionel Heng
 * Description: demo application of DBoW2 modified to work with ORB
 */

#include <iostream>
#include <vector>

// DBoW2
#include "DBoW2.h" // defines OrbVocabulary and OrbDatabase

#include "DUtils.h"
#include "DUtilsCV.h" // defines macros CVXX
#include "DVision.h"

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>

#ifdef HAVE_OPENCV3
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#else // HAVE_OpenCV3
#if CV24
#include <opencv2/nonfree/features2d.hpp>
#endif // CV24
#endif // HAVE_OpenCV3


using namespace DBoW2;
using namespace DUtils;
using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void loadFeatures(vector<vector<DVision::ORB::bitset> > &features);
void changeStructure(const cv::Mat &plain, vector<DVision::ORB::bitset> &out,
  int L);
void testVocCreation(const vector<vector<DVision::ORB::bitset> > &features);
void testDatabase(const vector<vector<DVision::ORB::bitset> > &features);


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// number of training images
const int NIMAGES = 4;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void wait()
{
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

// ----------------------------------------------------------------------------

int main()
{
  vector<vector<DVision::ORB::bitset> > features;
  loadFeatures(features);

  testVocCreation(features);

  wait();

  testDatabase(features);

  return 0;
}

// ----------------------------------------------------------------------------

void loadFeatures(vector<vector<DVision::ORB::bitset> > &features)
{
  features.clear();
  features.reserve(NIMAGES);

#ifdef HAVE_OPENCV3
  cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);
#else // HAVE_OPENCV3
  cv::Ptr<cv::ORB> orb = new cv::ORB(1000);
#endif // HAVE_OPENCV3

  cout << "Extracting ORB features..." << endl;
  for(int i = 0; i < NIMAGES; ++i)
  {
    stringstream ss;
    ss << "images/image" << i << ".png";

    cv::Mat image = cv::imread(ss.str(), 0);
    cv::Mat mask;
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;


#ifdef HAVE_OPENCV3
    orb->detectAndCompute(image, mask, keypoints, descriptors);
#else // HAVE_OPENCV3
    (*orb)(image, mask, keypoints, descriptors);
#endif // HAVE_OPENCV3

    features.push_back(vector<DVision::ORB::bitset>());
    changeStructure(descriptors, features.back(), orb->descriptorSize());
  }
}

// ----------------------------------------------------------------------------

void changeStructure(const cv::Mat &plain, vector<DVision::ORB::bitset> &out,
  int L)
{
  out.resize(plain.rows);

  for(int i = 0; i < plain.rows; ++i)
  {
    DVision::ORB::bitset bitset(L * 8);

    for(int j = 0; j < plain.cols; ++j)
    {
      unsigned char c = plain.at<unsigned char>(i,j);

      for(int k = 0; k < 8; ++k)
      {
          bitset[j * 8 + k] = (c & 0x1);
          c >>= 1;
      }
    }
    out.at(i) = bitset;
  }
}

// ----------------------------------------------------------------------------

void testVocCreation(const vector<vector<DVision::ORB::bitset> > &features)
{
  // branching factor and depth levels 
  const int k = 9;
  const int L = 3;
  const WeightingType weight = TF_IDF;
  const ScoringType score = L1_NORM;

  OrbVocabulary voc(k, L, weight, score);

  cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
  voc.create(features);
  cout << "... done!" << endl;

  cout << "Vocabulary information: " << endl
  << voc << endl << endl;

  // lets do something with this vocabulary
  cout << "Matching images against themselves (0 low, 1 high): " << endl;
  BowVector v1, v2;
  for(int i = 0; i < NIMAGES; i++)
  {
    voc.transform(features[i], v1);
    for(int j = 0; j < NIMAGES; j++)
    {
      voc.transform(features[j], v2);
      
      double score = voc.score(v1, v2);
      cout << "Image " << i << " vs Image " << j << ": " << score << endl;
    }
  }

  // save the vocabulary to disk
  cout << endl << "Saving vocabulary..." << endl;
  voc.save("small_voc.yml.gz");
  cout << "Done" << endl;
}

// ----------------------------------------------------------------------------

void testDatabase(const vector<vector<DVision::ORB::bitset> > &features)
{
  cout << "Creating a small database..." << endl;

  // load the vocabulary from disk
  OrbVocabulary voc("small_voc.yml.gz");
  
  OrbDatabase db(voc, true, 0);
  // The direct index is useful if we want to retrieve the features that 
  // belong to some vocabulary node.
  // db creates a copy of the vocabulary, we may get rid of "voc" now

  // add images to the database
  for(int i = 0; i < NIMAGES; i++)
  {
    db.add(features[i]);
  }

  cout << "... done!" << endl;

  cout << "Database information: " << endl << db << endl;

  // and query the database
  cout << "Querying the database: " << endl;

  QueryResults ret;
  for(int i = 0; i < NIMAGES; i++)
  {
    db.query(features[i], ret, 4);

    // ret[0] is always the same image in this case, because we added it to the 
    // database. ret[1] is the second best match.

    cout << "Searching for Image " << i << ". " << ret << endl;
  }

  cout << endl;

  // we can save the database. The created file includes the vocabulary
  // and the entries added
  cout << "Saving database..." << endl;
  db.save("small_db.yml.gz");
  cout << "... done!" << endl;
  
  // once saved, we can load it again  
  cout << "Retrieving database once again..." << endl;
  OrbDatabase db2("small_db.yml.gz");
  cout << "... done! This is: " << endl << db2 << endl;
}

// ----------------------------------------------------------------------------



