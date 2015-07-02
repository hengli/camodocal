/**
 * File: SurfSet.cpp
 * Project: DVision library
 * Author: Dorian Galvez-Lopez
 * Date: October 4, 2010
 * Description: Class to extract, manage, save and load surf features
 *
 * NOTE: this class does not offer a copy constructor or copy operator.
 *   This means the class cannot be copied when the Fast correspondences are
 *   used (in that case, both objects would share the Flann structure)
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

#include "SurfSet.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "DUtils.h"
#include "DUtilsCV.h" // defines macros CVXX


#ifdef HAVE_OPENCV3
#include <opencv2/flann.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#else // HAVE_OPENCV3

#include <opencv/cv.h>
#include <opencv/highgui.h>

#if CV24
#include <opencv2/nonfree/features2d.hpp>
#endif

#endif // HAVE_OPENCV3



using namespace std;
using namespace DVision;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::SaveCustom(const std::string &filename) const
{
	fstream f(filename.c_str(), ios::out);
	if(!f.is_open()) throw string("SurfSet: cannot open ") + filename;
	
	int L = 0;
	if(!keys.empty()) L = descriptors.size() / keys.size();

	f << keys.size() << " " << L << endl;
	
	vector<cv::KeyPoint>::const_iterator kit;
	vector<float>::const_iterator dit = descriptors.begin();
	for(kit = keys.begin(); kit != keys.end(); ++kit){
		f << kit->pt.x << " "
			<< kit->pt.y << " "
			<< kit->angle << " "
			<< kit->size << " "
			<< kit->response << " "
			<< kit->octave << " "
			<< laplacians[kit - keys.begin()] << " ";

		for(int i = 0; i < L; ++i, ++dit){
			f << *dit << " ";
		}
		f << endl;
	}
	
	f.close();
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::LoadCustom(const std::string &filename)
{
	keys.resize(0);
	descriptors.resize(0);
	laplacians.resize(0);
	delete m_index; m_index = NULL;
	
	fstream f(filename.c_str(), ios::in);
	if(!f.is_open()) throw string("SurfSet: cannot open ") + filename;
	
	int N, L;
	f >> N >> L;
	
	if (f.is_open() && !f.fail() ){
		keys.resize(N);
		descriptors.resize(N * L);
		laplacians.resize(N);
		
		vector<float>::iterator dit = descriptors.begin();
		for(int i = 0; i < N; ++i){
			cv::KeyPoint& k = keys[i];
			
			f >> k.pt.x >> k.pt.y >> k.angle >> k.size >> k.response
				>> k.octave >> laplacians[i];

			for(int j = 0; j < L; ++j, ++dit){
				f >> *dit;
			}
		}
	}else{
	  throw string("SurfSet: cannot read file ") + filename;
	}
	
	f.close();
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::Extract(const cv::Mat &image, double hessianTh, bool extended)
{
  CvSURFParams params = cvSURFParams(hessianTh, (extended ? 1: 0) );
  extract(image, params);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

#if USURF_SUPPORTED
void SurfSet::_ExtractUpright(const cv::Mat &image, double hessianTh, bool extended)
{
  CvSURFParams params = cvSURFParams(hessianTh, (extended ? 1: 0) );
  params.upright = 1;
  extract(image, params);
}
#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::extract(const cv::Mat &image, const CvSURFParams &params)
{
  #if CV24
    // OpenCV 2.4 implementation
  
    cv::SURF surf(params.hessianThreshold, params.nOctaves, params.nOctaveLayers,
      (params.extended == 1), false /* upright */);
    
    cv::Mat descs;
    surf(image, cv::Mat() /* mask */, this->keys, descs);
    
    const int L = (params.extended == 1 ? 128 : 64);
    this->descriptors.resize(this->keys.size() * L);
    this->laplacians.resize(this->keys.size());
    
    vector<cv::KeyPoint>::const_iterator kit;
    vector<int>::iterator lit = this->laplacians.begin();
    for(kit = this->keys.begin(); kit != this->keys.end(); ++kit, ++lit)
    {
      *lit = kit->class_id; // laplacian sign should be stored here
    }
    
    //assert(descs.type() == CV_32F && descs.isContinuous());
    
    std::copy(descs.ptr<float>(), descs.ptr<float>() + (descs.rows * descs.cols),
      this->descriptors.begin());
  
  #else
  
    // old implementation opencv 2.0
    CvSeq *kp1=NULL;
    CvSeq *desc1=NULL;
    CvMemStorage *storage = cvCreateMemStorage(0);
   
    IplImage ipl_image = IplImage(image);
    cvExtractSURF(&ipl_image, NULL, &kp1, &desc1, storage, params);

    CvSeqReader reader, kreader;
    cvStartReadSeq( desc1, &reader, 0 );
    cvStartReadSeq( kp1, &kreader, 0 );

    const int L = (params.extended ? 128 : 64);

    descriptors.resize(desc1->total * L);
    keys.resize(kp1->total);
    laplacians.resize(kp1->total);

    for( int i = 0; i < desc1->total; i++ )
    {
      const CvSURFPoint *p= (const CvSURFPoint*)kreader.ptr;
      const float* vec = (const float*)reader.ptr;
      CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
      CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );

      keys[i].pt.x = p->pt.x;
      keys[i].pt.y = p->pt.y;
      keys[i].angle = p->dir;
      keys[i].size = (float)p->size;
      keys[i].response = p->hessian;
      keys[i].octave = getPointOctave(*p, params);
      laplacians[i] = p->laplacian;

      for(int j = 0; j < L; ++j){
          descriptors[i*L + j] = vec[j];
      }
    }
    
    cvReleaseMemStorage(&storage);
  
  #endif
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::Compute(const cv::Mat &image,
    const std::vector<cv::KeyPoint> &keypoints, bool extended)
{
  CvSURFParams params = cvSURFParams(0, (extended ? 1: 0) );
  compute(image, keypoints, params);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

#if USURF_SUPPORTED

void SurfSet::_ComputeUpright(const cv::Mat &image,
    const std::vector<cv::KeyPoint> &keypoints, bool extended)
{
  CvSURFParams params = cvSURFParams(0, (extended ? 1: 0) );
  params.upright = 1;
  compute(image, keypoints, params);
}

#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::compute(const cv::Mat &image,
  const std::vector<cv::KeyPoint> &keypoints, const CvSURFParams &params)
{
  #if CV24
    // OpenCV 2.4 implementation
    if(keypoints.empty())
    {
      this->keys.clear();
      this->descriptors.clear();
      this->laplacians.clear();
    }
    else
    {
      this->keys = keypoints;

      cv::SURF surf(params.hessianThreshold, params.nOctaves, 
        params.nOctaveLayers, (params.extended == 1), false /* upright */);
      
      cv::Mat descs;
      surf(image, cv::Mat() /* mask */, this->keys, descs, true);
    
      this->descriptors.resize(this->keys.size() * descs.cols);
      this->laplacians.resize(this->keys.size());
      
      vector<cv::KeyPoint>::const_iterator kit;
      vector<int>::iterator lit = this->laplacians.begin();
      for(kit = this->keys.begin(); kit != this->keys.end(); ++kit, ++lit)
      {
        *lit = kit->class_id; // laplacian sign should be stored here
      }
      
      assert(descs.type() == CV_32F && descs.isContinuous());
      
      std::copy(descs.ptr<float>(), descs.ptr<float>() + (descs.rows * descs.cols),
        this->descriptors.begin());
    }
  
  #else
  
    const int L = (params.extended ? 128 : 64);
    
    keys = keypoints;
    laplacians.resize(keys.size());
    descriptors.resize(keys.size() * L);
    
    std::fill(laplacians.begin(), laplacians.end(), 1);

    CvSeqWriter writer;
    CvMemStorage *storage = cvCreateMemStorage(0);
    cvStartWriteSeq(0, sizeof(CvSeq), sizeof(CvSURFPoint), storage, &writer);
    CvSURFPoint p;

    std::vector<cv::KeyPoint>::const_iterator kit;
    for(kit = keypoints.begin(); kit != keypoints.end(); ++kit)
    {
      p.pt.x = kit->pt.x;
      p.pt.y = kit->pt.y;
      p.dir = kit->angle;
      p.size = kit->size;
      p.hessian = kit->response;
      p.laplacian = 1;
      
      CV_WRITE_SEQ_ELEM(p, writer);
    }
    
    CvSeq *kp1 = cvEndWriteSeq(&writer);
    CvSeq *desc1 = NULL;

    IplImage ipl_image = IplImage(image);
    cvExtractSURF(&ipl_image, NULL, &kp1, &desc1, storage,
      params, 1); // use provided keypoints);

    CvSeqReader reader;
    cvStartReadSeq( desc1, &reader, 0 );

    assert(desc1->total == (int)keys.size()); // ### check

    for( int i = 0; i < desc1->total; i++ )
    {
      const float* vec = (const float*)reader.ptr;
      CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );

      for(int j = 0; j < L; ++j){
          descriptors[i*L + j] = vec[j];
      }
    }
    
    cvReleaseMemStorage(&storage);
  #endif
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::CalculateCorrespondences(const SurfSet &B,
		vector<int> &corr_A, vector<int> &corr_B,
		vector<double> *distances, bool remove_duplicates, 
		double max_ratio) const
{
  calculateCorrespondencesNaive(B, corr_A, corr_B, distances, 
    remove_duplicates, max_ratio);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::CalculateFastCorrespondences(const SurfSet &B,
		vector<int> &corr_A, vector<int> &corr_B,
		vector<double> *distances, bool remove_duplicates, 
		double max_ratio) 
{
  calculateCorrespondencesApproximate(B, corr_A, corr_B, distances, 
    remove_duplicates, max_ratio);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::calculateCorrespondencesApproximate(const SurfSet &B,
		std::vector<int> &A_corr, std::vector<int> &B_corr,
		std::vector<double> *distances,
		bool remove_duplicates, double max_ratio) 
{    
  const int L = GetDescriptorLength();
    
  //const cv::Mat A_features( this->keys.size(), L, CV_32F, 
  //  const_cast<float*>(&this->descriptors[0]) );
  const cv::Mat B_features( B.keys.size(), L, CV_32F, 
    const_cast<float*>(&B.descriptors[0]) );
  
  //const cv::Mat *features, 
  const cv::Mat *queries;
  std::vector<int> *f_corr, *q_corr;
  /*
  if(this->keys.size() >= B.keys.size())
  {
    features = &A_features;
    queries = &B_features;
    f_corr = &A_corr;
    q_corr = &B_corr;
  }
  else
  {
    features = &B_features;
    queries = &A_features;
    f_corr = &B_corr;
    q_corr = &A_corr;
  }
  */
  queries = &B_features;
  f_corr = &A_corr;
  q_corr = &B_corr;
  
  cv::Mat indices(queries->rows, 2, CV_32S);
  cv::Mat dists(queries->rows, 2, CV_32F);

  if(!m_index) RecalculateApproximationTree();
  m_index->knnSearch(*queries, indices, dists, 2, cv::flann::SearchParams(64));

  A_corr.resize(0);
  B_corr.resize(0);
  if(distances) distances->resize(0);

  int* indices_ptr = indices.ptr<int>(0);
  float* dists_ptr = dists.ptr<float>(0);
  
  if(!remove_duplicates)
  {
    if(distances)
    {
      for (int i=0; i < indices.rows; ++i) {
        if (dists_ptr[2*i] < max_ratio * dists_ptr[2*i+1])
        {
          q_corr->push_back(i);
          f_corr->push_back(indices_ptr[2*i]);
          distances->push_back(dists_ptr[2*i]);
        }
      }
    } // if(distances)
    else
    {
      for (int i=0; i < indices.rows; ++i) {
        if (dists_ptr[2*i] < max_ratio * dists_ptr[2*i+1])
        {
          q_corr->push_back(i);
          f_corr->push_back(indices_ptr[2*i]);
        }
      }
    }
  } // if(!remove_duplicates)
  else
  {
    vector<int>::const_iterator it;
    if(distances)
    {
      for (int i=0; i < indices.rows; ++i) {
        if (dists_ptr[2*i] < max_ratio * dists_ptr[2*i+1])
        {
          int i_f = indices_ptr[2*i];
          float d_now = dists_ptr[2*i];
          
          it = find(f_corr->begin(), f_corr->end(), i_f);
          if(it != f_corr->end())
          {
            int i_before = it - f_corr->begin();
            float d_before = dists_ptr[2*i_before];
            
            if(d_now < d_before)
            {
              //update
              (*q_corr)[i_before] = i;
              (*f_corr)[i_before] = i_f;
              (*distances)[i_before] = d_now;
            }
          } // if(i_f is not in f_corr)
          else
          {
            q_corr->push_back(i);
            f_corr->push_back(indices_ptr[2*i]);
            distances->push_back(dists_ptr[2*i]);
          }
        } // if(d1/d2 < max_ratio)
      } // for each query sample
    } // if(distances)
    else
    {
      for (int i=0; i < indices.rows; ++i) {
        if (dists_ptr[2*i] < max_ratio * dists_ptr[2*i+1])
        {
          int i_f = indices_ptr[2*i];
          float d_now = dists_ptr[2*i];
          
          it = find(f_corr->begin(), f_corr->end(), i_f);
          if(it != f_corr->end())
          {
            int i_before = it - f_corr->begin();
            float d_before = dists_ptr[2*i_before];
            
            if(d_now < d_before)
            {
              //update
              (*q_corr)[i_before] = i;
              (*f_corr)[i_before] = i_f;
            }
          } // if(i_f is not in f_corr)
          else
          {
            q_corr->push_back(i);
            f_corr->push_back(indices_ptr[2*i]);
          }
        } // if(d1/d2 < max_ratio)
      } // for each query sample
    } // if(!distances)
  } // if(remove_duplicates)
  
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::calculateCorrespondencesNaive(const SurfSet &B,
		vector<int> &corr_A, vector<int> &corr_B,
		vector<double> *distances, bool remove_duplicates, 
		double max_ratio) const
{
  
  const SurfSet& A = *this;
	
  corr_A.clear();
  corr_B.clear();
  
  bool deallocate;
  if(distances)
  {
    deallocate = false;
  }else{
    distances = new vector<double>;
    deallocate = true;
  }

  int L = 0;
  if(!A.keys.empty()) L = A.descriptors.size() / A.keys.size();

  vector<float>::const_iterator da;
  vector<float>::const_iterator db;

  da = A.descriptors.begin(); 
  for(unsigned int a = 0; a < A.keys.size(); ++a, da += L)
  {
    double best_1 = 1e9;
    double best_2 = 1e9;
    int best_b = 0;

    db = B.descriptors.begin(); 
    for(unsigned int b = 0; b < B.keys.size(); ++b, db += L)
    {
      if(A.laplacians[a] == B.laplacians[b]){
        double d = calculateSqDistance(da, db, L);

        if(d < best_1){
          best_2 = best_1;
          best_1 = d;
          best_b = b;
        }else if(d < best_2){
          best_2 = d;
        }
      }
    }

    // best_ distances are square
    if(best_1 / best_2 < max_ratio*max_ratio)
    {
      // candidate found
      if(remove_duplicates)
      {
        // check that the B's point has not already been used
        // by another scene point.
        // in that case, select that with the minimum distance
        vector<int>::const_iterator it = 
          find(corr_B.begin(), corr_B.end(), best_b);

        if(it == corr_B.end()){
          // add it
          corr_A.push_back(a);
          corr_B.push_back(best_b);
          distances->push_back(best_1);
        }else{
          int corr_idx = it - corr_B.begin();
          if(best_1 < (*distances)[ corr_idx ]){
            // update 
            corr_A[corr_idx] = a;
            (*distances)[corr_idx] = best_1;
          }
        }
      } // if(remove_duplicates)
      else
      {
        // add it
        corr_A.push_back(a);
        corr_B.push_back(best_b);
        distances->push_back(best_1);
      }
    } // if ( < max_ratio )
         
  } // for each feature
  
  if(deallocate) 
    delete distances;
  else
  {
    // remove the "square" from the distance values
    vector<double>::iterator dit;
    for(dit = distances->begin(); dit != distances->end(); ++dit)
    {
      *dit = sqrt(*dit);
    }
  }
  
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

double SurfSet::calculateSqDistance(
    vector<float>::const_iterator ita, vector<float>::const_iterator itb,
    const int L) const
{
    double d = 0.0;
    for(int i = 0; i < L; i += 4, ita += 4, itb += 4)
    {
        d += (*ita - *itb)*(*ita - *itb);
        d += (*(ita+1) - *(itb+1))*(*(ita+1) - *(itb+1));
        d += (*(ita+2) - *(itb+2))*(*(ita+2) - *(itb+2));
        d += (*(ita+3) - *(itb+3))*(*(ita+3) - *(itb+3));
    }
    return d;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::RecalculateApproximationTree()
{
  if(m_index) delete m_index;

  const int L = GetDescriptorLength();
  const cv::Mat features( this->keys.size(), L, CV_32F, 
    const_cast<float*>(&this->descriptors[0]) );
  
  //m_index = new cv::flann::Index(features, cv::flann::KDTreeIndexParams(4));
  m_index = new cv::flann::Index(features, cv::flann::AutotunedIndexParams());    
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::Save(const std::string &filename) const
{
  cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
  save(fs, 0);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::save(cv::FileStorage &fs, int idx) const
{
  stringstream ss;
  ss << "set" << idx;
  
  fs << ss.str() << "{";
  
  fs << "descriptor_length" << GetDescriptorLength();
  cv::write(fs, "keypoints", keys);

  #if CV24  
    cv::write(fs, "descriptors", descriptors);
    cv::write(fs, "laplacians", laplacians);
  #else
  
    if(!descriptors.empty())
      fs << "descriptors" << "[" << descriptors << "]";
    else
      fs << "descriptors" << "[" << "]";
    
    if(!laplacians.empty())
      fs << "laplacians" << "[" << laplacians << "]";
    else
      fs << "laplacians" << "[" << "]";
      
  #endif
  
  fs << "}";
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::Load(const std::string &filename)
{
  cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
  load(fs, 0);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::load(cv::FileStorage &fs, int idx)
{
  keys.resize(0);
  descriptors.resize(0);
  laplacians.resize(0);
  delete m_index; m_index = NULL;
  
  stringstream ss;
  ss << "set" << idx;

  cv::FileNode fn = fs[ss.str()];
  
  int L = (int)fn["descriptor_length"];
  cv::read(fn["keypoints"], keys);

  descriptors.resize(keys.size() * L);
  laplacians.resize(keys.size());

  #if CV24
    cv::read(fn["descriptors"], descriptors);
    cv::read(fn["laplacians"], laplacians);
  #else
    fn["descriptors"] >> descriptors;
    fn["laplacians"] >> laplacians;
  #endif
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

int SurfSet::getPointOctave(const CvSURFPoint& kpt, const CvSURFParams& params)
  const 
{
  // these are as defined in opencv surf.cpp
  const int HAAR_SIZE0 = 9;
  const int HAAR_SIZE_INC = 6;
  
  int octave = 0, layer = 0, best_octave = 0;
  float min_diff = FLT_MAX;
  for( octave = 1; octave < params.nOctaves; octave++ )
    for( layer = 0; layer < params.nOctaveLayers; layer++ )
    {
      float diff = std::abs(kpt.size - 
        (float)((HAAR_SIZE0 + HAAR_SIZE_INC*layer) << octave));
      if( min_diff > diff )
      {
        min_diff = diff;
        best_octave = octave;
        if( min_diff == 0 )
          return best_octave;
      }
  }
  return best_octave;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::Remove(unsigned int i)
{
  if(i < keys.size())
  {
    const int L = GetDescriptorLength();
    
    vector<unsigned int> i_remove(1, i);
    vector<unsigned int> i_remove_desc;
    i_remove_desc.reserve(L);
    
    for(int j = 0; j < L; ++j)
    {
      i_remove_desc.push_back( i*L + j );
    }
    
    DUtils::STL::removeIndices(keys, i_remove, true);
    DUtils::STL::removeIndices(laplacians, i_remove, true);
    DUtils::STL::removeIndices(descriptors, i_remove_desc, true);
  }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void SurfSet::Remove(const std::vector<unsigned int> &ids)
{
  const int L = GetDescriptorLength();
  
  vector<unsigned int> i_remove_desc;
  i_remove_desc.reserve(L * ids.size());
  
  for(unsigned int k = 0; k < ids.size(); ++k)
  {
    unsigned int i = ids[k];
    for(int j = 0; j < L; ++j)
    {
      i_remove_desc.push_back( i*L + j );
    }
  }
  
  DUtils::STL::removeIndices(keys, ids, true);
  DUtils::STL::removeIndices(laplacians, ids, true);
  DUtils::STL::removeIndices(descriptors, i_remove_desc, true);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


