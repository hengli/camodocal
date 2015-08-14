/**
 * File: SurfSet.h
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

#ifndef __D_SURF_SET__
#define __D_SURF_SET__

// Set if U-SURF is supported by Opencv (probably by applying this patch:
// https://code.ros.org/trac/opencv/ticket/825 )
#define USURF_SUPPORTED 0

#ifdef HAVE_OPENCV3
#include <opencv2/flann.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif // HAVE_OPENCV3

#include <vector>
#include <string>

namespace DVision {

class Matches;

/// Collection of surf keypoints + descriptors
class SurfSet 
{
public:
  /// KeyPoint collection
	std::vector<cv::KeyPoint> keys;
	/// Descriptors associated to the keypoints
	std::vector<float> descriptors;
	/// Laplacian signs of the keypoints
	std::vector<int> laplacians;
	
public:

	SurfSet(): m_index(NULL) {}
	virtual ~SurfSet(){ delete m_index; }

	/** 
	 * Extract surf points from an image
	 * @param image
	 * @param hessianTh hessian threshold
	 * @param extended if true, 128-dimensional surf are used instead of 64-d
	 */
	void Extract(const cv::Mat &image, 
		double hessianTh = 400.0, bool extended = false);

#if USURF_SUPPORTED
	/** 
	 * Extract U-SURF points from an image
	 * NOTE: this function is only available if opencv supports U-SURF
   * (by applying the corresponding patch)
	 * @param image
	 * @param surf surf data
	 * @param hessianTh hessian threshold
	 * @param extended if true, 128-dimensional surf are used instead of 64-d
	 * @note the octave is not returned and it is always 1
	 */
	void _ExtractUpright(const cv::Mat &image, 
		double hessianTh = 400.0, bool extended = false);
#endif

  /**
   * Copies the given keypoints and computes their SURF descriptors
   * @param image
   * @param keypoints
   * @param extended if true, 128-dimensional surf are used instead of 64-d
   * @note the laplacian vector cannot be computed, so it is filled with ones
   */
  void Compute(const cv::Mat &image,
    const std::vector<cv::KeyPoint> &keypoints, bool extended = false);

#if USURF_SUPPORTED  
  /**
   * Copies the given keypoints and computes their U-SURF descriptors
   * NOTE: this function is only available if opencv supports U-SURF
   * (by applying the corresponding patch)
   * @param image
   * @param keypoints
   * @param extended if true, 128-dimensional surf are used instead of 64-d
   * @note the laplacian vector cannot be computed, so it is filled with ones
   */
  void _ComputeUpright(const cv::Mat &image,
    const std::vector<cv::KeyPoint> &keypoints, bool extended = false);
#endif
  
  /**
   * Returns the length of the descriptor vectors of the SURF features
   * or 0 if no features have been extracted yet
   * @return descriptor vector length
   */
  inline int GetDescriptorLength() const 
  {
    if(!keys.empty()) return descriptors.size() / keys.size();
    else return 0;
  }  

  /**
   * Returns the number of keypoints in the set
   */
  inline unsigned int size() const
  {
    return keys.size();
  }
  
  /**
   * Returns whether the key collection is empty
   */
  inline bool empty() const
  {
    return keys.empty();
  }
  
  /**
   * Returns a copy of the i-th descriptor
   * @param i
   * @return i-th descriptor
   */
  inline std::vector<float> get(unsigned int i) const;
  
  /**
   * Removes a point from the set, preserving the order
   * @param i index to remove
   */
  void Remove(unsigned int i);
  
  /**
   * Removes several poitns from the set preserving the order
   * @param ids indexes to remove
   */
  void Remove(const std::vector<unsigned int> &ids);
  
	/**
	 * Calculates correspondences between this set and the set B
	 * @param B other set
	 * @param A_corr indices of matched points from this set 
	 * @param B_corr indices of matched points from the set B
	 * @param distances if given, the distance between the correspondences is
	 *        stored here
	 * @param remove_duplicates if true, ambiguous matches are discarded
	 * @param max_ratio max ratio between the two nearest neighbours
	 */
	void CalculateCorrespondences(const SurfSet &B,
		std::vector<int> &A_corr, std::vector<int> &B_corr,
		std::vector<double> *distances = NULL,
		bool remove_duplicates = true, 
		double max_ratio = 0.6) const;

  /**
	 * Calculates correspondences between this set and the set B by using flann
	 * @param B other set
	 * @param A_corr indices of matched points from this set 
	 * @param B_corr indices of matched points from the set B
	 * @param distances if given, the distance between the correspondences is
	 *        stored here
	 * @param remove_duplicates if true, ambiguous matches are discarded
	 * @param max_ratio max ratio between the two nearest neighbours
	 */
  void CalculateFastCorrespondences(const SurfSet &B,
		std::vector<int> &A_corr, std::vector<int> &B_corr,
		std::vector<double> *distances = NULL,
		bool remove_duplicates = true, 
		double max_ratio = 0.6);

  /**
   * Creates the flann index to compute approximate correspondences.
   * This function is called automatically the first time when 
   * CalculateCorrespondences is invoked in the approximate mode
   */
  void RecalculateApproximationTree();

  /**
   * Saves the features into a file with a OpenCV writer
   * @param filename
   */
  void Save(const std::string &filename) const;

  /**
   * Loads the features from filename and removes the current ones
   * @param filename
   */
  void Load(const std::string &filename);

	/** Saves the features into a file in a custom format
	 * @param filename
	 * @note This function is DEPRECATED
	 * @note The file format starts with 2 integers giving the total
	 * number of keypoints and the size of descriptor vector for each
	 * keypoint. Then each keypoint is specified by the following fields:
	 * x y angle size hessian octave laplacian_sign descriptor_vector
	 */
	void SaveCustom(const std::string &filename) const;
	
	/** Loads the features from a custom file, removing the current ones
	 * @param filename
	 * @note This function is DEPRECATED
	 */
	void LoadCustom(const std::string &filename);

protected:
  friend class Matches;
  
  /**
   * Saves the surfset in the given file storage
   * The keys are created with the suffix idx
   * @param fs
   * @param idx index given to the saved set
   */
  void save(cv::FileStorage &fs, int idx) const;
  
  /**
   * Loads the surfset from the given file storage
   * The keys are created with the suffix idx
   * @param fs
   * @param idx index of set to load
   */
  void load(cv::FileStorage &fs, int idx);

protected:

  /**
   * Performs the actual extraction with the provided parameters
   * @param image
   * @param params surf detector params
   * @see SurfSet::Extract
   */
   
#ifdef HAVE_OPENCV3
  void extract(const cv::Mat &image, const cv::xfeatures2d::SURF &params);
#else // HAVE_OPENCV3
  void extract(const cv::Mat &image, const CvSURFParams &params);
#endif // HAVE_OPENCV3

  /**
   * Performs the actual computation with the provided parameters
   * @param image
   * @param keypoints keypoints to set
   * @param params surf detector parameters
   * @see SurfSet::Compute
   */
#ifdef HAVE_OPENCV3
  void compute(const cv::Mat &image,
    const std::vector<cv::KeyPoint> &keypoints, const cv::xfeatures2d::SURF &params);
#else // HAVE_OPENCV3
  void compute(const cv::Mat &image,
    const std::vector<cv::KeyPoint> &keypoints, const CvSURFParams &params);
    
#endif // HAVE_OPENCV3

	/** 
	 * Calculates the square distance between two descriptors
	 * @param ita beginning of one descriptor
	 * @param itb beginning of the other descriptor
	 * @param L length of descriptor vectors
	 */
	double calculateSqDistance(std::vector<float>::const_iterator ita, 
		std::vector<float>::const_iterator itb, const int L) const;

  /**
	 * Calculates correspondences between this set and the set B by brute force
	 * @param B other set
	 * @param A_corr indices of matched points from this set 
	 * @param B_corr indices of matched points from the set B
	 * @param distances the distance between the correspondences is stored here
	 * @param remove_duplicates if true, ambiguous matches are discarded
	 * @param max_ratio max ratio between the two nearest neighbours
	 */
	void calculateCorrespondencesNaive(const SurfSet &B,
		std::vector<int> &A_corr, std::vector<int> &B_corr,
		std::vector<double> *distances,
		bool remove_duplicates, double max_ratio) const;

  /**
	 * Calculates correspondences between this set and the set B by flann
	 * @param B other set
	 * @param A_corr indices of matched points from this set 
	 * @param B_corr indices of matched points from the set B
	 * @param distances the distance between the correspondences is stored here
	 * @param remove_duplicates if true, ambiguous matches are discarded
	 * @param max_ratio max ratio between the two nearest neighbours
	 */
	void calculateCorrespondencesApproximate(const SurfSet &B,
		std::vector<int> &A_corr, std::vector<int> &B_corr,
		std::vector<double> *distances,
		bool remove_duplicates, double max_ratio);

  /**
   * Returns the octave of a surf keypoint
   * @param kpt keypoint
   * @param params extractor parameters
   * @note This function is copied from the opencv surf.cpp file, written
   *    by Liu Liu
   */
#ifndef HAVE_OPENCV3
  int getPointOctave(const CvSURFPoint& kpt, const CvSURFParams& params) const;
#endif

protected:
  
  /// Flann index
  cv::flann::Index *m_index;
};

// ---------------------------------------------------------------------------

std::vector<float> SurfSet::get(unsigned int i) const
{
  std::vector<float> ret;
  if(i < keys.size())
  {
    const unsigned int L = descriptors.size() / keys.size();
    ret.resize(L);
    std::copy(this->descriptors.begin() + i*L, 
      this->descriptors.begin() + (i+1)*L, ret.begin());
  }
  return ret;
}

}

#endif

