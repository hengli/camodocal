#ifndef GPUWRAP_H
#define GPUWRAP_H

#include <iostream>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/gpu/gpu.hpp>

using namespace cv; 
using namespace cv::gpu; 

class SurfGpu: public Feature2D 
{
public:
    SurfGpu(double _hessianThreshold, int _nOctaves=4,
            int _nOctaveLayers=2, bool _extended=false, float _keypointsRatio=0.01f); 

    // returns the descriptor size in bytes
    int descriptorSize() const;
    // returns the descriptor type
    int descriptorType() const;

    // Compute the SurfGpu features and descriptors on an image
    void operator()(InputArray image, InputArray mask, vector<KeyPoint>& keypoints) const;

    // Compute the SurfGpu features and descriptors on an image
    void operator()( InputArray image, InputArray mask, vector<KeyPoint>& keypoints,
                     OutputArray descriptors, bool useProvidedKeypoints=false ) const;

//    AlgorithmInfo* info() const;

protected:

    void computeImpl( const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors ) const;
    void detectImpl( const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask=Mat() ) const;

    double hessianThreshold; 
    int nOctaves; 
    int nOctaveLayers; 
    bool extended; 
    float keypointsRatio; 


}; 


typedef SurfGpu SurfGpuFeatureDetector; 
typedef SurfGpu SurfGpuDescriptorExtractor;




class BFMatcherGpu: public DescriptorMatcher
{
public:
    BFMatcherGpu( int normType, bool crossCheck=false );
    virtual ~BFMatcherGpu() {}

    virtual bool isMaskSupported() const { return true; }
    virtual Ptr<DescriptorMatcher> clone( bool emptyTrainData=false ) const;

protected:
    virtual void knnMatchImpl( const Mat& queryDescriptors, vector<vector<DMatch> >& matches, int k,
           const vector<Mat>& masks=vector<Mat>(), bool compactResult=false );
    virtual void radiusMatchImpl( const Mat& queryDescriptors, vector<vector<DMatch> >& matches, float maxDistance,
           const vector<Mat>& masks=vector<Mat>(), bool compactResult=false );
    int normType;
    bool crossCheck;
}; 








#endif
