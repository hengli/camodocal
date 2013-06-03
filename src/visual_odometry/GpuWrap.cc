#include "GpuWrap.h"

using namespace cv; 
using namespace cv::gpu; 

SurfGpu::SurfGpu(double _hessianThreshold, int _nOctaves,
                 int _nOctaveLayers, bool _extended, float _keypointsRatio)
    : hessianThreshold(_hessianThreshold), nOctaves(_nOctaves), 
      nOctaveLayers(_nOctaveLayers), extended(_extended), 
      keypointsRatio(_keypointsRatio) 
{
    DeviceInfo info; 
    if (getCudaEnabledDeviceCount() > 0 && info.isCompatible())
    {
        setDevice(0); 
        std::cout << "Cuda Enabled Device is found!\n"; 
    }
    else 
    {
        std::cout << "No Cuda Enable Device found!\n"; 
        exit(0); 
    }
    
}
int SurfGpu::descriptorSize() const
{
    return extended ? 128 : 64;
}

int SurfGpu::descriptorType() const
{
    return CV_32F;
}
void 
SurfGpu::operator()(InputArray _image, InputArray _mask, vector<KeyPoint>& _keypoints,
                    OutputArray _descriptors, bool useProvidedKeypoints) const
{
    bool do_keypoints = !useProvidedKeypoints;
    bool do_descriptors = _descriptors.needed();

    if( (!do_keypoints && !do_descriptors) || _image.empty() )
        return;
    
    Mat image = _image.getMat(), mask = _mask.getMat(); 
    if(image.type() != CV_8UC1) cvtColor(_image, image, CV_BGR2GRAY); 

    GpuMat imageGpu, maskGpu; 
    imageGpu.upload(image); 
    maskGpu.upload(mask); 

    SURF_GPU surf(hessianThreshold, nOctaves, nOctaveLayers, extended, keypointsRatio);     
//    std::vector<KeyPoint> keypoints; 
    if (do_keypoints)
    {
        surf(imageGpu, maskGpu, _keypoints); 
    }
    
    if (do_descriptors)
    {
        GpuMat dtorsGpu; 
        Mat dtors; 
        
        surf(imageGpu, maskGpu, _keypoints, dtorsGpu, true); 
        dtorsGpu.download(dtors); 
        
        dtorsGpu.release(); 

        _descriptors.create(dtors.rows, dtors.cols, dtors.type()); 
        Mat wrap = _descriptors.getMat(); 
        dtors.copyTo(wrap); 
    }

    imageGpu.release(); 
    maskGpu.release(); 
    surf.releaseMemory(); 

}

void SurfGpu::operator()(InputArray image, InputArray mask, vector<KeyPoint>& keypoints) const
{
    (*this)(image, mask, keypoints, noArray(), false);
}

void SurfGpu::detectImpl( const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask) const
{
    (*this)(image, mask, keypoints, noArray(), false);
}

void SurfGpu::computeImpl( const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors) const
{
    (*this)(image, Mat(), keypoints, descriptors, true);
}




BFMatcherGpu::BFMatcherGpu(int _normType, bool _crossCheck)
//    : BFMatcher(_normType, _crossCheck)
{
    normType = _normType;
    crossCheck = _crossCheck;

    DeviceInfo info; 
    if (getCudaEnabledDeviceCount() > 0 && info.isCompatible())
    {
        setDevice(0); 
        std::cout << "Cuda Enabled Device is found!\n"; 
    }
    else 
    {
        std::cout << "No Cuda Enable Device found!\n"; 
        exit(0); 
    }
}

void 
BFMatcherGpu::knnMatchImpl(const Mat& queryDescriptors, vector<vector<DMatch> >& matches, int knn,
                           const vector<Mat>& masks, bool compactResult )
{
    if( queryDescriptors.empty() || trainDescCollection.empty() )
    {
        matches.clear();
        return;
    }
    CV_Assert( queryDescriptors.type() == trainDescCollection[0].type() );
    if (trainDescCollection.size() > 1)
    {
        std::cout << "Warning: only support matching from 1 image to 1 image. Only the 1st set of train descriptors is used. " << std::endl; 
    }
    
    matches.reserve(queryDescriptors.rows);
    
    GpuMat qDtorsGpu, tDtorsGpu; 
    qDtorsGpu.upload(queryDescriptors); 
    tDtorsGpu.upload(trainDescCollection[0]); 

    GpuMat maskGpu; 
    maskGpu.upload(masks[0]); 

    BruteForceMatcher_GPU<cv::L2<float> > matcher; 
       
    matcher.knnMatch(qDtorsGpu, tDtorsGpu, matches, knn, maskGpu, compactResult); 

    qDtorsGpu.release(); 
    tDtorsGpu.release(); 
    maskGpu.release(); 

/*    BruteForceMatcher_GPU<cv::L2<float> > matcher; 
    matcher.knnMatch(GpuMat(queryDescriptors), GpuMat(trainDescCollection[0]), matches, knn, GpuMat(masks[0]), compactResult); 
*/
   
}

void 
BFMatcherGpu::radiusMatchImpl(const Mat& queryDescriptors, vector<vector<DMatch> >& matches,
                              float maxDistance, const vector<Mat>& masks, bool compactResult )
{
    std::cout << "===================================\n"; 
    if( queryDescriptors.empty() || trainDescCollection.empty() )
    {
        matches.clear();
        return;
    }
    CV_Assert( queryDescriptors.type() == trainDescCollection[0].type() );
    
    if (trainDescCollection.size() > 1)
    {
        std::cout << "Warning: only support matching from 1 image to 1 image. Only the 1st set of train descriptors is used. " << std::endl; 
    }

    
    BruteForceMatcher_GPU<cv::L2<float> > matcher; 
    matcher.radiusMatch(GpuMat(queryDescriptors), GpuMat(trainDescCollection[0]), matches, maxDistance, GpuMat(masks[0]), compactResult); 


}
    

Ptr<DescriptorMatcher> BFMatcherGpu::clone( bool emptyTrainData ) const
{
    BFMatcherGpu* matcher = new BFMatcherGpu(normType, crossCheck);
    if( !emptyTrainData )
    {
        matcher->trainDescCollection.resize(trainDescCollection.size());
        std::transform( trainDescCollection.begin(), trainDescCollection.end(),
                        matcher->trainDescCollection.begin(), clone_op );
    }
    return matcher;
}



