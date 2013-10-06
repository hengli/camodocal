#ifndef SURFGPU_H
#define SURFGPU_H

#include <boost/thread/mutex.hpp>
#include <opencv2/nonfree/gpu.hpp>

namespace camodocal
{

class SurfGPU
{
public:
    SurfGPU(double hessianThreshold, int nOctaves=4,
            int nOctaveLayers=2, bool extended=false,
            float keypointsRatio=0.01f);

    ~SurfGPU();

    static cv::Ptr<SurfGPU> instance(double hessianThreshold, int nOctaves=4,
                                     int nOctaveLayers=2, bool extended=false,
                                     float keypointsRatio=0.01f);

    void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                const cv::Mat& mask = cv::Mat());
    void compute(const cv::Mat& image,
                 std::vector<cv::KeyPoint>& keypoints,
                 cv::Mat& descriptors);

    void knnMatch(const cv::Mat& queryDescriptors, const cv::Mat& trainDescriptors,
                  std::vector<std::vector<cv::DMatch> >& matches, int k,
                  const cv::Mat& mask = cv::Mat(), bool compactResult = false);
    void radiusMatch(const cv::Mat& queryDescriptors, const cv::Mat& trainDescriptors,
                     std::vector<std::vector<cv::DMatch> >& matches, float maxDistance,
                     const cv::Mat& mask = cv::Mat(), bool compactResult = false);

    void match(const cv::Mat& image1, std::vector<cv::KeyPoint>& keypoints1,
               const cv::Mat& mask1,
               const cv::Mat& image2, std::vector<cv::KeyPoint>& keypoints2,
               const cv::Mat& mask2,
               std::vector<cv::DMatch>& matches,
               bool useProvidedKeypoints = false);

private:
    static cv::Ptr<SurfGPU> m_instance;
    static boost::mutex m_instanceMutex;

    cv::gpu::SURF_GPU m_surfGPU;
    cv::gpu::BruteForceMatcher_GPU<cv::L2<float> > m_matcher;
};

}

#endif
