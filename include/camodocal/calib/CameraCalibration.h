#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <opencv2/core/core.hpp>

#include "camodocal/camera_models/Camera.h"

namespace camodocal
{

class CameraCalibration
{
public:
    CameraCalibration(Camera::ModelType modelType,
                      const std::string& cameraName,
                      const cv::Size& imageSize);

    void clear(void);

    void addChessboardData(const std::vector<cv::Point2f>& corners);

    bool calibrate(const cv::Size& boardSize, float squareSize);

    int sampleCount(void) const;
    std::vector< std::vector<cv::Point2f> >& imagePoints(void);
    const std::vector< std::vector<cv::Point2f> >& imagePoints(void) const;
    CameraPtr& camera(void);
    const CameraConstPtr camera(void) const;

    cv::Mat& cameraPoses(void);
    const cv::Mat& cameraPoses(void) const;

    void drawResults(const cv::Size& boardSize, float squareSize,
                     std::vector<cv::Mat>& images) const;

    void writeParams(const std::string& filename) const;
    void setVerbose(bool verbose);

private:
    bool calibrateHelper(const cv::Size& boardSize,
                         const std::vector< std::vector<cv::Point3f> >& objectPoints,
                         const std::vector< std::vector<cv::Point2f> >& imagePoints,
                         CameraPtr& camera,
                         std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const;

    void optimize(const std::vector< std::vector<cv::Point3f> >& objectPoints,
                  const std::vector< std::vector<cv::Point2f> >& imagePoints,
                  CameraPtr& camera,
                  std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const;

    cv::Mat mCameraPoses;
    cv::Mat mReprojErrs;
    float mAvgReprojErr;
    CameraPtr mCamera;

    std::vector< std::vector<cv::Point2f> > mImagePoints;

    bool mVerbose;
};

}

#endif
