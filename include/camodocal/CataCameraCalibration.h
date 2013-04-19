#ifndef CATACAMERACALIBRATION_H
#define CATACAMERACALIBRATION_H

#include <opencv2/core/core.hpp>

#include "CataCamera.h"

namespace camodocal
{

class CataCameraCalibration
{
public:
    CataCameraCalibration(const std::string& cameraName,
                          const cv::Size& imageSize);

    void clear(void);

    void addChessboardData(const std::vector<cv::Point2f>& corners);

    bool calibrate(const cv::Size& boardSize, float squareSize);

    int sampleCount(void) const;
    const std::vector< std::vector<cv::Point2f> >& imagePoints(void) const;
    CataCamera::Parameters& cameraParameters(void);
    const CataCamera::Parameters& cameraParameters(void) const;

    void drawResults(const cv::Size& boardSize, float squareSize,
                     std::vector<cv::Mat>& images) const;

    void writeParams(const std::string& filename) const;
    void setVerbose(bool verbose);

private:
    bool calibrateHelper(const cv::Size& boardSize,
                         const std::vector< std::vector<cv::Point3f> >& objectPoints,
                         const std::vector< std::vector<cv::Point2f> >& imagePoints,
                         CataCamera::Parameters& cameraParams,
                         std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const;

    void estimateExtrinsic(const CataCamera::Parameters& cameraParams,
                           const std::vector<cv::Point3f>& objectPoints,
                           const std::vector<cv::Point2f>& imagePoints,
                           cv::Mat& rvec, cv::Mat& tvec) const;

    void optimize(const std::vector< std::vector<cv::Point3f> >& objectPoints,
                  const std::vector< std::vector<cv::Point2f> >& imagePoints,
                  CataCamera::Parameters& cameraParams,
                  std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const;

    void projectPoints(const std::vector<cv::Point3f>& objectPoints,
                       const cv::Mat& rvec,
                       const cv::Mat& tvec,
                       const CataCamera::Parameters& cameraParams,
                       std::vector<cv::Point2f>& imagePoints) const;

    float computeReprojectionError(const std::vector< std::vector<cv::Point3f> >& objectPoints,
                                   const std::vector< std::vector<cv::Point2f> >& imagePoints,
                                   const CataCamera::Parameters& cameraParams,
                                   const std::vector<cv::Mat>& rvecs,
                                   const std::vector<cv::Mat>& tvecs,
                                   cv::Mat& perViewErrors) const;

    cv::Size mBoardSize;
    float mSquareSize;
    cv::Mat mExtrParams;
    cv::Mat mReprojErrs;
    float mAvgReprojErr;
    CataCamera::Parameters mCameraParams;

    std::vector< std::vector<cv::Point2f> > mImagePoints;

    bool mVerbose;
};

}

#endif
