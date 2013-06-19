#ifndef STEREOCAMERACALIBRATION_H
#define STEREOCAMERACALIBRATION_H

#include "CameraCalibration.h"

namespace camodocal
{

class StereoCameraCalibration
{
public:
    StereoCameraCalibration(Camera::ModelType modelType,
                            const std::string& cameraLeftName,
                            const std::string& cameraRightName,
                            const cv::Size& imageSize);

    void clear(void);

    void addChessboardData(const std::vector<cv::Point2f>& cornersLeft,
                           const std::vector<cv::Point2f>& cornersRight);

    bool calibrate(const cv::Size& boardSize, float squareSize);

    int sampleCount(void) const;
    const std::vector< std::vector<cv::Point2f> >& imagePointsLeft(void) const;
    const std::vector< std::vector<cv::Point2f> >& imagePointsRight(void) const;

    CameraPtr& cameraLeft(void);
    const CameraConstPtr cameraLeft(void) const;

    CameraPtr& cameraRight(void);
    const CameraConstPtr cameraRight(void) const;

    void drawResults(const cv::Size& boardSize, float squareSize,
                     std::vector<cv::Mat>& imagesLeft,
                     std::vector<cv::Mat>& imagesRight) const;

    void writeParams(const std::string& directory) const;
    void setVerbose(bool verbose);

private:
    CameraCalibration m_calibLeft;
    CameraCalibration m_calibRight;

    Eigen::Quaterniond m_q;
    Eigen::Vector3d m_t;

    bool m_verbose;
};

}

#endif
