#ifndef SPARSEGRAPH_H
#define SPARSEGRAPH_H

#include <boost/unordered_map.hpp>
#include <boost/weak_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <camodocal/sparse_graph/Odometry.h>
#include <camodocal/sparse_graph/Pose.h>

namespace camodocal
{

typedef struct
{
    int frameSetSegmentId;
    int frameSetId;
    int frameId;
} FrameTag;

class Point2DFeature;
class Point3DFeature;

typedef boost::shared_ptr<Point2DFeature> Point2DFeaturePtr;
typedef boost::weak_ptr<Point2DFeature> Point2DFeatureWPtr;
typedef boost::shared_ptr<const Point2DFeature> Point2DFeatureConstPtr;
typedef boost::weak_ptr<const Point2DFeature> Point2DFeatureConstWPtr;

typedef boost::shared_ptr<Point3DFeature> Point3DFeaturePtr;
typedef boost::weak_ptr<Point3DFeature> Point3DFeatureWPtr;
typedef boost::shared_ptr<const Point3DFeature> Point3DFeatureConstPtr;
typedef boost::weak_ptr<const Point3DFeature> Point3DFeatureConstWPtr;

class Frame
{
public:
    Frame();

    PosePtr& cameraPose(void);
    PoseConstPtr cameraPose(void) const;

    int& cameraId(void);
    int cameraId(void) const;

    OdometryPtr& systemPose(void);
    OdometryConstPtr systemPose(void) const;

    OdometryPtr& odometryMeasurement(void);
    OdometryConstPtr odometryMeasurement(void) const;

    PosePtr& gpsInsMeasurement(void);
    PoseConstPtr gpsInsMeasurement(void) const;

    std::vector<Point2DFeaturePtr>& features2D(void);
    const std::vector<Point2DFeaturePtr>& features2D(void) const;

    cv::Mat& image(void);
    const cv::Mat& image(void) const;

private:
    PosePtr m_cameraPose;
    int m_cameraId;

    OdometryPtr m_systemPose;
    OdometryPtr m_odometryMeasurement;
    PosePtr m_gpsInsMeasurement;

    std::vector<Point2DFeaturePtr> m_features2D;

    cv::Mat m_image;
};

typedef boost::shared_ptr<Frame> FramePtr;
typedef boost::weak_ptr<Frame> FrameWPtr;
typedef boost::shared_ptr<const Frame> FrameConstPtr;
typedef boost::weak_ptr<const Frame> FrameConstWPtr;

class Point2DFeature
{
public:
    Point2DFeature();

    cv::Mat& descriptor(void);
    const cv::Mat& descriptor(void) const;

    cv::KeyPoint& keypoint(void);
    const cv::KeyPoint& keypoint(void) const;

    unsigned int& index(void);
    unsigned int index(void) const;

    Point2DFeatureWPtr& prevMatch(void);
    Point2DFeatureConstWPtr prevMatch(void) const;

    std::vector<Point2DFeatureWPtr>& prevMatches(void);
    const std::vector<Point2DFeatureWPtr>& prevMatches(void) const;

    int& bestPrevMatchId(void);
    int bestPrevMatchId(void) const;

    Point2DFeatureWPtr& nextMatch(void);
    Point2DFeatureConstWPtr nextMatch(void) const;

    std::vector<Point2DFeatureWPtr>& nextMatches(void);
    const std::vector<Point2DFeatureWPtr>& nextMatches(void) const;

    int& bestNextMatchId(void);
    int bestNextMatchId(void) const;

    Point3DFeaturePtr& feature3D(void);
    Point3DFeatureConstPtr feature3D(void) const;

    FrameWPtr& frame(void);
    FrameConstWPtr frame(void) const;

protected:
    cv::Mat m_dtor;
    cv::KeyPoint m_keypoint;

    unsigned int m_index;

private:
    std::vector<Point2DFeatureWPtr> m_prevMatches;
    std::vector<Point2DFeatureWPtr> m_nextMatches;
    int m_bestPrevMatchId;
    int m_bestNextMatchId;
    Point3DFeaturePtr m_feature3D;
    FrameWPtr m_frame;
};

class Point3DFeature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum
    {
        LOCALLY_OBSERVED_BY_DIFFERENT_CAMERAS = 0x1
    };

    Point3DFeature();

    Eigen::Vector3d& point(void);
    const Eigen::Vector3d& point(void) const;

    double* pointData(void);
    const double* const pointData(void) const;

    Eigen::Matrix3d& pointCovariance(void);
    const Eigen::Matrix3d& pointCovariance(void) const;

    double* pointCovarianceData(void);
    const double* const pointCovarianceData(void) const;

    int& attributes(void);
    int attributes(void) const;

    double& weight(void);
    double weight(void) const;

    std::vector<Point2DFeatureWPtr>& features2D(void);
    const std::vector<Point2DFeatureWPtr>& features2D(void) const;

    bool removeFeatureObservation(const Point2DFeaturePtr& featureObs);

private:
    Eigen::Vector3d m_point;
    Eigen::Matrix3d m_pointCovariance;
    int m_attributes;
    double m_weight;
    std::vector<Point2DFeatureWPtr> m_features2D;
};

class FrameSet
{
public:
    FrameSet();

    FramePtr& frame(int cameraId);
    FrameConstPtr frame(int cameraId) const;

    std::vector<FramePtr>& frames(void);
    const std::vector<FramePtr>& frames(void) const;

    OdometryPtr& systemPose(void);
    OdometryConstPtr systemPose(void) const;

    OdometryPtr& odometryMeasurement(void);
    OdometryConstPtr odometryMeasurement(void) const;

    PosePtr& gpsInsMeasurement(void);
    PoseConstPtr gpsInsMeasurement(void) const;

private:
    std::vector<FramePtr> m_frames;

    OdometryPtr m_systemPose;
    OdometryPtr m_odometryMeasurement;
    PosePtr m_gpsInsMeasurement;
};

typedef boost::shared_ptr<FrameSet> FrameSetPtr;
typedef boost::shared_ptr<const FrameSet> FrameSetConstPtr;

typedef std::vector<FrameSetPtr> FrameSetSegment;

class SparseGraph
{
public:
    SparseGraph();

    FrameSetSegment& frameSetSegment(int segmentId);
    const FrameSetSegment& frameSetSegment(int segmentId) const;

    std::vector<FrameSetSegment>& frameSetSegments(void);
    const std::vector<FrameSetSegment>& frameSetSegments(void) const;

    size_t scenePointCount(void) const;

    bool readFromBinaryFile(const std::string& filename);
    void writeToBinaryFile(const std::string& filename) const;

private:
    template<typename T>
    void readData(std::ifstream& ifs, T& data) const;

    template<typename T>
    void writeData(std::ofstream& ofs, T data) const;

    std::vector<FrameSetSegment> m_frameSetSegments;
};

}

#endif
