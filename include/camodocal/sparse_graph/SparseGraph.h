#ifndef SPARSEGRAPH_H
#define SPARSEGRAPH_H

#include <boost/filesystem.hpp>
#include <boost/unordered_map.hpp>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Odometry.h"

// forward declarations
namespace pugi
{
class xml_node;
}

namespace camodocal
{

typedef struct
{
    int cameraIdx;
    int segmentIdx;
    int frameIdx;
} FrameID;

class Point2DFeature;
class Point3DFeature;

typedef boost::shared_ptr<Point2DFeature> Point2DFeaturePtr;
typedef boost::shared_ptr<const Point2DFeature> Point2DFeatureConstPtr;

typedef boost::shared_ptr<Point3DFeature> Point3DFeaturePtr;
typedef boost::shared_ptr<const Point3DFeature> Point3DFeatureConstPtr;

class Pose
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Pose();
    Pose(const Eigen::Matrix4d& H);

    uint64_t& timeStamp(void);
    uint64_t timeStamp(void) const;

    Eigen::Quaterniond& rotation(void);
    const Eigen::Quaterniond& rotation(void) const;
    double* rotationData(void);
    const double* const rotationData(void) const;

    Eigen::Vector3d& translation(void);
    const Eigen::Vector3d& translation(void) const;
    double* translationData(void);
    const double* const translationData(void) const;

    Eigen::Matrix4d pose(void) const;

    Eigen::Matrix<double,7,7>& covariance(void);
    const Eigen::Matrix<double,7,7>& covariance(void) const;
    double* covarianceData(void);
    const double* const covarianceData(void) const;

private:
    uint64_t m_timeStamp;
    Eigen::Quaterniond m_q;
    Eigen::Vector3d m_t;
    Eigen::Matrix<double,7,7> m_covariance;
};

typedef boost::shared_ptr<Pose> PosePtr;
typedef boost::shared_ptr<const Pose> PoseConstPtr;

class Frame
{
public:
    Frame();

    PosePtr& camera(void);
    PoseConstPtr camera(void) const;

    int& cameraId(void);
    int cameraId(void) const;

    OdometryPtr& odometryUnopt(void);
    OdometryConstPtr odometryUnopt(void) const;

    OdometryPtr& odometryOpt(void);
    OdometryConstPtr odometryOpt(void) const;

    PosePtr& gps_ins(void);
    PoseConstPtr gps_ins(void) const;

    std::vector<Point2DFeaturePtr>& features2D(void);
    const std::vector<Point2DFeaturePtr>& features2D(void) const;

    std::vector<Point3DFeaturePtr>& features3D(void);
    const std::vector<Point3DFeaturePtr>& features3D(void) const;

    unsigned int& id(void);
    unsigned int id(void) const;

    cv::Mat& image(void);
    const cv::Mat& image(void) const;

private:
    PosePtr m_camera;
    int m_cameraId;
    OdometryPtr m_odometryUnopt;
    OdometryPtr m_odometryOpt;
    PosePtr m_gpsIns;

    std::vector<Point2DFeaturePtr> m_features2D;
    std::vector<Point3DFeaturePtr> m_features3D;

    unsigned int m_id;
    cv::Mat m_image;
};

typedef boost::shared_ptr<Frame> FramePtr;
typedef boost::shared_ptr<const Frame> FrameConstPtr;

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

    Point2DFeaturePtr& prevMatch(void);
    Point2DFeatureConstPtr prevMatch(void) const;

    std::vector<Point2DFeaturePtr>& prevMatches(void);
    const std::vector<Point2DFeaturePtr>& prevMatches(void) const;

    int& bestPrevMatchIdx(void);
    int bestPrevMatchIdx(void) const;

    Point2DFeaturePtr& nextMatch(void);
    Point2DFeatureConstPtr nextMatch(void) const;

    std::vector<Point2DFeaturePtr>& nextMatches(void);
    const std::vector<Point2DFeaturePtr>& nextMatches(void) const;

    int& bestNextMatchIdx(void);
    int bestNextMatchIdx(void) const;

    Point3DFeaturePtr& feature3D(void);
    Point3DFeatureConstPtr feature3D(void) const;

    FramePtr& frame(void);
    FrameConstPtr frame(void) const;

protected:
    cv::Mat m_dtor;
    cv::KeyPoint m_keypoint;

    unsigned int m_index;

private:
    std::vector<Point2DFeaturePtr> m_prevMatches;
    std::vector<Point2DFeaturePtr> m_nextMatches;
    int m_bestPrevMatchIdx;
    int m_bestNextMatchIdx;
    Point3DFeaturePtr m_feature3D;
    FramePtr m_frame;
};

class Point2DFeatureLeft;
class Point2DFeatureRight;

typedef boost::shared_ptr<Point2DFeatureLeft> Point2DFeatureLeftPtr;
typedef boost::shared_ptr<Point2DFeatureRight> Point2DFeatureRightPtr;
typedef boost::shared_ptr<const Point2DFeatureLeft> Point2DFeatureLeftConstPtr;
typedef boost::shared_ptr<const Point2DFeatureRight> Point2DFeatureRightConstPtr;

class Point2DFeatureLeft: public Point2DFeature
{
public:
    Point2DFeatureRightPtr& rightCorrespondence(void);
    Point2DFeatureRightConstPtr rightCorrespondence(void) const;

    Point2DFeatureLeftPtr& prevCorrespondence(void);
    Point2DFeatureLeftConstPtr prevCorrespondence(void) const;

private:
    Point2DFeatureRightPtr m_rightCorrespondence;
    Point2DFeatureLeftPtr m_prevCorrespondence;
};

class Point2DFeatureRight: public Point2DFeature
{
public:
    Point2DFeatureLeftPtr& leftCorrespondence(void);
    Point2DFeatureLeftConstPtr leftCorrespondence(void) const;

    Point2DFeatureRightPtr& prevCorrespondence(void);
    Point2DFeatureRightConstPtr prevCorrespondence(void) const;

private:
    Point2DFeatureLeftPtr m_leftCorrespondence;
    Point2DFeatureRightPtr m_prevCorrespondence;
};

class Point3DFeature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Point3DFeature();

    Eigen::Vector3d& point(void);
    const Eigen::Vector3d& point(void) const;

    double* pointData(void);
    const double* const pointData(void) const;

    Eigen::Matrix3d& pointCovariance(void);
    const Eigen::Matrix3d& pointCovariance(void) const;

    double* pointCovarianceData(void);
    const double* const pointCovarianceData(void) const;

    std::vector<Point2DFeaturePtr>& features2D(void);
    const std::vector<Point2DFeaturePtr>& features2D(void) const;

private:
    Eigen::Vector3d m_point;
    Eigen::Matrix3d m_pointCovariance;
    std::vector<Point2DFeaturePtr> m_features2D;
};

typedef std::vector<FramePtr> FrameSegment;

class SparseGraph
{
public:
    SparseGraph();

    int cameraCount(void) const;

    std::vector<FrameSegment>& frameSegments(int cameraIdx);
    const std::vector<FrameSegment>& frameSegments(int cameraIdx) const;

    bool readFromBinaryFile(const std::string& filename);
    void writeToBinaryFile(const std::string& filename) const;

private:
    template<typename T>
    void readData(std::ifstream& ifs, T& data) const;

    template<typename T>
    void writeData(std::ofstream& ofs, T data) const;

    std::vector<std::vector<FrameSegment> > m_frameSegments;
};

}

#endif
