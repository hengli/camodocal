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

private:
    uint64_t mTimeStamp;
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

    OdometryPtr& odometry(void);
    OdometryConstPtr odometry(void) const;

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
    PosePtr mCamera;
    int mCameraId;
    OdometryPtr mOdometry;
    PosePtr mGpsIns;

    std::vector<Point2DFeaturePtr> mFeatures2D;
    std::vector<Point3DFeaturePtr> mFeatures3D;

    unsigned int mId;
    cv::Mat mImage;
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
    cv::Mat mDtor;
    cv::KeyPoint mKeypoint;

    unsigned int mIndex;

private:
    std::vector<Point2DFeaturePtr> mPrevMatches;
    std::vector<Point2DFeaturePtr> mNextMatches;
    int mBestPrevMatchIdx;
    int mBestNextMatchIdx;
    Point3DFeaturePtr mFeature3D;
    FramePtr mFrame;
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
    Point2DFeatureRightPtr mRightCorrespondence;
    Point2DFeatureLeftPtr mPrevCorrespondence;
};

class Point2DFeatureRight: public Point2DFeature
{
public:
    Point2DFeatureLeftPtr& leftCorrespondence(void);
    Point2DFeatureLeftConstPtr leftCorrespondence(void) const;

    Point2DFeatureRightPtr& prevCorrespondence(void);
    Point2DFeatureRightConstPtr prevCorrespondence(void) const;

private:
    Point2DFeatureLeftPtr mLeftCorrespondence;
    Point2DFeatureRightPtr mPrevCorrespondence;
};

class Point3DFeature
{
public:
    Point3DFeature();

    Eigen::Vector3d& point(void);
    const Eigen::Vector3d& point(void) const;

    double* pointData(void);
    const double* const pointData(void) const;

    std::vector<Point2DFeaturePtr>& features2D(void);
    const std::vector<Point2DFeaturePtr>& features2D(void) const;

private:
    Eigen::Vector3d mPoint;
    std::vector<Point2DFeaturePtr> mFeatures2D;
};

typedef std::vector<FramePtr> FrameSegment;

class SparseGraph
{
public:
    SparseGraph();

    std::vector<FrameSegment>& frameSegments(int cameraIdx);
    const std::vector<FrameSegment>& frameSegments(int cameraIdx) const;

    /**
     * XML file has the following structure:
     *
     * <root cameras_size=nCameras>
     *   <camera0 segments_size=nSegments>
     *     <segment0 frames_size>
     *       <frames frame0=? frame1=? ...?
     *       </frames>
     *     </segment0>
     *     ...
     *   </camera0>
     *   ...
     *   <frames size=nFrames>
     *     <frame0 image=? id=? camera=pose? odometry=odometry? gps_ins=pose? features2D_size=? features3D_size=?>
     *       <features2D features2D_0=F2D-? ...>
     *       </features2D>
     *       <features3D features3D_0=F3D-? ...>
     *       </features3D>
     *     </frame0>
     *     ...
     *   </frames>
     *   <poses size=nPoses>
     *     <pose0 q_x=? q_y=? q_z=? q_w=? t_x=? t_y=? t_z=?>
     *     </pose0>
     *     ...
     *   </poses>
     *   <odometry size=nOdometry>
     *     <odometry0 timestamp=? x=? y=? yaw=?>
     *     </odometry0>
     *     ...
     *   </odometry>
     *   <features2D=nFeatures2D>
     *     <F2D-0 kp_angle=? kp_class_id=? kp_octave=? kp_x=? kp_y=? kp_response=? kp_size=? index=? best_prev_match_idx=? best_next_match_idx=? feature3D=F3D-? frame=frame_?>
     *       <dtor type=? rows=? cols=?>
     *         <mat m-0-0=? ...>
     *         </mat
     *       </dtor>
     *       <prev_matches prev_matches_0=F2D_? ...>
     *       </prev_matches>
     *       <next_matches next_matches_0=F2D_? ...>
     *       </next_matches>
     *     </F2D-0>
     *     ...
     *   </features2D>
     *   <features3D=nFeatures3D>
     *     <F3D-0 x=? y=? z=? features2D_size=?>
     *       <features2D features2D_0=F2D-? ...>
     *       </features2D>
     *     </F3D-0>
     *     ...
     *   </features3D>
     * </root>
     *
     */

    bool readFromBinaryFile(const std::string& filename);
    void writeToBinaryFile(const std::string& filename) const;

    bool readFromXMLFile(const std::string& filename);
    void writeToXMLFile(const std::string& filename) const;

private:
    template<typename T>
    void readData(std::ifstream& ifs, T& data) const;

    template<typename T>
    void writeData(std::ofstream& ofs, T data) const;

    void XMLToFrames(pugi::xml_node& parent, unsigned int count,
                     FrameSegment& map,
                     std::vector<pugi::xml_node>& xmlMap,
                     const boost::filesystem::path& rootDir) const;
    void XMLToPoint2DFeatures(pugi::xml_node& parent, unsigned int count,
                              std::vector<Point2DFeaturePtr>& map,
                              std::vector<pugi::xml_node>& xmlMap) const;
    void XMLToPoint3DFeatures(pugi::xml_node& parent, unsigned int count,
                              std::vector<Point3DFeaturePtr>& map,
                              std::vector<pugi::xml_node>& xmlMap) const;
    void XMLToPoses(pugi::xml_node& parent, unsigned int count,
                    std::vector<PosePtr>& map) const;
    void XMLToOdometry(pugi::xml_node& parent, unsigned int count,
                       std::vector<OdometryPtr>& map) const;

    std::vector<std::vector<FrameSegment> > mFrameSegments;
};

}

#endif
