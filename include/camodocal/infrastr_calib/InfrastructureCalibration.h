#ifndef INFRASTRUCTURECALIBRATION_H
#define INFRASTRUCTURECALIBRATION_H

#include <boost/thread.hpp>

#include "camodocal/camera_systems/CameraSystem.h"
#include "camodocal/sparse_graph/SparseGraph.h"

#ifdef VCHARGE_VIZ
#include "../../../../../visualization/overlay/GLOverlayExtended.h"
#endif

namespace camodocal
{

// forward declaration
class LocationRecognition;

class InfrastructureCalibration
{
public:
    InfrastructureCalibration(std::vector<CameraPtr>& cameras,
                              bool verbose = false);

    bool loadMap(const std::string& mapDirectory);

    void addFrameSet(const std::vector<cv::Mat>& images,
                     uint64_t timestamp, bool preprocess);
    void addOdometry(double x, double y, double yaw, uint64_t timestamp);

    void reset(void);
    void run(void);

    void loadFrameSets(const std::string& filename);
    void saveFrameSets(const std::string& filename) const;

    const CameraSystem& cameraSystem(void) const;

private:
    void estimateCameraPose(const cv::Mat& image, uint64_t timestamp,
                            FramePtr& frame, bool preprocess = false);
    void optimize(bool optimizeScenePoints);

    cv::Mat buildDescriptorMat(const std::vector<Point2DFeaturePtr>& features,
                               std::vector<size_t>& indices,
                               bool hasScenePoint) const;
    std::vector<cv::DMatch> matchFeatures(const std::vector<Point2DFeaturePtr>& queryFeatures,
                                          const std::vector<Point2DFeaturePtr>& trainFeatures) const;

    void solveP3PRansac(const FrameConstPtr& frame1,
                        const FrameConstPtr& frame2,
                        const std::vector<cv::DMatch>& matches,
                        Eigen::Matrix4d& H,
                        std::vector<cv::DMatch>& inliers) const;

    double reprojectionError(const CameraConstPtr& camera,
                             const Eigen::Vector3d& P,
                             const Eigen::Quaterniond& cam_ref_q,
                             const Eigen::Vector3d& cam_ref_t,
                             const Eigen::Vector3d& ref_p,
                             const Eigen::Vector3d& ref_att,
                             const Eigen::Vector2d& observed_p) const;

    void frameReprojectionError(const FramePtr& frame,
                                const CameraConstPtr& camera,
                                const Pose& T_cam_ref,
                                double& minError, double& maxError, double& avgError,
                                size_t& featureCount) const;

    void frameReprojectionError(const FramePtr& frame,
                                const CameraConstPtr& camera,
                                double& minError, double& maxError, double& avgError,
                                size_t& featureCount) const;

    void reprojectionError(double& minError, double& maxError,
                           double& avgError, size_t& featureCount) const;

    // Compute the quaternion average using the Markley SVD method
    template <typename FloatT>
    Eigen::Quaternion<FloatT> quaternionAvg(const std::vector<Eigen::Quaternion<FloatT> >& points) const;

#ifdef VCHARGE_VIZ
    enum MapType
    {
        REFERENCE_MAP,
        REFERENCE_POINTS
    };

    void visualizeMap(const std::string& overlayName, MapType type) const;
    void visualizeCameraPose(const FrameConstPtr& frame,
                             bool showScenePoints);
    void visualizeCameraPoses(bool showScenePoints);
    void visualizeExtrinsics(void) const;
    void visualizeOdometry(void) const;
#endif

    typedef struct
    {
        uint64_t timestamp = 0;
        std::vector<FramePtr> frames;
    } FrameSet;

    // inputs
    std::vector<CameraPtr> m_cameras;
    SparseGraph m_refGraph;

    // working data
    boost::shared_ptr<LocationRecognition> m_locrec;
    boost::mutex m_feature3DMapMutex;
    boost::unordered_map<Point3DFeature*, Point3DFeaturePtr> m_feature3DMap;
    std::vector<FrameSet> m_framesets;
    double m_x_last;
    double m_y_last;
    double m_distance;
    bool m_verbose;

#ifdef VCHARGE_VIZ
    vcharge::GLOverlayExtended m_overlay;
#endif

    // output
    CameraSystem m_cameraSystem;

    // parameters
    const float k_maxDistanceRatio;
    const int k_minCorrespondences2D3D;
    const double k_minKeyFrameDistance;
    const int k_nearestImageMatches;
    const double k_reprojErrorThresh;
};

template <typename FloatT>
Eigen::Quaternion<FloatT>
InfrastructureCalibration::quaternionAvg(const std::vector<Eigen::Quaternion<FloatT> >& points) const
{
    using namespace Eigen;

    Matrix<FloatT, 3, 3> sum;
    sum.setZero();
    for (int i = 0, end = points.size(); i != end; ++i)
    {
        sum += points[i].toRotationMatrix();
    }

    JacobiSVD<Matrix<FloatT, 3, 3> > svd(sum, ComputeFullU | ComputeFullV);

    Matrix<FloatT, 3, 3> result = svd.matrixU()
        * (Matrix<FloatT, 3, 1>() << 1, 1, svd.matrixU().determinant()*svd.matrixV().determinant()).finished().asDiagonal()
        * svd.matrixV().transpose();
    return Quaternion<FloatT>(result);
}

}

#endif
