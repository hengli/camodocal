#ifndef CAMERARIGBA_H
#define CAMERARIGBA_H

#include <boost/thread/mutex.hpp>
#include <boost/tuple/tuple.hpp>
#include <Eigen/Dense>

#include <camodocal/calib/CameraCalibration.h>
#include <camodocal/camera_systems/CameraSystem.h>
#include <camodocal/sparse_graph/SparseGraph.h>

namespace camodocal
{

// forward declaration
class LocationRecognition;

class CameraRigBA
{
public:
    enum
    {
        CAMERA,
        ODOMETRY
    };

    enum
    {
        PRUNE_BEHIND_CAMERA = 0x1,
        PRUNE_FARAWAY = 0x2,
        PRUNE_HIGH_REPROJ_ERR = 0x4
    };

    CameraRigBA(CameraSystem& cameraSystem,
                SparseGraph& graph,
                double windowDistance = 3.0);

    void run(int beginStage = 1,
             bool optimizeIntrinsics = true,
             bool saveWorkingData = false, std::string dataDir = "data");

    void setVerbose(bool verbose);

    void frameReprojectionError(const FramePtr& frame,
                                const CameraConstPtr& camera,
                                const Pose& T_cam_odo,
                                double& minError, double& maxError, double& avgError,
                                size_t& featureCount,
                                int type) const;
    void reprojectionError(double& minError, double& maxError,
                           double& avgError, size_t& featureCount,
                           int type) const;

private:
    double reprojectionError(const CameraConstPtr& camera,
                             const Eigen::Vector3d& P,
                             const Eigen::Quaterniond& cam_odo_q,
                             const Eigen::Vector3d& cam_odo_t,
                             const Eigen::Vector3d& odo_p,
                             const Eigen::Vector3d& odo_att,
                             const Eigen::Vector2d& observed_p) const;

    void triangulateFeatureCorrespondences(void);

    void triangulateFeatures(FramePtr& frame1, FramePtr& frame2,
                             const CameraConstPtr& camera,
                             const Pose& T_cam_odo);

    void find2D2DCorrespondences(const std::vector<Point2DFeaturePtr>& features,
                                 int nViews,
                                 std::vector<std::vector<Point2DFeaturePtr> >& correspondences) const;

    typedef std::pair<Point2DFeaturePtr, Point2DFeaturePtr> Correspondence2D2D;
    typedef boost::tuple<FramePtr, FramePtr, Point2DFeaturePtr, Point3DFeaturePtr> Correspondence2D3D;
    typedef boost::tuple<FramePtr, FramePtr, Point3DFeaturePtr, Point3DFeaturePtr> Correspondence3D3D;

    void findLocalInterMap2D2DCorrespondences(std::vector<Correspondence2D2D>& correspondences2D2D,
                                              double reprojErrorThresh = 2.0);
    void matchFrameToWindow(FramePtr& frame1,
                            std::vector<FramePtr>& window,
                            std::vector<Correspondence2D2D>* correspondences2D2D,
                            double reprojErrorThresh = 2.0);
    void matchFrameToFrame(FramePtr& frame1, FramePtr& frame2,
                           std::vector<Correspondence2D2D>* corr2D2D,
                           double reprojErrorThresh = 2.0);

    void prune(int flags = PRUNE_BEHIND_CAMERA, int poseType = ODOMETRY);

    void optimize(int flags, bool optimizeZ = true, int nIterations = 500);

    void reweightScenePoints(void);

    bool estimateRigOdometryTransform(Eigen::Matrix4d& H_rig_odo) const;

    bool estimateAbsoluteGroundHeight(double& zGround) const;

    void applyTransform(const Eigen::Matrix4d& H,
                        bool applyToExtrinsics,
                        bool applyToSystemPoses,
                        bool applyToScenePoints);

    void writePosesToTextFile(const std::string& filename) const;

#ifdef VCHARGE_VIZ
    void visualize(const std::string& overlayPrefix, int type);

    void visualizeExtrinsics(const std::string& overlayName);

    void visualizeFrameFrameCorrespondences(const std::string& overlayName,
                                            const std::vector<std::pair<FramePtr, FramePtr> >& correspondencesFrameFrame) const;

    void visualizeSystemPoses(const std::string& overlayName);

    void visualize2D3DCorrespondences(const std::string& overlayName,
                                      const std::vector<Correspondence2D3D>& correspondences) const;

    void visualize3D3DCorrespondences(const std::string& overlayName,
                                      const std::vector<Correspondence3D3D>& correspondences) const;

    void visualize3D3DCorrespondences(const std::string& overlayName,
                                      const std::vector<Correspondence2D2D>& correspondences) const;

    void visualizeGroundPoints(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points) const;
#endif

    bool validateGraph(void) const;

    void dumpPointCloud(const std::string& dir, bool dumpPoses = false);

    typedef struct
    {
        double mean;
        double variance;
        std::vector<size_t> pointIndices;
    } ZPlaneModel;

    CameraSystem& m_cameraSystem;
    std::vector<boost::shared_ptr<CameraCalibration> > m_cameraCalibrations;
    SparseGraph m_graph;

    const size_t k_windowDistance;
    const float k_maxDistanceRatio;
    const double k_maxPoint3DDistance;
    const double k_maxReprojErr;
    const size_t k_minLoopCorrespondences2D3D;
    const size_t k_minWindowCorrespondences2D2D;
    const int k_nearestImageMatches;
    const double k_nominalFocalLength;

    bool m_verbose;
};

}

#endif
