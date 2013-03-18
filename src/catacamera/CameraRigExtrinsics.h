#ifndef CAMERARIGEXTRINSICS_H
#define CAMERARIGEXTRINSICS_H

#include <Eigen/Eigen>
#include <vector>

namespace camodocal
{

class CameraRigExtrinsics
{
public:
    // cameras are assumed to be indexed in an anti-clockwise direction
    CameraRigExtrinsics(int cameraCount);

    bool readFromFile(const std::string& filename);
    bool writeToFile(const std::string& filename) const;

    bool setReferenceCamera(int idx);

    // global camera pose is the transform from camera frame to odometry frame
    Eigen::Matrix4d getGlobalCameraPose(int idx) const;

    // local camera pose is the transform from camera frame to reference camera frame
    Eigen::Matrix4d getLocalCameraPose(int idx) const;

    void setGlobalCameraPose(int idx, const Eigen::Matrix4d& pose);
    void setLocalCameraPose(int idx, const Eigen::Matrix4d& pose);

    // pair index corresponds to the index of the right camera in the camera pair
    int leftCameraIdx(int cameraPairIdx) const;
    int rightCameraIdx(int cameraPairIdx) const;

    // relative transform is the transform from the left camera frame to the right camera frame
    Eigen::Matrix4d relativeTransformBetweenCameraPair(int pairIdx) const;
    double translationScaleBetweenCameraPair(int pairIdx) const;

    CameraRigExtrinsics& operator=(const CameraRigExtrinsics& other);

private:
    int mCameraCount;
    int mReferenceCameraIdx;

    std::vector< Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > mGlobalPoses;
};

}

#endif
