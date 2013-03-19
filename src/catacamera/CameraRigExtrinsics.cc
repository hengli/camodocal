#include "camodocal/CameraRigExtrinsics.h"

#include <fstream>
#include <iomanip>

namespace camodocal
{

CameraRigExtrinsics::CameraRigExtrinsics(int cameraCount)
 : mCameraCount(cameraCount)
 , mReferenceCameraIdx(0)
{
    mGlobalPoses.resize(cameraCount);
}

bool
CameraRigExtrinsics::readFromFile(const std::string& filename)
{
    std::ifstream ifs(filename.c_str());

    if (!ifs.is_open())
    {
        return false;
    }

    for (int i = 0; i < mCameraCount; ++i)
    {
        double H_data[12];
        for (int j = 0; j < 12; ++j)
        {
            ifs >> H_data[j];
        }

        Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
        for (int j = 0; j < 3; ++j)
        {
            for (int k = 0; k < 4; ++k)
            {
                H(j,k) = H_data[j * 4 + k];
            }
        }

        // compute pose with respect to EDN frame
        setGlobalCameraPose(i, H);
    }

    ifs.close();

    return true;
}

bool
CameraRigExtrinsics::writeToFile(const std::string& filename) const
{
    std::ofstream ofs(filename.c_str());

    if (!ofs.is_open())
    {
        return false;
    }

    ofs << std::fixed << std::setprecision(10);

    for (int i = 0; i < mCameraCount; ++i)
    {
        const Eigen::Matrix4d& globalPose = mGlobalPoses.at(i);

        for (int j = 0; j < 3; ++j)
        {
            for (int k = 0; k < 4; ++k)
            {
                ofs << globalPose(j,k);

                if (k < 3)
                {
                    ofs << " ";
                }
            }
            ofs << std::endl;
        }
        ofs << std::endl;
    }

    ofs.close();

    return true;
}

bool
CameraRigExtrinsics::setReferenceCamera(int idx)
{
    if (idx < 0 || idx >= mCameraCount)
    {
        return false;
    }

    mReferenceCameraIdx = idx;

    return true;
}

Eigen::Matrix4d
CameraRigExtrinsics::getGlobalCameraPose(int idx) const
{
    return mGlobalPoses.at(idx);
}

Eigen::Matrix4d
CameraRigExtrinsics::getLocalCameraPose(int idx) const
{
    return mGlobalPoses.at(mReferenceCameraIdx).inverse() * mGlobalPoses.at(idx);
}

void
CameraRigExtrinsics::setGlobalCameraPose(int idx, const Eigen::Matrix4d& pose)
{
    mGlobalPoses.at(idx) = pose;
}

void
CameraRigExtrinsics::setLocalCameraPose(int idx, const Eigen::Matrix4d& pose)
{
    mGlobalPoses.at(idx) = mGlobalPoses.at(mReferenceCameraIdx) * pose;
}

int
CameraRigExtrinsics::leftCameraIdx(int idx) const
{
    if (idx < 0 || idx >= mCameraCount)
    {
        return -1;
    }

    return (idx + 1) % mCameraCount;
}

int
CameraRigExtrinsics::rightCameraIdx(int idx) const
{
    if (idx < 0 || idx >= mCameraCount)
    {
        return -1;
    }

    return idx;
}

Eigen::Matrix4d
CameraRigExtrinsics::relativeTransformBetweenCameraPair(int pairIdx) const
{
    return mGlobalPoses.at(rightCameraIdx(pairIdx)).inverse() * mGlobalPoses.at(leftCameraIdx(pairIdx));
}

double
CameraRigExtrinsics::translationScaleBetweenCameraPair(int pairIdx) const
{
    Eigen::Matrix4d relativeTransform = relativeTransformBetweenCameraPair(pairIdx);

    return relativeTransform.block<3,1>(0, 3).norm();
}

CameraRigExtrinsics&
CameraRigExtrinsics::operator=(const CameraRigExtrinsics& other)
{
    if (this != &other) // protect against invalid self-assignment
    {
        mCameraCount = other.mCameraCount;
        mReferenceCameraIdx = other.mReferenceCameraIdx;
        mGlobalPoses = other.mGlobalPoses;
    }
}

}
