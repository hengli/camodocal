#include "camodocal/camera_systems/CameraRigExtrinsics.h"

#include <fstream>
#include <iomanip>

namespace camodocal
{

CameraRigExtrinsics::CameraRigExtrinsics(int cameraCount)
 : m_cameraCount(cameraCount)
 , m_referenceCameraIdx(-1)
{
    m_globalPoses.resize(cameraCount);
}

int
CameraRigExtrinsics::cameraCount(void) const
{
    return m_cameraCount;
}

void
CameraRigExtrinsics::reset(void)
{
    for (int i = 0; i < m_cameraCount; ++i)
    {
        m_globalPoses.at(i).setIdentity();
    }
}

bool
CameraRigExtrinsics::readFromFile(const std::string& filename)
{
    std::ifstream ifs(filename.c_str());

    if (!ifs.is_open())
    {
        return false;
    }

    for (int i = 0; i < m_cameraCount; ++i)
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

    for (int i = 0; i < m_cameraCount; ++i)
    {
        const Eigen::Matrix4d& globalPose = m_globalPoses.at(i);

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
    if (idx < 0 || idx >= m_cameraCount)
    {
        return false;
    }

    m_referenceCameraIdx = idx;

    return true;
}

Eigen::Matrix4d
CameraRigExtrinsics::getGlobalCameraPose(int idx) const
{
    return m_globalPoses.at(idx);
}

Eigen::Matrix4d
CameraRigExtrinsics::getLocalCameraPose(int idx) const
{
    return m_globalPoses.at(m_referenceCameraIdx).inverse() * m_globalPoses.at(idx);
}

void
CameraRigExtrinsics::setGlobalCameraPose(int idx, const Eigen::Matrix4d& pose)
{
    m_globalPoses.at(idx) = pose;
}

void
CameraRigExtrinsics::setLocalCameraPose(int idx, const Eigen::Matrix4d& pose)
{
    m_globalPoses.at(idx) = m_globalPoses.at(m_referenceCameraIdx) * pose;
}

int
CameraRigExtrinsics::leftCameraIdx(int idx) const
{
    if (idx < 0 || idx >= m_cameraCount)
    {
        return -1;
    }

    return (idx + 1) % m_cameraCount;
}

int
CameraRigExtrinsics::rightCameraIdx(int idx) const
{
    if (idx < 0 || idx >= m_cameraCount)
    {
        return -1;
    }

    return idx;
}

Eigen::Matrix4d
CameraRigExtrinsics::relativeTransformBetweenCameraPair(int pairIdx) const
{
    return m_globalPoses.at(rightCameraIdx(pairIdx)).inverse() * m_globalPoses.at(leftCameraIdx(pairIdx));
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
        m_cameraCount = other.m_cameraCount;
        m_referenceCameraIdx = other.m_referenceCameraIdx;
        m_globalPoses = other.m_globalPoses;
    }

    return *this;
}

}
