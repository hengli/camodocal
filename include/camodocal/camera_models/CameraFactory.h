#ifndef CAMERAFACTORY_H
#define CAMERAFACTORY_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "camodocal/camera_models/Camera.h"

namespace camodocal
{

class CameraFactory
{
public:
    CameraFactory();

    static boost::shared_ptr<CameraFactory> instance(void);

    CameraPtr generateCamera(Camera::ModelType modelType,
                             const std::string& cameraName,
                             cv::Size imageSize) const;

    CameraPtr generateCamera(const std::string& calibFilename);

private:
    static boost::shared_ptr<CameraFactory> m_instance;
};

}

#endif
