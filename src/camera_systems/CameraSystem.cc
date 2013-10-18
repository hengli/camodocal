#include <camodocal/camera_systems/CameraSystem.h>

#include <boost/filesystem.hpp>
#include <camodocal/camera_models/CameraFactory.h>
#include <camodocal/camera_models/CataCamera.h>
#include <camodocal/camera_models/EquidistantCamera.h>
#include <camodocal/camera_models/PinholeCamera.h>
#include <fstream>
#include <iomanip>

#include "../pugixml/pugixml.hpp"

namespace camodocal
{

CameraSystem::CameraSystem()
 : m_cameraCount(0)
 , m_referenceCameraIdx(-1)
{

}

CameraSystem::CameraSystem(int cameraCount)
 : m_cameraCount(cameraCount)
 , m_referenceCameraIdx(-1)
{
    m_cameras.resize(cameraCount);
    m_globalPoses.resize(cameraCount);
}

int
CameraSystem::cameraCount(void) const
{
    return m_cameraCount;
}

void
CameraSystem::reset(void)
{
    for (int i = 0; i < m_cameraCount; ++i)
    {
        m_cameras.at(i).reset();
        m_globalPoses.at(i).setIdentity();
    }
}

bool
CameraSystem::readPosesFromTextFile(const std::string& filename)
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
CameraSystem::writePosesToTextFile(const std::string& filename) const
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
CameraSystem::readFromDirectory(const std::string& directory)
{
    if (!boost::filesystem::is_directory(directory))
    {
        return false;
    }

    // read intrinsic data
    std::vector<std::string> intrinsicFilenames;

    boost::filesystem::directory_iterator it(directory);
    boost::filesystem::directory_iterator end_it;
    while (it != end_it)
    {
        if (boost::filesystem::is_regular_file(*it) && it->path().extension() == ".yaml")
        {
            intrinsicFilenames.push_back(it->path().string());
        }

        ++it;
    }

    if (intrinsicFilenames.empty())
    {
        return false;
    }

    std::sort(intrinsicFilenames.begin(), intrinsicFilenames.end());

    CameraSystem cameraSystem;

    for (size_t i = 0; i < intrinsicFilenames.size(); ++i)
    {
        CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(intrinsicFilenames.at(i));

        if (camera.get() == 0)
        {
            return false;
        }

        cameraSystem.m_cameras.push_back(camera);
    }

    cameraSystem.m_cameraCount = cameraSystem.m_cameras.size();
    cameraSystem.m_globalPoses.resize(cameraSystem.m_cameraCount);

    // read extrinsic data
    boost::filesystem::path extrinsicPath(directory);
    extrinsicPath /= "extrinsic.txt";

    cameraSystem.readPosesFromTextFile(extrinsicPath.string());

    *this = cameraSystem;

    return true;
}

bool
CameraSystem::writeToDirectory(const std::string& directory) const
{
    if (!boost::filesystem::is_directory(directory))
    {
        boost::filesystem::create_directory(directory);
    }

    // write extrinsic data
    boost::filesystem::path extrinsicPath(directory);
    extrinsicPath /= "extrinsic.txt";

    writePosesToTextFile(extrinsicPath.string());

    // write intrinsic data
    for (size_t i = 0; i < m_cameras.size(); ++i)
    {
        std::ostringstream oss;
        oss << "camera_intrinsic_" << i << ".yaml";

        boost::filesystem::path intrinsicPath(directory);
        intrinsicPath /= oss.str();

        m_cameras.at(i)->writeParametersToYamlFile(intrinsicPath.string());
    }

    return true;
}

bool
CameraSystem::readFromXmlFile(const std::string& filename)
{
    return true;
}

bool
CameraSystem::writeToXmlFile(const std::string& filename) const
{
    pugi::xml_document doc;

    // XML document declaration
    pugi::xml_node decl = doc.prepend_child(pugi::node_declaration);
    decl.append_attribute("version") = "1.0";
    decl.append_attribute("encoding") = "UTF-8";

    pugi::xml_node eSensors = doc.append_child("sensors");

    for (int i = 0; i < m_cameraCount; ++i)
    {
        pugi::xml_node eCamera = eSensors.append_child("camera");
        const CameraPtr& camera = m_cameras.at(i);

        std::ostringstream oss;
        oss << "mono_camera_" << i;

        eCamera.append_attribute("sensor-id") = oss.str().c_str();

        time_t t = time(NULL);
        tm* timePtr = localtime(&t);

        oss.clear(); oss.str("");
        oss << std::setfill('0') << std::setw(2)
            << timePtr->tm_mday << "." << timePtr->tm_mon + 1 << "." << timePtr->tm_year + 1900;

        eCamera.append_attribute("date") = oss.str().c_str();

        switch (camera->modelType())
        {
        case Camera::PINHOLE:
            eCamera.append_attribute("camera-model") = "pinhole";
            break;
        case Camera::KANNALA_BRANDT:
            eCamera.append_attribute("camera-model") = "equidistant";
            break;
        case Camera::MEI:
            eCamera.append_attribute("camera-model") = "christopher-mei";
            break;
        default:
            eCamera.append_attribute("camera-model") = "unknown";
        }

        pugi::xml_node eSupplier = eCamera.append_child("supplier");
        eSupplier.append_child(pugi::node_pcdata).set_value("CamOdoCal v2.0");

        pugi::xml_node eModel = eCamera.append_child("model");
        eModel.append_child(pugi::node_pcdata).set_value("TBD");

        pugi::xml_node eDescription = eCamera.append_child("description");
        eDescription.append_child(pugi::node_pcdata).set_value("Monocular Camera");

        pugi::xml_node eGeneralParameters = eCamera.append_child("general-parameters");

        pugi::xml_node eWidth = eGeneralParameters.append_child("width");
        oss.clear(); oss.str("");
        oss << camera->imageWidth();
        eWidth.append_child(pugi::node_pcdata).set_value(oss.str().c_str());
        oss.clear(); oss.str("");
        oss << camera->imageHeight();
        pugi::xml_node eHeight = eGeneralParameters.append_child("height");
        eHeight.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

        switch (camera->modelType())
        {
        case Camera::PINHOLE:
        {
            pugi::xml_node eIntrinsics = eCamera.append_child("intrinsic-parameters");

            PinholeCameraPtr pinholeCamera = boost::dynamic_pointer_cast<PinholeCamera>(camera);

            pugi::xml_node e_fx = eIntrinsics.append_child("fx");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << pinholeCamera->getParameters().fx();
            e_fx.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_fy = eIntrinsics.append_child("fy");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << pinholeCamera->getParameters().fy();
            e_fy.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_cx = eIntrinsics.append_child("cx");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << pinholeCamera->getParameters().cx();
            e_cx.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_cy = eIntrinsics.append_child("cy");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << pinholeCamera->getParameters().cy();
            e_cy.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_alpha = eIntrinsics.append_child("alpha");
            e_alpha.append_child(pugi::node_pcdata).set_value("0.0");

            pugi::xml_node e_kc = eIntrinsics.append_child("kc");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10)
                << pinholeCamera->getParameters().k1() << " "
                << pinholeCamera->getParameters().k2() << " "
                << pinholeCamera->getParameters().p1() << " "
                << pinholeCamera->getParameters().p2();
            e_kc.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            break;
        }
        case Camera::KANNALA_BRANDT:
        {
            pugi::xml_node eIntrinsics = eCamera.append_child("intrinsic-parameters");

            EquidistantCameraPtr equidCamera = boost::dynamic_pointer_cast<EquidistantCamera>(camera);

            pugi::xml_node e_fx = eIntrinsics.append_child("fx");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << equidCamera->getParameters().mu();
            e_fx.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_fy = eIntrinsics.append_child("fy");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << equidCamera->getParameters().mv();
            e_fy.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_cx = eIntrinsics.append_child("cx");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << equidCamera->getParameters().u0();
            e_cx.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_cy = eIntrinsics.append_child("cy");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << equidCamera->getParameters().v0();
            e_cy.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_alpha = eIntrinsics.append_child("alpha");
            e_alpha.append_child(pugi::node_pcdata).set_value("0.0");

            pugi::xml_node e_kc = eIntrinsics.append_child("kc");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10)
                << equidCamera->getParameters().k2() << " "
                << equidCamera->getParameters().k3() << " "
                << equidCamera->getParameters().k4() << " "
                << equidCamera->getParameters().k5();
            e_kc.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            break;
        }
        case Camera::MEI:
        {
            pugi::xml_node eIntrinsics = eCamera.append_child("intrinsic-parameters");

            CataCameraPtr cataCamera = boost::dynamic_pointer_cast<CataCamera>(camera);

            pugi::xml_node e_fx = eIntrinsics.append_child("fx");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << cataCamera->getParameters().gamma1();
            e_fx.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_fy = eIntrinsics.append_child("fy");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << cataCamera->getParameters().gamma2();
            e_fy.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_cx = eIntrinsics.append_child("cx");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << cataCamera->getParameters().u0();
            e_cx.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_cy = eIntrinsics.append_child("cy");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << cataCamera->getParameters().v0();
            e_cy.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_alpha = eIntrinsics.append_child("alpha");
            e_alpha.append_child(pugi::node_pcdata).set_value("0.0");

            pugi::xml_node e_xi = eIntrinsics.append_child("xi");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << cataCamera->getParameters().xi();
            e_xi.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_k1 = eIntrinsics.append_child("k1");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << cataCamera->getParameters().k1();
            e_k1.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_k2 = eIntrinsics.append_child("k2");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << cataCamera->getParameters().k2();
            e_k2.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_p1 = eIntrinsics.append_child("p1");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << cataCamera->getParameters().p1();
            e_p1.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            pugi::xml_node e_p2 = eIntrinsics.append_child("p2");
            oss.clear(); oss.str("");
            oss << std::fixed << std::setprecision(10) << cataCamera->getParameters().p2();
            e_p2.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

            break;
        }
        }

        Eigen::Matrix4d H = m_globalPoses.at(i);

        pugi::xml_node eExtrinsics = eCamera.append_child("extrinsic-parameters");

        pugi::xml_node eTranslation = eExtrinsics.append_child("translation");
        oss.clear(); oss.str("");
        oss << std::fixed << std::setprecision(10) << H(0,3) << " " << H(1,3) << " " << H(2,3);
        eTranslation.append_child(pugi::node_pcdata).set_value(oss.str().c_str());

        pugi::xml_node eRotation = eExtrinsics.append_child("rotation");
        oss.clear(); oss.str("");
        oss << std::fixed << std::setprecision(10)
            << H(0,0) << " " << H(0,1) << " " << H(0,2) << std::endl
            << "                   "
            << H(1,0) << " " << H(1,1) << " " << H(1,2) << std::endl
            << "                   "
            << H(2,0) << " " << H(2,1) << " " << H(2,2);
        eRotation.append_child(pugi::node_pcdata).set_value(oss.str().c_str());
    }

    doc.save_file(filename.c_str(), "   ", pugi::format_default, pugi::encoding_utf8);

    return true;
}

CameraPtr
CameraSystem::getCamera(int idx) const
{
    return m_cameras.at(idx);
}

void
CameraSystem::setCamera(int idx, CameraPtr& camera)
{
    m_cameras.at(idx) = camera;
}

bool
CameraSystem::setReferenceCamera(int idx)
{
    if (idx < 0 || idx >= m_cameraCount)
    {
        return false;
    }

    m_referenceCameraIdx = idx;

    return true;
}

Eigen::Matrix4d
CameraSystem::getGlobalCameraPose(int idx) const
{
    return m_globalPoses.at(idx);
}

Eigen::Matrix4d
CameraSystem::getLocalCameraPose(int idx) const
{
    return m_globalPoses.at(m_referenceCameraIdx).inverse() * m_globalPoses.at(idx);
}

void
CameraSystem::setGlobalCameraPose(int idx, const Eigen::Matrix4d& pose)
{
    m_globalPoses.at(idx) = pose;
}

void
CameraSystem::setLocalCameraPose(int idx, const Eigen::Matrix4d& pose)
{
    m_globalPoses.at(idx) = m_globalPoses.at(m_referenceCameraIdx) * pose;
}

int
CameraSystem::leftCameraIdx(int idx) const
{
    if (idx < 0 || idx >= m_cameraCount)
    {
        return -1;
    }

    return (idx + 1) % m_cameraCount;
}

int
CameraSystem::rightCameraIdx(int idx) const
{
    if (idx < 0 || idx >= m_cameraCount)
    {
        return -1;
    }

    return idx;
}

Eigen::Matrix4d
CameraSystem::relativeTransformBetweenCameraPair(int pairIdx) const
{
    return m_globalPoses.at(rightCameraIdx(pairIdx)).inverse() * m_globalPoses.at(leftCameraIdx(pairIdx));
}

double
CameraSystem::translationScaleBetweenCameraPair(int pairIdx) const
{
    Eigen::Matrix4d relativeTransform = relativeTransformBetweenCameraPair(pairIdx);

    return relativeTransform.block<3,1>(0, 3).norm();
}

CameraSystem&
CameraSystem::operator=(const CameraSystem& other)
{
    if (this != &other) // protect against invalid self-assignment
    {
        m_cameraCount = other.m_cameraCount;
        m_referenceCameraIdx = other.m_referenceCameraIdx;
        m_cameras = other.m_cameras;
        m_globalPoses = other.m_globalPoses;
    }

    return *this;
}

}
