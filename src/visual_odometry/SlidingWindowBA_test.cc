#include <Eigen/Eigen>
#include <gtest/gtest.h>
#include <iostream>
#include <opencv2/core/eigen.hpp>

#include "camodocal/camera_models/CataCamera.h"
#include "../gpl/gpl.h"
#include "SlidingWindowBA.h"

namespace camodocal
{

int nFrames = 30;
double cameraDist = 12.0;
double scenePointRange = 4.0;
int nScenePoints = 200;
double f = 300.0;
cv::Size imageSize(640, 480);

double rotationSigma = 0.05;
double translationSigma = 0.05;
double pointSigma = 0.05;

TEST(SlidingWindowBA, NoNoise1)
{
    // Simulate N scene points randomly distributed around (0,0)
    // and camera poses oriented towards (0,0)
    // and lying on a circle centered at (0,0)
    // All feature correspondences are assumed to be perfect.

    CataCamera::Parameters cameraParameters("", imageSize.width, imageSize.height,
                                            0.9, 0.0, 0.0, 0.0, 0.0, f, f,
                                            imageSize.width / 2.0, imageSize.height / 2.0);
    CataCameraPtr camera(new CataCamera(cameraParameters));

    SlidingWindowBA sba(camera);

    std::vector<Point3DFeaturePtr> scenePoints;
    for (int i = 0; i < nScenePoints; ++i)
    {
        Point3DFeaturePtr point(new Point3DFeature);
        point->point() = Eigen::Vector3d::Random() * scenePointRange;

        scenePoints.push_back(point);
    }

    std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>, Eigen::aligned_allocator<std::pair<Eigen::Quaterniond, Eigen::Vector3d> > > cameraPoses(nFrames);
    std::vector< std::vector<Point2DFeaturePtr> > features2D(nFrames);
    for (int i = 0; i < nFrames; ++i)
    {
        double theta = M_PI * 2.0 / nFrames * i;

        double z = cameraDist * cos(theta);
        double x = cameraDist * sin(theta);
        double y = 0.0;

        double orientation = normalizeTheta(M_PI + theta);

        // camera to world
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(orientation, Eigen::Vector3d::UnitY());
        Eigen::Vector3d t;
        t << x, y, z;

        // world to camera
        q = q.conjugate();
        t = - q.toRotationMatrix() * t;

        for (int j = 0; j < nScenePoints; ++j)
        {
            Eigen::Vector3d P = q.toRotationMatrix() * scenePoints.at(j)->point() + t;

            if (P(2) < 0.0)
            {
                std::cout << "# ERROR: Point is behind camera." << std::endl;
                exit(0);
            }

            Eigen::Vector2d p;
            camera->spaceToPlane(P, p);

            if (p(0) < 0.0 || p(1) < 0.0 || p(0) >= imageSize.width || p(1) >= imageSize.height)
            {
                std::cout << "# ERROR: Point is outside image." << std::endl;
                exit(0);
            }

            Point2DFeaturePtr feature2D(new Point2DFeature);
            feature2D->keypoint().pt = cv::Point2f(p(0), p(1));

            if (i != 0)
            {
                feature2D->prevMatches().push_back(features2D.at(i - 1).at(j));
                feature2D->bestPrevMatchId() = 0;
            }

            scenePoints.at(j)->features2D().push_back(feature2D);

            features2D.at(i).push_back(feature2D);
        }

        cameraPoses.at(i) = std::make_pair(q,t);
    }

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > posesEst;

    Eigen::Quaterniond q_prev;
    Eigen::Vector3d t_prev;
    for (int i = 0; i < nFrames; ++i)
    {
        Frame frameGround;

        Eigen::Quaterniond q = cameraPoses.at(i).first;
        Eigen::Vector3d t = cameraPoses.at(i).second;

        PosePtr cameraPose(new Pose);
        frameGround.cameraPose() = cameraPose;

        if (i == 0)
        {
            cameraPose->rotation() = q;
            cameraPose->translation() = t;
        }
        else
        {
            cameraPose->rotation() = q * q_prev.conjugate();
            cameraPose->translation() = - cameraPose->rotation().toRotationMatrix() * t_prev + t;
        }

        frameGround.features2D() = features2D.at(i);

        FramePtr frame(new Frame);
        frame->features2D() = frameGround.features2D();
        for (size_t j = 0; j < frameGround.features2D().size(); ++j)
        {
            frame->features2D().at(j)->frame() = frame;
        }

        sba.addFrame(frame);

        Eigen::Matrix4d H_est = frame->cameraPose()->toMatrix();
        posesEst.push_back(H_est);

        // update all windowed poses
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > window = sba.poses();
        for (size_t j = 0; j < window.size(); ++j)
        {
            posesEst.at(i + 1 + j - window.size()) = window.at(j);
        }

        q_prev = q;
        t_prev = t;
    }

    EXPECT_EQ(posesEst.size(), nFrames);

    Eigen::Matrix3d R = cameraPoses.at(2).first.toRotationMatrix() * cameraPoses.at(0).first.toRotationMatrix().inverse();
    Eigen::Vector3d t = -R * cameraPoses.at(0).second + cameraPoses.at(2).second;

    double scale = t.norm() / (posesEst.at(2) * posesEst.at(0).inverse()).block<3,1>(0,3).norm();

    for (size_t i = 0; i < posesEst.size(); ++i)
    {
        Eigen::Matrix4d H;
        H.setIdentity();

        H.block<3,3>(0,0) = cameraPoses.at(i).first.toRotationMatrix() * cameraPoses.at(0).first.toRotationMatrix().inverse();
        H.block<3,1>(0,3) = -H.block<3,3>(0,0) * cameraPoses.at(0).second + cameraPoses.at(i).second;

        H.block<3,1>(0,3) /= scale;

        Eigen::Matrix4d H_expected = posesEst.at(i);

        for (int j = 0; j < 4; ++j)
        {
            for (int k = 0; k < 4; ++k)
            {
                EXPECT_NEAR(H_expected(j,k), H(j,k), 0.00001) << "Elements differ at (" << j << "," << k << ")";
            }
        }
    }
}

TEST(SlidingWindowBA, Noise1)
{
    // Simulate N scene points randomly distributed around (0,0)
    // and camera poses oriented towards (0,0)
    // and lying on a circle centered at (0,0)
    // All feature correspondences are assumed to be perfect.

    CataCamera::Parameters cameraParameters("", imageSize.width, imageSize.height,
                                            0.9, 0.0, 0.0, 0.0, 0.0, f, f,
                                            imageSize.width / 2.0, imageSize.height / 2.0);
    CataCameraPtr camera(new CataCamera(cameraParameters));

    SlidingWindowBA sba(camera);

    std::vector<Point3DFeaturePtr> scenePoints;
    for (int i = 0; i < nScenePoints; ++i)
    {
        Point3DFeaturePtr point(new Point3DFeature);
        point->point() = Eigen::Vector3d::Random() * scenePointRange;

        scenePoints.push_back(point);
    }

    std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>, Eigen::aligned_allocator<std::pair<Eigen::Quaterniond, Eigen::Vector3d> > > cameraPoses(nFrames);
    std::vector< std::vector<Point2DFeaturePtr> > features2D(nFrames);
    for (int i = 0; i < nFrames; ++i)
    {
        double theta = M_PI * 2.0 / nFrames * i;

        double z = cameraDist * cos(theta);
        double x = cameraDist * sin(theta);
        double y = 0.0;

        double orientation = normalizeTheta(M_PI + theta);

        // camera to world
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(randomNormal(rotationSigma), Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(orientation + randomNormal(rotationSigma), Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(randomNormal(rotationSigma), Eigen::Vector3d::UnitX());
        Eigen::Vector3d t;
        t << x, y, z;

        // world to camera
        q = q.conjugate();
        t = - q.toRotationMatrix() * t;

        t(0) += randomNormal(translationSigma);
        t(1) += randomNormal(translationSigma);
        t(2) += randomNormal(translationSigma);

        for (int j = 0; j < nScenePoints; ++j)
        {
            Eigen::Vector3d P = scenePoints.at(j)->point();
            P(0) += randomNormal(pointSigma);
            P(1) += randomNormal(pointSigma);
            P(2) += randomNormal(pointSigma);

            P = q.toRotationMatrix() * P + t;

            if (P(2) < 0.0)
            {
                std::cout << "# ERROR: Point is behind camera." << std::endl;
                exit(0);
            }

            Eigen::Vector2d p;
            camera->spaceToPlane(P, p);

            if (p(0) < 0.0 || p(1) < 0.0 || p(0) >= imageSize.width || p(1) >= imageSize.height)
            {
                std::cout << "# ERROR: Point is outside image." << std::endl;
                exit(0);
            }

            Point2DFeaturePtr feature2D(new Point2DFeature);
            feature2D->keypoint().pt = cv::Point2f(p(0), p(1));

            if (i != 0)
            {
                feature2D->prevMatches().push_back(features2D.at(i - 1).at(j));
                feature2D->bestPrevMatchId() = 0;
            }

            scenePoints.at(j)->features2D().push_back(feature2D);

            features2D.at(i).push_back(feature2D);
        }

        cameraPoses.at(i) = std::make_pair(q,t);
    }

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > posesEst;

    Eigen::Quaterniond q_prev;
    Eigen::Vector3d t_prev;
    for (int i = 0; i < nFrames; ++i)
    {
        Frame frameGround;

        Eigen::Quaterniond q = cameraPoses.at(i).first;
        Eigen::Vector3d t = cameraPoses.at(i).second;

        PosePtr cameraPose(new Pose);
        frameGround.cameraPose() = cameraPose;

        if (i == 0)
        {
            cameraPose->rotation() = q;
            cameraPose->translation() = t;
        }
        else
        {
            cameraPose->rotation() = q * q_prev.conjugate();
            cameraPose->translation() = - cameraPose->rotation().toRotationMatrix() * t_prev + t;
        }

        frameGround.features2D() = features2D.at(i);

        FramePtr frame(new Frame);
        frame->features2D() = frameGround.features2D();
        for (size_t j = 0; j < frameGround.features2D().size(); ++j)
        {
            frame->features2D().at(j)->frame() = frame;
        }

        sba.addFrame(frame);

        Eigen::Matrix4d H_est = frame->cameraPose()->toMatrix();
        posesEst.push_back(H_est);

        // update all windowed poses
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > window = sba.poses();
        for (size_t j = 0; j < window.size(); ++j)
        {
            posesEst.at(i + 1 + j - window.size()) = window.at(j);
        }

        q_prev = q;
        t_prev = t;
    }

    EXPECT_EQ(posesEst.size(), nFrames);

    Eigen::Matrix3d R = cameraPoses.at(2).first.toRotationMatrix() * cameraPoses.at(0).first.toRotationMatrix().inverse();
    Eigen::Vector3d t = -R * cameraPoses.at(0).second + cameraPoses.at(2).second;

    double scale = t.norm() / (posesEst.at(2) * posesEst.at(0).inverse()).block<3,1>(0,3).norm();

    for (size_t i = 0; i < posesEst.size(); ++i)
    {
        Eigen::Matrix4d H;
        H.setIdentity();

        H.block<3,3>(0,0) = cameraPoses.at(i).first.toRotationMatrix() * cameraPoses.at(0).first.toRotationMatrix().inverse();
        H.block<3,1>(0,3) = -H.block<3,3>(0,0) * cameraPoses.at(0).second + cameraPoses.at(i).second;

        H.block<3,1>(0,3) /= scale;

        Eigen::Matrix4d H_expected = posesEst.at(i);

        for (int j = 0; j < 4; ++j)
        {
            for (int k = 0; k < 4; ++k)
            {
                EXPECT_NEAR(H_expected(j,k), H(j,k), 0.025) << "Elements differ at (" << j << "," << k << ")";
            }
        }
    }
}

TEST(SlidingWindowBA, NoNoise2)
{
    // Simulate N scene points randomly distributed around (0,0)
    // and camera poses oriented towards (0,0)
    // and lying on a circle centered at (0,0)
    // All feature correspondences are assumed to be perfect.
    // Only some features may have correspondences.

    CataCamera::Parameters cameraParameters("", imageSize.width, imageSize.height,
                                            0.9, 0.0, 0.0, 0.0, 0.0, f, f,
                                            imageSize.width / 2.0, imageSize.height / 2.0);
    CataCameraPtr camera(new CataCamera(cameraParameters));

    SlidingWindowBA sba(camera);

    std::vector<Point3DFeaturePtr> scenePoints;
    for (int i = 0; i < nScenePoints; ++i)
    {
        Point3DFeaturePtr point(new Point3DFeature);
        point->point() = Eigen::Vector3d::Random() * scenePointRange;

        scenePoints.push_back(point);
    }

    std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>, Eigen::aligned_allocator<std::pair<Eigen::Quaterniond, Eigen::Vector3d> > > cameraPoses(nFrames);
    std::vector< std::vector<Point2DFeaturePtr> > features2D(nFrames);
    for (int i = 0; i < nFrames; ++i)
    {
        double theta = M_PI * 2.0 / nFrames * i;

        double z = cameraDist * cos(theta);
        double x = cameraDist * sin(theta);
        double y = 0.0;

        double orientation = normalizeTheta(M_PI + theta);

        // camera to world
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(orientation, Eigen::Vector3d::UnitY());
        Eigen::Vector3d t;
        t << x, y, z;

        // world to camera
        q = q.conjugate();
        t = - q.toRotationMatrix() * t;

        for (int j = 0; j < nScenePoints; ++j)
        {
            Eigen::Vector3d P = q.toRotationMatrix() * scenePoints.at(j)->point() + t;

            if (P(2) < 0.0)
            {
                std::cout << "# ERROR: Point is behind camera." << std::endl;
                exit(0);
            }

            Eigen::Vector2d p;
            camera->spaceToPlane(P, p);

            if (p(0) < 0.0 || p(1) < 0.0 || p(0) >= imageSize.width || p(1) >= imageSize.height)
            {
                std::cout << "# ERROR: Point is outside image." << std::endl;
                exit(0);
            }

            Point2DFeaturePtr feature2D(new Point2DFeature);
            feature2D->keypoint().pt = cv::Point2f(p(0), p(1));

            bool hasCorrespondence = (random(-1.0, 1.0) > 0.0);

            if (i != 0 && hasCorrespondence)
            {
                feature2D->prevMatches().push_back(features2D.at(i - 1).at(j));
                feature2D->bestPrevMatchId() = 0;

                scenePoints.at(j)->features2D().push_back(feature2D);
            }
            else
            {
                feature2D->bestPrevMatchId() = -1;
            }

            features2D.at(i).push_back(feature2D);
        }

        cameraPoses.at(i) = std::make_pair(q,t);
    }

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > posesEst;

    Eigen::Quaterniond q_prev;
    Eigen::Vector3d t_prev;
    for (int i = 0; i < nFrames; ++i)
    {
        Frame frameGround;

        Eigen::Quaterniond q = cameraPoses.at(i).first;
        Eigen::Vector3d t = cameraPoses.at(i).second;

        PosePtr cameraPose(new Pose);
        frameGround.cameraPose() = cameraPose;

        if (i == 0)
        {
            cameraPose->rotation() = q;
            cameraPose->translation() = t;
        }
        else
        {
            cameraPose->rotation() = q * q_prev.conjugate();
            cameraPose->translation() = - cameraPose->rotation().toRotationMatrix() * t_prev + t;
        }

        frameGround.features2D() = features2D.at(i);

        FramePtr frame(new Frame);
        frame->features2D() = frameGround.features2D();
        for (size_t j = 0; j < frameGround.features2D().size(); ++j)
        {
            frame->features2D().at(j)->frame() = frame;
        }

        sba.addFrame(frame);

        Eigen::Matrix4d H_est = frame->cameraPose()->toMatrix();
        posesEst.push_back(H_est);

        // update all windowed poses
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > window = sba.poses();
        for (size_t j = 0; j < window.size(); ++j)
        {
            posesEst.at(i + 1 + j - window.size()) = window.at(j);
        }

        q_prev = q;
        t_prev = t;
    }

    EXPECT_EQ(posesEst.size(), nFrames);

    Eigen::Matrix3d R = cameraPoses.at(2).first.toRotationMatrix() * cameraPoses.at(0).first.toRotationMatrix().inverse();
    Eigen::Vector3d t = -R * cameraPoses.at(0).second + cameraPoses.at(2).second;

    double scale = t.norm() / (posesEst.at(2) * posesEst.at(0).inverse()).block<3,1>(0,3).norm();

    for (size_t i = 0; i < posesEst.size(); ++i)
    {
        Eigen::Matrix4d H;
        H.setIdentity();

        H.block<3,3>(0,0) = cameraPoses.at(i).first.toRotationMatrix() * cameraPoses.at(0).first.toRotationMatrix().inverse();
        H.block<3,1>(0,3) = -H.block<3,3>(0,0) * cameraPoses.at(0).second + cameraPoses.at(i).second;

        H.block<3,1>(0,3) /= scale;

        Eigen::Matrix4d H_expected = posesEst.at(i);

        for (int j = 0; j < 4; ++j)
        {
            for (int k = 0; k < 4; ++k)
            {
                EXPECT_NEAR(H_expected(j,k), H(j,k), 0.01) << "Elements differ at (" << j << "," << k << ")";
            }
        }
    }
}

}
