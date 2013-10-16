#include <camodocal/sparse_graph/SparseGraphUtils.h>

namespace camodocal
{

cv::Mat
buildDescriptorMat(const std::vector<Point2DFeaturePtr>& features,
                   std::vector<size_t>& indices)
{
    for (size_t i = 0; i < features.size(); ++i)
    {
         if (features.at(i)->feature3D().get() != 0)
         {
             indices.push_back(i);
         }
    }

    cv::Mat dtor(indices.size(), features.at(0)->descriptor().cols, features.at(0)->descriptor().type());

    for (size_t i = 0; i < indices.size(); ++i)
    {
         features.at(indices.at(i))->descriptor().copyTo(dtor.row(i));
    }

    return dtor;
}

std::vector<cv::DMatch>
matchFeatures(const std::vector<Point2DFeaturePtr>& features1,
              const std::vector<Point2DFeaturePtr>& features2,
              float maxDistanceRatio)
{
    cv::BFMatcher descriptorMatcher(cv::NORM_L2, false);

    std::vector<size_t> indices1, indices2;
    cv::Mat dtor1 = buildDescriptorMat(features1, indices1);
    cv::Mat dtor2 = buildDescriptorMat(features2, indices2);

    std::vector<std::vector<cv::DMatch> > candidateFwdMatches;
    descriptorMatcher.knnMatch(dtor1, dtor2, candidateFwdMatches, 2);

    std::vector<std::vector<cv::DMatch> > candidateRevMatches;
    descriptorMatcher.knnMatch(dtor2, dtor1, candidateRevMatches, 2);

    std::vector<std::vector<cv::DMatch> > fwdMatches(candidateFwdMatches.size());
    for (size_t i = 0; i < candidateFwdMatches.size(); ++i)
    {
        std::vector<cv::DMatch>& match = candidateFwdMatches.at(i);

        if (match.size() < 2)
        {
            continue;
        }

        float distanceRatio = match.at(0).distance / match.at(1).distance;

        if (distanceRatio < maxDistanceRatio)
        {
            fwdMatches.at(i).push_back(match.at(0));
        }
    }

    std::vector<std::vector<cv::DMatch> > revMatches(candidateRevMatches.size());
    for (size_t i = 0; i < candidateRevMatches.size(); ++i)
    {
        std::vector<cv::DMatch>& match = candidateRevMatches.at(i);

        if (match.size() < 2)
        {
            continue;
        }

        float distanceRatio = match.at(0).distance / match.at(1).distance;

        if (distanceRatio < maxDistanceRatio)
        {
            revMatches.at(i).push_back(match.at(0));
        }
    }

    // cross-check
    std::vector<cv::DMatch> matches;
    for (size_t i = 0; i < fwdMatches.size(); ++i)
    {
        if (fwdMatches.at(i).empty())
        {
            continue;
        }

        cv::DMatch& fwdMatch = fwdMatches.at(i).at(0);

        if (revMatches.at(fwdMatch.trainIdx).empty())
        {
            continue;
        }

        cv::DMatch& revMatch = revMatches.at(fwdMatch.trainIdx).at(0);

        if (fwdMatch.queryIdx == revMatch.trainIdx &&
            fwdMatch.trainIdx == revMatch.queryIdx)
        {
            cv::DMatch match;
            match.queryIdx = indices1.at(fwdMatch.queryIdx);
            match.trainIdx = indices2.at(revMatch.queryIdx);

            matches.push_back(match);
        }
    }

    return matches;
}

void
rectifyImagePoint(const CameraConstPtr& camera,
                  const cv::Point2f& src, cv::Point2f& dst)
{
    Eigen::Vector3d P;

    camera->liftProjective(Eigen::Vector2d(src.x, src.y), P);

    P /= P(2);

    dst.x = P(0);
    dst.y = P(1);
}

void
rectifyImagePoint(const CameraConstPtr& camera,
                  const Eigen::Vector2d& src, Eigen::Vector2d& dst)
{
    Eigen::Vector3d P;

    camera->liftProjective(src, P);

    P /= P(2);

    dst = P.block<2,1>(0,0);
}

void
rectifyImagePoints(const CameraConstPtr& camera,
                   const std::vector<cv::Point2f>& src,
                   std::vector<cv::Point2f>& dst)
{
    dst.resize(src.size());

    for (size_t i = 0; i < src.size(); ++i)
    {
        const cv::Point2f& p = src.at(i);

        Eigen::Vector3d P;
        camera->liftProjective(Eigen::Vector2d(p.x, p.y), P);

        P /= P(2);

        dst.at(i) = cv::Point2f(P(0), P(1));
    }
}

}
