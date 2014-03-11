#include <camodocal/sparse_graph/SparseGraphUtils.h>

namespace camodocal
{

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
