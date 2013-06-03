#ifndef ONE_POINT_HPP
#define ONE_POINT_HPP

#include <opencv2/opencv.hpp>

void findPose_1pt(cv::InputArray points1, cv::InputArray points2, 
              double focal, cv::Point2d pp, 
              cv::OutputArray rvec, cv::OutputArray tvec, 
              int method, double prob, double threshold, cv::OutputArray _mask); 

#endif
