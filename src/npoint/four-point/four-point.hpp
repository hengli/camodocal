#ifndef FOUR_POINT_HPP
#define FOUR_POINT_HPP

#include <opencv2/opencv.hpp>

void findPose_4pt(cv::InputArray points1, cv::InputArray points2, 
              double angle, double focal, cv::Point2d pp, 
              cv::OutputArrayOfArrays rvecs, cv::OutputArrayOfArrays tvecs, 
              int method, double prob, double threshold, cv::OutputArray _mask); 

void four_point(cv::InputArray points1, cv::InputArray points2, 
                double angle, double focal, cv::Point2d pp, 
                cv::OutputArrayOfArrays rvecs, cv::OutputArrayOfArrays tvecs); 

void four_point_get_ab(double k1, double k2, double k3, 
                       double x1[4], double y1[4], double x2[4], double y2[4], 
                       double a[55], double b[55]); 

void four_point_get_M(double k1, double k2, double k3, 
                      double x1[4], double y1[4], double x2[4], double y2[4], 
                      double rx, double ry, double rz, double M[12]); 

#endif
