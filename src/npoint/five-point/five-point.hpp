#ifndef FIVE_POINT_HPP
#define FIVE_POINT_HPP
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv; 

Mat findEssentialMat( InputArray points1, InputArray points2, double focal = 1.0, Point2d pp = Point2d(0, 0), 
					int method = CV_FM_RANSAC, 
					double prob = 0.999, double threshold = 1, int maxIters = 1000, OutputArray mask = noArray() ); 

void decomposeEssentialMat( const Mat & E, Mat & R1, Mat & R2, Mat & t ); 

int recoverPose( const Mat & E, InputArray points1, InputArray points2, Mat & R, Mat & t, 
					double focal = 1.0, Point2d pp = Point2d(0, 0), 
					InputOutputArray mask = noArray()); 


#endif
