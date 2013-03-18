#include "five-point.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <ctime>
#include <iostream>
#include <fstream>
#include <vector>
int main()
{

	int t0 = clock(); 
//    CvEMEstimator em; 

	std::vector<cv::Point2f> m1, m2; 
	std::fstream fin("../test/large.txt"); 
	do
	{
		cv::Point2f p; 
		fin >> p.x >> p.y; 
		m1.push_back(p); 
		fin >> p.x >> p.y; 
		m2.push_back(p); 
		std::cout << m1.size() << " "<< p.x << std::endl; 

	} while (!fin.eof()); 
	m1.resize(m1.size() - 1); 
	m2.resize(m2.size() - 1); 
//	std::cout << cv::Mat(m1) << std::endl; 
//	std::cout << cv::Mat(m2) << std::endl; 

//    int count = em.runKernel(&_m1, &_m2, E); 
	    
//	cv::Mat E = findEssentialMat(m1, m2); 
	srand(time(0)); 
	cv::Mat mask; 
	cv::Mat E = findEssentialMat(m1, m2, 300, cv::Point2d(320, 400), CV_FM_RANSAC, 1.0 - 1e-10, 1, mask); 
	std::cout << E << std::endl; 
	cv::Mat R1, R2, t; 
	decomposeEssentialMat(E, R1, R2, t); 
	std::cout << R1 << std::endl; 
	std::cout << R2 << std::endl; 
	std::cout << t << std::endl; 
	std::cout << countNonZero(mask) << std::endl; 
	std::cout << "==============" << std::endl; 

	std::cout << recoverPose(E, m1, m2, R1, t, 300, cv::Point2d(320, 400), mask); 
	std::cout << R1 << std::endl; 
	std::cout << t << std::endl; 
//	cv::Mat F = cv::findFundamentalMat(m1, m2); 
	
}
