/*
    BRISK - Binary Robust Invariant Scalable Keypoints
    Reference implementation of
    [1] Stefan Leutenegger,Margarita Chli and Roland Siegwart, BRISK:
    	Binary Robust Invariant Scalable Keypoints, in Proceedings of
    	the IEEE International Conference on Computer Vision (ICCV2011).

    Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
    Stefan Leutenegger, Simon Lynen and Margarita Chli.

    This file is part of BRISK.

    BRISK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    BRISK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with BRISK.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <opencv2/opencv.hpp>
#include "../include/brisk/brisk.h"
#include <fstream>
#include <iostream>
#include <list>

//standard configuration for the case of no file given
const int n=12;
const float r=2.5; // found 8-9-11, r=3.6, exponent 1.5

void help(char** argv){
	std::cout << "This command line tool lets you evaluate different keypoint "
			<< "detectors, descriptors and matchers." << std::endl
			<< "usage:" << std::endl
			<< argv[0] << " <dataset> <2nd> <detector> <descriptor> [descFile1 descFile1]" << std::endl
			<< "    " << "dataset:    Folder containing the images. The images must be of .ppm "<< std::endl
			<< "    " << "            format. They must be named img#.ppm, and there must be "<< std::endl
			<< "    " << "            corresponding homographies named H1to#." << std::endl
			<< "    " << "            You can also use the prefix rot-, then 2nd will be the" << std::endl
			<< "    " << "            rotation in degrees." << std::endl
			<< "    " << "2nd:        Number of the 2nd image (the 1st one is always img1.ppm)"<< std::endl
			<< "    " << "            or the rotation in degrees, if rot- used." << std::endl
			<< "    " << "detector:   Feature detector, e.g. AGAST, or BRISK. You can add the "<< std::endl
			<< "    " << "            threshold, e.g. BRISK80 or SURF2000"<< std::endl
			<< "    " << "descriptor: Feature descriptor, e.g. SURF, BRIEF, BRISK or U-BRISK."<< std::endl
			<< "    " << "[descFile]: Optional: files with descriptors to act as detected points."<< std::endl;
}

int main(int argc, char ** argv) {

	//std::cout<<sizeof(cv::Point2i)<<" "<<sizeof(CvPoint)<<std::endl;

	// process command line args
	if(argc != 5 && argc != 7 && argc != 1){
		help(argv);
		return 1;
	}

	// names of the two image files
	std::string fname1;
	std::string fname2;
	cv::Mat imgRGB1;
	cv::Mat imgRGB2;
	cv::Mat imgRGB3;
	bool do_rot=false;
	// standard file extensions
	std::vector<std::string> fextensions;
	fextensions.push_back(".bmp");
	fextensions.push_back(".jpeg");
	fextensions.push_back(".jpg");
	fextensions.push_back(".jpe");
	fextensions.push_back(".jp2");
	fextensions.push_back(".png");
	fextensions.push_back(".pgm");
	fextensions.push_back(".ppm");
	fextensions.push_back(".sr");
	fextensions.push_back(".ras");
	fextensions.push_back(".tiff");
	fextensions.push_back(".tif");

	// if no arguments are passed: 
	if(argc==1){
		int i=0;
		int fextensions_size=fextensions.size();
		while(imgRGB1.empty()||imgRGB2.empty()){
			fname1 = "../../images/img1"+fextensions[i];
			fname2 = "../../images/img2"+fextensions[i];
			imgRGB1 = cv::imread(fname1);
			imgRGB2 = cv::imread(fname2);
			i++;
			if(i>=fextensions_size) break;
		}
		if (imgRGB2.empty()||imgRGB2.empty())
		{
			std::cout<<"image(s) "<<fname1<<", "<<fname2<<" not found." << std::endl;
			return 2;
		}
	}
	else{
		if(strncmp("rot-", argv[1], 4)==0){
			do_rot=true;
			int i=0;
			int fextensions_size=fextensions.size();
			while(imgRGB1.empty()){
				fname1 = std::string(argv[1]+4)+"/img1"+fextensions[i];
				imgRGB1 = cv::imread(fname1);
				i++;
				if(i>=fextensions_size) break;
			}
			if (imgRGB2.empty())
			{
				std::cout<<"image not found." << std::endl;
				return 2;
			}
		}
		else{
			int i=0;
			int fextensions_size=fextensions.size();
			while(imgRGB1.empty()||imgRGB2.empty()){
				fname1 = std::string(argv[1])+"/img1"+fextensions[i];
				fname2 = std::string(argv[1])+"/img"+std::string(argv[2])+fextensions[i];
				imgRGB1 = cv::imread(fname1);
				imgRGB2 = cv::imread(fname2);
				i++;
				if(i>=fextensions_size) break;
			}
			if (imgRGB2.empty()||imgRGB2.empty())
			{
				std::cout<<"image(s)"<<fname1<<", "<<fname2<<" not found." << std::endl;
				return 2;
			}
		}
		//unsigned int N=atoi(argv[3]);
		if (imgRGB1.empty())
		{
			fname1 = std::string(argv[1]+4)+"/img1.pgm";
			imgRGB1 = cv::imread(fname1);
			if (imgRGB1.empty()){
				std::cout<<"image not found at " << fname1 << std::endl;
				return 2;
			}
		}
	}

	// convert to grayscale
	cv::Mat imgGray1;
	cv::cvtColor(imgRGB1, imgGray1, CV_BGR2GRAY);
	cv::Mat imgGray2;
	if(!do_rot){
		cv::cvtColor(imgRGB2, imgGray2, CV_BGR2GRAY);
	}

	// run FAST in first image
	std::vector<cv::KeyPoint> keypoints, keypoints2;
	int threshold;

	// create the detector:
	cv::Ptr<cv::FeatureDetector> detector;
	if(argc==1){
		detector = new cv::BriskFeatureDetector(60,4);
	}
	else{
		if(strncmp("FAST", argv[3], 4 )==0){
			threshold = atoi(argv[3]+4);
			if(threshold==0)
				threshold = 30;
			detector = new cv::FastFeatureDetector(threshold,true);
		}
		else if(strncmp("AGAST", argv[3], 5 )==0){
			threshold = atoi(argv[3]+5);
			if(threshold==0)
				threshold = 30;
			detector = new cv::BriskFeatureDetector(threshold,0);
		}
		else if(strncmp("BRISK", argv[3], 5 )==0){
			threshold = atoi(argv[3]+5);
			if(threshold==0)
				threshold = 30;
			detector = new cv::BriskFeatureDetector(threshold,4);
		}
		else if(strncmp("SURF", argv[3], 4 )==0){
			threshold = atoi(argv[3]+4);
			if(threshold==0)
				threshold = 400;
			detector = new cv::SurfFeatureDetector(threshold);
		}
		else if(strncmp("SIFT", argv[3], 4 )==0){
			float thresh = 0.04 / cv::SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS / 2.0;
			float edgeThreshold=atof(argv[3]+4);
			if(edgeThreshold==0)
				thresh = 10.0;
			detector = new cv::SiftFeatureDetector(thresh,edgeThreshold);
		}
		else{
			detector = cv::FeatureDetector::create( argv[3] );
		}
		if (detector.empty()){
			std::cout << "Detector " << argv[3] << " not recognized. Check spelling!" << std::endl;
			return 3;
		}
	}

	// run the detector:
	if(argc == 7){
		// try to read descriptor files
		std::string desc1 = std::string(argv[5]);
		std::string desc2 = std::string(argv[6]);
		std::ifstream descf1(desc1.c_str());
		if(!descf1.good()){
			std::cout<<"Descriptor file not found at " << desc1<<std::endl;
			return 3;
		}
		std::ifstream descf2(desc2.c_str());
		if(!descf2.good()){
			std::cout<<"Descriptor file not found at " << desc2<<std::endl;
			return 3;
		}

		// fill the keypoints
		std::string str1;
		std::stringstream strstrm1;
		std::getline(descf1,str1);
		std::getline(descf1,str1);
		while(!descf1.eof()){
			std::getline(descf1,str1);
			float x,y,a;
			strstrm1.str(str1);
			strstrm1>>x;
			strstrm1>>y;
			strstrm1>>a;
			float r=sqrt(1.0/a);
			keypoints.push_back(cv::KeyPoint(x, y, 4.0*r));
		}
		std::string str2;
		std::stringstream strstrm2;
		std::getline(descf2,str2);
		std::getline(descf2,str2);
		while(!descf2.eof()){
			std::getline(descf2,str2);
			float x,y,a;
			strstrm2.str(str2);
			strstrm2>>x;
			strstrm2>>y;
			strstrm2>>a;
			float r=sqrt(1.0/a);
			keypoints2.push_back(cv::KeyPoint(x, y, 4.0*r));
		}

		// clean up
		descf1.close();
		descf2.close();
	}
	else{

		detector->detect(imgGray1,keypoints);
		detector->detect(imgGray2,keypoints2);
	}

	// now the extractor:
	bool hamming=true;
	cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
	// now the extractor:
	if(argc==1){
		descriptorExtractor = new cv::BriskDescriptorExtractor();
	}
	else{
		if(std::string(argv[4])=="BRISK"){
			descriptorExtractor = new cv::BriskDescriptorExtractor();
		}
		else if(std::string(argv[4])=="U-BRISK"){
			descriptorExtractor = new cv::BriskDescriptorExtractor(false);
		}
		else if(std::string(argv[4])=="SU-BRISK"){
			descriptorExtractor = new cv::BriskDescriptorExtractor(false,false);
		}
		else if(std::string(argv[4])=="S-BRISK"){
			descriptorExtractor = new cv::BriskDescriptorExtractor(true,false);
		}
		else if(std::string(argv[4])=="BRIEF"){
			descriptorExtractor = new cv::BriefDescriptorExtractor(64);
		}
		else if(std::string(argv[4])=="CALONDER"){
			descriptorExtractor = new cv::CalonderDescriptorExtractor<float>("current.rtc");
			hamming=false;
		}
		else if(std::string(argv[4])=="SURF"){
			descriptorExtractor = new cv::SurfDescriptorExtractor();
			hamming=false;
		}
		else if(std::string(argv[4])=="SIFT"){
			descriptorExtractor = new cv::SiftDescriptorExtractor();
			hamming=false;
		}
		else{
			descriptorExtractor = cv::DescriptorExtractor::create( argv[4] );
		}
		if (descriptorExtractor.empty()){
			hamming=false;
			std::cout << "Descriptor " << argv[4] << " not recognized. Check spelling!" << std::endl;
			return 4;
		}
	}

	// get the descriptors
	cv::Mat descriptors, descriptors2;
	std::vector<cv::DMatch> indices;
	// first image
	descriptorExtractor->compute(imgGray2,keypoints2,descriptors2);
	// and the second one
	descriptorExtractor->compute(imgGray1,keypoints,descriptors);

	// matching
	std::vector<std::vector<cv::DMatch> > matches;
	cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
	if(hamming)
		descriptorMatcher = new cv::BruteForceMatcher<cv::HammingSse>();
	else
		descriptorMatcher = new cv::BruteForceMatcher<cv::L2<float> >();
	if(hamming)
		descriptorMatcher->radiusMatch(descriptors2,descriptors,matches,100.0);
	else
		descriptorMatcher->radiusMatch(descriptors2,descriptors,matches,0.21);
	cv::Mat outimg;
	
	// drawing
	drawMatches(imgRGB2, keypoints2, imgRGB1, keypoints,matches,outimg,
			 cv::Scalar(0,255,0), cv::Scalar(0,0,255),
			std::vector<std::vector<char> >(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	cv::namedWindow("Matches");
	cv::imshow("Matches", outimg);
	cv::waitKey();

	return 0;
}
