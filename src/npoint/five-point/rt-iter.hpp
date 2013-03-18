#include <iostream>
#include "precomp.hpp"
#include "_modelest.h"

#include <eigen3/Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class CvTranslationIterator : public CvModelEstimator2
{
public:	
	CvTranslationIterator(const CvMat* _R); 
	virtual int runKernel( const CvMat* m1, const CvMat* m2, CvMat* model ); 

protected:
	CvMat* R; 
    virtual void computeReprojError( const CvMat* m1, const CvMat* m2,
                                     const CvMat* model, CvMat* error );
}; 

CvTranslationIterator::CvTranslationIterator(const CvMat* _R)
: CvModelEstimator2( 2, cvSize(3, 1), 1 )
{
	R = cvCreateMat(3, 3, CV_64FC1); 
	cvConvert(_R, R); 
}

int CvTranslationIterator::runKernel( const CvMat* m1, const CvMat* m2, CvMat* model )
{
	Eigen::MatrixXd M1, M2; 
	cv::cv2eigen(cv::Mat(m1).reshape(1, m1->cols), M1); 
	cv::cv2eigen(cv::Mat(m2).reshape(1, m2->cols), M2); 

	Eigen::MatrixXd A(M1.rows(), 3), T; 
	A.col(0) = M2.col(1) - M1.col(1); 
	A.col(1) = -(M2.col(0) - M1.col(0)); 
	A.col(2).array() = M2.col(0).array() * M1.col(1).array() - M1.col(0).array() * M2.col(1).array(); 
	
	T = A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(2); 
	
	double* t = model->data.db; 
	memcpy(t, T.data(), sizeof(double) * 3); 
	return 1; 

}


// Same as the runKernel, m1 and m2 should be
// 1 row x n col x 2 channels. 
// And also, error has to be of CV_32FC1. 
void CvTranslationIterator::computeReprojError( const CvMat* m1, const CvMat* m2,
                                     const CvMat* model, CvMat* error )
{
	Eigen::MatrixXd X1t, X2t; 
	cv::cv2eigen(cv::Mat(m1).reshape(1, m1->cols), X1t); 
	cv::cv2eigen(cv::Mat(m2).reshape(1, m2->cols), X2t); 
	Eigen::MatrixXd X1(3, X1t.rows()); 
	Eigen::MatrixXd X2(3, X2t.rows()); 
	X1.topRows(2) = X1t.transpose(); 
	X2.topRows(2) = X2t.transpose(); 
	X1.row(2).setOnes(); 
	X2.row(2).setOnes(); 

	Eigen::MatrixXd E, t, t_skew(3, 3), _R; 
	cv::cv2eigen(cv::Mat(model), t); 
	t_skew << 0, -t(2), t(1), t(2), 0, -t(0), -t(1), t(0), 0; 
	cv::cv2eigen(cv::Mat(R), _R); 
	E = t_skew * _R; 
	
	// Compute Simpson's error
	Eigen::MatrixXd Ex1, x2tEx1, Etx2, SimpsonError; 
	Ex1 = E * X1; 
	x2tEx1 = (X2.array() * Ex1.array()).matrix().colwise().sum(); 
	Etx2 = E.transpose() * X2; 
	SimpsonError = x2tEx1.array().square() / (Ex1.row(0).array().square() + Ex1.row(1).array().square() + Etx2.row(0).array().square() + Etx2.row(1).array().square()); 
	
	assert( CV_IS_MAT_CONT(error->type) ); 
	int i; 
	for (int i = 0; i < SimpsonError.cols(); i++) 
		error->data.fl[i] = SimpsonError(0, i); 

}

Mat iterateTranslation( InputArray _points1, InputArray _points2, const cv::Mat & R, double focal = 1.0, Point2d pp = Point2d(0, 0), 
					int method = CV_FM_RANSAC, 
					double prob = 0.999, double threshold = 1, OutputArray _mask = noArray() ) 
{
	
	Mat points1, points2; 
	_points1.getMat().copyTo(points1); 
	_points2.getMat().copyTo(points2); 

	int npoints = points1.checkVector(2);
    CV_Assert( npoints >= 0 && points2.checkVector(2) == npoints &&
				              points1.type() == points2.type());
	CV_Assert( R.cols == 3 && R.rows == 3 ); 

	if (points1.channels() > 1)
	{
		points1 = points1.reshape(1, npoints); 
		points2 = points2.reshape(1, npoints); 
	}
	points1.convertTo(points1, CV_64F); 
	points2.convertTo(points2, CV_64F); 

	points1.col(0) = (points1.col(0) - pp.x) / focal; 
	points2.col(0) = (points2.col(0) - pp.x) / focal; 
	points1.col(1) = (points1.col(1) - pp.y) / focal; 
	points2.col(1) = (points2.col(1) - pp.y) / focal; 
	
	// Reshape data to fit opencv ransac function
	points1 = points1.reshape(2, 1); 
	points2 = points2.reshape(2, 1); 
	
	cv::Mat t(1, 3, CV_64F); 
	CvMat _R = R; 
	CvTranslationIterator iterator(&_R); 

	CvMat p1 = points1; 
	CvMat p2 = points2; 
	CvMat _t = t; 
	CvMat* tempMask = cvCreateMat(1, npoints, CV_8U); 
	
	assert(npoints >= 2); 
	threshold /= focal; 
	if (method = CV_FM_RANSAC)
	{
		iterator.runRANSAC(&p1, &p2, &_t, tempMask, threshold, prob); 
	}
	else
	{
		iterator.runLMeDS(&p1, &p2, &_t, tempMask, prob); 
	}

	if (_mask.needed()) 
	{
		_mask.create(1, npoints, CV_8U, -1, true); 
		Mat mask = _mask.getMat(); 
		mask = Mat(tempMask) * 1.0; 
	}
	
	return t; 

}
