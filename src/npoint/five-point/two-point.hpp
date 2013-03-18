#include <iostream>
#include "precomp.hpp"
#include "_modelest.h"

#include <eigen3/Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "five-point.hpp"
#include <ctime>

using namespace cv; 
using namespace std; 

template<typename T> int icvCompressPoints( T* ptr, const uchar* mask, int mstep, int count )
{
    int i, j;
    for( i = j = 0; i < count; i++ )
		if( mask[i*mstep] )
        {
            if( i > j )
                ptr[j] = ptr[i];
            j++;
        }
    return j;
}

class CvRotationEstimator : public CvModelEstimator2
{
public:
    CvRotationEstimator(); 
    virtual int runKernel( const CvMat* m1, const CvMat* m2, CvMat* model ); 
    virtual int run2PointSVD( const CvMat* _q1, const CvMat* _q2, CvMat* _rmatrix ); 
	virtual bool refine( const CvMat * m1, const CvMat * m2, CvMat * model, int maxIters ); 

protected: 
    virtual void computeReprojError( const CvMat* m1, const CvMat* m2,
                                     const CvMat* model, CvMat* error );
}; 

CvRotationEstimator::CvRotationEstimator()
: CvModelEstimator2( 2, cvSize(3, 3), 1)
{
}

int CvRotationEstimator::runKernel( const CvMat* m1, const CvMat* m2, CvMat* model )
{
	run2PointSVD(m1, m2, model); 
}

// Implementation of Kabsch algorithm
int CvRotationEstimator::run2PointSVD( const CvMat* m1, const CvMat* m2, CvMat* rmatrix )
{

	Eigen::MatrixXd M1, M2; 
	cv::cv2eigen(cv::Mat(m1).reshape(1, m1->cols), M1); 
	cv::cv2eigen(cv::Mat(m2).reshape(1, m2->cols), M2); 

	int n = M1.rows(); 
	Eigen::MatrixXd X1(n, 3), X2(n, 3), A, U, V, W; 
	X1.leftCols(2) = M1; 
	X1.col(2).setOnes(); 
	X2.leftCols(2) = M2; 
	X2.col(2).setOnes(); 

	X1.array() /= X1.rowwise().norm().array().replicate<1, 3>(); 
	X2.array() /= X2.rowwise().norm().array().replicate<1, 3>(); 

	A = X1.transpose() * X2; 
	U = A.jacobiSvd(Eigen::ComputeFullU|Eigen::ComputeFullV).matrixU(); 
	V = A.jacobiSvd(Eigen::ComputeFullU|Eigen::ComputeFullV).matrixV(); 
	W = Eigen::MatrixXd::Identity(3, 3); 
	W(2, 2) = (U * V.transpose()).determinant() > 0 ? 1 : -1; 

	Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R; 
	R = U * W * V.transpose(); 

	assert( CV_IS_MAT_CONT(rmatrix->type) ); 
	double* r = rmatrix->data.db; 
	memcpy(r, R.data(), sizeof(double) * 9); 
//	std::cout << r[0] << " " << r[1] << " " << r[2] << " " << r[3] << " " << r[4] << " " << r[5] << " " << r[6] << " " << r[7] << " " << r[8] << "\n"; 
//	exit(0); 
	
	return 1; 
	
}

// Same as the runKernel (run2Point), m1 and m2 should be
// 1 row x n col x 2 channels. 
// And also, error has to be of CV_32FC1. 
void CvRotationEstimator::computeReprojError( const CvMat* m1, const CvMat* m2,
                                     const CvMat* model, CvMat* _error )
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

	Eigen::MatrixXd R; 
	cv::cv2eigen(cv::Mat(model), R); 

	int i; 
	X1.array() /= X1.colwise().norm().array().replicate<3, 1>(); 
	X1 = R * X1; 
	X1.array() /= X1.row(2).array().replicate<3, 1>(); 
	
	Eigen::MatrixXd error; 
	error = (X1 - X2).colwise().norm(); 
	assert( CV_IS_MAT_CONT(_error->type) ); 

	for (i = 0; i < error.cols(); i++)
		_error->data.fl[i] = error(0, i) * error(0, i); 

}	

bool CvRotationEstimator::refine(const CvMat * m1, const CvMat * m2, CvMat * model, int maxIter )
{
	run2PointSVD(m1, m2, model); 
	return true; 
}

Mat findRotationMat( InputArray _points1, InputArray _points2, double focal = 1.0, Point2d pp = Point2d(0, 0), 
					int method = CV_FM_RANSAC, 
					double prob = 0.999, double threshold = 1, OutputArray _mask = noArray() ) 
{
	Mat points1, points2; 
	_points1.getMat().copyTo(points1); 
	_points2.getMat().copyTo(points2); 

	int npoints = points1.checkVector(2);
    CV_Assert( npoints >= 0 && points2.checkVector(2) == npoints &&
				              points1.type() == points2.type());

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

	Mat R(3, 3, CV_64F); 
	CvRotationEstimator estimator; 

	CvMat p1 = points1; 
	CvMat p2 = points2; 
	CvMat _R = R;  
	CvMat* tempMask = cvCreateMat(1, npoints, CV_8U); 
	
	assert(npoints >= 5); 
	threshold /= focal; 
	if (method = CV_FM_RANSAC)
	{
		estimator.runRANSAC(&p1, &p2, &_R, tempMask, threshold, prob); 
	}
	else
	{
		estimator.runLMeDS(&p1, &p2, &_R, tempMask, prob); 
	}

	icvCompressPoints( (CvPoint2D64f*)p1.data.ptr, tempMask->data.ptr, 1, npoints ); 
	int ninliers = icvCompressPoints( (CvPoint2D64f*)p2.data.ptr, tempMask->data.ptr, 1, npoints ); 
	p1.cols = p2.cols = ninliers; 

	std::cout << cv::Mat(&_R) << std::endl; 
	estimator.refine(&p1, &p2, &_R, 0); 
	std::cout << cv::Mat(&_R) << std::endl; 


	if (_mask.needed()) 
	{
		_mask.create(1, npoints, CV_8U, -1, true); 
		Mat mask = _mask.getMat(); 
		mask = Mat(tempMask) * 1.0; 
	}

	return R; 

}

