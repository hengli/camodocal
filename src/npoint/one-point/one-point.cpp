
#include <opencv2/opencv.hpp>

#include "_modelest.h"
#include "one-point.hpp"

using namespace cv; 



class CvOnePointEstimator : public CvModelEstimator2
{
public:
    CvOnePointEstimator(); 
    virtual int runKernel( const CvMat* m1, const CvMat* m2, CvMat* model ); 
protected: 
    virtual void computeReprojError( const CvMat* m1, const CvMat* m2,
                                     const CvMat* model, CvMat* error );
}; 


CvOnePointEstimator::CvOnePointEstimator()
: CvModelEstimator2( 1, cvSize(1, 1), 1 ) 
{
}


// Notice to keep compatibility with opencv ransac, q1 and q2 have
// to be of 1 row x n col x 2 channel. 
int CvOnePointEstimator::runKernel( const CvMat* q1, const CvMat* q2, CvMat* _theta )
{
    double x1, y1, x2, y2; 
    x1 = q1->data.db[0]; 
    y1 = q1->data.db[1]; 
    x2 = q2->data.db[0]; 
    y2 = q2->data.db[1]; 

    // Transform the coord to be consistent with Scaramuzza's paper. 
    double x, y, z, x_, y_, z_; 
    x = x_ = 1; 
    y = -x1; y_ = -x2; 
    z = -y1; z_ = y2; 

    double theta = -2.0 * atan( (y_ * z - z_ * y) / (x_ * z + z_ * x) ); 
    if (theta > CV_PI / 2) theta -= CV_PI; 
    if (theta < -CV_PI / 2) theta += CV_PI; 

    // Back transform angle.
    // This angle (-theta) is the vehicle turning angle in image coord system. 
    // Note the rotation angle for image coord system should be theta. 
    _theta->data.db[0] = -theta; 
 
    return 1 ; 
}

// Same as the runKernel, m1 and m2 should be
// 1 row x n col x 2 channels. 
// And also, error has to be of CV_32FC1. 
void CvOnePointEstimator::computeReprojError( const CvMat* m1, const CvMat* m2,
                                     const CvMat* model, CvMat* error )
{
    Mat X1(m1), X2(m2); 
    int n = X1.cols; 
    X1 = X1.reshape(1, n); 
    X2 = X2.reshape(1, n); 

    X1.convertTo(X1, CV_64F); 
    X2.convertTo(X2, CV_64F); 

    double theta = model->data.db[0]; 

    // Note this E is for image normal camera system, not the system in 1-pt paper
    Mat E = (Mat_<double>(3, 3) << 0, -cos(theta * 0.5), 0, 
                                   cos(theta * 0.5), 0, -sin(theta * 0.5), 
                                   0, -sin(theta * 0.5), 0); 
    for (int i = 0; i < n; i++)
    {
        Mat x1 = (Mat_<double>(3, 1) << X1.at<double>(i, 0), X1.at<double>(i, 1), 1.0); 
        Mat x2 = (Mat_<double>(3, 1) << X2.at<double>(i, 0), X2.at<double>(i, 1), 1.0); 
        double x2tEx1 = x2.dot(E * x1); 
        Mat Ex1 = E * x1; 
        Mat Etx2 = E * x2; 
        double a = Ex1.at<double>(0) * Ex1.at<double>(0); 
        double b = Ex1.at<double>(1) * Ex1.at<double>(1); 
        double c = Etx2.at<double>(0) * Etx2.at<double>(0); 
        double d = Etx2.at<double>(0) * Etx2.at<double>(0); 

        error->data.fl[i] = x2tEx1 * x2tEx1 / (a + b + c + d); 

    }

}    

void findPose_1pt(cv::InputArray _points1, cv::InputArray _points2, 
              double focal, cv::Point2d pp, 
              cv::OutputArray _rvec, cv::OutputArray _tvec, 
              int method, double prob, double threshold, OutputArray _mask) 
{
	Mat points1, points2; 
	_points1.getMat().copyTo(points1); 
	_points2.getMat().copyTo(points2); 

	int npoints = points1.checkVector(2);
    CV_Assert( npoints >= 1 && points2.checkVector(2) == npoints &&
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

    CvOnePointEstimator estimator; 
    Mat theta(1, 1, CV_64F); 

	CvMat p1 = points1; 
	CvMat p2 = points2; 
	CvMat _theta = theta; 
	CvMat* tempMask = cvCreateMat(1, npoints, CV_8U); 
	
	assert(npoints >= 1); 
	threshold /= focal; 
    if (method == CV_RANSAC)
	{
		estimator.runRANSAC(&p1, &p2, &_theta, tempMask, threshold, prob); 
	}
	else
	{
		estimator.runLMeDS(&p1, &p2, &_theta, tempMask, prob); 
	}

    if (_mask.needed())
    {
    	_mask.create(1, npoints, CV_8U, -1, true); 
    	Mat mask = _mask.getMat(); 
    	Mat(tempMask).copyTo(mask); 
    }


    _rvec.create(3, 2, CV_64F, -1, true); 
    _tvec.create(3, 2, CV_64F, -1, true); 

    double t = theta.at<double>(0); 
    Mat rvec = (Mat_<double>(3, 2) << 0, 0, 
                                     -t, -t, 
                                      0, 0); 
    Mat tvec = (Mat_<double>(3, 2) << -sin(t / 2.0), sin(t / 2.0),
                                                  0, 0, 
                                       cos(t / 2.0), -cos(t / 2.0)); 
    
    rvec.copyTo(_rvec.getMat()); 
    tvec.copyTo(_tvec.getMat()); 

}
