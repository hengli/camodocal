/**
 * File: Drawing.cpp
 * Project: DUtilsCV library
 * Author: Dorian Galvez-Lopez
 * Date: September 23, 2010
 * Description: drawing functions
 *
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "Drawing.h"


#ifdef HAVE_OPENCV3
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#endif



using namespace std;
using namespace DUtilsCV;

// ---------------------------------------------------------------------------

void Drawing::drawKeyPoints(cv::Mat &image, 
    const std::vector<cv::KeyPoint> &keypoints,
    bool colorOctave, bool useCartesianAngle)
{
  CvScalar colors[4] = {
    cvScalar(0, 0, 255),
    cvScalar(0, 255, 0),
    cvScalar(255, 0, 0),
    cvScalar(255, 255, 255) 
  };

  const double PI = 3.14159265;

  vector<cv::KeyPoint>::const_iterator it;
  for(it = keypoints.begin(); it != keypoints.end(); ++it)
  {
    //float s = ((9.0f/1.2f) * it->size/10.0f) / 3.0f;
    float s = it->size / 2.f;
    //if(s < 3.f) s = 3.f;
    
    const CvScalar *color;
    if(!colorOctave || it->octave < 1 || it->octave > 3)
      color = &colors[3];
    else
      color = &colors[it->octave-1];
    
    int r1 = (int)(it->pt.y + 0.5);
    int c1 = (int)(it->pt.x + 0.5);
    
    cv::circle(image, cvPoint(c1, r1), (int)s, *color, 1);
    
    if(it->angle >= 0)
    {
      // angle is in [0..360]
      float o = (float)(it->angle * PI / 180.);
      int c2 = (int)(s * cos(o) + it->pt.x + 0.5);
      int r2;
      
      if(useCartesianAngle)
        r2 = (int)(- s * sin(o) + it->pt.y + 0.5);
      else
        r2 = (int)(s * sin(o) + it->pt.y + 0.5);

      cv::line(image, cvPoint(c1, r1), cvPoint(c2, r2), *color);
    }
  }
}

// ---------------------------------------------------------------------------

void Drawing::saveKeyPointImage(const std::string &filename,
    const cv::Mat &image, const std::vector<cv::KeyPoint> &keypoints)
{
  cv::Mat im = image.clone();
  Drawing::drawKeyPoints(im, keypoints);
  cv::imwrite(filename, im);
}

// ---------------------------------------------------------------------------

void Drawing::saveCorrespondenceImage(const std::string &filename, 
    const cv::Mat &im1,
    const cv::Mat &im2, const std::vector<cv::KeyPoint> &kp1,
    const std::vector<cv::KeyPoint> &kp2,
    const std::vector<int> &c1, const std::vector<int> &c2)
{
  cv::Mat image;
  Drawing::drawCorrespondences(image, im1, im2, kp1, kp2, c1, c2);
  cv::imwrite(filename, image);
}

// ---------------------------------------------------------------------------

void Drawing::drawCorrespondences(cv::Mat &image, const cv::Mat &img1,
    const cv::Mat &img2, const std::vector<cv::KeyPoint> &kp1,
    const std::vector<cv::KeyPoint> &kp2,
    const std::vector<int> &c1, const std::vector<int> &c2)
{
  int rows = img1.rows + img2.rows;
  int cols = (img1.cols > img2.cols ? img1.cols : img2.cols);
  
  cv::Mat aux1, aux2;
  if(img1.channels() > 1)
    cv::cvtColor(img1, aux1, CV_RGB2GRAY);
  else
    aux1 = img1.clone();
  
  if(img2.channels() > 1)
    cv::cvtColor(img2, aux2, CV_RGB2GRAY);
  else
    aux2 = img2.clone();

  Drawing::drawKeyPoints(aux1, kp1);
  Drawing::drawKeyPoints(aux2, kp2);

  cv::Mat im = cv::Mat::zeros(rows, cols, CV_8UC1);
  IplImage ipl_im = IplImage(im);
  IplImage* ipl_ret = &ipl_im;

  CvRect roi;
  roi.x = 0;
  roi.y = 0;
  roi.width = img1.cols;
  roi.height = img1.rows;
	
  cvSetImageROI(ipl_ret, roi);
  IplImage ipl_aux1 = IplImage(aux1);
#ifdef HAVE_OPENCV3
  cvCopy(&ipl_aux1, ipl_ret);
#else // HAVE_OPENCV3
  cvCopyImage(&ipl_aux1, ipl_ret);
#endif // HAVE_OPENCV3
  
  roi.x = 0;
  roi.y = img1.rows;
  roi.width = img2.cols;
  roi.height = img2.rows;
	
  cvSetImageROI(ipl_ret, roi);
  IplImage ipl_aux2 = IplImage(aux2);
#ifdef HAVE_OPENCV3
  cvCopy(&ipl_aux2, ipl_ret);
#else // HAVE_OPENCV3
  cvCopyImage(&ipl_aux2, ipl_ret);
#endif // HAVE_OPENCV3

	cvResetImageROI(ipl_ret);

	// draw correspondences
	cv::cvtColor(im, image, CV_GRAY2RGB);
	
	for(unsigned int i = 0; i < c1.size(); ++i)
	{
	  int mx = (int)kp1[ c1[i] ].pt.x;
	  int my = (int)kp1[ c1[i] ].pt.y;
	  int px = (int)kp2[ c2[i] ].pt.x;
	  int py = (int)kp2[ c2[i] ].pt.y;
	  
	  py += img1.rows;
	  
    CvScalar color = cvScalar( 
      int(((double)rand()/((double)RAND_MAX + 1.0)) * 256.0),
      int(((double)rand()/((double)RAND_MAX + 1.0)) * 256.0),
      int(((double)rand()/((double)RAND_MAX + 1.0)) * 256.0));

    cv::line(image, cvPoint(mx, my), cvPoint(px, py), color, 1);
	}
}

// ---------------------------------------------------------------------------

void Drawing::drawReferenceSystem(cv::Mat &image, const cv::Mat &cTo,
    const cv::Mat &A, const cv::Mat &K, float length)
{
  const cv::Mat cRo(cTo, cv::Range(0,3), cv::Range(0,3));
  cv::Mat cto = cTo(cv::Range(0,3), cv::Range(3,4)).clone();
  
  if(cTo.type() == CV_32F)
  {
    cto.at<float>(0,0) /= cTo.at<float>(3,3);
    cto.at<float>(1,0) /= cTo.at<float>(3,3);
    cto.at<float>(2,0) /= cTo.at<float>(3,3);
  }
  else
  {
    cto.at<double>(0,0) /= cTo.at<double>(3,3);
    cto.at<double>(1,0) /= cTo.at<double>(3,3);
    cto.at<double>(2,0) /= cTo.at<double>(3,3);
  }
  
  Drawing::drawReferenceSystem(image, cRo, cto, A, K, length);
}
    
// ---------------------------------------------------------------------------

void Drawing::drawReferenceSystem(cv::Mat &image, const cv::Mat &cRo,
  const cv::Mat &cto, const cv::Mat &A, const cv::Mat &K,
  float length)
{
  cv::Mat k;
  if(K.empty())
    k = cv::Mat::zeros(4,1, cRo.type());
  else
    k = K;

  std::vector<cv::Point3f> oP;
  oP.push_back(cv::Point3f(0,0,0));
  oP.push_back(cv::Point3f(length,0,0));
  oP.push_back(cv::Point3f(0,length,0));
  oP.push_back(cv::Point3f(0,0,length));

  vector<cv::Point2f> points2d;
  cv::projectPoints(cv::Mat(oP), cRo, cto, A, k, points2d);
  
  // draw axis
  CvScalar bluez, greeny, redx;
  
  if(image.channels() == 3 )
  {
    bluez = cvScalar(255,0,0);
    greeny = cvScalar(0,255,0);
    redx = cvScalar(0,0,255);
  }
  else
  {
    bluez = cvScalar(18,18,18);
    greeny = cvScalar(182,182,182);
    redx = cvScalar(120,120,120);
  }

  cv::line(image, points2d[0], points2d[1], redx, 2);
  cv::line(image, points2d[0], points2d[2], greeny, 2);
  cv::line(image, points2d[0], points2d[3], bluez, 2);
}

// ---------------------------------------------------------------------------

void Drawing::drawBox(cv::Mat &image, const cv::Mat &cRo,
  const cv::Mat &cto, float width, float height,
  const cv::Mat &A, const cv::Mat &K,
  std::vector<cv::Point2f> *_box,
  const Plot::Style &style) 
{
  vector<cv::Point2f> auxbox;
  vector<cv::Point2f>* pbox = (_box ? _box : &auxbox);
  vector<cv::Point2f>& box = *pbox;

  cv::Mat k;
  if(K.empty())
    k = cv::Mat::zeros(4,1, cRo.type());
  else
    k = K;
  
  const float w = width / 2.f;
  const float h = height / 2.f;
  cv::Mat oBox = (cv::Mat_<float>(4,3) <<
    -w, -h, 0,
     w, -h, 0,
     w,  h, 0,
    -w,  h, 0);
  
  cv::projectPoints(oBox, cRo, cto, A, k, box);

	cv::line(image, box[0], box[1], style.color, style.thickness);
	cv::line(image, box[1], box[2], style.color, style.thickness);
	cv::line(image, box[2], box[3], style.color, style.thickness);
	cv::line(image, box[3], box[0], style.color, style.thickness);
}

// ---------------------------------------------------------------------------

void Drawing::drawBox(cv::Mat &image, const cv::Mat &sHb, int cols, int rows,
  std::vector<cv::Point2f> *_box, const Plot::Style &style)
{
  vector<cv::Point2f> auxbox;
  vector<cv::Point2f>* pbox = (_box ? _box : &auxbox);
  vector<cv::Point2f>& box = *pbox;
  
  box.resize(4);

  cv::Mat P;
  if(sHb.type() == CV_32F)
  {
    P = sHb * (cv::Mat_<float>(3,4) <<
      0, cols, cols,    0,
      0,    0, rows, rows,
      1,    1,    1,    1);
    
    for(short i = 0; i < 4; ++i)
    {
      box[i].x = P.at<float>(0,i) / P.at<float>(2,i);
      box[i].y = P.at<float>(1,i) / P.at<float>(2,i);
    }
  }
  else
  {
    P = sHb * (cv::Mat_<double>(3,4) <<
      0, cols, cols,    0,
      0,    0, rows, rows,
      1,    1,    1,    1);
    
    for(short i = 0; i < 4; ++i)
    {
      box[i].x = P.at<double>(0,i) / P.at<double>(2,i);
      box[i].y = P.at<double>(1,i) / P.at<double>(2,i);
    }
  }

  cv::line(image, box[0], box[1], style.color, style.thickness);
	cv::line(image, box[1], box[2], style.color, style.thickness);
	cv::line(image, box[2], box[3], style.color, style.thickness);
	cv::line(image, box[3], box[0], style.color, style.thickness);

}

// ---------------------------------------------------------------------------

Drawing::Plot::Style::Style(char c, int _thickness)
{
  int r, g, b;
  r = g = b = 0;
  
  thickness = _thickness;
  
  switch(c)
  {
    case 'b': b = 255; break;
    case 'g': g = 255; break;
    case 'r': r = 255; break;
    case 'c': b = g = 255; break;
    case 'm': r = b = 255; break;
    case 'y': r = g = 255; break;
    case 'w': r = g = b = 255; break;
    default: break;
  }
  
  color = cv::Scalar(b, g, r);
}

// ---------------------------------------------------------------------------

Drawing::Plot::Style::Style(int _thickness, char c)
{  
  Drawing::Plot::Style(c, _thickness);
}

// ---------------------------------------------------------------------------

Drawing::Plot::Plot(int rows, int cols, double minx, double maxx, 
  double miny, double maxy, int margin)
{  
  m_bg = cv::Scalar(255,255,255);
  m_margin = margin;
  m_canvas.create(rows, cols, CV_8UC3);
  m_canvas = m_bg;
  setAxes(minx, maxx, miny, maxy, margin);
}

// ---------------------------------------------------------------------------

void Drawing::Plot::create(int rows, int cols, 
  double minx, double maxx, double miny, double maxy, int margin)
{
  m_margin = margin;
  m_canvas.create(rows, cols, CV_8UC3);
  setAxes(minx, maxx, miny, maxy, margin);
  m_canvas = m_bg;
}

// ---------------------------------------------------------------------------

void Drawing::Plot::create(int rows, int cols, 
  double minx, double maxx, double miny, double maxy)
{
  m_canvas.create(rows, cols, CV_8UC3);
  setAxes(minx, maxx, miny, maxy, m_margin);
  m_canvas = m_bg;
}

// ---------------------------------------------------------------------------

void Drawing::Plot::create(double minx, double maxx, double miny, double maxy,
  int margin)
{
  m_margin = margin;
  setAxes(minx, maxx, miny, maxy, m_margin);
  m_canvas = m_bg;
}

// ---------------------------------------------------------------------------

void Drawing::Plot::create(double minx, double maxx, double miny, double maxy)
{
  setAxes(minx, maxx, miny, maxy, m_margin);
  m_canvas = m_bg;
}

// ---------------------------------------------------------------------------

void Drawing::Plot::setAxes(double minx, double maxx, double miny, double maxy, 
  int margin)
{    
  m_cx = (minx + maxx) / 2;
  m_cy = (miny + maxy) / 2;
  double xdif = maxx - minx;
  double ydif = maxy - miny;
  
  m_uppx = xdif / double(m_canvas.cols - m_margin*2); // units per pixel
  m_uppy = ydif / double(m_canvas.rows - m_margin*2);
  
  // axis equal
  if(m_uppx > m_uppy)
    m_uppy = m_uppx;
  else
    m_uppx = m_uppy;
}

// ---------------------------------------------------------------------------


void Drawing::Plot::line(double x1, double y1, double x2, double y2, 
  const Style &style)
{
  cv::Point a(toPxX(x1), toPxY(y1));
  cv::Point b(toPxX(x2), toPxY(y2));
    
  cv::line(m_canvas, a, b, style.color, style.thickness);
}

// ---------------------------------------------------------------------------
