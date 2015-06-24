/**
 * File: Drawing.h
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

#ifndef __D_CV_DRAWING__
#define __D_CV_DRAWING__

#include <vector>
#include <opencv/cv.h>
#ifdef HAVE_OPENCV3
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif // HAVE_OPENCV3

namespace DUtilsCV
{

/// Drawing functions
class Drawing
{
public:

  /// Allows to plot numeric data 
  class Plot
  {
  public:

    /// Line style for plotting
    struct Style
    {
      /// Color
      cv::Scalar color;
      /// Thickness
      int thickness;
      
      /**
       * Style by default
       */
      Style(): color(cv::Scalar(0,0,0)), thickness(1) {}
      
      /**
       * Style with color and thickness
       * @param _color 
       * @param _thickness
       */
      Style(const cv::Scalar &_color, int _thickness = 1)
        : color(_color), thickness(_thickness){}
      
      /**
       * Style with color and thickness
       * @param _thickness
       * @param _color 
       */
      Style(int _thickness, const cv::Scalar &_color = cv::Scalar(0,0,0) )
        : color(_color), thickness(_thickness){}
      
      /**
       * Style with a char-coded color
       * @param c char-coded color: those supported by matlab (bgrcmykw)
       * @param _thickness
       */
      Style(char c, int _thickness = 1);
      
      /**
       * Style with a char-coded color
       * @param _thickness
       * @param c char-coded color: those supported by matlab (bgrcmykw)
       */
      Style(int _thickness, char c);
    };
    
  public:

    /**
     * Creates a plotting object
     * @param rows image height
     * @param cols image width
     * @param minx min value of x axis
     * @param maxx max value of x axis
     * @param miny min value of y axis
     * @param maxy max value of y axis
     * @param margin width of blank area close to image borders
     */
    Plot(int rows = 240, int cols = 320, 
      double minx = 0, double maxx = 1, double miny = 0, double maxy = 1,
      int margin = 0);
      
    ~Plot(){}
    
    /**
     * Changes the parameters of the axes and clears the image
     * @param rows image height
     * @param cols image width
     * @param minx min value of x axis
     * @param maxx max value of x axis
     * @param miny min value of y axis
     * @param maxy max value of y axis
     */
    void create(int rows, int cols, 
      double minx, double maxx, double miny, double maxy);

    /**
     * Changes the parameters of the axes and clears the image
     * @param rows image height
     * @param cols image width
     * @param minx min value of x axis
     * @param maxx max value of x axis
     * @param miny min value of y axis
     * @param maxy max value of y axis
     * @param margin width of blank area close to image borders
     */
    void create(int rows, int cols, 
      double minx, double maxx, double miny, double maxy, int margin);
    
    /**
     * Changes the parameters of the axes and clears the image
     * @param minx min value of x axis
     * @param maxx max value of x axis
     * @param miny min value of y axis
     * @param maxy max value of y axis
     */
    void create(double minx, double maxx, double miny, double maxy);

    /**
     * Changes the parameters of the axes and clears the image
     * @param minx min value of x axis
     * @param maxx max value of x axis
     * @param miny min value of y axis
     * @param maxy max value of y axis
     * @param margin width of blank area close to image borders
     */    
    void create(double minx, double maxx, double miny, double maxy, int margin);
    
    /**
     * Sets the background color of the plot area, but does not modify the image
     * @param color background color
     */
    inline void setBackgroundColor(const cv::Scalar &color);
    
    /**
     * Clears the image with the background color
     */
    inline void clear();
    
    /**
     * Gets the canvas image
     * @return a reference to the canvas image
     */
    inline cv::Mat& getImage();
    
    /**
     * Gets the canvas image
     * @return a const reference to the canvas image
     */
    inline const cv::Mat& getImage() const;
    
    /**
     * Draws a line
     * @param x1 first point x
     * @param y1 first point y
     * @param x2 second point x
     * @param y2 second point y
     * @param style
     */
    void line(double x1, double y1, double x2, double y2, 
      const Style &style = Style());
    
    /**
     * Plots a ser of lines
     * @param x vector of x coordinates
     * @param y vector of y coordinates
     * @param style
     */
    template<class T>
    void polyline(const std::vector<T> &x, const std::vector<T> &y,
      const Style &style = Style());
  
  protected:
  
    /**
     * Sets axis parameters
     * @param minx min value of x axis
     * @param maxx max value of x axis
     * @param miny min value of y axis
     * @param maxy max value of y axis
     * @param margin width of blank area close to image borders
     */
    void setAxes(double minx, double maxx, double miny, double maxy, 
      int margin);
    
    /**
     * Converts a x coordinate into its pixel value
     * @param x axis coordinate
     * @return u coordinate in pixels
     */
    inline int toPxX(double x) const;
    
    /**
     * Converts a y coordinate into its pixel value
     * @param y axis coordinate
     * @return v coordinate in pixels
     */
    inline int toPxY(double y) const;
  
  protected:
  
    /// Background color
    cv::Scalar m_bg;
    
    /// Area margin
    int m_margin;
    
    /// Canvas image
    cv::Mat m_canvas;
    
    /// Axis parameters
    double m_cx, m_cy, m_uppx, m_uppy;
  };
  
public:
  
  /**
   * Draws keypoints on an image
   * @param image
   * @param keypoints
   * @param useCartesianAngle if true, the angle of the keypoints is assumed
   *   to be in Cartesian axis. If false, the vision reference is used instead
   *   (X points right, Y points up)
   * @param colorOctave if true, the keypoint color codes the keypoint octave
   *   (1: red, 2: green, 3: blue, other: whie)
   * @note OpenCV SURF keypoints use Cartesian coordinates
   * @note OpenCV ORB keypoints use vision coordinates
   */
  static void drawKeyPoints(cv::Mat &image, 
    const std::vector<cv::KeyPoint> &keypoints, 
    bool colorOctave = false,
    bool useCartesianAngle = false);

  /**
   * Draws and saves keypoints on an image
   * @param filename
   * @param image
   * @param keypoints
   */
  static void saveKeyPointImage(const std::string &filename,
    const cv::Mat &image, const std::vector<cv::KeyPoint> &keypoints);
  
  /**
   * Creates an image with correspondences
   * @param image return image
   * @param im1 
   * @param im2
   * @param kp1 keypoints from im1
   * @param kp2 keypoints from im2
   * @param c1 indices of correspondences from kp1 
   * @param c2 indices of correspondences from kp2
   */
  static void drawCorrespondences(cv::Mat &image, const cv::Mat &im1,
    const cv::Mat &im2, const std::vector<cv::KeyPoint> &kp1,
    const std::vector<cv::KeyPoint> &kp2,
    const std::vector<int> &c1, const std::vector<int> &c2);

  /**
   * Creates and saves an image with correspondences
   * @param filename file to create
   * @param im1 
   * @param im2
   * @param kp1 keypoints from im1
   * @param kp2 keypoints from im2
   * @param c1 indices of correspondences from kp1 
   * @param c2 indices of correspondences from kp2
   */
  static void saveCorrespondenceImage(const std::string &filename, 
    const cv::Mat &im1,
    const cv::Mat &im2, const std::vector<cv::KeyPoint> &kp1,
    const std::vector<cv::KeyPoint> &kp2,
    const std::vector<int> &c1, const std::vector<int> &c2);

  /**
   * Draws a reference system in the given image with axes x red, y green, z
   * blue
   * @param image image to draw
   * @param cTo transformation from camera to the origin of the drawn reference
   * @param A intrinsic camera parameters 
   * @param K distortion of the camera
   * @param length length of axes
   */
  static void drawReferenceSystem(cv::Mat &image, const cv::Mat &cTo,
    const cv::Mat &A, const cv::Mat &K = cv::Mat(), 
    float length = 0.1);

  /**
   * Draws a reference system in the given image with axes x red, y green, z
   * blue
   * @param image image to draw
   * @param cRo rotation from camera to the origin of the drawn reference
   * @param cto translation from camera to the origin of the drawn reference
   * @param A intrinsic camera parameters 
   * @param K distortion of the camera
   * @param length length of axes
   */
  static void drawReferenceSystem(cv::Mat &image, const cv::Mat &cRo,
    const cv::Mat &cto, const cv::Mat &A, const cv::Mat &K = cv::Mat(), 
    float length = 0.1);

  /**
   * Draws a rectangle in the image from its location in the 3D space
   * @param image image to draw
   * @param cRo rotation from camera to the center of the rectangle
   * @param cto translation from camera to the center of the rectangle
   * @param width
   * @param height dimensions of the rectangle in metres
   * @param A intrinsic camera parameters 
   * @param K distortion of the camera
   * @param box if given, the image coordinates of the box corners are returned 
   *   here
   */
  static void drawBox(cv::Mat &image, const cv::Mat &cRo,
    const cv::Mat &cto, float width, float height,
    const cv::Mat &A, const cv::Mat &K = cv::Mat(),
    std::vector<cv::Point2f> *box = NULL,
    const Plot::Style &style = Plot::Style('r', 2));

  /** 
   * Draws a rectangle in the image from an homography
   * @param image image to draw
   * @param sHb homography from a orthonormal plane to the image
   * @param cols
   * @param rows dimensions of the plane in pixels
   * @param box if given, the image coordinates of the box corners are returned 
   *   here
   */
  static void drawBox(cv::Mat &image, const cv::Mat &sHb, int cols, int rows,
    std::vector<cv::Point2f> *box = NULL,
    const Plot::Style &style = Plot::Style('r', 2));

  /**
   * Creates an image with data plotted
   * @param image image in which create the plot 
   * @param x x coordinates of points (any unit)
   * @param y y coordinates of points (any unit)
   * @param thickness thickness of lines
   * @param color line color
   * @param bgcolor background color
   */
  template<class T>
  static void plot(cv::Mat &image, const std::vector<T>& x,
    const std::vector<T>& y, int thickness = 1,
    const cv::Scalar& color = cv::Scalar(0),
    const cv::Scalar& bgcolor = cv::Scalar(255,255,255));

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

  

};

// ---------------------------------------------------------------------------

inline void Drawing::Plot::setBackgroundColor(const cv::Scalar &color)
{
  m_bg = color;
}

// ---------------------------------------------------------------------------

inline void Drawing::Plot::clear()
{
  m_canvas = m_bg;
}

// ---------------------------------------------------------------------------

inline const cv::Mat& Drawing::Plot::getImage() const
{
  return m_canvas;
}

// ---------------------------------------------------------------------------

inline cv::Mat& Drawing::Plot::getImage()
{
  return m_canvas;
}

// ---------------------------------------------------------------------------

inline int Drawing::Plot::toPxX(double x) const
{
  return((int)( (x - m_cx) / m_uppx + m_canvas.cols/2 ));
}
    
// ---------------------------------------------------------------------------
    
inline int Drawing::Plot::toPxY(double y) const
{
  return((int)( (y - m_cy) / m_uppy + m_canvas.rows/2 ));
}

// ---------------------------------------------------------------------------

template<class T>
void Drawing::Plot::polyline(const std::vector<T>& x,
    const std::vector<T>& y, const Drawing::Plot::Style &style)
{
  if(x.empty() || y.empty()) 
    return;
  
  const unsigned int N = (x.size() < y.size() ? x.size() : y.size());
  
  if(N > 1)
  {
    // draw lines
    cv::Point a(toPxX(x[0]), toPxY(y[0]));
    for(unsigned int i = 1; i < N; ++i)
    {
      cv::Point b(toPxX(x[i]), toPxY(y[i]));
      cv::line(m_canvas, a, b, style.color, style.thickness);
      a = b;
    }
  }
  else
  {
    // draw a single point
    cv::circle(m_canvas, cv::Point(toPxX(x[0]), toPxY(y[0])), 1, 
      style.color, style.thickness);
  }
}

// ---------------------------------------------------------------------------

}

#endif
