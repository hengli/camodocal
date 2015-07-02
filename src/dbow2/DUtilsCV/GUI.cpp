/**
 * File: GUI.cpp
 * Project: DUtilsCV library
 * Author: Dorian Galvez-Lopez
 * Date: September 24, 2010
 * Description: OpenCV-related GUI functions
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
#include <queue>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cstdio>
#include <string>
#include <sstream>
#include <algorithm>
#include "GUI.h"

#include <iostream>

#ifdef HAVE_OPENCV3
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#endif

using namespace std;
using namespace DUtilsCV;

// ---------------------------------------------------------------------------

int GUI::showImage(const cv::Mat &image, bool autosize, 
  GUI::tWinHandler *hwnd, int timeout)
{
  static int id = 0;
  
  string name;
  if(hwnd == NULL || hwnd->empty())
  {
    stringstream ssname;
    ssname << "win" << id++;
    name = ssname.str();
    
    int flags = 0;
    if(autosize) flags |= CV_WINDOW_AUTOSIZE;
    
    cv::namedWindow( name.c_str(), flags );
    
    if(hwnd) *hwnd = name;
  }
  else
  {
    name = *hwnd;
  }
  
  cv::imshow( name, image );
  char key = cv::waitKey(timeout);
  
  if(key == -1) key = NO_KEY;
  
  return key;
}

// ---------------------------------------------------------------------------

int GUI::showImageInfo(const cv::Mat &image, bool autosize,
  tWinHandler *_hwnd)
{
  const cv::Scalar STATUSBAR_COLOR(0, 0, 0);
  const cv::Scalar AXIS_COLOR(255, 255, 255);
  const cv::Scalar TEXT_COLOR(0, 255, 255);
  
  const int STATUSBAR_HEIGHT = 30;
  const int STATUSBAR_LEFT_MARGIN = 10;
  
  const int W = image.cols; // real image width
  const int H = image.rows; // real image height
  
  cv::Mat canvas(image.rows + STATUSBAR_HEIGHT, image.cols, CV_8UC3);
  cv::Mat image_rgb;
  cv::Mat aux;
  
  tWinHandler haux;
  tWinHandler *hwnd = (_hwnd ? _hwnd : &haux);
  
  if(image.type() == CV_8U)
    cv::cvtColor(image, image_rgb, CV_GRAY2RGB);
  else
    image_rgb = image;
  
  // creates the window for the first time
  {
    // image
    aux = canvas.rowRange(0, image.rows);
    image_rgb.copyTo(aux);
    // status bar
    aux = canvas.rowRange(image.rows, canvas.rows);
    aux.setTo(STATUSBAR_COLOR);
  }
  
  // and attach the mouse
  MouseHandler mouse;
  
  showImage(canvas, autosize, hwnd, 5);
  mouse.attachToMotions(*hwnd);
  
  MouseHandler::tEvent event;
  bool refresh = false;
  
  int key;
  for(bool goon = true; goon; )
  {
    // refresh
    if(refresh)
    {
      // create image
      // image
      aux = canvas.rowRange(0, image.rows);
      image_rgb.copyTo(aux);
      // status bar
      aux = canvas.rowRange(image.rows, canvas.rows);
      aux.setTo(STATUSBAR_COLOR);
      
      // info about mouse
      
      // axis
      // vertical
      if(event.x >= 0 && event.x < W)
      {
        cv::line(canvas, cv::Point(event.x, 0), cv::Point(event.x, H-1),
          AXIS_COLOR);
      }
      // horizontal
      if(event.y >= 0 && event.y < H)
      {
        cv::line(canvas, cv::Point(0, event.y), cv::Point(W-1, event.y),
          AXIS_COLOR);
      }
      
      // text
      stringstream ss;
      ss << "Image size: " << W << "x" << H;
      
      if(event.x >= 0 && event.x < W && event.y >= 0 && event.y < H)
      {
        ss << ". Cursor position: x = " << event.x << ", y = " << event.y;
      }
      
      int baseline;
      cv::Size sz = cv::getTextSize(ss.str(), cv::FONT_HERSHEY_PLAIN, 1, 1,
        &baseline);
      
      cv::putText(canvas, ss.str(), 
        cv::Point(STATUSBAR_LEFT_MARGIN, 
          canvas.rows - (STATUSBAR_HEIGHT - sz.height)/2), 
        cv::FONT_HERSHEY_PLAIN, 1, TEXT_COLOR, 1);
    }
    
    // control
    {
      key = showImage(canvas, autosize, hwnd, 5);
      refresh = mouse.get(event);
      
      // check if the window was closed
      if(!windowExists(*hwnd) || key != NO_KEY)
      {
        goon = false;
      }
    }
  }
  
  //destroyWindow(hwnd);
  
  return key;
}
  

// ---------------------------------------------------------------------------

bool GUI::windowExists(const tWinHandler &hwnd)
{
  return cvGetWindowHandle(hwnd.c_str()) != NULL;
}

// ---------------------------------------------------------------------------

void GUI::destroyWindow(const tWinHandler &hwnd)
{
  cv::destroyWindow(hwnd);
}

// ---------------------------------------------------------------------------

bool GUI::showSystemImage(const cv::Mat &image,
  const std::string &tmp_file, const std::string &app)
{
  cv::imwrite(tmp_file, image);
  
  stringstream ss;
  ss << app << " " << tmp_file;

  return(0 == system(ss.str().c_str()));
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------

GUI::MouseHandler::MouseHandler(): m_attached(false)
{
  m_valid_events.reserve(10);
}

// ---------------------------------------------------------------------------

GUI::MouseHandler::~MouseHandler()
{
}

// ---------------------------------------------------------------------------

void GUI::MouseHandler::attach(const tWinHandler &hwnd, 
  const std::vector<int> &events)
{
  if(events.empty())
  {
    if(m_valid_events.empty()) listenToAll();
  }
  else
  {
    m_valid_events = events;
    std::sort(m_valid_events.begin(), m_valid_events.end());
  }
  
  m_attached = true;
  cv::setMouseCallback(hwnd, GUI::MouseHandler::callbackHandlerFunction, this);
}

// ---------------------------------------------------------------------------

void GUI::MouseHandler::attachToClicks(const tWinHandler &hwnd)
{
  m_valid_events.resize(1);
  m_valid_events[0] = CV_EVENT_LBUTTONUP;
  m_attached = true;
  cv::setMouseCallback(hwnd, GUI::MouseHandler::callbackHandlerFunction, this);
}

// ---------------------------------------------------------------------------

void GUI::MouseHandler::attachToMotions(const tWinHandler &hwnd)
{
  m_valid_events.resize(1);
  m_valid_events[0] = CV_EVENT_MOUSEMOVE;
  m_attached = true;
  cv::setMouseCallback(hwnd, GUI::MouseHandler::callbackHandlerFunction, this);
}

// ---------------------------------------------------------------------------

void GUI::MouseHandler::listen(int event)
{
  //@note RACE CONDITION!
  
  vector<int>::iterator vit = 
    std::lower_bound(m_valid_events.begin(), m_valid_events.end(), event);
  // vit == end || first position where *vit >= event
  
  if(vit == m_valid_events.end() || *vit != event)
  {
    m_valid_events.insert(vit, event);
  }
}

// ---------------------------------------------------------------------------

void GUI::MouseHandler::listenToAll()
{
  //@note RACE CONDITION!
  
  m_valid_events.resize(10);
  m_valid_events[0] = CV_EVENT_LBUTTONDOWN;
  m_valid_events[1] = CV_EVENT_LBUTTONUP;
  m_valid_events[2] = CV_EVENT_LBUTTONDBLCLK;
  m_valid_events[3] = CV_EVENT_RBUTTONDOWN;
  m_valid_events[4] = CV_EVENT_RBUTTONUP;
  m_valid_events[5] = CV_EVENT_RBUTTONDBLCLK;
  m_valid_events[6] = CV_EVENT_MBUTTONDOWN;
  m_valid_events[7] = CV_EVENT_MBUTTONUP;
  m_valid_events[8] = CV_EVENT_MBUTTONDBLCLK;
  m_valid_events[9] = CV_EVENT_MOUSEMOVE;
  
  std::sort(m_valid_events.begin(), m_valid_events.end());
}

// ---------------------------------------------------------------------------

void GUI::MouseHandler::ignore(int event)
{
  //@note RACE CONDITION!
  
  vector<int>::iterator vit = 
    std::lower_bound(m_valid_events.begin(), m_valid_events.end(), event);
  // vit == end || first position where *vit >= event
  
  if(vit != m_valid_events.end() && *vit == event)
  {
    m_valid_events.erase(vit);
  }
}

// ---------------------------------------------------------------------------

bool GUI::MouseHandler::get(tEvent &event)
{
  //@note RACE CONDITION!
  
  if(m_events.empty()) return false;
  
  event = m_events.front();
  m_events.pop();
  
  return true;
}

// ---------------------------------------------------------------------------

void GUI::MouseHandler::callbackHandlerFunction(int event, int x, int y, 
  int flags, void *pMouseHandler)
{
  //@note RACE CONDITION!
  
  // check if we are interested in this event
  MouseHandler& mh = *((MouseHandler*)pMouseHandler);

  if (std::binary_search(mh.m_valid_events.begin(), mh.m_valid_events.end(), 
    event))
  {
    mh.m_events.push(tEvent(x, y, event, flags));
  } 
  // ignore otherwise
  
}

// ---------------------------------------------------------------------------


