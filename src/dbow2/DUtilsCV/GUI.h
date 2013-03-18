/**
 * File: GUI.h
 * Project: DUtilsCV library
 * Author: Dorian Galvez
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

#ifndef __D_CV_GUI__
#define __D_CV_GUI__

#include <vector>
#include <queue>
#include <opencv/cv.h>
#include <string>

namespace DUtilsCV
{

/// Window management for image visualization
class GUI
{
public:
  class MouseHandler;
  
public:
  
  /// Key codes
  enum tKey
  {
    NO_KEY = 0, // special value
    BACKSPACE = 8,
    TAB = 9,
    ESC = 27,
    ENTER = 10,
    SPACEBAR = 32,
    LEFT_ARROW = 81,
    UP_ARROW = 82,
    RIGHT_ARROW = 83,
    DOWN_ARROW = 84,
    PAGE_UP = 85,
    PAGE_DOWN = 86,
    HOME = 80,
    END = 87,
    INS = 99,
    
    // numeric keyboard
    KP_CENTER = -99,
    KP_SW = -100,
    KP_SE = -101,
    KP_NE = -102,
    KP_S = -103,
    KP_E = -104,
    KP_N = -105,
    KP_W = -106,
    KP_NW = -107,
    KP_INS = -98,
    KP_SUP = -97,
    KP_ENTER = -115,
    KP_PLUS = -85,
    KP_MINUS = -83,
    KP_SLASH = -81,
    KP_ASTERISK = -86,
    KP_POINT = -82,
    
    KP_0 = -80,
    KP_1 = -79,
    KP_2 = -78,
    KP_3 = -77,
    KP_4 = -76,
    KP_5 = -75,
    KP_6 = -74,
    KP_7 = -73,
    KP_8 = -72,
    KP_9 = -71
  };
  
  /// Handler for windows
  typedef std::string tWinHandler;
  
public:
  
  /**
   * Creates a windows showing the given image and waits untils some key
   * is pressed
   * @param image
   * @param autosize param used the first time the window is created
   * @param hwnd (in/out) if given, the image is shown in the given window. 
   *   If not, a new window is created and its name is returned here
   * @param timeout time to wait for user input (in ms). If 0 or not given,
   *   there is no time limit. In case of timeout, NO_KEY is returned.
   * @returns the pressed key by the user, or NO_KEY in case of timeout
   */
  static int showImage(const cv::Mat &image, bool autosize = true,
    tWinHandler *hwnd = NULL, int timeout = 0);
  
  /**
   * Creates a window showing the given image and a status bar with 
   * info about the image. The control is blocked until some key is pressed,
   * which is returned later.
   * @param image
   * @param autosize param used the first time the window is created
   * @param hwnd (in/out) if given, the image is shown in the given window. 
   *   If not, a new window is created and its name is returned here
   * @param hwnd (in/out) if given, the image is shown in the given window. 
   *   If not, a new window is created and its name is returned here
   * @returns the pressed key by the user
   */
  static int showImageInfo(const cv::Mat &image, bool autosize = true,
    tWinHandler *hwnd = NULL);
  
  /**
   * Checks if window exists
   * @param hwnd
   * @return true iif window exists
   */
  static bool windowExists(const tWinHandler &hwnd);
  
  /**
   * Closes a window
   * @param hwnd
   */
  static void destroyWindow(const tWinHandler &hwnd);
  
  /**
   * Saves the image in a temporary file to visualize it with a system
   * application
   * @param image
   * @param tmp_file file where the image is stored
   * @param app application invoked to visualize the image
   * @return true iff success
   */
  static bool showSystemImage(const cv::Mat &image,
    const std::string &tmp_file = "tmp.png",
    const std::string &app = "eog");

};

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

/// Synchronous mouse event handler
/// @note Important! this class is still non thread
/// aware, so race conditions can occur. Apart from that, it is fully functional
class GUI::MouseHandler
{
public:

  /// Mouse event
  struct tEvent
  {
    int x, y;  // mouse coordinates
    int event; // some CV_EVENT_* value (see below)
    int flags; // flags composed of CV_EVENT_FLAG_* values
    
    tEvent(){}
    tEvent(int _x, int _y, int _event, int _flags)
      : x(_x), y(_y), event(_event), flags(_flags){}
    
    // Events defined by OpenCV
    // CV_EVENT_LBUTTONDOWN;
    // CV_EVENT_LBUTTONUP;
    // CV_EVENT_LBUTTONDBLCLK;
    // CV_EVENT_RBUTTONDOWN;
    // CV_EVENT_RBUTTONUP;
    // CV_EVENT_RBUTTONDBLCLK;
    // CV_EVENT_MBUTTONDOWN;
    // CV_EVENT_MBUTTONUP;
    // CV_EVENT_MBUTTONDBLCLK;
    // CV_EVENT_MOUSEMOVE;
  };

public:

  /** 
   * Creates the handler without attaching it to any window
   */
  MouseHandler();
  
  /**
   * Destroyer
   */
  virtual ~MouseHandler();

  /**
   * Attaches the handler to a window. It can only be attached once
   * @param hwnd window
   * @param events (optional) events to listen to. If not given, the existing 
   *   list of valid events is not modified, except if it was empty. In that
   *   case, all the events are added, even mouse move.
   */
  void attach(const tWinHandler &hwnd, 
    const std::vector<int> &events = std::vector<int>());
  
  /**
   * Attaches the handler to a window and makes it listen to left single clicks
   * only
   * @param hwnd window
   */
  void attachToClicks(const tWinHandler &hwnd);

  /**
   * Attaches the handler to a window and makes it listen to mouse movements
   * only
   * @param hwnd window
   */
  void attachToMotions(const tWinHandler &hwnd);

  /**
   * Says if the mouse handler is already attached to some window
   * @return true iif attached
   */
  inline bool attached() const { return m_attached; }

  /** 
   * Adds an event to listen to
   * @param event
   */
  void listen(int event);

  /**
   * Adds all the events to the list of valid events
   */
  void listenToAll();

  /**
   * Stops listening to some event
   * @param event
   */
  void ignore(int event);

  /**
   * Gets the next event from the queue
   * @param event (out) event fetched
   * @return false iif queue was empty and no event could be retrieved
   */
  bool get(tEvent &event);
  
  /**
   * Says whether there is some event to get
   * @return true iif no event available
   */
  inline bool empty() const;

protected:

  /**
   * Callback function invoked when there is a new mouse event
   * @param event 
   * @param x
   * @param y
   * @param falgs
   * @param pMouseHandler pointer to the MouseHandler parent
   */
  static void callbackHandlerFunction(int event, int x, int y, int flags, 
    void *pMouseHandler);

protected:

  bool m_attached;

  /// Valid event queue
  std::queue<tEvent> m_events;
  
  /// List of events to listen to
  std::vector<int> m_valid_events; // in ascending order all the time

};

inline bool GUI::MouseHandler::empty() const
{
  return m_events.empty();
}

}

#endif

