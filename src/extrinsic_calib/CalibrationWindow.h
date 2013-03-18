#ifndef CALIBRATIONWINDOW_H
#define CALIBRATIONWINDOW_H

#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>

#include <boost/thread/mutex.hpp>
#include <glibmm.h>
#include <opencv2/core/core.hpp>
#include <string>

namespace camodocal
{

class CalibrationWindow
{
public:
    CalibrationWindow();

    static CalibrationWindow* instance(void);

    static void setKeyboardHandler(void (*keyboardHandler)(unsigned char key, int x, int y));

    void open(const std::string& title,
              int imageWidth, int imageHeight, int channels = 1);
    void close(void);

    cv::Mat& frontView(void);
    cv::Mat& leftView(void);
    cv::Mat& rearView(void);
    cv::Mat& rightView(void);

    std::string& frontText(void);
    std::string& leftText(void);
    std::string& rearText(void);
    std::string& rightText(void);

    boost::mutex& dataMutex(void);

private:
    void initGL(void);

    static void displayHandler(CalibrationWindow* window);
    static void drawGLScene(void);
    static void resizeGLScene(int width, int height);
    static void drawGLTimer(int extra);

    static CalibrationWindow* mInstance;
    static void (*mKeyboardHandler)(unsigned char key, int x, int y);

    std::string mTitle;
    int mImageWidth;
    int mImageHeight;
    int mChannels;
    Glib::Thread* mDisplayThread;
    int mWindow;
    GLuint mGLTex[CAMERA_COUNT];
    cv::Mat mView[CAMERA_COUNT];
    std::string mText[CAMERA_COUNT];
    bool mQuit;

    boost::mutex mDataMutex;
};

}

#endif
