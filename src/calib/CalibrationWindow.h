#ifndef CALIBRATIONWINDOW_H
#define CALIBRATIONWINDOW_H

#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>

#include <boost/thread.hpp>
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

    static CalibrationWindow* m_instance;
    static void (*m_keyboardHandler)(unsigned char key, int x, int y);

    std::string m_title;
    int m_imageWidth;
    int m_imageHeight;
    int m_channels;
    boost::shared_ptr<boost::thread> m_displayThread;
    int m_window;
    GLuint m_glTex[4];
    cv::Mat m_view[4];
    std::string mText[4];
    bool m_quit;

    boost::mutex m_dataMutex;
};

}

#endif
