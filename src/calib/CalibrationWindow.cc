#include "CalibrationWindow.h"

#include <boost/make_shared.hpp>
#include <GL/freeglut.h>

#include "../../../../library/gpl/CameraEnums.h"

namespace camodocal
{

CalibrationWindow* CalibrationWindow::m_instance = 0;
void (*CalibrationWindow::m_keyboardHandler)(unsigned char key, int x, int y) = 0;

CalibrationWindow::CalibrationWindow()
 : m_quit(false)
{
    m_window = 0;
}

CalibrationWindow*
CalibrationWindow::instance(void)
{
    if (m_instance == 0)
    {
        m_instance = new CalibrationWindow;
    }

    return m_instance;
}

void
CalibrationWindow::setKeyboardHandler(void (*keyboardHandler)(unsigned char key, int x, int y))
{
    m_keyboardHandler = keyboardHandler;
}

void
CalibrationWindow::open(const std::string& title,
                        int imageWidth, int imageHeight, int channels)
{
    m_title = title;
    m_imageWidth = imageWidth;
    m_imageHeight = imageHeight;
    m_channels = channels;
    m_quit = false;

    for (int i = 0; i < 4; ++i)
    {
        m_view[i] = cv::Mat(imageHeight, imageWidth, CV_8UC(channels));
        m_view[i] = cv::Scalar(0);
    }

    m_displayThread = boost::make_shared<boost::thread>(&CalibrationWindow::displayHandler, this);
}

void
CalibrationWindow::close(void)
{
    m_quit = true;

    m_displayThread->join();
}

cv::Mat&
CalibrationWindow::frontView(void)
{
    return m_view[vcharge::CAMERA_FRONT];
}

cv::Mat&
CalibrationWindow::leftView(void)
{
    return m_view[vcharge::CAMERA_LEFT];
}

cv::Mat&
CalibrationWindow::rearView(void)
{
    return m_view[vcharge::CAMERA_REAR];
}

cv::Mat&
CalibrationWindow::rightView(void)
{
    return m_view[vcharge::CAMERA_RIGHT];
}

std::string&
CalibrationWindow::frontText(void)
{
    return mText[vcharge::CAMERA_FRONT];
}

std::string&
CalibrationWindow::leftText(void)
{
    return mText[vcharge::CAMERA_LEFT];
}

std::string&
CalibrationWindow::rearText(void)
{
    return mText[vcharge::CAMERA_REAR];
}

std::string&
CalibrationWindow::rightText(void)
{
    return mText[vcharge::CAMERA_RIGHT];
}

boost::mutex&
CalibrationWindow::dataMutex(void)
{
    return m_dataMutex;
}

void
CalibrationWindow::initGL(void)
{
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_FALSE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_ALPHA_TEST);
    glEnable(GL_TEXTURE_2D);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel(GL_FLAT);

    for (int i = 0; i < 4; ++i)
    {
        glGenTextures(1, &m_glTex[i]);
        glBindTexture(GL_TEXTURE_2D, m_glTex[i]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }

    resizeGLScene(m_imageWidth, m_imageHeight);
}

void
CalibrationWindow::displayHandler(CalibrationWindow* window)
{
    int argc = 0;
    char** argv = 0;

    if (glutGet(GLUT_INIT_STATE) != 1)
    {
        glutInit(&argc, argv);
    }

    glutInitContextProfile(GLUT_CORE_PROFILE);
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,
                  GLUT_ACTION_GLUTMAINLOOP_RETURNS);

    glutInitDisplayMode(GLUT_RGBA | GLUT_ALPHA | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(window->m_imageWidth, window->m_imageHeight);
    glutInitWindowPosition(0, 0);

    window->m_window = glutCreateWindow(window->m_title.c_str());
    glutDisplayFunc(&drawGLScene);
    if (m_keyboardHandler)
    {
        glutKeyboardFunc(m_keyboardHandler);
    }
    glutTimerFunc(0, drawGLTimer, 0);
    glutReshapeFunc(&resizeGLScene);

    window->initGL();

    glutMainLoop();
}

void
CalibrationWindow::drawGLScene(void)
{
    if (m_instance == 0)
    {
        return;
    }

    m_instance->m_dataMutex.lock();

    if (glutGetWindow() == m_instance->m_window)
    {
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        for (int i = 0; i < 4; ++i)
        {
            glBindTexture(GL_TEXTURE_2D, m_instance->m_glTex[i]);
            if (m_instance->m_channels == 1)
            {
                glTexImage2D(GL_TEXTURE_2D, 0, 1,
                             m_instance->m_imageWidth, m_instance->m_imageHeight,
                             0, GL_LUMINANCE, GL_UNSIGNED_BYTE,
                             m_instance->m_view[i].data);
            }
            else
            {
                glTexImage2D(GL_TEXTURE_2D, 0, 3,
                             m_instance->m_imageWidth, m_instance->m_imageHeight,
                             0, GL_BGR_EXT, GL_UNSIGNED_BYTE,
                             m_instance->m_view[i].data);
            }

            int offsetX = 0;
            int offsetY = 0;

            switch (i)
            {
            case vcharge::CAMERA_FRONT:
                offsetX = 0; offsetY = 0;
                break;
            case vcharge::CAMERA_LEFT:
                offsetX = m_instance->m_imageWidth / 2; offsetY = 0;
                break;
            case vcharge::CAMERA_REAR:
                offsetX = 0; offsetY = m_instance->m_imageHeight / 2;
                break;
            case vcharge::CAMERA_RIGHT:
                offsetX = m_instance->m_imageWidth / 2; offsetY = m_instance->m_imageHeight / 2;
                break;
            }

            glBegin(GL_TRIANGLE_FAN);
            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
            glTexCoord2f(0, 0); glVertex3f(offsetX, offsetY, 0);
            glTexCoord2f(1, 0); glVertex3f(offsetX + m_instance->m_imageWidth / 2, offsetY, 0);
            glTexCoord2f(1, 1); glVertex3f(offsetX + m_instance->m_imageWidth / 2, offsetY + m_instance->m_imageHeight / 2, 0);
            glTexCoord2f(0, 1); glVertex3f(offsetX, offsetY + m_instance->m_imageHeight / 2, 0);
            glEnd();

            glBindTexture(GL_TEXTURE_2D, 0);

            int textOffsetX = offsetX + 10;
            int textOffsetY = offsetY + 25;

            glRasterPos2i(textOffsetX, textOffsetY);
            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

            switch (i)
            {
            case vcharge::CAMERA_FRONT:
                glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char*>("front"));
                break;
            case vcharge::CAMERA_LEFT:
                glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char*>("left"));
                break;
            case vcharge::CAMERA_REAR:
                glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char*>("rear"));
                break;
            case vcharge::CAMERA_RIGHT:
                glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char*>("right"));
                break;
            }

            textOffsetX = offsetX + m_instance->m_imageWidth / 4;
            textOffsetY = offsetY + m_instance->m_imageHeight / 2 - 20;

            if (!m_instance->mText[i].empty())
            {
                int textWidth = glutBitmapLength(GLUT_BITMAP_HELVETICA_18,
                                                 reinterpret_cast<unsigned const char*>(m_instance->mText[i].c_str()));

                glRasterPos2i(textOffsetX - textWidth / 2, textOffsetY);
                glColor4f(0.0f, 1.0f, 0.0f, 1.0f);

                glutBitmapString(GLUT_BITMAP_HELVETICA_18,
                                 reinterpret_cast<unsigned const char*>(m_instance->mText[i].c_str()));
            }
        }
    }

    m_instance->m_dataMutex.unlock();

    glFlush();
    glutSwapBuffers();
}

void
CalibrationWindow::resizeGLScene(int width, int height)
{
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width, height, 0, -1.0f, 1.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void
CalibrationWindow::drawGLTimer(int extra)
{
    if (m_instance == 0)
    {
        return;
    }

    if (m_instance->m_quit)
    {
        glutDestroyWindow(m_instance->m_window);
        return;
    }

    glutSetWindow(m_instance->m_window);
    glutPostRedisplay();
    glutTimerFunc(100, drawGLTimer, 0);
}

}
