#include "CalibrationWindow.h"

#include <GL/freeglut.h>

namespace camodocal
{

CalibrationWindow* CalibrationWindow::mInstance = 0;
void (*CalibrationWindow::mKeyboardHandler)(unsigned char key, int x, int y) = 0;

CalibrationWindow::CalibrationWindow()
 : mDisplayThread(0)
 , mQuit(false)
{
    mWindow = 0;
}

CalibrationWindow*
CalibrationWindow::instance(void)
{
    if (mInstance == 0)
    {
        mInstance = new CalibrationWindow;
    }

    return mInstance;
}

void
CalibrationWindow::setKeyboardHandler(void (*keyboardHandler)(unsigned char key, int x, int y))
{
    mKeyboardHandler = keyboardHandler;
}

void
CalibrationWindow::open(const std::string& title,
                        int imageWidth, int imageHeight, int channels)
{
    if (!Glib::thread_supported())
    {
        Glib::thread_init();
    }

    mTitle = title;
    mImageWidth = imageWidth;
    mImageHeight = imageHeight;
    mChannels = channels;
    mQuit = false;

    for (int i = 0; i < CAMERA_COUNT; ++i)
    {
        mView[i] = cv::Mat(imageHeight, imageWidth, CV_8UC(channels));
        mView[i] = cv::Scalar(0);
    }

    mDisplayThread = Glib::Thread::create(sigc::bind(sigc::ptr_fun(&CalibrationWindow::displayHandler), this), true);
}

void
CalibrationWindow::close(void)
{
    mQuit = true;

    mDisplayThread->join();
}

cv::Mat&
CalibrationWindow::frontView(void)
{
    return mView[CAMERA_FRONT];
}

cv::Mat&
CalibrationWindow::leftView(void)
{
    return mView[CAMERA_LEFT];
}

cv::Mat&
CalibrationWindow::rearView(void)
{
    return mView[CAMERA_REAR];
}

cv::Mat&
CalibrationWindow::rightView(void)
{
    return mView[CAMERA_RIGHT];
}

std::string&
CalibrationWindow::frontText(void)
{
    return mText[CAMERA_FRONT];
}

std::string&
CalibrationWindow::leftText(void)
{
    return mText[CAMERA_LEFT];
}

std::string&
CalibrationWindow::rearText(void)
{
    return mText[CAMERA_REAR];
}

std::string&
CalibrationWindow::rightText(void)
{
    return mText[CAMERA_RIGHT];
}

boost::mutex&
CalibrationWindow::dataMutex(void)
{
    return mDataMutex;
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

    for (int i = 0; i < CAMERA_COUNT; ++i)
    {
        glGenTextures(1, &mGLTex[i]);
        glBindTexture(GL_TEXTURE_2D, mGLTex[i]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }

    resizeGLScene(mImageWidth, mImageHeight);
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
    glutInitWindowSize(window->mImageWidth, window->mImageHeight);
    glutInitWindowPosition(0, 0);

    window->mWindow = glutCreateWindow(window->mTitle.c_str());
    glutDisplayFunc(&drawGLScene);
    if (mKeyboardHandler)
    {
        glutKeyboardFunc(mKeyboardHandler);
    }
    glutTimerFunc(0, drawGLTimer, 0);
    glutReshapeFunc(&resizeGLScene);

    window->initGL();

    glutMainLoop();
}

void
CalibrationWindow::drawGLScene(void)
{
    if (mInstance == 0)
    {
        return;
    }

    mInstance->mDataMutex.lock();

    if (glutGetWindow() == mInstance->mWindow)
    {
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        for (int i = 0; i < CAMERA_COUNT; ++i)
        {
            glBindTexture(GL_TEXTURE_2D, mInstance->mGLTex[i]);
            if (mInstance->mChannels == 1)
            {
                glTexImage2D(GL_TEXTURE_2D, 0, 1,
                             mInstance->mImageWidth, mInstance->mImageHeight,
                             0, GL_LUMINANCE, GL_UNSIGNED_BYTE,
                             mInstance->mView[i].data);
            }
            else
            {
                glTexImage2D(GL_TEXTURE_2D, 0, 3,
                             mInstance->mImageWidth, mInstance->mImageHeight,
                             0, GL_BGR_EXT, GL_UNSIGNED_BYTE,
                             mInstance->mView[i].data);
            }

            int offsetX = 0;
            int offsetY = 0;

            switch (i)
            {
            case CAMERA_FRONT:
                offsetX = 0; offsetY = 0;
                break;
            case CAMERA_LEFT:
                offsetX = mInstance->mImageWidth / 2; offsetY = 0;
                break;
            case CAMERA_REAR:
                offsetX = 0; offsetY = mInstance->mImageHeight / 2;
                break;
            case CAMERA_RIGHT:
                offsetX = mInstance->mImageWidth / 2; offsetY = mInstance->mImageHeight / 2;
                break;
            }

            glBegin(GL_TRIANGLE_FAN);
            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
            glTexCoord2f(0, 0); glVertex3f(offsetX, offsetY, 0);
            glTexCoord2f(1, 0); glVertex3f(offsetX + mInstance->mImageWidth / 2, offsetY, 0);
            glTexCoord2f(1, 1); glVertex3f(offsetX + mInstance->mImageWidth / 2, offsetY + mInstance->mImageHeight / 2, 0);
            glTexCoord2f(0, 1); glVertex3f(offsetX, offsetY + mInstance->mImageHeight / 2, 0);
            glEnd();

            glBindTexture(GL_TEXTURE_2D, 0);

            int textOffsetX = offsetX + 10;
            int textOffsetY = offsetY + 25;

            glRasterPos2i(textOffsetX, textOffsetY);
            glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

            switch (i)
            {
            case CAMERA_FRONT:
                glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char*>("front"));
                break;
            case CAMERA_LEFT:
                glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char*>("left"));
                break;
            case CAMERA_REAR:
                glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char*>("rear"));
                break;
            case CAMERA_RIGHT:
                glutBitmapString(GLUT_BITMAP_HELVETICA_18, reinterpret_cast<const unsigned char*>("right"));
                break;
            }

            textOffsetX = offsetX + mInstance->mImageWidth / 4;
            textOffsetY = offsetY + mInstance->mImageHeight / 2 - 20;

            if (!mInstance->mText[i].empty())
            {
                int textWidth = glutBitmapLength(GLUT_BITMAP_HELVETICA_18,
                                                 reinterpret_cast<unsigned const char*>(mInstance->mText[i].c_str()));

                glRasterPos2i(textOffsetX - textWidth / 2, textOffsetY);
                glColor4f(0.0f, 1.0f, 0.0f, 1.0f);

                glutBitmapString(GLUT_BITMAP_HELVETICA_18,
                                 reinterpret_cast<unsigned const char*>(mInstance->mText[i].c_str()));
            }
        }
    }

    mInstance->mDataMutex.unlock();

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
    if (mInstance == 0)
    {
        return;
    }

    if (mInstance->mQuit)
    {
        glutDestroyWindow(mInstance->mWindow);
        return;
    }

    glutSetWindow(mInstance->mWindow);
    glutPostRedisplay();
    glutTimerFunc(100, drawGLTimer, 0);
}

}
