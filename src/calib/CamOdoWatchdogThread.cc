#include "CamOdoWatchdogThread.h"

namespace camodocal
{

CamOdoWatchdogThread::CamOdoWatchdogThread(boost::multi_array<bool, 1>& completed,
                                           bool& stop)
 : mCompleted(completed)
 , mStop(stop)
{

}

CamOdoWatchdogThread::~CamOdoWatchdogThread()
{
    g_return_if_fail(mThread == 0);
}

void
CamOdoWatchdogThread::launch(void)
{
    mThread = Glib::Threads::Thread::create(sigc::mem_fun(*this, &CamOdoWatchdogThread::threadFunction));
}

void
CamOdoWatchdogThread::join(void)
{
    if (mRunning)
    {
        mThread->join();
    }
    mThread = 0;
}

bool
CamOdoWatchdogThread::running(void) const
{
    return mRunning;
}

sigc::signal<void>&
CamOdoWatchdogThread::signalFinished(void)
{
    return mSignalFinished;
}

void
CamOdoWatchdogThread::threadFunction(void)
{
    mRunning = true;

    while (!mStop)
    {
        bool stop = true;

        for (size_t i = 0; i < mCompleted.size(); ++i)
        {
            if (!mCompleted[i])
            {
                stop = false;
                break;
            }
        }

        if (stop)
        {
            mStop = stop;
        }

        usleep(1000);
    }

    mRunning = false;

    mSignalFinished();
}

}
