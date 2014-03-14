#include "CamOdoWatchdogThread.h"

#include <boost/make_shared.hpp>

namespace camodocal
{

CamOdoWatchdogThread::CamOdoWatchdogThread(boost::multi_array<bool, 1>& completed,
                                           bool& stop)
 : m_running(false)
 , m_completed(completed)
 , m_stop(stop)
{

}

CamOdoWatchdogThread::~CamOdoWatchdogThread()
{

}

void
CamOdoWatchdogThread::launch(void)
{
    m_running = true;

    m_thread = boost::make_shared<boost::thread>(&CamOdoWatchdogThread::threadFunction, this);
}

void
CamOdoWatchdogThread::join(void)
{
    if (m_running)
    {
        m_thread->join();
    }
}

bool
CamOdoWatchdogThread::running(void) const
{
    return m_running;
}

boost::signals2::signal<void ()>&
CamOdoWatchdogThread::signalFinished(void)
{
    return m_signalFinished;
}

void
CamOdoWatchdogThread::threadFunction(void)
{
    while (!m_stop)
    {
        bool stop = true;

        for (size_t i = 0; i < m_completed.size(); ++i)
        {
            if (!m_completed[i])
            {
                stop = false;
                break;
            }
        }

        if (stop)
        {
            m_stop = stop;
        }

        usleep(1000);
    }

    m_running = false;

    m_signalFinished();
}

}
