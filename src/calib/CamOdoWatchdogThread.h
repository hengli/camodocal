#ifndef CAMODOWATCHDOG_THREAD_H
#define CAMODOWATCHDOG_THREAD_H

#include <boost/multi_array.hpp>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>

namespace camodocal
{

class CamOdoWatchdogThread
{
public:
    explicit CamOdoWatchdogThread(boost::multi_array<bool, 1>& completed,
                                  bool& stop);
    virtual ~CamOdoWatchdogThread();

    void launch(void);
    void join(void);
    bool running(void) const;
    boost::signals2::signal<void ()>& signalFinished(void);

private:
    void threadFunction(void);

    boost::shared_ptr<boost::thread> m_thread;
    bool m_running;
    boost::signals2::signal<void ()> m_signalFinished;

    boost::multi_array<bool, 1>& m_completed;
    bool& m_stop;
};

}

#endif
