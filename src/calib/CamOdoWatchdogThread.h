#ifndef CAMODOWATCHDOG_THREAD_H
#define CAMODOWATCHDOG_THREAD_H

#include <boost/multi_array.hpp>
#include <glibmm.h>

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
    sigc::signal<void>& signalFinished(void);

private:
    void threadFunction(void);

    Glib::Threads::Thread* mThread;
    bool mRunning;
    sigc::signal<void> mSignalFinished;

    boost::multi_array<bool, 1>& mCompleted;
    bool& mStop;
};

}

#endif
