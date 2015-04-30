#ifndef ATOMICDATA_H
#define ATOMICDATA_H

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <opencv2/core/core.hpp>

namespace camodocal
{

template<class T>
class AtomicData
{
public:
    AtomicData();

    T& data(void);
    uint64_t& timeStamp(); 
    bool& available(void);
    bool& process(void);

    void lockData(void);
    bool tryLockData(void);
    void unlockData(void);

    void notifyData(void);
    bool timedWaitForData(const boost::system_time& timeout);
    void waitForData(void);

    void notifyProcessingDone(void);
    void waitForProcessingDone(void);

private:
    T mData;
    uint64_t mTimeStamp; 
    bool mAvailable;
    bool mProcess;

    boost::condition_variable mDataCond;
    boost::mutex mDataMutex;

    boost::condition_variable mProcessCond;
    boost::mutex mProcessMutex;
};

template<class T>
AtomicData<T>::AtomicData()
 : mAvailable(false)
 , mProcess(false)
{

}

template<class T>
T&
AtomicData<T>::data(void)
{
    return mData;
}

template<class T>
uint64_t&
AtomicData<T>::timeStamp()
{
    return mTimeStamp; 
}

template<class T>
bool&
AtomicData<T>::available(void)
{
    return mAvailable;
}

template<class T>
bool&
AtomicData<T>::process(void)
{
    return mProcess;
}

template<class T>
void
AtomicData<T>::lockData(void)
{
    mDataMutex.lock();
}

template<class T>
bool
AtomicData<T>::tryLockData(void)
{
    return mDataMutex.try_lock();
}

template<class T>
void
AtomicData<T>::unlockData(void)
{
    mDataMutex.unlock();
}

template<class T>
void
AtomicData<T>::notifyData(void)
{
    {
        boost::lock_guard<boost::mutex> lock(mDataMutex);

        mAvailable = true;
    }

    mDataCond.notify_one();
}

template<class T>
void
AtomicData<T>::waitForData(void)
{
    boost::unique_lock<boost::mutex> lock(mDataMutex);

    while (!mAvailable)
    {
        mDataCond.wait(lock);
    }
}

template<class T>
bool
AtomicData<T>::timedWaitForData(const boost::system_time& timeout)
{
    boost::unique_lock<boost::mutex> lock(mDataMutex);

    bool ret = mDataCond.timed_wait(lock, timeout);
    (void)ret;

    return mAvailable;
}

template<class T>
void
AtomicData<T>::notifyProcessingDone(void)
{
    {
        boost::lock_guard<boost::mutex> lock(mProcessMutex);

        mProcess = false;
    }

    mProcessCond.notify_one();
}

template<class T>
void
AtomicData<T>::waitForProcessingDone(void)
{
    boost::unique_lock<boost::mutex> lock(mProcessMutex);

    mProcess = true;

    notifyData();

    while (mProcess)
    {
        mProcessCond.wait(lock);
    }
}

}

#endif
