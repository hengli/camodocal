#ifndef SENSORDATABUFFER_H
#define SENSORDATABUFFER_H

#include <boost/thread/mutex.hpp>
#include <vector>

namespace camodocal
{

template <class T>
class SensorDataBuffer
{
public:
    explicit SensorDataBuffer(size_t size = 100);

    void clear(void);
    bool empty(void);
    size_t size(void);

    bool before(uint64_t timestamp, T& data);
    bool after(uint64_t timestamp, T& data);

    bool nearest(uint64_t timestamp, T& data);
    bool nearest(uint64_t timestamp, T& dataBefore, T& dataAfter);

    bool current(T& data);
    void push(uint64_t timestamp, const T& data);

    bool find(uint64_t timestamp, T& data);

private:
    long int timestampDiff(uint64_t t1, uint64_t t2) const
    {
        if (t2 > t1)
        {
            uint64_t d = t2 - t1;

            if (d > std::numeric_limits<long int>::max())
            {
                return std::numeric_limits<long int>::max();
            }
            else
            {
                return d;
            }
        }
        else
        {
            uint64_t d = t1 - t2;

            if (d > std::numeric_limits<long int>::max())
            {
                return std::numeric_limits<long int>::min();
            }
            else
            {
                return - static_cast<long int>(d);
            }
        }
    }

    std::vector< std::pair<uint64_t, T> > mBuffer;
    int mIndex;

    boost::mutex mGlobalMutex;
};

template <class T>
SensorDataBuffer<T>::SensorDataBuffer(size_t size)
 : mIndex(-1)
{
    mBuffer.reserve(size);
}

template <class T>
void
SensorDataBuffer<T>::clear(void)
{
    boost::mutex::scoped_lock lock(mGlobalMutex);

    mIndex = -1;
    mBuffer.clear();
}

template <class T>
bool
SensorDataBuffer<T>::empty(void)
{
    boost::mutex::scoped_lock lock(mGlobalMutex);

    return mBuffer.empty();
}

template <class T>
size_t
SensorDataBuffer<T>::size(void)
{
    boost::mutex::scoped_lock lock(mGlobalMutex);

    return mBuffer.size();
}

template <class T>
bool
SensorDataBuffer<T>::before(uint64_t timestamp, T& data)
{
    boost::mutex::scoped_lock lock(mGlobalMutex);

    if (mBuffer.empty())
    {
        return false;
    }

    int endMark = 0;
    if (mBuffer.size() == mBuffer.capacity())
    {
        endMark = mIndex;
    }
    else
    {
        endMark = mBuffer.capacity() - 1;
    }

    int mark = mIndex;
    do
    {
        long int tsDiff = timestampDiff(timestamp, mBuffer.at(mark).first);
        if (tsDiff < 0)
        {
            if (mark == mIndex)
            {
                // no data after timestamp
                return false;
            }
            else
            {
                data = mBuffer.at(mark).second;

                return true;
            }
        }

        --mark;
        if (mark < 0)
        {
            mark += mBuffer.capacity();
        }
    }
    while (mark != endMark);

    return false;
}

template <class T>
bool
SensorDataBuffer<T>::after(uint64_t timestamp, T& data)
{
    boost::mutex::scoped_lock lock(mGlobalMutex);

    if (mBuffer.empty())
    {
        return false;
    }

    int endMark = 0;
    if (mBuffer.size() == mBuffer.capacity())
    {
        endMark = mIndex;
    }
    else
    {
        endMark = mBuffer.capacity() - 1;
    }

    int mark = mIndex;
    do
    {
        long int tsDiff = timestampDiff(timestamp, mBuffer.at(mark).first);
        if (tsDiff < 0)
        {
            if (mark == mIndex)
            {
                // no data after timestamp
                return false;
            }
            else
            {
                data = mBuffer.at((mark + 1) % mBuffer.capacity()).second;

                return true;
            }
        }

        --mark;
        if (mark < 0)
        {
            mark += mBuffer.capacity();
        }
    }
    while (mark != endMark);

    return false;
}

template <class T>
bool
SensorDataBuffer<T>::nearest(uint64_t timestamp, T& data)
{
    boost::mutex::scoped_lock lock(mGlobalMutex);

    if (mBuffer.empty())
    {
        return false;
    }

    int endMark = 0;
    if (mBuffer.size() == mBuffer.capacity())
    {
        endMark = mIndex;
    }
    else
    {
        endMark = mBuffer.capacity() - 1;
    }

    long int tsDiffMin = std::numeric_limits<long int>::max();
    int mark = mIndex;
    do
    {
        long int tsDiff = std::abs(timestampDiff(timestamp, mBuffer.at(mark).first));
        if (tsDiff < tsDiffMin)
        {
            tsDiffMin = tsDiff;
        }
        else
        {
            data = mBuffer.at(mark).second;
            return true;
        }

        --mark;
        if (mark < 0)
        {
            mark += mBuffer.capacity();
        }
    }
    while (mark != endMark);

    return false;
}

template <class T>
bool
SensorDataBuffer<T>::nearest(uint64_t timestamp, T& dataBefore, T& dataAfter)
{
    boost::mutex::scoped_lock lock(mGlobalMutex);

    if (mBuffer.empty())
    {
        return false;
    }

    int endMark = 0;
    if (mBuffer.size() == mBuffer.capacity())
    {
        endMark = mIndex;
    }
    else
    {
        endMark = mBuffer.capacity() - 1;
    }

    int mark = mIndex;
    do
    {
        long int tsDiff = timestampDiff(timestamp, mBuffer.at(mark).first);
        if (tsDiff < 0)
        {
            if (mark == mIndex)
            {
                // no data after timestamp
                return false;
            }
            else
            {
                dataBefore = mBuffer.at(mark).second;
                dataAfter = mBuffer.at((mark + 1) % mBuffer.capacity()).second;

                return true;
            }
        }

        --mark;
        if (mark < 0)
        {
            mark += mBuffer.capacity();
        }
    }
    while (mark != endMark);

    return false;
}

template <class T>
bool
SensorDataBuffer<T>::current(T& data)
{
    boost::mutex::scoped_lock lock(mGlobalMutex);

    if (mBuffer.empty())
    {
        return false;
    }
    else
    {
        data = mBuffer.at(mIndex).second;
        return true;
    }
}

template <class T>
void
SensorDataBuffer<T>::push(uint64_t timestamp, const T& data)
{
    boost::mutex::scoped_lock lock(mGlobalMutex);

    if (!mBuffer.empty() && timestamp == mBuffer.back().first)
    {
        return;
    }

    if (mBuffer.size() == mBuffer.capacity())
    {
        mIndex = (mIndex + 1) % mBuffer.capacity();
        mBuffer.at(mIndex).first = timestamp;
        mBuffer.at(mIndex).second = data;
    }
    else
    {
        mBuffer.push_back(std::make_pair(timestamp, data));
        ++mIndex;
    }
}

template <class T>
bool
SensorDataBuffer<T>::find(uint64_t timestamp, T& data)
{
    boost::mutex::scoped_lock lock(mGlobalMutex);

    for (typename std::vector< std::pair<uint64_t, T> >::iterator it = mBuffer.begin(); it != mBuffer.end(); ++it)
    {
        if (it->first == timestamp)
        {
            data = it->second;

            return true;
        }
        else if (it->first > timestamp)
        {
            return false;
        }
    }

    return false;
}

}

#endif
