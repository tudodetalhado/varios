//
// Copyright (c) 2013 Juan Palacios juan.palacios.puyana@gmail.com
// Subject to the BSD 2-Clause License
// - see < http://opensource.org/licenses/BSD-2-Clause>
//

#ifndef CONCURRENT_QUEUE_
#define CONCURRENT_QUEUE_

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

template <typename T>
class FIFOBuffer
{
 public:

  T pop()
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
      cond_.wait(mlock);
    }
    auto val = queue_.front();
    queue_.pop();
    return val;
  }

  void pop(T& item)
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    while (queue_.empty())
    {
      cond_.wait(mlock);
    }
    item = queue_.front();
    queue_.pop();
  }

  void push(const T& item)
  {
    std::unique_lock<std::mutex> mlock(mutex_);
    queue_.push(item);
    mlock.unlock();
    cond_.notify_one();
  }
  FIFOBuffer()=default;
  FIFOBuffer(const FIFOBuffer&) = delete;            // disable copying
  FIFOBuffer& operator=(const FIFOBuffer&) = delete; // disable assignment

 private:
  std::queue<T> queue_;
  std::mutex mutex_;
  std::condition_variable cond_;
};

#endif



//#ifndef VERYSIMPLETHREADSAFEFIFOBUFFER_H
//#define VERYSIMPLETHREADSAFEFIFOBUFFER_H

//#include <queue>
//#include <thread>
//#include <mutex>

//#include <opencv2/opencv.hpp>

//template <class T>
//class FIFOBuffer
//{
//private:
//    std::queue<T> buffer;
//    std::mutex bufferMutex;
//    size_t maxBufSize, maxSize;

//public:
//    FIFOBuffer(size_t _maxSize = 20) :
//        maxBufSize(0), maxSize(_maxSize)
//    {
//    }

//    ~FIFOBuffer()
//    {
//        //calls destructor for all elements if any
//        //if the elements are pointers, the pointed to objects are not destroyed.
//        while (!buffer.empty())
//        {
//            buffer.pop(); //you never should be here
//        }
//    }

//    void push(const T item)
//    {
//        lock_guard<mutex> lock(bufferMutex);
//        if (buffer.size() > 20)
//        {
//            buffer.pop();
//            buffer.push(item);
//        }
//    }

//    T pop()
//    {
//        lock_guard<mutex> lock(bufferMutex);
//        T item = buffer.front();
//        buffer.pop();
//        return item;
//    }

//    size_t Size() {
//        lock_guard<mutex> lock(bufferMutex);
//        return buffer.size();
//    }
//};

//#endif // VERYSIMPLETHREADSAFEFIFOBUFFER_H
