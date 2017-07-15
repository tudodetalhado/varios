#ifndef QUEUE_H
#define QUEUE_H

#include <queue>
#include <opencv2/opencv.hpp>

template <typename T>
class Queue : public queue
{
public:
    typedef boost::mutex::scoped_lock lock;

    CircularBuffer()
    {
    }

    CircularBuffer(int tamanho)
    {
        buffer.set_capacity(tamanho);
    }

    void push(T object)
    {
        lock lk(monitor);
        buffer.push_back(object);
        bufferNotEmpty.notify_one();
    }

    T pop()
    {
        lock lk(monitor);
        while (buffer.empty())
        {
            bufferNotEmpty.wait(lk);
        }
        T object = buffer.front();
        buffer.pop_front();
        return object;
    }
    void clear()
    {
        lock lk(monitor);
        buffer.clear();
    }

    int size()
    {
        lock lk(monitor);
        return buffer.size();
    }

    void setCapacity(int capacity)
    {
        lock lk(monitor);
        buffer.set_capacity(capacity);
    }

private:
    boost::condition bufferNotEmpty;
    boost::circular_buffer<T> buffer;
    boost::mutex monitor;

};

#endif // QUEUE_H
