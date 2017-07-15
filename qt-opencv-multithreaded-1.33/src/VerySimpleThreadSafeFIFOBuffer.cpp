/** @brief a Very Simple ThreadSafe FIFO Buffer
std::queue with cv::Mat is like a circular buffer.
Even if you store N frames and create N x Mat in the queue
only few blocks will be really allocated thanks to std::queue and Mat memory recycling
compare GetDataMemoryCount() and GetMatMemoryCount() with GetItemCount()
@warning THIS IS AN EXAMPLE, MANY IMPROVEMENT CAN BE DONE
*/

template <class T>
class VerySimpleThreadSafeFIFOBuffer
{
public:
    /* _maxSize=0 :size is unlimited (up to available memory) */
    VerySimpleThreadSafeFIFOBuffer(size_t _maxSize = 20) :
        maxBufSize(0), itemCount(0), maxSize(_maxSize)
    {}
    ~VerySimpleThreadSafeFIFOBuffer()
    {
        //calls destructor for all elements if any
        //if the elements are pointers, the pointed to objects are not destroyed.
        while (!m_buffer.empty())
            m_buffer.pop(); //you never should be here
    }

    /** @brief  Add an item to the queue
     class T should have a copy constructor overload
     */
    bool Push(const T &item)
    {
        //mutex is automatically released when
        //lock goes out of scope
        lock_guard<mutex> lock(m_queueMtx);

        size_t size = m_buffer.size();
        if (maxSize > 0 && size > maxSize)
            return false;

        m_buffer.push(item);//calls T::copy constructor
#ifdef _DEBUG
        //collect some stats
        itemCount++;
        maxBufSize = max(size, maxBufSize);
        MyData *dataPtr = &m_buffer.back();
        unsigned char *matPtr = m_buffer.back().GetMatPointer();
        dataMemoryCounter[dataPtr]++;
        matMemoryCounter[matPtr]++;
#endif
        return true;
    }

    /** @brief Get oldest queued item
     class T should have a =operator overload
     */
    bool Pop(T &item)
    {
        lock_guard<mutex> lock(m_queueMtx);
        if (m_buffer.empty())
            return false;
        item = m_buffer.front(); //calls T::=operator
        m_buffer.pop();
        return true;
    }

    size_t Size() {
        lock_guard<mutex> lock(m_queueMtx);
        return m_buffer.size();
    }
#ifdef _DEBUG
    size_t GetItemCount(){
        lock_guard<mutex> lock(m_queueMtx);
        return itemCount;
    }
    size_t GetBufSizeMax(){
        lock_guard<mutex> lock(m_queueMtx);
        return maxBufSize;
    }
    size_t GetDataMemoryCount(){
        lock_guard<mutex> lock(m_queueMtx);
        return dataMemoryCounter.size();
    }
    size_t GetMatMemoryCount(){
        lock_guard<mutex> lock(m_queueMtx);
        return matMemoryCounter.size();
    }
#endif
private:
    queue<T> m_buffer;
    mutex m_queueMtx;
    size_t maxBufSize, maxSize;
    size_t itemCount;
    map<void*, int> dataMemoryCounter;
    map<void*, int> matMemoryCounter;
};
