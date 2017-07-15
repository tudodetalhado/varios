#include <queue>
#include <thread>
#include <mutex>

class MyData
{
public:
    //we use a struct to have simple copy without forget some members
    struct MyDataFlatMembers {
        int64 timestamp = 0;
        int64 frameNum = 0;
        //...
    };

    MyDataFlatMembers vars;
    Mat ir, depth, bgr;

    MyData() { }

    /** @brief class destructor.(USED BY queue::Pop)
     delete object pointer here if you have
     */
    ~MyData() { }

    /** @brief overload for the copy constructor (USED BY buffer::Push)*/
    MyData(const MyData& src) {
        Clone(src);//call clone Mat
    }

    /** @brief Assignment (=) Operator overloading (USED BY from buffer::Pop)
    Because buffer::Pop calls queue:pop who calls MyData destructor we will lost all members
    pointers.
    We are safe with cv::Mat, clone is not needed because of internal cv::Mat memory counter.
    @warning  This operator is not needed ONLY IF object members pointers are only cv::Mat
    in case we have some member pointer object we need to clone them
    */
    MyData& operator=(const MyData&src)
    {
        if (this == &src) return *this;
        Copy(src);
        return *this;
    }

    // this is just to collect stats
    unsigned char* GetMatPointer(){ return ir.data; }

private:
    /** @brief Copy all members (for Mat member =operator will be used)
    USED BY =operator
    */
    void Copy(const MyData& src)
    {
        vars = src.vars; //Copy all flat members using struct copy
        ir = src.ir;
        depth = src.depth;
        bgr = src.bgr;
    }

    /** @brief Copy all members (for Mat clone will be used)
    USED BY copy constructor
    */
    void Clone(const MyData& src)
    {
        vars = src.vars;//Copy all flat members using struct copy
        src.ir.copyTo(ir);
        src.depth.copyTo(depth);
        src.bgr.copyTo(bgr);
    }
};
