#include "opencv2\opencv.hpp"
#include<iostream>

using namespace std;
using namespace cv;
//By Shridhar Kini(Subscribe to: OpenCVstyle in YouTube)

string winName="Image";
Mat rgbIm;
int h=0;
int s=255;
int v=255;
void hMoved(int pos,void *x)
{
    h=pos;
    Mat hsvIm;
    cvtColor(rgbIm,hsvIm,CV_BGR2HSV);
    Scalar hsv=Scalar((uchar)h,(uchar)s,(uchar)v,0);
    hsvIm.setTo(hsv);
    cvtColor(hsvIm,rgbIm,CV_HSV2BGR);
    imshow(winName,rgbIm);
}
void sMoved(int pos,void *x)
{
    s=pos;
    Mat hsvIm;
    cvtColor(rgbIm,hsvIm,CV_BGR2HSV);
    Scalar hsv=Scalar((uchar)h,(uchar)s,(uchar)v,0);
    hsvIm.setTo(hsv);
    cvtColor(hsvIm,rgbIm,CV_HSV2BGR);
    imshow(winName,rgbIm);
}
void vMoved(int pos,void *x)
{
    v=pos;
    Mat hsvIm;
    cvtColor(rgbIm,hsvIm,CV_BGR2HSV);
    Scalar hsv=Scalar((uchar)h,(uchar)s,(uchar)v,0);
    hsvIm.setTo(hsv);
    cvtColor(hsvIm,rgbIm,CV_HSV2BGR);
    imshow(winName,rgbIm);
}
int main()
{
    rgbIm.create(500,500,CV_8UC3);
    rgbIm.setTo(Scalar(0,0,255));
    namedWindow(winName);
    imshow(winName,rgbIm);
    createTrackbar("H",winName,&h,180,hMoved,NULL);
    createTrackbar("S",winName,&s,255,sMoved,NULL);
    createTrackbar("V",winName,&v,255,vMoved,NULL);

    while(1)
    {
        int c=waitKey(0);
        if(c == 27)
        {
            break;
        }
    }
    waitKey(0);
    return 0;
}
