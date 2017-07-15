#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <numeric>

#include <opencv/cv.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

using namespace std;
using namespace cv;

int _width = 560;
int _height = 420;

int _x, _y;

Mat lambda(2, 4, CV_32FC1);
vector<vector<Point>> tetragons;
int THRESHOLD_ANIMAL_VS_FLOOR = 200;
int THRESHOLD_WALL_VS_FLOOR = 80;
RNG rng(12345);

int main()
{
    VideoCapture cap("D:/Qt/OpenCV/FirstMorris/src/Morris1.avi");
    //cap.open("D:/Qt/_FirstMorris/src/Morris1.avi");
    Mat frameOriginal, frameColor, frameGray, frameBlur,
            frameThreshold, frameTemp;

    Mat frameCropped = Mat::zeros(_height, _height, CV_8UC1);
    Mat frameTrack = Mat::zeros(_width -190, _height-60, CV_8UC3);
    Mat frameLine = Mat::zeros(_width -190, _height-60, CV_8UC3);
    vector<vector<Point>> contours;

//    int totalFrameNumber = (int)cap.get(CV_CAP_PROP_FRAME_COUNT);

//    // give camera some extra time to get ready:
//    std::this_thread::sleep_for(std::chrono::milliseconds(200));

//    if (!cap.isOpened()) {
//         std::cout << "Unable to read stream from specified device." << std::endl;
//         return 1;
//    }

    while(true)
    {
        cap.read(frameOriginal);

        if (frameOriginal.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        resize(frameOriginal, frameOriginal, Size(_width,_height), 0, 0, INTER_CUBIC);
        frameColor = frameOriginal(Rect(100, 40, _width -190, _height-60));
        cvtColor(frameColor, frameGray, COLOR_BGR2GRAY);

        Mat frameMask(frameGray.rows, frameGray.cols, CV_8UC1, Scalar(0,0,0));
        ellipse(frameMask, Point(180,180), Size(140,140), 0, 0, 360, Scalar( 255, 255, 255), -1, 8 );
        Mat frameRedondo;
        bitwise_and(frameGray,frameMask,frameRedondo);

        imshow("Recorte", frameRedondo);

        GaussianBlur(frameRedondo, frameBlur, Size(25,25), 0);
        threshold(frameBlur, frameThreshold, THRESHOLD_ANIMAL_VS_FLOOR, 255, THRESH_BINARY);
        findContours(frameThreshold, contours, RETR_TREE, CHAIN_APPROX_NONE);

        vector<int> indices(contours.size());
        iota(indices.begin(), indices.end(), 0);

        //O primeiro contorno no índice será o maior.
        sort(indices.begin(), indices.end(), [&contours](int lhs, int rhs) {
            return contours[lhs].size() > contours[rhs].size();
        });

        if (indices.size() > 0){

           frameTemp = Mat::zeros(_width -190, _height-60, CV_8UC3);
           drawContours(frameTemp, contours, indices[0], Scalar(0, 150, 0), 2);

           circle(frameTemp,Point(180,180),140, Scalar(0,0,255),3);

            vector<Point> contour = contours[0];
            Moments mmts = moments(contour,false);
            int mx = (int)(mmts.m10 / mmts.m00);
            int my = (int)(mmts.m01 / mmts.m00);

            if (mx > 0 && my > 0)
            {
                Point centroid = Point(mx, my);
                if (_x > 0 && _y > 0)
                {
                    Point origem = Point(_x, _y);
                    line(frameLine, centroid, origem, Scalar(150, 0, 0), 1);
                    add(frameLine, frameTrack, frameTrack);
                    addWeighted(frameTemp, 0.7, frameTrack, 1, 1, frameTemp);
                }
                _x = centroid.x;
                _y = centroid.y;
            }
        }

        imshow("Track", frameTemp);
        imshow("Original", frameOriginal);

        int c = cvWaitKey(10);
        if((char)c==27 ) break;
    }
    cap.release();
    return EXIT_SUCCESS;
}
