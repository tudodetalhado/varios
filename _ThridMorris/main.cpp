#include <opencv/cv.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <vector>
#include <numeric>
#include <iostream>

using namespace std;
using namespace cv;

int _width = 560;
int _height = 420;

int _x, _y;

Mat lambda(2, 4, CV_32FC1);
vector<vector<Point>> tetragons;
int THRESHOLD_ANIMAL_VS_FLOOR = 110;
RNG rng(12345);

static Mat frameBackground; // = Mat(_height,_width, CV_8UC1, Scalar(0));

void processarFundo()
{
    VideoCapture cap;
    cap.open("D:/Expt-o/QtCpp/ThridMorris/Morris3.mpg");

    Mat frameOriginal;
    Mat frameTotal = Mat(_height,_width, CV_32FC1, Scalar(0));

    int index = 0;

    while(true && index < 2)
    {
        cap.read(frameOriginal);
        if (frameOriginal.empty()) {
            cerr << "Acabou o vídeo!\n";
            break;
        }

        resize(frameOriginal, frameOriginal, Size(_width,_height), 0, 0, INTER_CUBIC);

        if (frameBackground.empty())
        {
            cvtColor(frameOriginal, frameBackground, CV_BGR2GRAY);
        }

        Mat frameFloat = Mat(_height,_width, CV_32FC1, Scalar(0));
        Mat frameProcessamento = Mat(_height,_width, CV_8UC1, Scalar(0));

        cvtColor(frameOriginal, frameProcessamento, CV_BGR2GRAY);

        //adaptiveThreshold(frameProcessamento, frameProcessamento, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11,2);

        frameProcessamento.convertTo(frameFloat, CV_32FC1);
        frameTotal +=  frameFloat;
        Mat frameFinal = (frameTotal / index++);
        frameFinal.convertTo(frameBackground, CV_8UC1);

//        imshow("Processando", frameBackground);

//        waitKey(5);
    }
}

int main()
{

    processarFundo();

    if (!frameBackground.empty())
    {

        VideoCapture cap;
        cap.open("D:/Expt-o/QtCpp/ThridMorris/Morris3.mpg");
        Mat frameOriginal, frameColor, frameGray, frameBlur,
                frmThreshold, frameTemp, frameAnterior, frmDiferenca;

        Mat frameTrack = Mat(_width, _height, CV_8UC3, Scalar(0));
        Mat frameLine = Mat::zeros(_width, _height, CV_8UC3);
        vector<vector<Point>> contours;

        Mat frameForeground = Mat(frameBackground.size().height,
                                  frameBackground.size().width, CV_8UC1, Scalar(0));

        while(true)
        {
            cap.read(frameOriginal);
            if (frameOriginal.empty()) {
                cerr << "ERROR! blank frame grabbed\n";
                break;
            }

            resize(frameOriginal, frameOriginal, Size(_width,_height), 0, 0, INTER_CUBIC);

            imshow("Original", frameOriginal);

            cvtColor(frameOriginal, frameGray, COLOR_BGR2GRAY);

            if (frameAnterior.empty()){
                frameOriginal.copyTo(frameAnterior);
            }

//            dilate(frameGray, frameGray, getStructuringElement(MORPH_DILATE, Size(5, 5)));
//            absdiff(frameOriginal, frameAnterior, frmDiferenca);
//            int LEVEL = 10;
//            threshold(frmDiferenca, frmThreshold, LEVEL, 255, THRESH_BINARY);
//            blur(frmThreshold, frmThreshold, Size(20, 20));
//            threshold(frmThreshold, frmThreshold, LEVEL, 255, THRESH_BINARY);


            //GaussianBlur(frameGray, frameBlur, Size(9,9), BORDER_CONSTANT);
            //threshold((frameBlur - frameBackground), frameForeground, 30, 255, THRESH_BINARY);
            //adaptiveThreshold((frameGray - frameBackground), frameForeground, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11,0);
            //adaptiveThreshold(frameBlur, frameForeground, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11,0);
            //adaptiveThreshold(frameForeground, frameForeground, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11,2);
            //absdiff(frameGray, frameBackground, frameForeground);
            //morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=4)
            //dilate(frameForeground, frameForeground, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
            //erode(frameForeground, frameForeground, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
            //threshold(frameForeground, frameForeground, 20, 255, THRESH_BINARY);
            //adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
            //adaptiveThreshold(frameForeground, frameForeground, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11,2);


            //imshow("Background", frameBackground);  // * 1.2;
            imshow("Foreground", frameOriginal * 1.5);

            //frameOriginal.copyTo(frameAnterior);

//            resize(frameOriginal, frameOriginal, Size(_width,_height), 0, 0, INTER_CUBIC);
//            frameColor = frameOriginal(Rect(85, 0, _height, _height));
//            cvtColor(frameColor, frameGray, COLOR_BGR2GRAY);

//            Mat frameMask(frameGray.rows, frameGray.cols, CV_8UC1, Scalar(255));
//            ellipse(frameMask, Point(205,200), Size(190,190), 0, 0, 360, Scalar(0), -1, 8 );
//            Mat frameRedondo;
//            bitwise_or(frameGray,frameMask,frameRedondo);

//            imshow("Recorte", frameRedondo);

//            GaussianBlur(frameRedondo, frameBlur, Size(25,25), 0);
//            threshold(frameBlur, frameThreshold, THRESHOLD_ANIMAL_VS_FLOOR, 255, THRESH_BINARY_INV);
//            findContours(frameThreshold, contours, RETR_TREE, CHAIN_APPROX_NONE);

//            imshow("Threshold", frameThreshold);

//            vector<int> indices(contours.size());
//            iota(indices.begin(), indices.end(), 0);

//            //O primeiro contorno no índice será o maior.
//            sort(indices.begin(), indices.end(), [&contours](int lhs, int rhs) {
//                return contours[lhs].size() > contours[rhs].size();
//            });

//            if (indices.size() > 0){

//                frameTemp = Mat::zeros(_height, _height, CV_8UC3);
//                drawContours(frameTemp, contours, indices[0], Scalar(0, 150, 0), 2);

//                circle(frameTemp,Point(205,200), 190, Scalar(0,0,255),3);

//                vector<Point> contour = contours[0];
//                Moments mmts = moments(contour,false);
//                int mx = (int)(mmts.m10 / mmts.m00);
//                int my = (int)(mmts.m01 / mmts.m00);

//                //            if (mx > 0 && my > 0)
//                //            {
//                //                Point centroid = Point(mx, my);
//                //               if (_x > 0 && _y > 0)
//                //                {
//                //                    Point origem = Point(_x, _y);
//                //                    line(frameLine, centroid, origem, Scalar(150, 0, 0), 1);
//                //                    add(frameLine, frameTrack, frameTrack);
//                //                    addWeighted(frameTemp, 0.7, frameTrack, 1, 1, frameTemp);
//                //                }
//                //                _x = centroid.x;
//                //                _y = centroid.y;
//                //            }
//                imshow("Track", frameTemp);
//            }

            //drawContours(frameColor, tetragons, 0, Scalar(0, 0, 255), 2);


            int c = cvWaitKey(10);
            if((char)c==27 ) break;
        }
        cap.release();
    }
    return EXIT_SUCCESS;
}




//int main()
//{

//    processarFundo();

//    VideoCapture cap;
//    cap.open("D:/Expt-o/QtCpp/ThridMorris/Morris3.mpg");
//    Mat frameOriginal, frameColor, frameGray, frameBlur,
//            frameThreshold, frameTemp;

//    Mat frameTrack = Mat(_width, _height, CV_8UC3, Scalar(0));
//    Mat frameLine = Mat::zeros(_width, _height, CV_8UC3);
//    vector<vector<Point>> contours;

//    while(true)
//    {
//        cap.read(frameOriginal);
//        if (frameOriginal.empty()) {
//            cerr << "ERROR! blank frame grabbed\n";
//            break;
//        }

//        //imshow("Original", frameOriginal);

//        resize(frameOriginal, frameOriginal, Size(_width,_height), 0, 0, INTER_CUBIC);
//        frameColor = frameOriginal(Rect(85, 0, _height, _height));
//        cvtColor(frameColor, frameGray, COLOR_BGR2GRAY);

//        Mat frameMask(frameGray.rows, frameGray.cols, CV_8UC1, Scalar(255));
//        ellipse(frameMask, Point(205,200), Size(190,190), 0, 0, 360, Scalar(0), -1, 8 );
//        Mat frameRedondo;
//        bitwise_or(frameGray,frameMask,frameRedondo);

//        imshow("Recorte", frameRedondo);

//        GaussianBlur(frameRedondo, frameBlur, Size(25,25), 0);
//        threshold(frameBlur, frameThreshold, THRESHOLD_ANIMAL_VS_FLOOR, 255, THRESH_BINARY_INV);
//        findContours(frameThreshold, contours, RETR_TREE, CHAIN_APPROX_NONE);

//        imshow("Threshold", frameThreshold);

//        vector<int> indices(contours.size());
//        iota(indices.begin(), indices.end(), 0);

//        //O primeiro contorno no índice será o maior.
//        sort(indices.begin(), indices.end(), [&contours](int lhs, int rhs) {
//            return contours[lhs].size() > contours[rhs].size();
//        });

//        if (indices.size() > 0){

//            frameTemp = Mat::zeros(_height, _height, CV_8UC3);
//            drawContours(frameTemp, contours, indices[0], Scalar(0, 150, 0), 2);

//            circle(frameTemp,Point(205,200), 190, Scalar(0,0,255),3);

//            vector<Point> contour = contours[0];
//            Moments mmts = moments(contour,false);
//            int mx = (int)(mmts.m10 / mmts.m00);
//            int my = (int)(mmts.m01 / mmts.m00);

////            if (mx > 0 && my > 0)
////            {
////                Point centroid = Point(mx, my);
////               if (_x > 0 && _y > 0)
////                {
////                    Point origem = Point(_x, _y);
////                    line(frameLine, centroid, origem, Scalar(150, 0, 0), 1);
////                    add(frameLine, frameTrack, frameTrack);
////                    addWeighted(frameTemp, 0.7, frameTrack, 1, 1, frameTemp);
////                }
////                _x = centroid.x;
////                _y = centroid.y;
////            }
//            imshow("Track", frameTemp);
//        }

//        //drawContours(frameColor, tetragons, 0, Scalar(0, 0, 255), 2);


//        int c = cvWaitKey(10);
//        if((char)c==27 ) break;
//    }
//    cap.release();
//    return EXIT_SUCCESS;
//}
