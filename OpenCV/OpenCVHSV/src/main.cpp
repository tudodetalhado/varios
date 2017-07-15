#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace cv;
using namespace std;

/// Global variables
Mat src,img,image;

int elem1 = 255;
int elem2 = 255;
int elem3 = 255;

int const max_elem = 500;


/** Function Headers */

void Hue( int, void* );
void Saturation( int, void* );
void Value( int, void* );
void Processing();

/** @function main */
int main( int argc, char** argv )
{
    /// Load an image
    namedWindow("image");

    createTrackbar( "Huse", "image",&elem1, max_elem,Hue);
    createTrackbar( "Saturation", "image",&elem2, max_elem,Saturation);
    createTrackbar( "Value", "image",&elem3, max_elem,Value);

    src = imread( "D:\\Qt\\OpenCV\\OpenCVHSV\\src\\1.jpg" );

    waitKey(0);
    return 0;
}

void Hue(int, void *)
{
    Processing();
}

void Saturation(int, void *)
{
    Processing();
}

void Value(int, void *)
{
    Processing();
}


void Processing()
{
    cvtColor(src,img,CV_RGB2HSV);

    int hue = elem1 - 255;
    int saturation = elem2 - 255;
    int value = elem3 - 255;

    for(int y=0; y<img.cols; y++)
    {
        for(int x=0; x<img.rows; x++)
        {
            int cur1 = img.at<Vec3b>(Point(y,x))[0];
            int cur2 = img.at<Vec3b>(Point(y,x))[1];
            int cur3 = img.at<Vec3b>(Point(y,x))[2];
            cur1 += hue;
            cur2 += saturation;
            cur3 += value;

            if(cur1 < 0) cur1= 0; else if(cur1 > 255) cur1 = 255;
            if(cur2 < 0) cur2= 0; else if(cur2 > 255) cur2 = 255;
            if(cur3 < 0) cur3= 0; else if(cur3 > 255) cur3 = 255;

            img.at<Vec3b>(Point(y,x))[0] = cur1;
            img.at<Vec3b>(Point(y,x))[1] = cur2;
            img.at<Vec3b>(Point(y,x))[2] = cur3;
        }
    }

    cvtColor(img,image,CV_HSV2RGB);
    imshow( "image", image );

}
