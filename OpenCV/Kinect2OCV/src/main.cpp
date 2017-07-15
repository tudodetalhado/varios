#include "KinectSource.hpp"
//#include "helper.hpp"

using namespace K2OCV;
using namespace cv;

int main() {

    Mat dimg , iimg , cimg;
    CKinectSource* src = new CKinectSource();
    src->initSensor();
    src->initSourceReader(IR_S | COLOR_S | DEPTH_S | BODY_S | FACE_S);

    while (true) {
        cimg = src->getFrame(COLOR_F);
        iimg = src->getFrame(IR_F);
        dimg = src->getFrame(DEPTH_F);

        if (cimg.data) {
            imshow("color", cimg);
        }

        if (iimg.data) {
            imshow("ir", iimg);
        }

        if (dimg.data) {
            imshow("depth", dimg);
        }

        if (waitKey(20) == 27) {
            break;
        }
    }

    return 0;
}
