#include "faceDetectionHaar.h"

int faceDetector::loadClassifier() {
    face.load( "C:/OpenCV/sources/data/haarcascades/haarcascade_frontalface_alt.xml");
    if(!face.load("C:/OpenCV/sources/data/haarcascades/haarcascade_frontalface_alt.xml"))
		return -1;
	return 0;
}

int faceDetector::detection(cv::Mat &image) {
	if(!image.data) {
		return -1;
	}
	face.detectMultiScale(image,detectedFace,1.1,3,CV_HAAR_FIND_BIGGEST_OBJECT|CV_HAAR_SCALE_IMAGE,cv::Size(30,30));
	if(detectedFace.size() == 0) {
		return -1;
	}
	return 0;
}
