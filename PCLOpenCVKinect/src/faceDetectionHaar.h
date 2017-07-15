// OpenCV
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class faceDetector {

//friend class KinectViewer;

public:
	cv::CascadeClassifier face; // Object
	std::vector<cv::Rect> detectedFace; // Detected face

public:
	int loadClassifier(); // Load classifier
	int detection(cv::Mat &image); // Face detection		
};