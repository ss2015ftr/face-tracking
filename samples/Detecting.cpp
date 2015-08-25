#include "Detecting.hpp"


 using namespace std;
 using namespace cv;


void Detector::Detect(const Mat& fr, std::vector<Rect> &fcs)
{
	  Mat frame_gray;
	  cvtColor( fr, frame_gray, CV_BGR2GRAY );
      equalizeHist( frame_gray, frame_gray );
	  face_cascade.detectMultiScale( frame_gray, fcs, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
}


