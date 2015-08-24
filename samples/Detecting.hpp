#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;
class Detector{
	 String face_cascade_name;
     CascadeClassifier face_cascade;
public:
	 Detector( String F_c_n = "haarcascade_frontalface_alt.xml")
	 {
          face_cascade_name = F_c_n;
		  RNG rng(12345);
	 }
	 void Detect(const Mat& fr, std::vector<Rect> &fcs);
};
