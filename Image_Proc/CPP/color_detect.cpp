/* OPEN CV LIBRARIES */

#include "opencv2/core/core.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"

/* PID LIBRARY */

#include "PID.h"

/* GENERAL PURPOSE LIBRARIES */

#include <iostream>
#include <cmath>

/* USING THE NAMESPACE FOR OPENCV OBJECTS */

using namespace cv;

/* USING NAMESPACE FOR THE GENERAL PURPOSE OBJECTS */

using namespace std;

/*================================================
=============== GENERAL DEFINITIONS ==============
================================================*/



int center_frame[2] = {320,240};    // The (x,y) coordinates of the center of the frame with the resolution 640*480

int radius_frame = 90;        // The minimum desired radius of the object being tracked
int area_frame = 25447;         // The desired area of the object that is being tracked
int radius_frame_max = 160;    // The maximum desired radius of the object being tracked
int area_frame_max = 80425;   // The maximum desired area of the object being tracked
int size[2] = {640,480};			    // The resolution of the camera

float PID_input_hor = 0;
float PID_output_hor = 0;
float Setpoint_hor = 320;          // The desired position of the quadcopter in centimeters
float Kp_hor = 0.135;                // Proportionality constant for the PID calculation of the Distance
float Ki_hor = 0.001;
float Kd_hor = 0.03;               // Derivative constant for the PID calculation of Distance
float Upper_Limit_hor = 10.0;       // Upper limit for the PID output of the distance correction
float Lower_Limit_hor = -10.0;     // Lower limit for the PID output of the distance correction



int lowH = 35;
int highH = 60;

int lowS = 50; 
int highS = 255;

int lowV = 50;
int highV = 255;

int largest_area = 15000;
int largest_contour_index;
cv::Rect bounding_rect;

int thresh = 100;
int max_thresh = 255;


PID myPID(&PID_input_hor,&PID_output_hor,&Setpoint_hor,Kp_hor,Ki_hor,Kd_hor);

int main(int argc, char* argv[]){

	myPID.SetLimits(&Upper_Limit_hor,&Lower_Limit_hor);

	VideoCapture source(0);
	if (!source.isOpened())  // if not success, exit program
		{
			cout << "Did you forget to switch the camera # ???" << endl;
			return -1;
		}

	double Width = source.set(CV_CAP_PROP_FRAME_WIDTH,size[0]); //get the width of frames of the video
	double Height = source.set(CV_CAP_PROP_FRAME_HEIGHT,size[1]); //get the height of frames of the video

	/* Setting up the Windows for projections */

	namedWindow("Original",CV_WINDOW_AUTOSIZE);
	namedWindow("HSV",CV_WINDOW_AUTOSIZE);
	namedWindow("Thresholding",CV_WINDOW_AUTOSIZE);

	RNG rng(12345);

	while(true){
		
		myPID.Compute();

		Mat frame;
		bool Success = source.read(frame);
		if (!Success) //if not success, break loop
		{
			cout << "Sorry forgot how to camera.read" << endl;
			break;
		}

		/* HSV PROCESSING */

		Mat imageHSV;
		cvtColor(frame, imageHSV, COLOR_BGR2HSV);		// Conversion from BGR to HSV

		/* THRESHOLDING */

		Mat imageTHR;

		inRange(imageHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), imageTHR);

		/* OPENING EROSION/DILUTION */

		erode(imageTHR, imageTHR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
		dilate(imageTHR, imageTHR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));


		/* CLOSING EROSION/DILUTION */

		dilate(imageTHR, imageTHR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8))); 
		erode(imageTHR, imageTHR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));


		/* PREPPING THE CONTOURS */

		/* BETTER KEEP THOSE REFERENCES AND POINTERS RIGHT */
		/* REALLY DOES MESS THINGS UP A LOT OF TIMES */

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		Mat canny_output;

		/// Detect edges using canny
		Canny(imageTHR, canny_output, thresh, thresh*2, 3 );
		/// Find contours
		findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

		vector<vector<Point>> contours_poly(contours.size());
  		vector<Point2f> center(contours.size());
  		vector<float> radius(contours.size());


		for( int i = 0; i < contours.size(); i++ )
	    {
	        minEnclosingCircle( Mat (contours[i]), center[i], radius[i] );
	    }

	    Scalar color = Scalar(255,255,255);

		Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
		for( int i = 0; i< contours.size(); i++ )
		{
		    circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );

		    
			cout << int(center[i].x) << "\t" << int(center[i].y) << endl;
		}
		
		namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
		imshow( "Contours", drawing );

		/* Displaying all three outputs */
		
		imshow("Original", frame);
		imshow("HSV", imageHSV);
		imshow("Thresholding", imageTHR);

		if (waitKey(30) == 27)
		{
			break; 
		}
	}

	return 0;

}

