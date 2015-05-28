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

		erode(imageTHR, imageTHR, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imageTHR, imageTHR, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


		/* CLOSING EROSION/DILUTION */

		dilate(imageTHR, imageTHR, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); 
		erode(imageTHR, imageTHR, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


		/* Displaying all three outputs */

		imshow("Original", frame);
		imshow("HSV", imageHSV);
		imshow("Thresholding", imageTHR);

		if (waitKey(30) == 27)
		{
			break; 
		}

		cout << PID_output_hor << endl;
	}

	return 0;

}

