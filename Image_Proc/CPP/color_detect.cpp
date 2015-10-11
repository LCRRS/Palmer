/*===============================================================
=================         COLOR TRACKING          ===============
===============================================================*/

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

#include "PID.h" // PID LIBRARY

#include <iostream> // GENERAL PURPOSE LIBRARIES
#include <cmath> // GENERAL PURPOSE LIBRARIES

/*=============================================
====== Uncomment the following if you are =====
======= planning to use self-made POSIX =======
============   Serial Communication   =========
==============================================*/

// #include <termios.h>
// #include <fcntl.h>
// #include <sys/ioctl.h>
// #include <sys/stat.h>
// #include <sys/types.h>

#include <string.h>
#include <unistd.h> 
#include <stdio.h>

/*=============================================
=== In order to be able to use SerialStream ===
==== you need to install libserial locally ====
==============================================*/
#include <SerialStream.h> //POSIX and termios based library for Serial communication

using namespace cv; // USING THE NAMESPACE FOR OPENCV OBJECTS
using namespace std; // USING NAMESPACE FOR THE GENERAL PURPOSE OBJECTS
using namespace LibSerial; // USING NAMESPACE FOR THE Serial

/*================================================
=============== GENERAL DEFINITIONS ==============
================================================*/

int center_frame[2] = {320,240};  // The (x,y) coordinates of the center of the frame with the resolution 640*480
int radius_frame = 90;        	 // The minimum desired radius of the object being tracked
int area_frame = 25447;         // The desired area of the object that is being tracked
int radius_frame_max = 160;    // The maximum desired radius of the object being tracked
int area_frame_max = 80425;   // The maximum desired area of the object being tracked
int size[2] = {640,480};	 // The resolution of the camera

float PID_input_hor = 0;
float PID_output_hor = 0;
float Setpoint_hor = 320;          // The desired position of the quadcopter in centimeters
float Kp_hor = 0.135;              // Proportionality constant for the PID calculation of the Distance
float Ki_hor = 0.001;			   // Integral Constant for the PID calculation of Distance
float Kd_hor = 0.03;               // Derivative constant for the PID calculation of Distance
float Upper_Limit_hor = 10.0;      // Upper limit for the PID output of the distance correction
float Lower_Limit_hor = -10.0;     // Lower limit for the PID output of the distance correction
PID myPID(&PID_input_hor,&PID_output_hor,&Setpoint_hor,Kp_hor,Ki_hor,Kd_hor); //Initializing the PID instance

int lowH = 35;			//Low pass value for Hue
int highH = 60;			//High pass value for Hue
int lowS = 50;			//Low pass value for Saturation
int highS = 255;		//High pass value for Saturation
int lowV = 50;			//Low pass value for Value
int highV = 255;		//High pass value for Value

int largest_area = 15000;
int largest_contour_index;
int thresh = 100;
int max_thresh = 255;

int main(int argc, char* argv[]){

	SerialStream serial_port;			// Initialize Serial instance
	serial_port.Open("/dev/ttyACM0");	// Binding Serial to the Port
	serial_port.SetBaudRate( SerialStreamBuf::BAUD_9600 );	//Baud rate specification
	serial_port.SetVTime(1);	// Input Flow Control. Time to wait for data in tenths of a second
	serial_port.SetVMin(0);		// Input Flow Control. Minimum number of characters to read

	myPID.SetLimits(&Upper_Limit_hor,&Lower_Limit_hor); // Setting the upper and lower limit for the PID calculation

	VideoCapture source(0);	 // Capturng on the default port
	if (!source.isOpened())  // if not success, exit program
		{
			cout << "Did you forget to switch the camera # ???" << endl;
			return -1;
		}

	double Width = source.set(CV_CAP_PROP_FRAME_WIDTH,size[0]); //get the width of frames of the video
	double Height = source.set(CV_CAP_PROP_FRAME_HEIGHT,size[1]); //get the height of frames of the video

	while(true){
		
		myPID.Compute();	//Compute the PID values

		Mat original_frame;
		bool Success = source.read(original_frame);
		if (!Success)	//if not success, break loop
		{
			cout << "Sorry forgot how to camera.read" << endl;
			break;
		}

		/* HSV PROCESSING */
		Mat imageHSV;
		cvtColor(original_frame, imageHSV, COLOR_BGR2HSV);		// Conversion from BGR to HSV

		/* THRESHOLDING */
		Mat imageTHR;
		inRange(imageHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), imageTHR);	//Thresholding the HSV in accordance with the previously declared values

		/* OPENING EROSION/DILUTION */
		erode(imageTHR, imageTHR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
		dilate(imageTHR, imageTHR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
		dilate(imageTHR, imageTHR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8))); 
		erode(imageTHR, imageTHR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));

		/* PREPPING THE CONTOURS */
		/* BETTER KEEP THOSE REFERENCES AND POINTERS RIGHT */
		/* REALLY DOES MESS THINGS UP A LOT OF TIMES */
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		Mat canny_output;

		/* CANNY EDGE DETECTION */
		Canny(imageTHR, canny_output, thresh, thresh*2, 3 ); // Detect edges using canny
		findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) ); // Find contours

		vector<vector<Point>> contours_poly(contours.size());
  		vector<Point2f> center(contours.size());
  		vector<float> radius(contours.size());

		for( int i = 0; i < contours.size(); i++ )
	    {
	        minEnclosingCircle( Mat (contours[i]), center[i], radius[i] ); // Allows better estimation of the real size of the object, independent of the rotation
	        cout << int(center[i].x) << "\t" << int(center[i].y) << endl;
	        serial_port << int(center[i].x) << "\n";
	    }

	    /*==================================================================
	    ========================	  VISUALS		========================
		====================================================================
		============  Uncomment the following code in order	================
		============	to see the visual representation	================
		==================================================================*/

	    // Scalar color = Scalar(255,255,255); // The color of the drawn contour
		// Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3 );
		// for( int i = 0; i< contours.size(); i++ )
		// {
		//     circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
		// }
		
		// namedWindow("Original",CV_WINDOW_AUTOSIZE);
		// namedWindow("HSV",CV_WINDOW_AUTOSIZE);
		// namedWindow("Thresholding",CV_WINDOW_AUTOSIZE);
		// namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
		// imshow( "Contours", drawing );		
		// imshow("Original", frame);
		// imshow("HSV", imageHSV);
		// imshow("Thresholding", imageTHR);

		if (waitKey(30) == 27)
		{
			break; 
		}
	}

	return 0;

}

