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

int thresh1 = 100;
int max_thresh1 = 255;
int thresh2 = 100;
int max_thresh2 = 255;

PID myPID(&PID_input_hor,&PID_output_hor,&Setpoint_hor,Kp_hor,Ki_hor,Kd_hor);

/*================================================
=============== MAIN CALCULATIONS ================
================================================*/

int main(int argc, char* argv[]){

	myPID.SetLimits(&Upper_Limit_hor,&Lower_Limit_hor);

	VideoCapture source1(1);
	VideoCapture source2(0);
	if (!source1.isOpened() && !source2.isOpened())  // if not success, exit program
		{
			cout << "Did you forget to switch the camera # ???" << endl;
			return -1;
		}

	double Width1 = source1.set(CV_CAP_PROP_FRAME_WIDTH,size[0]); //get the width of frames of the video
	double Height1 = source1.set(CV_CAP_PROP_FRAME_HEIGHT,size[1]); //get the height of frames of the video

	double Width2 = source2.set(CV_CAP_PROP_FRAME_WIDTH,size[0]); //get the width of frames of the video
	double Height2 = source2.set(CV_CAP_PROP_FRAME_HEIGHT,size[1]); //get the height of frames of the video

	/* Setting up the Windows for projections */
	namedWindow("2nd",CV_WINDOW_AUTOSIZE);
	namedWindow("Original",CV_WINDOW_AUTOSIZE);
	namedWindow("HSV",CV_WINDOW_AUTOSIZE);
	namedWindow("Thresholding",CV_WINDOW_AUTOSIZE);

	while(true){
		
		myPID.Compute();

		Mat frame1;
		Mat frame2;

		source1 >> frame1;
		source2 >> frame2;

		bool Success1 = source1.read(frame1);
		bool Success2 = source2.read(frame2);

		if (!Success2 || !Success1) //if not success, break loop
		{
			cout << "Sorry forgot how to camera.read" << endl;
			break;
		}

		/* HSV PROCESSING */

		Mat image1HSV;
		Mat image2HSV;
		cvtColor(frame1, image1HSV, COLOR_BGR2HSV);		// Conversion from BGR to HSV
		cvtColor(frame2, image2HSV, COLOR_BGR2HSV);		// Conversion from BGR to HSV

		/* THRESHOLDING */

		Mat image1THR;
		Mat image2THR;
		inRange(image1HSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), image1THR);
		inRange(image2HSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), image2THR);

		/* OPENING EROSION/DILUTION */

		erode(image1THR, image1THR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
		erode(image2THR, image2THR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
		dilate(image1THR, image1THR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
		dilate(image2THR, image2THR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));


		/* CLOSING EROSION/DILUTION */

		dilate(image1THR, image1THR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
		dilate(image2THR, image2THR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8))); 
		erode(image1THR, image1THR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));
		erode(image2THR, image2THR, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));


		/* PREPPING THE CONTOURS */

		/* BETTER KEEP THOSE REFERENCES AND POINTERS RIGHT */
		/* REALLY DOES MESS THINGS UP A LOT OF TIMES */

		vector<vector<Point> > contours1;
		vector<Vec4i> hierarchy1;
		Mat canny_output1;

		vector<vector<Point> > contours2;
		vector<Vec4i> hierarchy2;
		Mat canny_output2;

		/// Detect edges using canny
		Canny(image1THR, canny_output1, thresh1, thresh1*2, 3 );
		Canny(image2THR, canny_output2, thresh2, thresh2*2, 3 );
		/// Find contours
		findContours(canny_output1, contours1, hierarchy1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		findContours(canny_output2, contours2, hierarchy2, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

		vector<vector<Point>> contours_poly1(contours1.size());
  		vector<Point2f> center1(contours1.size());
  		vector<float> radius1(contours1.size());

  		vector<vector<Point>> contours_poly2(contours2.size());
  		vector<Point2f> center2(contours2.size());
  		vector<float> radius2(contours2.size());


		for( int i = 0; i < contours1.size(); i++ )
	    {
	        minEnclosingCircle( Mat (contours1[i]), center1[i], radius1[i] );
	        cout << int(center1[i].x) << "\t" << int(center1[i].y) << endl;
	        
	    }

	    for( int i = 0; i < contours2.size(); i++ )
	    {
	        minEnclosingCircle( Mat (contours2[i]), center2[i], radius2[i] );
	        cout << int(center2[i].x) << "\t" << int(center2[i].y) << endl;
	        
	    }	    

	    Scalar color = Scalar(255,255,255);

		Mat drawing1 = Mat::zeros( canny_output1.size(), CV_8UC3 );
		Mat drawing2 = Mat::zeros( canny_output2.size(), CV_8UC3 );
		for( int i = 0; i< contours1.size(); i++ )
		{
		    circle( drawing1, center1[i], (int)radius1[i], color, 2, 8, 0 );
		}

		for( int i = 0; i< contours2.size(); i++ )
		{
		    circle( drawing2, center2[i], (int)radius2[i], color, 2, 8, 0 );
		}
		
		namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
		imshow( "Contours", drawing1 );
		namedWindow( "Contours2", CV_WINDOW_AUTOSIZE );
		imshow( "Contours2", drawing2 );

		/* Displaying all three outputs */
		
		imshow("Original", frame1);
		imshow("2nd", frame2);
		imshow("HSV", image1HSV);
		imshow("Thresholding", image1THR);

		if (waitKey(30) == 27)
		{
			break; 
		}
	}

	return 0;

}

