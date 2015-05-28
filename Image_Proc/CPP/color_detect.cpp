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



int center_frame[2] = {88,72};    // The (x,y) coordinates of the center of the frame with the resolution 640*480

int radius_frame = 30;        // The minimum desired radius of the object being tracked
int area_frame = 70;         // The desired area of the object that is being tracked
int radius_frame_max = 50;    // The maximum desired radius of the object being tracked
int area_frame_max = 150;   // The maximum desired area of the object being tracked
int size[2] = {240,180};			    // The resolution of the camera

float PID_input_hor = 0;
float PID_output_hor = 0;
float Setpoint_hor = 100;          // The desired position of the quadcopter in centimeters
float Kp_hor = 0.135;                // Proportionality constant for the PID calculation of the Distance
float Ki_hor = 0.001;
float Kd_hor = 0.03;               // Derivative constant for the PID calculation of Distance
float Upper_Limit_hor = 10.0;       // Upper limit for the PID output of the distance correction
float Lower_Limit_hor = -10.0;     // Lower limit for the PID output of the distance correction

bool first_calculation = false;
float Kp_ver = 0.125;


PID myPID(&PID_input_hor,&PID_output_hor,&Setpoint_hor,Kp_hor,Ki_hor,Kd_hor);


int main(){

	VideoCapture source(0);

	if (!source.isOpened())  // if not success, exit program
		{
			cout << "Did you forget to switch the camera # ???" << endl;
			return -1;
		}

	double Width = cap.set(CV_CAP_PROP_FRAME_WIDTH,size[0]);
	double Height = cap.set(CV_CAP_PROP_FRAME_HEIGHT,size[1]);

	while(true){
	
		myPID.SetLimits(&Upper_Limit_hor,&Lower_Limit_hor);
		myPID.Compute();

		

		cout << PID_output_hor << endl;
	}

}

