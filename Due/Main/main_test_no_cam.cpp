/*==========================================
========       MAIN FILE      =========
============================================
=== Using this file does not output any  ===
=== Serial readings for the benefit of   ===
=== performance. If you want to receive  ===
=== information any information while    ===
=== the program executes - use the test  ===
=== file.                                ===
=== DO NOT FORGET TO UPDATE THE VALUES!  ===
==========================================*/


/*=========================================
===========================================
=============       PALMR       ===========
===========================================
==  Sign value (-/+) in the diagram      ==
==  represent the gyroscope readings     ==
==  when the respective arm is tilted up ==
===========================================
==  The M#-P# represent the Arduino Pins ==
==  to which respective ESC is attached  ==
===========================================
				yaw ---> +
		  pitch -           roll +
			CW               CCW
		  M1-P6             M2-P9
			 \    FORWARD    /
			  \      ^      /
			   \     |     /
				\ _______ /
				 |CENTRAL|
				 |CONTROL|
				 |_______|
				/         \
			   /           \
			  /             \
			 /               \
		  M4-P11           M3-P10
		   CCW               CW
		  roll -           pitch +
==========================================*/
#include <PID_v1.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif

#define OUTPUT_READABLE_YAWPITCHROLL

#define MIN_SIGNAL 1000
#define LOOPTIME 10
#define START_SPEED 1450    // Speed at which the thrust is not yet sufficient to lift up the drone
#define BASE_SPEED 1745     // Standard operational speed of the drone
#define TAKEOFF_SPEED 1650  // Speed at which the drone has suddenly taken off the ground
#define SAMPLE_TIME 10

#define KP_PR 7.05
#define KI_PR 0.0
#define KD_PR 0.9
#define KP_YAW 6.0
#define KI_YAW 2.3
#define KD_YAW 0.0

#define LOWER_LIMIT_YAW -39
#define UPPER_LIMIT_YAW 39
#define LOWER_LIMIT_PR -48   // The lowest possible output that the PID can produce
#define UPPER_LIMIT_PR 48 // The maximum possible output that the PID can produce (anything higher will be set back to this value)

#define ERROR_CODE_GYRO_CONNECTION 	4
#define ERROR_CODE_GYRO_DMP 		3
#define ERROR_TIME_GYRO_CONNECTION 	300
#define ERROR_TIME_GYRO_DMP 		600
#define NO_ERROR 			0
#define LED_RED_ALARM 		53
#define LED_BLUE_WARNING 	49
#define LED_GREEN_OK 		51

MPU6050 mpu;

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;

float speed_myservo1, speed_myservo2, speed_myservo3, speed_myservo4;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];

bool initiation_count = false; // Used to initiate motor start/warmup routine inside the loop function only once
bool yaw_orientation_update = true; //Used to determining whether the yaw axis should update its orientation setpoing for the PID

float ypr0;
float ypr1;
float ypr2;

/*=================================================
================= PID CONTROLLER ==================
=================================================*/

float pid_setPoint_yaw = 0;    // The desired value of the gyroscope. PID produces as an output until the value is reached
float pid_setPoint_pitch = 0;
float pid_setPoint_roll = 0;

float pid_input_yaw, pid_output_yaw;
float pid_input_pitch, pid_output_pitch;
float pid_input_roll, pid_output_roll;   // PID containers to hold the gyroscope reading and the correction output respectively

PID myPID_yaw(&pid_input_yaw, &pid_output_yaw, &pid_setPoint_yaw, KP_YAW, KI_YAW, KD_YAW, DIRECT);
PID myPID_pitch(&pid_input_pitch, &pid_output_pitch, &pid_setPoint_pitch, KP_PR, KI_PR, KD_PR, DIRECT);
PID myPID_roll(&pid_input_roll, &pid_output_roll, &pid_setPoint_roll, KP_PR, KI_PR, KD_PR, DIRECT);        //PID class object that is associated with respected variables for the roll plane

int takeoff_speed = START_SPEED; // Initializing the takeoff variables

/*=============================================
======          AIR STABILIZATION        ======
=============================================*/

void stabilize(float current_speed){

	pid_input_yaw = ypr0;
	pid_input_pitch = ypr1;
	pid_input_roll = ypr2;

	myPID_yaw.Compute(&pid_setPoint_yaw);
	myPID_pitch.Compute(&pid_setPoint_pitch);
	myPID_roll.Compute(&pid_setPoint_roll);

	// Speed is being calculated from the preset base_speed compensated with the respective pid oututs
	// and is adjusted to move in a vertical plane in accordance with the object being tracked (if present)

	speed_myservo1 = current_speed - pid_output_pitch - pid_output_yaw;
	speed_myservo2 = current_speed + pid_output_roll + pid_output_yaw;
	speed_myservo3 = current_speed + pid_output_pitch - pid_output_yaw;
	speed_myservo4 = current_speed - pid_output_roll + pid_output_yaw;

	indivSpeed(myservo1, speed_myservo1);
	indivSpeed(myservo2, speed_myservo2);
	indivSpeed(myservo3, speed_myservo3);
	indivSpeed(myservo4, speed_myservo4);
}

/*=================================================
=================== SERIAL READ ===================
=================================================*/
/*	
int x_axis = 0;
int y_axis = 0;

void serial_read() {
    
    if (Serial.find("s")) {
    	x_axis = Serial.parseInt(); // parses numeric characters before the comma
    	y_axis = Serial.parseInt();// parses numeric characters after the comma
    }


	
	//The point of the following if statements is to stop the ypr0 values from updating when we are
	//not receiving values from the pi.  This is to prevent drifting in the yaw axis when the pi values are not present.
	
	if(x_axis <= 10 && x_axis >= -10){ //Check if receiving values
		if(yaw_orientation_update != false){ //check if this is the first cycle without a value
			pid_setPoint_yaw = ypr0;  //Since this is the first cycle, take the current value to keep until we get another reading from the pi
			yaw_orientation_update = false; //since not receiving values, keep old ypr0
		}
	}
	else{
		pid_setPoint_yaw = ypr0 + x_axis; //if an object is being tracked sets the new pid target value
		yaw_orientation_update = true; //since receiving values, update ypr0
	}
	pid_setPoint_pitch = 0;
	pid_setPoint_roll = 0;
}
*/
/*============================================================
===============         WARM-UP ROUTINE       ================
============================================================*/

void warmup(){
	Serial.println("inside warmup");
	myPID_yaw.SetOutputLimits(LOWER_LIMIT_YAW, UPPER_LIMIT_YAW);
	myPID_pitch.SetOutputLimits(LOWER_LIMIT_PR, UPPER_LIMIT_PR);
	myPID_roll.SetOutputLimits(LOWER_LIMIT_PR, UPPER_LIMIT_PR);

	myPID_yaw.SetSampleTime(SAMPLE_TIME); //Time to pass between the values of PID are recomputed (in milliseconds)
	myPID_pitch.SetSampleTime(SAMPLE_TIME);
	myPID_roll.SetSampleTime(SAMPLE_TIME);

	while (!initiation_count){
		//serial_read();
		//if (Serial.find("s")){
			for (int speed = 1000; speed < BASE_SPEED; speed++){
				setSpeed(speed);
				delay(10);
			}
			initiation_count = true;
			//setSpeed(START_SPEED);
			//setSpeed(TAKEOFF_SPEED);
		//}
	}
}

/*==============================================================
===                     AUTOMATED TAKEOFF                    ===
================================================================
===      This function is used to automate the takeoff       ===
===                      of the quadcopter                   ===
==============================================================*/

void takeoff(){
	Serial.println("inside takeoff");
	get_ypr();
	pid_input_yaw = ypr0;
	pid_input_pitch = ypr1;
	pid_input_roll = ypr2;
	if(ypr1 < -2 || ypr1 > 2 || ypr2 <-2 || ypr2 > 2){
		stabilize(takeoff_speed);
	}
	else{
		takeoff_speed++;
	}

}

/*==============================================================
===               SET SPEED ON ALL MOTORS                    ===
================================================================
===    This function is used to set speed of all motors      ===
===                  to the same value                       ===
==============================================================*/

void setSpeed(float speed){
	myservo1.writeMicroseconds(speed);
	myservo2.writeMicroseconds(speed);
	myservo3.writeMicroseconds(speed);
	myservo4.writeMicroseconds(speed);
}

/*==============================================================
===               SET SPEED ON INDIVIDUAL MOTORS             ===
================================================================
===    Individual Set Speed function used to set speed of    ===
===   individual motorsHas to be used in the implementation  ===
===                 of stabilization routine                 ===
==============================================================*/

void indivSpeed(Servo servo, float speed){
	servo.writeMicroseconds(speed);
}

/*==============================================================
===               INTERRUPT DETECTION ROUTINE                ===
==============================================================*/

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady(){
	mpuInterrupt = true;
}

/*==============================================================
=======                   GET GYRO/ACCEL                ========
==============================================================*/

void get_ypr(){
	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();

	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		#ifdef OUTPUT_READABLE_YAWPITCHROLL
			// display Euler angles in degrees
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

			ypr0 = (ypr[0] * 180/M_PI)+180;
			ypr1 = (ypr[1] * 180/M_PI)-3.42;
			ypr2 = (ypr[2] * 180/M_PI)-2.02;
		#endif
	}
}

/*==============================================================
=====                     STATUS BLINKING                  =====
================================================================
=== ErrorCode = 0 means no error, ErrorCode != 0 means error ===
==============================================================*/

void statusBlinking(int error,int timing){
	if (error != 0){
		Serial.println("blink");
		digitalWrite(LED_GREEN_OK,LOW);
		digitalWrite(LED_BLUE_WARNING,LOW);
		for (int i = 0;i < error;i++){
			digitalWrite(LED_RED_ALARM,HIGH);
			delay(timing);	
			digitalWrite(LED_RED_ALARM,LOW);
			delay(timing);
		}
	}
}

/*==============================================================
=====                      INITIAL SETUP                   =====
==============================================================*/

void setup(){

	pinMode(LED_GREEN_OK, OUTPUT);
	pinMode(LED_BLUE_WARNING,OUTPUT);
	pinMode(LED_RED_ALARM,OUTPUT);

	Serial.begin(9600);
	pid_input_yaw = ypr0;
	pid_input_pitch = ypr1;
	pid_input_roll = ypr2;

	myservo1.attach(6, 1000, 2000);
	myservo2.attach(9, 1000, 2000);
	myservo3.attach(10, 1000, 2000);
	myservo4.attach(11, 1000, 2000);

	myservo1.writeMicroseconds(MIN_SIGNAL);
	myservo2.writeMicroseconds(MIN_SIGNAL);
	myservo3.writeMicroseconds(MIN_SIGNAL);
	myservo4.writeMicroseconds(MIN_SIGNAL);
	delay(4000);

	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire1.begin();
		Wire1.setClock(400000);
//		TWBR = 12; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif

	while (!Serial);

	Serial.println("starting gyro setup");

	// initialize device
	mpu.initialize();

	// verify connection
	if (mpu.testConnection() != 1){
		statusBlinking(ERROR_CODE_GYRO_CONNECTION,ERROR_TIME_GYRO_CONNECTION);
		exit(0);
	}

	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(-8);
	mpu.setYGyroOffset(-75);
	mpu.setZGyroOffset(11);
	mpu.setZAccelOffset(1063); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0){
		// turn on the DMP, now that it's ready
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection

		attachInterrupt(0, dmpDataReady, RISING);

		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}

	else {
        statusBlinking(ERROR_CODE_GYRO_DMP,ERROR_TIME_GYRO_DMP);
        exit(0);
    }

    Serial.println("got past gyro setup");

	myPID_yaw.SetMode(AUTOMATIC);
	myPID_roll.SetMode(AUTOMATIC);
	myPID_pitch.SetMode(AUTOMATIC);

	// Serial.println("verifying sserial_read");
	// bool cat = false;
	// while(!cat) {
	// 	serial_read();
	// 	if (Serial.find("s")) {
	// 		cat = true;
	// 	}
	// }

	digitalWrite(LED_GREEN_OK,HIGH);
	Serial.println("setup warmup starts");
	warmup();
	//while(takeoff_speed != BASE_SPEED){
	//	Serial.println("takeoff is workig");
	//	takeoff();
	//}
}

int counter = 0;

void loop() {
	// if programming failed, don't try to do anything
	if (!dmpReady) {
		return;
	}
	// unsigned long time1 = millis();
	// for (int i=0; i<100; i++) {
	// 	get_ypr();
	// }
	// unsigned long time2 = millis();
	// Serial.println(time2-time1);
	// wait for MPU interrupt or extra packet(s) available
	if (!mpuInterrupt){
		// if (counter == 45){
		// 	serial_read();
		// 	counter=0;
		// }
		// else{
		// 	counter++;
		// }

		stabilize(BASE_SPEED);
	}
	get_ypr();

}