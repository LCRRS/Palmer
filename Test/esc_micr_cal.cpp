#include <Servo.h>
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 800
#define GOD_DAMN_IT 800

/*================================

================================*/

int motorpin1 = 5;
int motorpin2 = 6;
int motorpin3 = 9;
int motorpin4 = 10;

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

/*================================
SETUP

================================*/

void setup() 
{
	Serial.begin(9600);
	Serial.println("Program begin...");
	Serial.println("This program will calibrate the ESC.");
	motor1.attach(motorpin1);
	motor2.attach(motorpin2);
	motor3.attach(motorpin3);
	motor4.attach(motorpin4);
	Serial.println("Now writing maximum output.");
	Serial.println("Turn on power source, then wait 2 seconds and press any key.");
	motor1.writeMicroseconds(MAX_SIGNAL);
	motor2.writeMicroseconds(MAX_SIGNAL);
	motor3.writeMicroseconds(MAX_SIGNAL);
	motor4.writeMicroseconds(MAX_SIGNAL);
	// Wait for input
	while (!Serial.available());
	Serial.read();
	// Send min output
	Serial.println("Sending minimum output");
	motor1.writeMicroseconds(GOD_DAMN_IT);
	motor2.writeMicroseconds(MIN_SIGNAL);
	motor3.writeMicroseconds(GOD_DAMN_IT);
	motor4.writeMicroseconds(MIN_SIGNAL);
}

/*================================
LOOP

================================*/

void loop() 
{

}