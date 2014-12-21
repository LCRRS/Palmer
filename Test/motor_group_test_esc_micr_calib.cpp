// This script is used to test perform calibration routin and test the synchonization of all four motors
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700
#include <Servo.h> 

int mpin1 = 9;
int mpin2 = 10;
int mpin3 = 5;
int mpin4 = 6;

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;


void setSpeed(int speed){
 // speed is from 0 to 100 where 0 is off and 100 is maximum speed
 //the following maps speed values of 0-100 to angles from 0-180,
 // some speed controllers may need different values, see the ESC instructions
	int angle = speed;
	myservo1.write(angle);
	myservo2.write(angle); 
	myservo3.write(angle);
	myservo4.write(angle); 
}

void setup()
{
	Serial.begin(9600);
	if(Serial.available() > 0)
	{
        
		myservo1.attach(mpin1);
		myservo2.attach(mpin2);
		myservo3.attach(mpin3);
		myservo4.attach(mpin4);
		 
		Serial.println("Now writing maximum output.");
		Serial.println("Turn on power source, then wait 2 seconds and press any key.");
		myservo1.writeMicroseconds(MAX_SIGNAL);
	        myservo2.writeMicroseconds(MAX_SIGNAL);
	        myservo3.writeMicroseconds(MAX_SIGNAL);
	        myservo4.writeMicroseconds(MAX_SIGNAL);
		// Wait for input
		while (!Serial.available());
		Serial.read();
		// Send min output
		Serial.println("Sending minimum output");
		myservo1.writeMicroseconds(MIN_SIGNAL);
	        myservo2.writeMicroseconds(MIN_SIGNAL);
	        myservo3.writeMicroseconds(MIN_SIGNAL);
	        myservo4.writeMicroseconds(MIN_SIGNAL);
		delay(5000);
	}
}


void loop()
{
 
	int speed;

	 // sweep up from 0 to to maximum speed in 20 seconds
	for(speed = 0; speed <= 60; speed += 5) 
	{
		setSpeed(speed);
		delay(1000);
	}

	// sweep back down to 0 speed.
	for(speed = 55; speed > 0; speed -= 5) 
	{
		setSpeed(speed);
		delay(1000);
	}
	
	setSpeed(0);  
	delay(5000); // stop the motor for 5 seconds
}