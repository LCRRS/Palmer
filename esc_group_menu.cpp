/*
Coded by Marjan Olesch
Sketch from Insctructables.com
Open source - do what you want with this code!
*/
#include <Servo.h>


int value = 0; // set values you need to zero

Servo ESC1, ESC2, ESC3, ESC4; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time

void setup() 
{

	ESC1.attach(9);
	ESC2.attach(10);
	ESC3.attach(5);
	ESC4.attach(6);    
	Serial.begin(9600);    // start serial at 9600 baud

}

void loop() 
{

	//First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions
 
	ESC1.writeMicroseconds(value);
	ESC2.writeMicroseconds(value);
	ESC3.writeMicroseconds(value);
	ESC4.writeMicroseconds(value);
 
	if(Serial.available()) 
	{
    	value = Serial.parseInt();    // Parse an Integer from Serial
    }
}