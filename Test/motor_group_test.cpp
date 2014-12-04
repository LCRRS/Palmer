// this uses the Arduino servo library included with version 0012 

// caution, this code sweeps the motor up to maximum speed !
// make sure the motor is mounted securily before running.

#include <Servo.h> 

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;


void setSpeed(int speed)
{
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
	myservo1.attach(9);
	myservo2.attach(10);
	myservo3.attach(11);
	myservo4.attach(6);

}


void loop()
{
	if (millis()<3000)
	{
		setSpeed(23);
		delay(1000);
	}
 
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