// The following script is used for motor-testing

#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 600

//Servo myservo1;
Servo myservo2;
//Servo myservo3;
//Servo myservo4;


void setSpeed(int speed)
{
 // speed is from 0 to 100 where 0 is off and 100 is maximum speed
 //the following maps speed values of 0-100 to angles from 0-180,
 // some speed controllers may need different values, see the ESC instructions
	int angle = speed;
//	myservo1.writeMicroseconds(angle);
	myservo2.writeMicroseconds(angle);
//	myservo3.writeMicroseconds(angle);
//	myservo4.writeMicroseconds(angle);
}

void setup()
{
	Serial.begin(9600);
//	myservo1.attach(9);
	myservo2.attach(10);
//	myservo3.attach(11);
//	myservo4.attach(12);

}


void loop()
{
	int speed;

	 // sweep up from 0 to to maximum speed in 20 seconds
	for(speed = 600; speed <= 1200; speed += 5)
	{
		setSpeed(speed);
		delay(200);
	}

	// sweep back down to 0 speed.
	for(speed = 1200; speed > 600; speed -= 5)
	{
		setSpeed(speed);
		delay(1000);
	}

	setSpeed(0);
	delay(5000); // stop the motor for 5 seconds
}
