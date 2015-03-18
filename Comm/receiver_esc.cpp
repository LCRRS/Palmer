#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>
#include <I2Cdev.h>

#define MIN_SIGNAL 1000

#define LOOPTIME 100


Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;

double speed_myservo1, speed_myservo2, speed_myservo3, speed_myservo4;

bool initiation_count = false;
float ypr0;
float ypr1;
float ypr2;

int ypr;
float ypr_precision;

/*=================================================
================= PID CONTROLLER ==================
=================================================*/

double pid_setPoint = 0;    //The desired value of the gyroscope. PID produces as an output until the value is reached
  //PID containers to hold the gyroscope reading and the correction output respectively
double pid_input_pitch, pid_output_pitch;
    //Constant PID values determined experimentally through trial and error
double Kp_pitch = 5.0, Ki_pitch = 0.0, Kd_pitch = 2.0;
//The maximum possible output that the PID can produce (anything higher will be set back to this value)
double lower_limit_pitch = -100;
double upper_limit_pitch = 100;
       //PID class object that is associated with respected variables for the roll plane
PID myPID_pitch(&pid_input_pitch, &pid_output_pitch, &pid_setPoint, Kp_pitch, Ki_pitch, Kd_pitch, DIRECT);

/*============================================================
===============         WARM-UP ROUTINE       ================
============================================================*/

void warmup()
{
    myPID_pitch.SetOutputLimits(lower_limit_pitch, upper_limit_pitch);

    myPID_pitch.SetSampleTime(10);

    while (!initiation_count)
    {
        for (int speed = 1000; speed < 1400; speed++)
        {
            setSpeed(speed);
            delay(10);
        }
        initiation_count = true;
        setSpeed(1400);
    }
}


void receiveEvent(int howMany)
{
    String ypr_str;
    while(Wire.available()) // loop through all but the last
    {
        char c = Wire.read(); // receive byte as a character
        ypr_str = String (ypr_str + c);
    }
    ypr = ypr_str.toInt();
    ypr1 = float(ypr);
    ypr_precision = ypr1/100;
}

/*==============================================================
===               SET SPEED ON ALL MOTORS                    ===
================================================================
===    This function is used to set speed of all motors      ===
===                  to the same value                       ===
==============================================================*/

void setSpeed(double speed)
{
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

void indivSpeed(Servo servo, double speed)
{
    servo.writeMicroseconds(speed);
}


/*==============================================================
=====                      INITIAL SETUP                   =====
==============================================================*/

void setup()
{
    Serial.begin(115200);
    Wire.begin(4);

    Wire.onReceive(receiveEvent);

    myservo1.attach(9, 1000, 2000);
    myservo2.attach(10, 1000, 2000);
    myservo3.attach(11, 1000, 2000);
    myservo4.attach(6, 1000, 2000);

    myservo1.writeMicroseconds(MIN_SIGNAL);
    myservo2.writeMicroseconds(MIN_SIGNAL);
    myservo3.writeMicroseconds(MIN_SIGNAL);
    myservo4.writeMicroseconds(MIN_SIGNAL);
    delay(4000);

    myPID_pitch.SetMode(AUTOMATIC);
}

void loop() {
    warmup();
    pid_input_pitch = ypr_precision;
    myPID_pitch.Compute();
    speed_myservo2 = 1400 + pid_output_pitch;
    speed_myservo4 = 1400 - pid_output_pitch;

    indivSpeed(myservo2, speed_myservo2);
    indivSpeed(myservo4, speed_myservo4);

    Serial.println(ypr_precision);


}
