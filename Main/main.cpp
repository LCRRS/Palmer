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
#define OUTPUT_READABLE_WORLDACCEL

#define MIN_SIGNAL 1000
#define LOOPTIME 100
#define START_SPEED 1750
#define SAMPLE_TIME 25

#define KP_PR 5.0
#define KI_PR 0.2
#define KD_PR 2.0
#define KP_YAW 2.0
#define KI_YAW 0.0
#define KD_YAW 0.7

#define LOWER_LIMIT_YAW -30
#define UPPER_LIMIT_YAW 30
#define LOWER_LIMIT_PR -120   // The lowest possible output that the PID can produce
#define UPPER_LIMIT_PR 120 // The maximum possible output that the PID can produce (anything higher will be set back to this value)

MPU6050 mpu;

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;

double speed_myservo1, speed_myservo2, speed_myservo3, speed_myservo4;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];

bool initiation_count = false; // used to initiate motor start/warmup routine inside the loop function only once

float ypr0;
float ypr1;
float ypr2;

float accel_x;
float accel_y;
float accel_z;

/*=================================================
================= PID CONTROLLER ==================
=================================================*/

double pid_setPoint = 0;    // The desired value of the gyroscope. PID produces as an output until the value is reached

double pid_input_yaw, pid_output_yaw;
double pid_input_pitch, pid_output_pitch;
double pid_input_roll, pid_output_roll;   // PID containers to hold the gyroscope reading and the correction output respectively

PID myPID_yaw(&pid_input_yaw, &pid_output_yaw, &pid_setPoint, KP_YAW, KI_YAW, KD_YAW, DIRECT);
PID myPID_pitch(&pid_input_pitch, &pid_output_pitch, &pid_setPoint, KP_PR, KI_PR, KD_PR, DIRECT);
PID myPID_roll(&pid_input_roll, &pid_output_roll, &pid_setPoint, KP_PR, KI_PR, KD_PR, DIRECT);        //PID class object that is associated with respected variables for the roll plane

/*============================================================
===============         WARM-UP ROUTINE       ================
============================================================*/

void warmup()
{
    myPID_yaw.SetOutputLimits(LOWER_LIMIT_YAW, UPPER_LIMIT_YAW);
    myPID_pitch.SetOutputLimits(LOWER_LIMIT_PR, UPPER_LIMIT_PR);
    myPID_roll.SetOutputLimits(LOWER_LIMIT_PR, UPPER_LIMIT_PR);

    myPID_yaw.SetSampleTime(SAMPLE_TIME); //Time to pass between the values of PID are recomputed (in milliseconds)
    myPID_pitch.SetSampleTime(SAMPLE_TIME);
    myPID_roll.SetSampleTime(SAMPLE_TIME);

    while (!initiation_count)
    {
        for (int speed = 1000; speed < START_SPEED; speed++)
        {
            setSpeed(speed);
            delay(10);
        }
        initiation_count = true;
        setSpeed(START_SPEED);
    }
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
===               INTERRUPT DETECTION ROUTINE                ===
==============================================================*/

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

/*==============================================================
=======                   GET GYRO/ACCEL                ========
==============================================================*/

void get_ypr()
{
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

            ypr0 = (ypr[0] * 180/M_PI);
            ypr1 = (ypr[1] * 180/M_PI)-1.95;
            ypr2 = (ypr[2] * 180/M_PI)-2.75;
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

            accel_x = aaWorld.x;
            accel_y = aaWorld.y;
            accel_z = aaWorld.z;
        #endif
    }
}

/*==============================================================
=====                      INITIAL SETUP                   =====
==============================================================*/

void setup()
{
    Serial.begin(115200);
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
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    while (!Serial);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-8);
    mpu.setYGyroOffset(-75);
    mpu.setZGyroOffset(11);
    mpu.setZAccelOffset(1063); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));

        attachInterrupt(0, dmpDataReady, RISING);

        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    myPID_yaw.SetMode(AUTOMATIC);
    myPID_roll.SetMode(AUTOMATIC);
    myPID_pitch.SetMode(AUTOMATIC);
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    if (millis() > 50000 && !mpuInterrupt && fifoCount < packetSize)
    {
        warmup();

        /*=============================================
        ======          AIR STABILIZATION        ======
        =============================================*/
        pid_input_yaw = ypr0;
        pid_input_pitch = ypr1;
        pid_input_roll = ypr2;
        myPID_yaw.Compute();
        myPID_pitch.Compute();
        myPID_roll.Compute();

        speed_myservo1 = START_SPEED - pid_output_pitch + pid_output_yaw;
        speed_myservo2 = START_SPEED + pid_output_roll - pid_output_yaw;
        speed_myservo3 = START_SPEED + pid_output_pitch + pid_output_yaw;
        speed_myservo4 = START_SPEED - pid_output_roll - pid_output_yaw;

        indivSpeed(myservo1, speed_myservo1);
        indivSpeed(myservo2, speed_myservo2);
        indivSpeed(myservo3, speed_myservo3);
        indivSpeed(myservo4, speed_myservo4);

    }
}
