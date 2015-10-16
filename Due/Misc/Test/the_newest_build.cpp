#include <PID_v1.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define MIN_SIGNAL 1000

#define OUTPUT_READABLE_YAWPITCHROLL

#define LOOPTIME 100

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

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];

bool initiation_count = false; // used to initiate motor start/warmup routine inside the loop function only once
bool gyro_stable = true; // if gyroscope is stable, it allows the readings to be used for interrupts

double old_ypr0; // comparison value used to check whether the gyroscope is stable
double old_ypr1; // comparison value used to check whether the gyroscope is stable
double old_ypr2; // comparison value used to check whether the gyroscope is stable

double ypr0;
double ypr1;
double ypr2;

int increase_count = 0;

int cycle_count = 0; // used to count how many gyroscope readings were taken. Is used to check the stability of gyroscope

/*=================================================
================= PID CONTROLLER ==================
=================================================*/

double pid_setPoint = 0;    //The desired value of the gyroscope. PID produces as an output until the value is reached

double pid_input_roll, pid_output_roll;   //PID containers to hold the gyroscope reading and the correction output respectively
double pid_input_pitch, pid_output_pitch;

double Kp_roll = 2.2, Ki_roll = 0.0, Kd_roll = 1.4;     //Constant PID values determined experimentally through trial and error
double Kp_pitch = 2.2, Ki_pitch = 0.0, Kd_pitch = 1.4;

double lower_limit_roll = -50;   //The lowest possible output that the PID can produce
double upper_limit_roll = 50; //The maximum possible output that the PID can produce (anything higher will be set back to this value)
double lower_limit_pitch = -50;
double upper_limit_pitch = 50;

PID myPID_roll(&pid_input_roll, &pid_output_roll, &pid_setPoint, Kp_roll, Ki_roll, Kd_roll, DIRECT);        //PID class object that is associated with respected variables for the roll plane
PID myPID_pitch(&pid_input_pitch, &pid_output_pitch, &pid_setPoint, Kp_pitch, Ki_pitch, Kd_pitch, DIRECT);

/*============================================================
===============         WARM-UP ROUTINE       ================
============================================================*/

void warmup()
{
  myPID_roll.SetOutputLimits(lower_limit_roll, upper_limit_roll);
  myPID_pitch.SetOutputLimits(lower_limit_pitch, upper_limit_pitch);

  myPID_roll.SetSampleTime(10); //Time to pass between the values of PID are recomputed (in milliseconds)
  myPID_pitch.SetSampleTime(10);

    while (!initiation_count)
    {
        for (int speed = 1000; speed < 1300; speed++)
        {
            setSpeed(speed);
            delay(10);
        }
        initiation_count = true;
        setSpeed(1330);
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
=====                      INITIAL SETUP                   =====
==============================================================*/

void setup()
{
    Serial.begin(115200);
    pid_input_roll = ypr2;
    pid_input_pitch = ypr1;

    myservo1.attach(9, 1000, 2000);
    myservo2.attach(10, 1000, 2000);
    myservo3.attach(11, 1000, 2000);
    myservo4.attach(12, 1000, 2000);

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
    mpu.setXGyroOffset(-229);
    mpu.setYGyroOffset(12);
    mpu.setZGyroOffset(43);
    mpu.setZAccelOffset(1733); // 1688 factory default for my test chip

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

    myPID_roll.SetMode(AUTOMATIC);
    myPID_pitch.SetMode(AUTOMATIC);
}

double error = 0;
double lasterror = 0;
bool error_correction = false;

void loop() {
  warmup();

  ypr1 = (ypr[1] * 180/M_PI) - 0.4;
  ypr2 = (ypr[2] * 180/M_PI) + 1.6;

  if (error - lasterror > 20)
  {
    error_correction = true;
  }

  if (error - lasterror < 20)
  {
    error_correction = false;
  }

    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize)
    {
        if (!gyro_stable)
        {

            if (ypr2 != 0 && !error_correction)
            {
                /*=============================================
                ======          AIR STABILIZATION        ======
                =============================================*/

                pid_input_roll = ypr2;
                myPID_roll.Compute();
                speed_myservo1 = 1330;
                speed_myservo3 = 1330;
                speed_myservo1 =  speed_myservo1 + pid_output_roll;
                speed_myservo3 = speed_myservo3 - pid_output_roll;
                indivSpeed(myservo1, speed_myservo1);
                indivSpeed(myservo3, speed_myservo3);
                Serial.print(ypr2);
                Serial.print("      ");
                Serial.println(speed_myservo2);


            if (ypr1 != 0 && !error_correction)
            {

                /*=============================================
                ======          AIR STABILIZATION        ======
                =============================================*/

                pid_input_pitch = ypr1;
                myPID_pitch.Compute();
                speed_myservo2 = 1330;
                speed_myservo4 = 1330;
                speed_myservo2 =  speed_myservo2 + pid_output_pitch;
                speed_myservo4 = speed_myservo4 - pid_output_pitch;
                indivSpeed(myservo2, speed_myservo2);
                indivSpeed(myservo4, speed_myservo4);
                Serial.print(ypr1);
                Serial.print("      ");
                Serial.println(speed_myservo2);
            }
        }
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

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
        #endif
    }
    lasterror = error;
    error = ypr2;


    /*=========================================================
    ======              GYROSCOPE VERIFICATION          =======
    =========================================================*/

    if (cycle_count < 1000)
    {
        cycle_count = cycle_count + 1;
    }

    if (cycle_count == 200)
    {
        old_ypr0 = ypr0;
        old_ypr1 = ypr1;
        old_ypr2 = ypr2;
    }

    if (cycle_count == 1000)
    {
        if (old_ypr1 - ypr1 == 0 && old_ypr2 - ypr2 == 0)
        {
            gyro_stable = false;
            cycle_count = cycle_count + 1;
        }

        else
        {
            cycle_count = 0;
        }
    }
}
