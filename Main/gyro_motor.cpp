#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 800

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

// Definitions needed for the mpu6050 board control

int led1 = 12;

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



bool blinkState = false;

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
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



/*=================================================================
=====                      INITIAL SETUP                      =====
=================================================================*/

void setup() {
    pinMode(led1, OUTPUT);

    Serial.begin(115200);

 
    myservo1.attach(6);
    myservo2.attach(5);
    myservo3.attach(9);
    myservo4.attach(10);
     
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

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));

        /*=================================
         MPU Interrupt function definition
        ==================================*/

        attachInterrupt(0, dmpDataReady, RISING);


        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

}



int old_ypr0; // comparison value used to check whether the gyroscope is stable
int old_ypr1; // comparison value used to check whether the gyroscope is stable
int old_ypr2; // comparison value used to check whether the gyroscope is stable

int cycle_count = 0; // used to count how many gyroscope readings were taken. Is used to check the stability of gyroscope

bool initiation_count = true; // used to initiate motor start/warmup only once
bool gyro_stable = false; // if gyroscope is stable, it allows the readings to be used for interrupts

/*=================================================================
======                    MAIN PROGRAM LOOP                  ======
=================================================================*/

void loop() {

    while (initiation_count == true)
    {
        for (int speed = 30; speed < 45; speed++)
        {
            setSpeed(speed);
            delay(1000);
        }
        initiation_count = false;
    }

    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) 
    {
        if (gyro_stable == true)
        {
            digitalWrite(led1, HIGH);
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
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

    }

    /*=========================================================
    ======              GYROSCOPE VERIFICATION          =======
    =========================================================*/

    if (cycle_count < 400)
    {
        cycle_count = cycle_count + 1;
    }

    if (cycle_count == 200)
    {
        old_ypr0 = int (ypr[0] * 180/M_PI);
        old_ypr1 = int (ypr[1] * 180/M_PI);
        old_ypr2 = int (ypr[2] * 180/M_PI);
    }

    if (cycle_count == 400)
    {
        if (old_ypr1 - int (ypr[1] * 180/M_PI) == 0 && old_ypr2 - int (ypr[2] * 180/M_PI) == 0)
        {
            gyro_stable = true;
            delay (2000);
            cycle_count = cycle_count + 1;
            Serial.println("gyro_stable = true");
        }

        else
        {
            cycle_count = 0;
        }
    }
}