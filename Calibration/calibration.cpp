#include <Motors_PWM.h>

#define NB_MOTOR_4
#define NB_MOTOR 4
#define NB_MOTOR_CONFIG FOUR_Motors
#define LOWEST_SPEED 1000
#define IDLE_SPEED 1200
#define HIGHEST_SPEED 2000

void initMotors(NB_Motors motorConfig) {
  initializeMotors(motorConfig);
}


char getCommand() {
  char cmd = 0;
  if (Serial.available() > 0) {
    cmd = Serial.read();
  }
  return cmd;
}


void setSpeed(int motor, int throttle) {
  motorCommand[motor] = throttle;
  writeMotors();
  Serial.print("Motor: ");
  Serial.print(motor);
  Serial.print(" Speed set to: ");
  Serial.println(throttle);
}


void turnAllMotorsOff() {
  for (int i = 0; i < NB_MOTOR; i++) {
    setSpeed(i, LOWEST_SPEED);
  }
}


void setup() {
  Serial.begin(115200);
  Serial.println("Hello Motormouth");
  initMotors(NB_MOTOR_CONFIG);
  turnAllMotorsOff();
}


boolean gChanged = false;
int gMotor = 0;
int gThrottle = LOWEST_SPEED;

void loop() {
  char cmd;

  if (gChanged) {
    setSpeed(gMotor, gThrottle);
    gChanged = !gChanged;
  }

  if (cmd = getCommand()) {
    switch (cmd) {
      case 'h':
           gThrottle = HIGHEST_SPEED;
           gChanged = true;
           break;
       case 'l':
           gThrottle = LOWEST_SPEED;
           gChanged = true;
           break;
       case 'i':
           gThrottle = IDLE_SPEED;
           gChanged = true;
           break;
       case '0':
           gMotor = 0;
           gThrottle = LOWEST_SPEED;
           gChanged=true;
           break;
       case '1':
           gMotor = 1;
           gThrottle = LOWEST_SPEED;
           gChanged=true;
           break;
       case '2':
           gMotor = 2;
           gThrottle = LOWEST_SPEED;
           gChanged=true;
           break;
       case '3':
           gMotor = 3;
           gThrottle = LOWEST_SPEED;
           gChanged=true;
           break;
       default:
           turnAllMotorsOff();
           gMotor = 0;
           gThrottle = LOWEST_SPEED;
           Serial.print("I do not know how to process ");
           Serial.println(cmd);
     }
  }
}
