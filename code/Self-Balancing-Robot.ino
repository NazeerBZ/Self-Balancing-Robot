#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#include <SoftwareSerial.h>
//onst int rxpin = 12; // pin used to receive (not used in this version)
///const int txpin = 13; // pin used to send to LCD
//SoftwareSerial blue(rxpin, txpin); // new serial port on pins 2 and 3

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define BLUE
#define debug
#define LOG_INPUT 0
#define MANUAL_TUNING 0
#define LOG_PID_CONSTANTS 0 //MANUAL_TUNING must be 1
#define MOVE_BACK_FORTH 0

#define MIN_ABS_SPEED 30

//MPU

MPU6050 mpu;

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
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


//PID



#if MANUAL_TUNING
double kp , ki, kd;
double prevKp, prevKi, prevKd;
#endif
double originalSetpoint = 174.29;//1740.29
double setpoint = originalSetpoint;
double movingAngleOffset = 0.4;
double input, output;
int moveState = 0; //0 = balance; 1 = back; 2 = forth

#if MANUAL_TUNING
PID pid(&input, &output, &setpoint, 0, 0, 0, DIRECT);

#else
PID pid(&input, &output, &setpoint, 70, 240, 1.9, DIRECT);

#endif


int motorPin1 = 11; // pin 2 on L293D IC
int motorPin2 = 10; // pin 7 on L293D IC
int enablePin1 = 4; // pin 1 on L293D IC
int motorPin3 = 9; // pin 10 on L293D IC
int motorPin4 = 8; // pin 15 on L293D IC
int enablePin2 = 5; // pin 9 on L293D IC
int state;
int flag = 0;


//MOTOR CONTROLLER


int ENA = 4;//5
int IN1 = 11;//3
int IN2 = 10;//4
int IN3 = 9;//8
int IN4 = 8;//9
int ENB = 5;


LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, 0.6, 1);


//timers


long time1Hz = 0;
long time5Hz = 0;


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}


void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)




  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin1, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  // sets enablePin high so that motor can turn on:
  digitalWrite(enablePin1, HIGH);
  digitalWrite(enablePin2, HIGH);




  Serial.begin(115200);
  Serial.println(F("-----------------------------"));
  Serial.println(F("Microprocessor and Assembly Language"));
  Serial.println(F("-----------------------------"));
  Serial.println(F("Self Balancing Robot"));
  Serial.println(F("-----------------------------"));
  Serial.println(F("Group Members:"));
  Serial.println(F("   "));
  Serial.println(F("Nazeer Bin Zafar"));
  Serial.println(F("Abdul Rehman"));
  Serial.println(F("Muhammad Osama Khan"));





  //Serial.begin(115200);
  //    while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);//220
  mpu.setYGyroOffset(76);//76
  mpu.setZGyroOffset(-85);//-85
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    //setup PID

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
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

}


void loop()
{



  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    //no mpu data - performing PID calculations and output to motors

    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);

    unsigned long currentMillis = millis();

    if (currentMillis - time1Hz >= 1000)
    {
      loopAt1Hz();
      time1Hz = currentMillis;
    }

    if (currentMillis - time5Hz >= 5000)
    {
      loopAt5Hz();
      time5Hz = currentMillis;
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)


  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#if LOG_INPUT
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
#endif

    input = ypr[1] * 180 / M_PI + 180;
  }
  bluetooth();          //App control
}


void loopAt1Hz()
{
#if MANUAL_TUNING
  setPIDTuningValues();
#endif
}


void loopAt5Hz()
{
#if MOVE_BACK_FORTH
  moveBackForth();
#endif
}


//move back and forth


void moveBackForth()
{
  moveState++;
  if (moveState > 2) moveState = 0;

  if (moveState == 0)
    setpoint = originalSetpoint;
  else if (moveState == 1)
    setpoint = originalSetpoint - movingAngleOffset;
  else
    setpoint = originalSetpoint + movingAngleOffset;
}




void bluetooth()
{

  if (Serial.available() > 0) {


    state = Serial.read();
    flag = 0;

    // }
    // if the state is '0' the DC motor will turn off
    if (state == 'E') {
      digitalWrite(motorPin1, LOW); // set pin 2 on L293D low
      digitalWrite(motorPin2, LOW); // set pin 7 on L293D low
      digitalWrite(motorPin3, LOW); // set pin 2 on L293D low
      digitalWrite(motorPin4, LOW); // set pin 7 on L293D low
      if (flag == 0) {
        Serial.println("Motor: off");
        flag = 1;
      }
    }
    // if the state is '1' the motor will turn right
    else if (state == 'F') {
      digitalWrite(motorPin1, LOW); // set pin 2 on L293D low
      digitalWrite(motorPin2, HIGH); // set pin 7 on L293D high
      digitalWrite(motorPin3, LOW); // set pin 2 on L293D low
      digitalWrite(motorPin4, HIGH); // set pin 7 on L293D high
      if (flag == 0) {
        Serial.println("Motor: Farword");
        flag = 1;
      }
    }
    // if the state is '2' the motor will turn left
    else if (state == 'B') {
      digitalWrite(motorPin1, HIGH); // set pin 2 on L293D high
      digitalWrite(motorPin2, LOW); // set pin 7 on L293D low
      digitalWrite(motorPin3, HIGH); // set pin 2 on L293D high
      digitalWrite(motorPin4, LOW); // set pin 7 on L293D low
      if (flag == 0) {
        Serial.println("Motor: Reverse");
        flag = 1;
      }
    }
    else if (state == 'R') {
      digitalWrite(motorPin1, HIGH); // set pin 2 on L293D high
      digitalWrite(motorPin2, LOW); // set pin 7 on L293D low
      digitalWrite(motorPin3, LOW); // set pin 2 on L293D high
      digitalWrite(motorPin4, HIGH); // set pin 7 on L293D low
      if (flag == 0) {
        Serial.println("Motor: Right");
        flag = 1;
      }
    }
    else if (state == 'L') {
      digitalWrite(motorPin1, LOW); // set pin 2 on L293D high
      digitalWrite(motorPin2, HIGH); // set pin 7 on L293D low
      digitalWrite(motorPin3, HIGH); // set pin 2 on L293D high
      digitalWrite(motorPin4, LOW); // set pin 7 on L293D low
      if (flag == 0) {
        Serial.println("Motor: Left");
        flag = 1;
      }
    }
  }
}

//void bt_control(){
//
//}


//PID Tuning (3 potentiometers)

#if MANUAL_TUNING
void setPIDTuningValues()
{
  readPIDTuningValues();

  if (kp != prevKp || ki != prevKi || kd != prevKd)
  {
#if LOG_PID_CONSTANTS
    Serial.print(kp); Serial.print(", "); Serial.print(ki); Serial.print(", "); Serial.println(kd);
#endif

    pid.SetTunings(kp, ki, kd);
    prevKp = kp; prevKi = ki; prevKd = kd;
  }
}


void readPIDTuningValues()
{
  int potKp = analogRead(A0);
  int potKi = analogRead(A1);
  int potKd = analogRead(A2);

  kp = map(potKp, 0, 1023, 0, 25000) / 100.0; //0 - 250
  ki = map(potKi, 0, 1023, 0, 100000) / 100.0; //0 - 1000
  kd = map(potKd, 0, 1023, 0, 500) / 100.0; //0 - 5
}
#endif

