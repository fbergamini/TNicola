#include <Servo.h>

///Pins that are connected to the receiver's channels
#define ch1 2
#define ch2 3
///Pins that are connected to the sensors
#define linePin1 A4
///Pins that are connected to the RoboClaw controller
#define thCh 9  //Connect to S1
#define stCh 10 //Connect to S2

///Functions that are called upon when a matching interruption event occurs
///Calculates the width of the servo signal pulse
///No parameters
///Returns nothing
void pulseCalcCh1();
void pulseCalcCh2();

///Function that translates pulse width to a matching velocity
///Parameter is the pulse time variable (volatile unsigned long)
///Returns the velocity (int)
int pulseToVel(volatile unsigned long pulseTime, int chType);

///Variables related to pulse width measurement
volatile unsigned long lastTimeCh1 = 0, lastTimeCh2 = 0, riseTimeCh1 = 0, riseTimeCh2 = 0, pulseTimeCh1 = 0, pulseTimeCh2 = 0;

///Servo variables related to the RoboClaw channels - Throttle and Steer
Servo rcThrotle, rcSteering;

///Variables that store the sensor values
int  lineV;

///Variable that stores the velocity of the actions - Throttle and Steer
int velTh = 90;
int velSt = 90;

///thType e stType são variáveis que controlam o canal
int thType = 0;
int stType = 1;

void setup() {
  attachInterrupt(digitalPinToInterrupt(ch1), pulseCalcCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch2), pulseCalcCh2, CHANGE);
  rcThrotle.attach(thCh);
  rcSteering.attach(stCh);
  Serial.begin(9600);
}

void loop() {
  ///Stores the value read by the line sensor
  //lineV = analogRead(linePin1);
  lineV = 400;    

  if ((millis() - lastTimeCh1) > 200) pulseTimeCh1 = 1468;
  if ((millis() - lastTimeCh2) > 200) pulseTimeCh2 = 1468;

  ///Calculates throttle velocity
  velTh = pulseToVel(pulseTimeCh1, thType);
  velSt = pulseToVel(pulseTimeCh2, stType);
  //Serial.print(pulseTimeCh1); Serial.print("  ");
  Serial.print(velTh); Serial.print(" ");
  Serial.println(velSt);
  rcThrotle.write(velTh);
  rcSteering.write(velSt);  
}

int pulseToVel(volatile unsigned long pulseTime, int chType) {
  int vel;
  if (!chType) {
    if (pulseTime > 1600)  vel = map(pulseTime, 1468, 2028, 90, 180);
    else if (pulseTime < 1350)  vel = map(pulseTime, 1100, 1468, 0, 90);
    else vel = 90;
    if (lineV < 80) vel = 90;
  }
  else {
    if (pulseTime > 1550)  vel = map(pulseTime, 1480, 1760, 90, 180);
    else if (pulseTime < 1350)  vel = map(pulseTime, 1100, 1480, 0, 90);
    else vel = 90;
    if (lineV < 80) vel = 90;
  }
  return vel;
}

void pulseCalcCh1() {
  //record the interrupt time so that we can tell if the receiver has a signal from the transmitter
  lastTimeCh1 = millis();
  //if the pin has gone HIGH, record the microseconds since the Arduino started up
  if (digitalRead(ch1) == HIGH)   riseTimeCh1 = micros();
  //otherwise, the pin has gone LOW
  else  {
    //only worry about this if the timer has actually started
    if (riseTimeCh1 != 0) {
      //record the pulse time
      pulseTimeCh1 = (volatile unsigned long)micros() - riseTimeCh1;
      //restart the timer
      riseTimeCh1 = 0;
    }
  }
}

void pulseCalcCh2() {
  //record the interrupt time so that we can tell if the receiver has a signal from the transmitter
  lastTimeCh2 = millis();
  //if the pin has gone HIGH, record the microseconds since the Arduino started up
  if (digitalRead(ch2) == HIGH)   riseTimeCh2 = micros();
  //otherwise, the pin has gone LOW
  else  {
    //only worry about this if the timer has actually started
    if (riseTimeCh2 != 0) {
      //record the pulse time
      pulseTimeCh2 = (volatile unsigned long)micros() - riseTimeCh2;
      //restart the timer
      riseTimeCh2 = 0;
    }
  }
}
