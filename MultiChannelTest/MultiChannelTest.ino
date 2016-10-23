#include <PinChangeInt.h>
#include <Servo.h>

#define NO_PORTD_PINCHANGES
#define NO_PORTC_PINCHANGES

// Assign your channel in pins
#define THROTTLE_IN_PIN 2
#define STEERING_IN_PIN 3
#define AUX1 10
#define AUX2 11

// Assign your channel out pins
#define MOTDIR 5
#define MOTESQ 6

// Servo objects generate the signals expected by Electronic Speed Controllers and Servos
// We will use the objects to output the signals we read in
// this example code provides a straight pass through of the signal with no custom processing
Servo servoDir;
Servo servoEsq;

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2
#define AUX1_FLAG 3
#define AUX2_FLAG 4

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
volatile uint16_t unAux1InShared;
volatile uint16_t unAux2InShared;
volatile uint32_t lastInterrupt;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;
uint32_t ulAux1Start;
uint32_t ulAux2Start;

void setup()
{
  Serial.begin(9600);

  Serial.println("multiChannels");

  // attach servo objects, these will generate the correct
  // pulses for driving Electronic speed controllers, servos or other devices
  // designed to interface directly with RC Receivers
  servoDir.attach(MOTDIR);
  servoEsq.attach(MOTESQ);

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  attachInterrupt(digitalPinToInterrupt(THROTTLE_IN_PIN), calcThrottle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEERING_IN_PIN), calcSteering, CHANGE);
  PCintPort::attachInterrupt(AUX1, calcAux, CHANGE);
  PCintPort::attachInterrupt(AUX2, calcAux2, CHANGE);
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint16_t unAux1In;
  static uint16_t unAux2In;
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if (bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;

    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.


    unThrottleIn = unThrottleInShared;



    unSteeringIn = unSteeringInShared;


    if (bUpdateFlags & AUX1_FLAG)
    {
      unAux1In = unAux1InShared;
    }

    if (bUpdateFlags & AUX2_FLAG)
    {
      unAux2In = unAux2InShared;
    }

    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;

    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }

  if (unThrottleIn > 1500 || unThrottleIn < 1400)
  {
    //Serial.println("Entrei");
    servoDir.writeMicroseconds(unThrottleIn);
    servoEsq.writeMicroseconds(unThrottleIn);
  }
  else
  {
    if (unSteeringIn > 1500 || unSteeringIn < 1400)
    {
      servoDir.writeMicroseconds(1460 + (1460 - unSteeringIn));
      servoEsq.writeMicroseconds(unSteeringIn);
    }
    else
    {
      servoDir.writeMicroseconds(1460);
      servoEsq.writeMicroseconds(1460);
    }
  }





  if ((millis() - lastInterrupt) > 200)
  {
    servoDir.writeMicroseconds(1460);
    servoEsq.writeMicroseconds(1460);
  }


  Serial.print("Th = "); Serial.print(unThrottleIn); Serial.print(" "); Serial.print("St = "); Serial.print(unSteeringIn); Serial.println(" ");
  //Serial.print("AUX1 = "); Serial.println(1500 + (1500-unSteeringIn));// Serial.print(" "); Serial.print("AUX2 = "); Serial.println(unAux2In);
  //Serial.print("lastint = "); Serial.println(millis() - lastInterrupt);

  bUpdateFlags = 0;
}


// simple interrupt service routine
void calcThrottle()
{
  lastInterrupt = millis();
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if (digitalRead(THROTTLE_IN_PIN) == HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  lastInterrupt = millis();
  if (digitalRead(STEERING_IN_PIN) == HIGH)
  {
    ulSteeringStart = micros();
  }
  else
  {
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

void calcAux()
{
  //lastInterrupt = millis();
  if (digitalRead(AUX1) == HIGH)
  {
    ulAux1Start = micros();
  }
  else
  {
    unAux1InShared = (uint16_t)(micros() - ulAux1Start);
    bUpdateFlagsShared |= AUX1_FLAG;
  }
}

void calcAux2()
{
  //lastInterrupt = millis();
  if (digitalRead(AUX2) == HIGH)
  {
    ulAux2Start = micros();
  }
  else
  {
    unAux2InShared = (uint16_t)(micros() - ulAux2Start);
    bUpdateFlagsShared |= AUX2_FLAG;
  }
}
