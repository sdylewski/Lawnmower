/*
  RC Remote Car
  fsi6x-RC-car-spin.ino
  Uses Flysky FS-I6X RC receiver & FS-I6X 6-ch Controller
  Uses TB6612FNG H-Bridge Motor Controller
  Drives two DC Motors

  Two Drive Modes - Normal and Spin
  Map channel 6 on controller to switch SWA for mode control

  Right stick controls direction in Normal mode (CH1 & CH2)
  VRA controls speed and direction in Spin mode (CH5)
  Left stick is acceleration in both modes (CH3)

  Channel functions by Ricardo Paiva - https://gist.github.com/werneckpaiva/

  DroneBot Workshop 2021
  https://dronebotworkshop.com

  
*/

// Include iBus Library
//#include <IBusBM.h>

// Create iBus Object
//IBusBM ibus;

// TODO:
// Switch to ibus?
// 

// Define Input Connections from remote receiver (pin numbers)
#define CH1 9 // white
#define CH2 10 // blue
#define CH3 11 // red
#define CH4 12 // green
//#define CH5 17 // green optional?  or use for sensing the battery voltage?
//#define CH6 16 // blue optional?

// Receiver Channel Values
int rcCH1 = 0; // Left - Right
int rcCH2 = 0; // Forward - Reverse
int rcCH3 = 0; // Acceleration
int rcCH4 = 0; // Spin mode?  (scott) 
//int rcCH5 = 0; // Spin Control
//bool rcCH6 = 0; // Mode Control

// LED Connection
#define carLED 13 // pin for LED 
//#define carLED LED_BUILTIN

// UNO has PWM on D3, D5, D6, D9, D10, D11 only.  be sure to use these for pwmA/B
// Motor Control Connections for TB6612FNG
// A is Right ?
// B is Left ?
// Nano has PWM on D3, D5, D6, D9, D10, and D11
#define Front_pwmA 7 // white GPIO7 for pwm output to driver for speed
#define Front_in2A 6 // blue GPIO6 for output to motor driver for direction
#define Front_in1A 5 // red GPIO4 for output to motor driver for direction
//#define Front_stby 1 // green GPIO1 for STBY/Enable 
#define Front_in1B 4 // pin for output to motor driver for direction
#define Front_in2B 3 // pin for output to motor driver for direction
#define Front_pwmB 2 // pin for pwm output to driver for speed

//#define Rear_stby 9 // GPIO1 for STBY/Enable 
// gnd pin 13
//#define Rear_pwmB 10 // GPI10 for pwm output to driver for speed
//#define Rear_in2B 11 // GPIO6 for output to motor driver for direction
//#define Rear_in1B 12 // GPIO4 for output to motor driver for direction
//#define Rear_in1A 13 // pin for output to motor driver for direction
//#define Rear_in2A 14 // pin for output to motor driver for direction
//#define Rear_pwmA 15 // pin for pwm output to driver for speed


// Motor Speed Values - Start at zero
int MotorSpeedA = 0;
int MotorSpeedB = 0;

// Motor Direction Values - 0 = backward, 1 = forward
int MotorDirA = 1;
int MotorDirB = 1;

// Control Motor A (Left?)
void mControlA(int mspeed, int mdir) {

  // Determine direction
  if (mdir == 0) {
    // Motor backward
    digitalWrite(Front_in1A, LOW);
    digitalWrite(Front_in2A, HIGH);
    //digitalWrite(Rear_in1A, LOW);
    //digitalWrite(Rear_in2A, HIGH);
  } else {
    // Motor forward
    digitalWrite(Front_in1A, HIGH);
    digitalWrite(Front_in2A, LOW);
    //digitalWrite(Rear_in1A, HIGH);
    //digitalWrite(Rear_in2A, LOW);
  }

  // Control motor
  analogWrite(Front_pwmA, mspeed);
  //analogWrite(Rear_pwmA, mspeed);
}

// Control Motor B
void mControlB(int mspeed, int mdir) {

  // Determine direction
  if (mdir == 0) {
    // Motor backward
    digitalWrite(Front_in1B, LOW);
    digitalWrite(Front_in2B, HIGH);
    //digitalWrite(Rear_in1B, LOW);
    //digitalWrite(Rear_in2B, HIGH);
  } else {
    // Motor forward
    digitalWrite(Front_in1B, HIGH);
    digitalWrite(Front_in2B, LOW);
    //digitalWrite(Rear_in1B, HIGH);
    //digitalWrite(Rear_in2B, LOW);
  }

  // Control motor
  // for arduino, use the analogWrite:
  analogWrite(Front_pwmB, mspeed);
  //analogWrite(Rear_pwmB, mspeed);
  // for pi pico, need to use PWM slices:
  
}

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
//int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
//  uint16_t ch = ibus.readChannel(channelInput);  // ibus example
//  if (ch < 100) return defaultValue;
//  return map(ch, 1000, 2000, minLimit, maxLimit);
//}
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void setup()

{

  // Start serial monitor for debugging
  Serial.begin(115200);

  // Attach iBus object to serial port
  //ibus.begin(Serial1);

  // Set all the motor control pins to outputs
  pinMode(Front_pwmA, OUTPUT);
  pinMode(Front_pwmB, OUTPUT);
  pinMode(Front_in1A, OUTPUT);
  pinMode(Front_in2A, OUTPUT);
  pinMode(Front_in1B, OUTPUT);
  pinMode(Front_in2B, OUTPUT);
  //pinMode(Front_stby, OUTPUT);

  //pinMode(Rear_pwmA, OUTPUT);
  //pinMode(Rear_pwmB, OUTPUT);
  //pinMode(Rear_in1A, OUTPUT);
  //pinMode(Rear_in2A, OUTPUT);
  //pinMode(Rear_in1B, OUTPUT);
  //pinMode(Rear_in2B, OUTPUT);
  //pinMode(Rear_stby, OUTPUT);

  // Set LED pin as output
  pinMode(carLED, OUTPUT);

  // Keep motors on standby for two seconds & flash LED
  //digitalWrite(stby, LOW);
  digitalWrite(carLED, HIGH);
  delay (1000);
  digitalWrite(carLED, LOW);
  delay (1000);
  digitalWrite(carLED, HIGH);
  delay (1000);
  digitalWrite(carLED, LOW);
  //digitalWrite(stby, HIGH);

  // enable motor controller:
  //digitalWrite(stby, HIGH);
}

void loop() {

  // Get RC channel values
  rcCH1 = readChannel(CH1, -100, 100, 0);
  rcCH2 = readChannel(CH2, -100, 100, 0);
  rcCH3 = readChannel(CH3, 0, 155, 0);  // speed/throttle?  
  //rcCH3 = readChannel(CH3, 
  rcCH4 = readChannel(CH4, -100, 100, 0);
  //rcCH5 = readChannel(4, -100, 100, 0);
  //rcCH6 = readSwitch(5, false);

  // Print values to serial monitor for debugging
  Serial.print("Ch1 = ");
  Serial.print(rcCH1);

  Serial.print(" Ch2 = ");
  Serial.print(rcCH2);

  Serial.print(" Ch3 = ");
  Serial.print(rcCH3);

  Serial.print(" Ch4 = ");
  Serial.print(rcCH4);
  
  //Serial.print(" Ch5 = ");
  //Serial.print(rcCH5);

  //Serial.print(" Ch6 = ");
  //Serial.println(rcCH6);

  // Set speeds with channel 3 value (throttle?)
  MotorSpeedA = rcCH3;
  MotorSpeedB = rcCH3;

  // Set Mode with channel 6 value
//  if (rcCH6) {
//    // Spin Mode
//
//    // Turn on LED
//    digitalWrite(carLED, HIGH);
//
//    // Get Direction from channel 5 value
//    if (rcCH5 >= 0) {
//      //Clockwise
//      MotorDirA = 0;
//      MotorDirB = 1;
//      Serial.println("Clockwise");
//    } else {
//      //Counter-Clockwise
//      MotorDirA = 1;
//      MotorDirB = 0;
//      Serial.println("Counter-Clockwise");
//    }
//
//    // Add Motorspeed to channel 5 value
//    MotorSpeedA = MotorSpeedA + abs(rcCH5);
//    MotorSpeedB = MotorSpeedB + abs(rcCH5);
//
//  } else {
    // Normal Mode

    // Turn off LED
    //digitalWrite(carLED, LOW);

    // Set forward/backward direction with channel 2 value
    if (rcCH2 >= 0) {
      //Forward
      MotorDirA = 1;
      MotorDirB = 1;
      Serial.println(" Forward");
    } else {
      //Backward
      MotorDirA = 0;
      MotorDirB = 0;
      Serial.println(" Backward");
    }

    // Add channel 2 speed
    MotorSpeedA = MotorSpeedA + abs(rcCH2);
    MotorSpeedB = MotorSpeedB + abs(rcCH2);

    // Set left/right offset with channel 1 value
    MotorSpeedA = MotorSpeedA - rcCH1;
    MotorSpeedB = MotorSpeedB + rcCH1;

//  }

  // Ensure that speeds are between 0 and 255
  MotorSpeedA = constrain(MotorSpeedA, 0, 255);
  MotorSpeedB = constrain(MotorSpeedB, 0, 255);

  //Drive Motors
  //mControlA(MotorSpeedA, MotorDirA);
  //mControlB(MotorSpeedB, MotorDirB);

  // Print speed values to serial monitor for debugging
  Serial.print("Motor A Speed = ");
  Serial.print(MotorSpeedA);
  Serial.print(" | Motor B Speed = ");
  Serial.println(MotorSpeedB);

  // blink LED so we know it's running:
  digitalWrite(carLED, HIGH);
  // Slight delay
  delay(50);
  digitalWrite(carLED, LOW);


}
