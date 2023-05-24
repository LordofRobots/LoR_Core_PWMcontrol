/* LORD of ROBOTS - LoR_Core_PWMcontrol - 202305222118
 
   Control inputs - LED Indication:
    RC pwm - Blue LED
    none/Stop/standby - Red LED

  Drive configurations:
    Mecanum 
    Standard tank style

*/

#include <Adafruit_NeoPixel.h>

// version control and major control function settings
String Version = "Base Version : LoR Core PWM Control : 1.0.0";


// IO Interface Definitions
#define LED_DataPin 12
#define LED_COUNT 36
#define ControllerSelectPin 34
#define MotorEnablePin 13
#define channel1Pin 16
#define channel2Pin 17
#define channel3Pin 21
#define channel4Pin 22


// Motor Pin Definitions
#define motorPin_M1_A 5
#define motorPin_M1_B 14
#define motorPin_M2_A 18
#define motorPin_M2_B 26
#define motorPin_M3_A 23
#define motorPin_M3_B 19
#define motorPin_M4_A 15
#define motorPin_M4_B 33
#define motorPin_M5_A 27
#define motorPin_M5_B 25
#define motorPin_M6_A 32
#define motorPin_M6_B 4
const int motorPins_A[] = { motorPin_M1_A, motorPin_M2_A, motorPin_M3_A, motorPin_M4_A, motorPin_M5_A, motorPin_M6_A };
const int motorPins_B[] = { motorPin_M1_B, motorPin_M2_B, motorPin_M3_B, motorPin_M4_B, motorPin_M5_B, motorPin_M6_B };

// PWM Configuration Definitions
const int Motor_M1_A = 0;
const int Motor_M1_B = 1;
const int Motor_M2_A = 2;
const int Motor_M2_B = 3;
const int Motor_M3_A = 4;
const int Motor_M3_B = 5;
const int Motor_M4_A = 6;
const int Motor_M4_B = 7;
const int Motor_M5_A = 8;
const int Motor_M5_B = 9;
const int Motor_M6_A = 10;
const int Motor_M6_B = 11;
const int MOTOR_PWM_Channel_A[] = { Motor_M1_A, Motor_M2_A, Motor_M3_A, Motor_M4_A, Motor_M5_A, Motor_M6_A };
const int MOTOR_PWM_Channel_B[] = { Motor_M1_B, Motor_M2_B, Motor_M3_B, Motor_M4_B, Motor_M5_B, Motor_M6_B };
const int PWM_FREQUENCY = 20000;
const int PWM_RESOLUTION = 8;

// Define variables to store the PWM signal values
int PWM_Input_LY = 0;
int PWM_Input_LX = 0;
int PWM_Input_RY = 0;
int PWM_Input_RX = 0;

// Define variables to store the PWM signal values
volatile int channel1Value = 0;
volatile int channel2Value = 0;
volatile int channel3Value = 0;
volatile int channel4Value = 0;

// Define variables to store the time of the last pulse on each PWM channel
volatile unsigned long channel1LastRising = 0;
volatile unsigned long channel1LastFalling = 0;
volatile unsigned long channel2LastRising = 0;
volatile unsigned long channel2LastFalling = 0;
volatile unsigned long channel3LastRising = 0;
volatile unsigned long channel3LastFalling = 0;
volatile unsigned long channel4LastRising = 0;
volatile unsigned long channel4LastFalling = 0;

// Define the interrupt service routines for each PWM channel
void IRAM_ATTR channel1ISR() {
  unsigned long now = micros();
  if (digitalRead(channel1Pin) == HIGH) {
    channel1LastRising = now;
  } else {
    channel1Value = now - channel1LastRising;
    channel1LastFalling = now;
  }
}

void IRAM_ATTR channel2ISR() {
  unsigned long now = micros();
  if (digitalRead(channel2Pin) == HIGH) {
    channel2LastRising = now;
  } else {
    channel2Value = now - channel2LastRising;
    channel2LastFalling = now;
  }
}

void IRAM_ATTR channel3ISR() {
  unsigned long now = micros();
  if (digitalRead(channel3Pin) == HIGH) {
    channel3LastRising = now;
  } else {
    channel3Value = now - channel3LastRising;
    channel3LastFalling = now;
  }
}

void IRAM_ATTR channel4ISR() {
  unsigned long now = micros();
  if (digitalRead(channel4Pin) == HIGH) {
    channel4LastRising = now;
  } else {
    channel4Value = now - channel4LastRising;
    channel4LastFalling = now;
  }
}


// Set a specific color for the entire NeoPixel strip
// NeoPixel Configurations
Adafruit_NeoPixel strip(LED_COUNT, LED_DataPin, NEO_GRB + NEO_KHZ800);
const uint32_t RED = strip.Color(255, 0, 0, 0);
const uint32_t GREEN = strip.Color(0, 255, 0, 0);
const uint32_t BLUE = strip.Color(0, 0, 255, 0);
const uint32_t WHITE = strip.Color(0, 0, 0, 255);
const uint32_t PURPLE = strip.Color(255, 0, 255, 0);
const uint32_t CYAN = strip.Color(0, 255, 255, 0);
const uint32_t YELLOW = strip.Color(255, 255, 0, 0);
void NeoPixel_SetColour(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {  // For each pixel in strip...
    strip.setPixelColor(i, color);               //  Set pixel's color (in RAM)
    strip.show();                                // Update strip with new contents
  }
}

// Rainbow pattern for NeoPixel strip
long firstPixelHue = 0;
void NeoPixel_Rainbow() {
  strip.rainbow(firstPixelHue);
  strip.show();  // Update strip with new contents
  firstPixelHue += 256;
  if (firstPixelHue >= 5 * 65536) firstPixelHue = 0;
}

// Process input and calculate motor speeds 
bool MecanumDrive_Enabled = true;
const int DEAD_BAND = 20;
const float TURN_RATE = 1.5;

bool INVERT = false;
int Motor_FrontLeft_SetValue, Motor_FrontRight_SetValue, Motor_BackLeft_SetValue, Motor_BackRight_SetValue = 0;
void Motion_Control(int LY_Axis, int LX_Axis, int RX_Axis) {

  int FrontLeft_TargetValue, FrontRight_TargetValue, BackLeft_TargetValue, BackRight_TargetValue = 0;
  int ForwardBackward_Axis = LY_Axis;
  int StrafeLeftRight_Axis = LX_Axis;
  int TurnLeftRight_Axis = -RX_Axis;

  //Set deadband
  if (abs(ForwardBackward_Axis) < DEAD_BAND) ForwardBackward_Axis = 0;
  if (abs(StrafeLeftRight_Axis) < DEAD_BAND) StrafeLeftRight_Axis = 0;
  if (abs(TurnLeftRight_Axis) < DEAD_BAND) TurnLeftRight_Axis = 0;

  //Calculate strafe values
  FrontLeft_TargetValue = -ForwardBackward_Axis + (StrafeLeftRight_Axis * MecanumDrive_Enabled);
  BackLeft_TargetValue = -ForwardBackward_Axis - (StrafeLeftRight_Axis * MecanumDrive_Enabled);
  FrontRight_TargetValue = ForwardBackward_Axis + (StrafeLeftRight_Axis * MecanumDrive_Enabled);
  BackRight_TargetValue = ForwardBackward_Axis - (StrafeLeftRight_Axis * MecanumDrive_Enabled);

  //calculate rotation values
  if (abs(TurnLeftRight_Axis) > DEAD_BAND) {
    FrontLeft_TargetValue += (TURN_RATE * TurnLeftRight_Axis);
    BackLeft_TargetValue += (TURN_RATE * TurnLeftRight_Axis);
    FrontRight_TargetValue += (TURN_RATE * TurnLeftRight_Axis);
    BackRight_TargetValue += (TURN_RATE * TurnLeftRight_Axis);
  }

  //constrain to joystick range
  FrontLeft_TargetValue = constrain(FrontLeft_TargetValue, -127, 127);
  BackLeft_TargetValue = constrain(BackLeft_TargetValue, -127, 127);
  FrontRight_TargetValue = constrain(FrontRight_TargetValue, -127, 127);
  BackRight_TargetValue = constrain(BackRight_TargetValue, -127, 127);

  //set motor speed through slew rate function
  Motor_FrontLeft_SetValue = SlewRateFunction(FrontLeft_TargetValue, Motor_FrontLeft_SetValue);
  Motor_FrontRight_SetValue = SlewRateFunction(FrontRight_TargetValue, Motor_FrontRight_SetValue);
  Motor_BackLeft_SetValue = SlewRateFunction(BackLeft_TargetValue, Motor_BackLeft_SetValue);
  Motor_BackRight_SetValue = SlewRateFunction(BackRight_TargetValue, Motor_BackRight_SetValue);
}

// Function to handle slew rate for motor speed ramping
// Slew rate for ramping motor speed
const int SLEW_RATE_MS = 20;
int SlewRateFunction(int Input_Target, int Input_Current) {
  int speedDiff = Input_Target - Input_Current;
  if (speedDiff > 0) Input_Current += min(speedDiff, SLEW_RATE_MS);
  else if (speedDiff < 0) Input_Current -= min(-speedDiff, SLEW_RATE_MS);
  constrain(Input_Current, -127, 127);
  return Input_Current;
}

// Function to control motor output based on input values
const int MAX_SPEED = 255;
const int MIN_SPEED = -255;
const int MIN_STARTING_SPEED = 140;
const int STOP = 0;
const int SerialControl_SPEED = 110;
void Set_Motor_Output(int Output, int Motor_ChA, int Motor_ChB) {
  if (INVERT) Output = -Output;

  Output = constrain(Output, -127, 127);

  int Mapped_Value = map(abs(Output), 0, 127, MIN_STARTING_SPEED, MAX_SPEED);
  int A, B = 0;
  if (Output < -DEAD_BAND) {  // Rotate Clockwise
    A = 0;
    B = Mapped_Value;
  } else if (Output > DEAD_BAND) {  // Rotate Counter-Clockwise
    A = Mapped_Value;
    B = 0;
  } else {  // Rotation Stop
    A = STOP;
    B = STOP;
  }
  ledcWrite(Motor_ChA, A);  //send to motor control pins
  ledcWrite(Motor_ChB, B);
}

void Motor_Control() {
  Set_Motor_Output(Motor_FrontLeft_SetValue, Motor_M1_A, Motor_M1_B);
  Set_Motor_Output(Motor_BackLeft_SetValue, Motor_M2_A, Motor_M2_B);
  Set_Motor_Output(Motor_FrontRight_SetValue, Motor_M5_A, Motor_M5_B);
  Set_Motor_Output(Motor_BackRight_SetValue, Motor_M6_A, Motor_M6_B);
}

void Motor_STOP() {
  Set_Motor_Output(STOP, Motor_M1_A, Motor_M1_B);
  Set_Motor_Output(STOP, Motor_M2_A, Motor_M2_B);
  Set_Motor_Output(STOP, Motor_M5_A, Motor_M5_B);
  Set_Motor_Output(STOP, Motor_M6_A, Motor_M6_B);
}

// measueing PWM duty cycles and mapping for output values
boolean PWM_Control() {
  PWM_Input_LY = int(map(channel1Value, 1000, 2000, -127, 127) * 2);
  PWM_Input_LX = int(map(channel2Value, 1000, 2000, -127, 127) * 2);
  PWM_Input_RY = int(map(channel3Value, 1000, 2000, -127, 127));
  PWM_Input_RX = int(map(channel4Value, 1000, 2000, -127, 127));
  if (PWM_Input_LY + PWM_Input_LX + PWM_Input_RX + PWM_Input_RY < (-128 * 4)) return false;  // no signal threshold
  return true;
}

// Set up pins, LED PWM functionalities and begin PS4 controller, Serial and Serial2 communication
void setup() {
  // Set up the pins
  pinMode(LED_DataPin, OUTPUT);
  pinMode(ControllerSelectPin, INPUT_PULLUP);
  pinMode(MotorEnablePin, OUTPUT);

  for (int i = 0; i < 6; i++) {
    pinMode(motorPins_A[i], OUTPUT);
    pinMode(motorPins_B[i], OUTPUT);
    digitalWrite(motorPins_A[i], 0);
    digitalWrite(motorPins_B[i], 0);
  }

  // configureation of aux pins for pwm control (alternative control method)
  
    pinMode(channel1Pin, INPUT_PULLDOWN);
    pinMode(channel2Pin, INPUT_PULLDOWN);
    pinMode(channel3Pin, INPUT_PULLDOWN);
    pinMode(channel4Pin, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(channel1Pin), channel1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel2Pin), channel2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel3Pin), channel3ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel4Pin), channel4ISR, CHANGE);


  // output preset bias
  digitalWrite(LED_DataPin, 0);
  digitalWrite(MotorEnablePin, 1);

  // configure LED PWM functionalitites
  for (int i = 0; i < 6; i++) {
    ledcSetup(MOTOR_PWM_Channel_A[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(MOTOR_PWM_Channel_B[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(motorPins_A[i], MOTOR_PWM_Channel_A[i]);
    ledcAttachPin(motorPins_B[i], MOTOR_PWM_Channel_B[i]);
  }

  // Neopixels Configuration
  strip.begin();            // INITIALIZE NeoPixel strip object
  strip.show();             // Turn OFF all pixels ASAP
  strip.setBrightness(50);  // Set BRIGHTNESS to about 1/5 (max = 255)

  // Serial comms configurations (USB for debug messages)
  Serial.begin(115200);  // USB Serial
  delay(1500);

 
  Serial.print("Mecanum Drive: ");
  if (MecanumDrive_Enabled) Serial.println("Enabled");
  else Serial.println("Disabled");

  Serial.println("CORE System Ready! " + Version);
}

void loop() {
  if (PWM_Control()) {
    NeoPixel_SetColour(BLUE);
    Motion_Control(PWM_Input_LY, PWM_Input_LX, PWM_Input_RX);  // RC control
    Motor_Control();

    //Stop/Standby
  } else {
    NeoPixel_SetColour(RED);
    Motor_STOP();
  }
}
