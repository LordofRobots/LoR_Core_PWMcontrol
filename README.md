# LoR_Core_PWMcontrol
This holds all configurations for a MiniBot controlled via PWM signals from a RC controller.

Board: ESP32 dev module

This code is designed for controlling a MiniBot with an ESP32 microcontroller via Pulse Width Modulation (PWM) signals. The ESP32 is programmed using the Arduino platform, and the system is capable of controlling motors through PWM inputs and indicating statuses through an LED system. 

1. **Control Inputs and Indications**: 
The control inputs are taken from PWM signals. LED indications are configured to show the state of the control input, with a blue LED indicating a PWM signal and a red LED indicating standby mode.

2. **Drive Configurations**: 
The system supports two types of drive configurations - Mecanum (omni-directional drive) and Standard Tank style. These configurations dictate how the robot moves in response to input signals.

3. **Version Control and Interface Definitions**: 
The script includes a version control string and defines various interface and motor pins for ESP32.

4. **PWM Configuration Definitions**: 
The ESP32's hardware PWM channels are defined and configured. 

5. **Motor Pin Definitions**: 
The code specifies which GPIO pins on the ESP32 are connected to each motor of the robot.

6. **Interrupt Service Routines (ISR)**:
The ISRs are triggered by changes in the PWM signals. They record the time of the last rising and falling edge of each PWM signal, which is then used to calculate the PWM duty cycle.

7. **NeoPixel Setup**: 
The code sets up and controls a NeoPixel LED strip for visual indications. 

8. **Motion Control**: 
This function calculates the motor speeds based on input from the controller. It handles both Mecanum (omni-directional drive) and standard movement. 

9. **Slew Rate Function**: 
A slew rate function is included to gradually change the motor speeds, preventing abrupt changes that could damage the motors or cause the robot to behave erratically.

10. **Motor Control**: 
This function controls the motor output based on the processed inputs, and it includes a mechanism for ramping motor speed.

11. **PWM Control**: 
This function measures PWM duty cycles from the inputs and maps them to output values suitable for the motor control function.

12. **Setup**: 
The setup function initializes the ESP32's hardware and software resources necessary for robot operation, such as setting up the motor control pins, setting up the NeoPixel strip, configuring the PWM pins, setting up the serial communication, and attaching the interrupt handlers for each PWM input pin.
