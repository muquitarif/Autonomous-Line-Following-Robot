# Autonomous Line Follower Robot Using QTR-8A (Reflectance Sensor Array)

This repository contains the code for a line-following robot implemented on an Arduino platform. The robot utilizes infrared (IR) sensors to detect the position of a line on the ground and adjusts its movements accordingly to follow the line. The control algorithm implemented is Proportional-Integral-Derivative (PID) control to ensure smooth and accurate tracking of the line.

### Components Used:
- **Arduino Board**: The code is written for an Arduino-compatible microcontroller.
- **L298N Dual H-Bridge Motor Driver**: Used to control the motors that drive the robot.
- **QTR Sensors**: Infrared sensors used for line detection.

### Key Features:
- **Calibration**: The robot calibrates its sensor readings at startup to adapt to varying light conditions.
- **Line Following**: The main loop of the code continuously reads sensor values and adjusts motor speeds to keep the robot on the line.
- **PID Control**: Implements a PID controller to compute motor speeds based on the deviation of the robot from the desired line position.
- **Motor Control**: Utilizes PWM signals to control motor speed and direction through the L298N motor driver.

### Constants and Parameters:
- **PID Constants (Kp, Kd)**: Tunable parameters to adjust the response of the PID controller.
- **MaxSpeed, BaseSpeed, SpeedTurn**: Configurable parameters defining maximum speed, base speed, and turning speed of the robot.
- **CheckPoint**: Threshold value for detecting the line position.

### Hardware Setup:
- Motors A and B are connected to the L298N motor driver for controlling movement.
- IR sensors are connected to analog pins for line detection.

### Dependencies

- L298NX2
    https://github.com/AndreaLombardo/L298N
- QTRSensors
    https://github.com/pololu/qtr-sensors-arduino

### Schematics
![Schematics](/Schematics/Line-follower-circuit-diagram.png)

### Usage:
1. Ensure all hardware connections are properly set up.
2. Upload the code to the Arduino board.
3. The LED indicator will signal during calibration.
4. The robot will then continuously follow the line based on sensor readings.

### PID Line Follower Tuning
Complete Guide on How to Tune PID Line Follower
    http://robotresearchlab.com/2019/02/16/pid-line-follower-tuning/

### Note:
- Fine-tune PID constants and other parameters based on specific requirements and environmental conditions.
- Adjust sensor positions and calibration steps for optimal performance on different surfaces.

Feel free to contribute, report issues, or suggest improvements to this project.





