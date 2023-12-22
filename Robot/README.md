# Eurobot Robotics Project - Arduino Code and Schematics
## Introduction
This repository contains the Arduino code and schematics of our robot.


## Components

- **Arduino Uno:** The primary microcontroller for managing robot operations.
- **ESP32:** Used for Wi-Fi communication with an external positioning beacon and I2C communication with the Arduino Uno.
- **MPU 6050:** A motion tracking device that includes a gyroscope and accelerometer.
- **Ultrasonic Sensor:** For obstacle detection and distance measurement.
- **Motor Shield:** To control the motors with the Arduino Uno.
- **2 Motors:** Driving the movement of the robot.

## System Overview

The robot's architecture is built around the Arduino Uno, which controls the overall functionality. The ESP32 module, integrated via I2C, provides wireless communication capabilities and interacts with the external positioning system. The MPU 6050 aids in rotation mesurment, while the ultrasonic sensor is utilized for obstacle avoidance and navigation. The motors are managed through the motor shield connected to the Arduino.

The PAMI operates similarly to an RC car. The Raspberry Pi sends power and angle data to the ESP, which then relays it back to the Arduino. We wanted the Arduino to perform as little work as possible and programmed the algorithm on the Raspberry Pi, which has a more global view of the system.

We discovered that using the encoder to measure the angle was inaccurate, so we switched to using a gyroscope. By integrating the velocity, we obtain the angle. The angle is subject to drift proportional to time, but the PAMI's lifetime is only 10 seconds. During this time, the drift is well less than 1 degree.

## Design and Fabrication

We employed Fusion 360 to design the robot. The robot's structure was 3D printed. We designed the robot in multiple smaller parts, ensuring that if any component needed redesigning or replacement, it could be done quickly and efficiently without the need to reprint the entire structure.
Here is the final robot whith all the components : 
![Screenshot](https://raw.githubusercontent.com/TardyNoe/ECAM_Eurobot_2024_TeamErasmus/main/Robot/3D.png)
Here is the 3D parts :
![Screenshot](https://raw.githubusercontent.com/TardyNoe/ECAM_Eurobot_2024_TeamErasmus/main/Robot/3D2.png)

## Code Structure

- **Arduino.ino:** The main Arduino program file.
- **Esp.ino:** Code for ESP32 and Arduino communication.

## Schematic
![Screenshot](https://raw.githubusercontent.com/TardyNoe/ECAM_Eurobot_2024_TeamErasmus/main/Robot/circuit.png)
