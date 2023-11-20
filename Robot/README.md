# Eurobot Robotics Project - Arduino Code and Schematics
![alt text]([http://url/to/img.png](https://github.com/TardyNoe/ECAM_Eurobot_2024_TeamErasmus/blob/main/Robot/3D.png?raw=true))
## Introduction

This repository contains the Arduino code and schematics for a key part of our robotic project developed for the Eurobot contest. The robot is designed with a focus on efficiency, accuracy, and adaptability.

## Components

- **Arduino Uno:** The primary microcontroller for managing robot operations.
- **ESP32:** Used for Wi-Fi communication with an external positioning beacon and I2C communication with the Arduino Uno.
- **MPU 6050:** A motion tracking device that includes a gyroscope and accelerometer.
- **Ultrasonic Sensor:** For obstacle detection and distance measurement.
- **Motor Shield:** To control the motors with the Arduino Uno.
- **2 Motors:** Driving the movement of the robot.

## System Overview

The robot's architecture is built around the Arduino Uno, which controls the overall functionality. The ESP32 module, integrated via I2C, provides wireless communication capabilities and interacts with the external positioning system. The MPU 6050 aids in rotation mesurment, while the ultrasonic sensor is utilized for obstacle avoidance and navigation. The motors are managed through the motor shield connected to the Arduino.

## Code Structure

- **Arduino.ino:** The main Arduino program file.
- **Esp.ino:** Code for ESP32 and Arduino communication.

## Schematic
