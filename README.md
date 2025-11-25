# Drone Control System

This repository contains all code developed for our Swedish upper-secondary _Gymnasiearbete_ project.  
The system implements a complete communication chain for controlling a quadcopter using BLE, LoRa, and an onboard IMU-based flight controller.

All programming in this repository was developed by **Hannes Göök**.

## Repository Structure

```
arduino_rx_drone_control/
mobile_app_2_raspberry/
raspberry_2_arduino/
```

## System Overview

### 1. Mobile App to Raspberry Pi

A Flutter application that sends joystick values, throttle, and PID parameters over BLE.

### 2. Raspberry Pi to Arduino

The Raspberry Pi receives BLE packets, bridges them through LoRa, and transmits them to the Arduino flight controller.

### 3. Arduino Flight Controller

The Arduino receives LoRa packets, parses control inputs, reads IMU data (MPU6050), runs a Madgwick filter, computes PID corrections, and drives four ESCs for quadcopter stabilization.

## License

This project is licensed under the MIT License.
See the `LICENSE` file for details.
