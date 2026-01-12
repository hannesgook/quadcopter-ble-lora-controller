# Drone Control System

This repository contains all code developed for our Swedish upper-secondary _Gymnasiearbete_ project.  
The system implements a complete communication chain for controlling a quadcopter using BLE, LoRa, and an onboard IMU-based flight controller.

All programming in this repository was developed by **Hannes Göök**.

## Repository Structure

```
arduino_rx_drone_control/
mobile_app_2_raspberry/
raspberry_2_arduino/
images/
```

## System Overview

### 1. Mobile App to Raspberry Pi

A Flutter app that sends joystick values, throttle, and PID parameters over BLE.
BLE is used only for short-range communication between the mobile app and the Raspberry Pi.

### 2. Raspberry Pi to Arduino

The Raspberry Pi receives BLE packets, bridges them through LoRa, and transmits them to the Arduino flight controller.
LoRa is used here to get reliable long-range control without relying on Wi-Fi or BLE.

### 3. Arduino Flight Controller

The Arduino (ATmega328P in our case) receives LoRa packets, parses control inputs, reads IMU data (MPU6050), runs a Madgwick filter, computes PID corrections, and drives four ESCs for quadcopter stabilization.

### Failsafe Behavior

The flight controller implements a communication failsafe based on LoRa packet reception.

If no valid packet is received for **1000 ms**, failsafe is triggered:
- All ESCs are immediately set to **1000 µs**
- PID computations are skipped
- Motors remain at minimum throttle until communication resumes

The failsafe state is automatically cleared as soon as a new valid LoRa packet is received.

Failsafe triggers **~1000 ms** after pressing **Disconnect** in the app, since no further LoRa packets are received.

### Control Notes

Setting throttle to zero does not always fully stop all motors if yaw PID gains are not set to zero.
Yaw corrections can still introduce motor output at low throttle.

To guarantee full motor cutoff, the communication failsafe must be triggered or the system must be disarmed.

## Quadcopter Prototype

![Quadcopter top view](images/quadcopter_top_view.jpg)

## Full System Setup

![Full system setup](images/full_system_setup.jpg)

## Status

The system has been tested on a real quadcopter built together with my project partners, and is able to take off and fly under manual control from the mobile app.

Due to a simple frame design, limited mechanical tuning, and non-final PID parameters, the quadcopter is not perfectly stable and tends to drift. The controller is intended as a working prototype rather than a production-grade flight controller.

## Dependencies and Notes

This project relies on a modified version of the RadioHead library.

Changes made:
- The file `RH_ASK.cpp` was disabled (renamed to `RH_ASK.cpp.disabled`) to avoid conflicts with the LoRa module.
- Atomic sections (`ATOMIC_BLOCK_START` / `ATOMIC_BLOCK_END`) were removed in `RHSPIDriver.cpp` to prevent blocking behavior that interfered with motor timing.

These changes were required to ensure stable real-time motor control. Without them, motor timing was unreliable.

### Mobile App Notes

The mobile app is provided as Flutter source code (`mobile_app_2_raspberry/lib/main.dart`) together with dependency definitions. Platform-specific files can be generated using `flutter create`.

### Possible Next Steps

- Add a barometer module to estimate altitude and enable basic altitude hold.
- Integrate GPS to compensate for horizontal drift.

## License

This project is licensed under the MIT License.
See the `LICENSE` file for details.
