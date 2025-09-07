# ESP32 IMU Pan-Tilt Control

This repo contains the code for my ESP32-based wireless pan–tilt servo control project.  
The system uses an MPU6050 IMU to measure orientation on one ESP32 (the **sender**) and reproduces that motion with a pan–tilt servo mount on another ESP32 (the **receiver**) via ESP-NOW.

For a full write-up with wiring diagrams, photos, latency measurements, and demo videos, check out the blog post here: 👉 

## sender-imu

Reads orientation data from the MPU6050 and sends it to the receiver over ESP-NOW.

## receiver-servo

Receives orientation data and drives the pan–tilt servos through a PCA9685 servo driver.

## Note on Calibration

Some values in the code (like `SERVOMIN`, `SERVOMAX`, and offsets) are based on my hardware and calibration.  
You should adjust these for your own setup.  