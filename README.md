# 3D-Spatial-Scanning-and-Data-Visualization-Project

## Project Description

The Embedded Spatial Measurement System is designed to scan and create a detailed 3D rendering of indoor environments. Utilizing an MSP432E401Y microcontroller, a VL53L1X Time-of-Flight (ToF) sensor, and a stepper motor, this system captures distance measurements and generates a comprehensive spatial map of its surroundings. The integration of various hardware and software components ensures precise data collection and visualization, providing an effective and cost-efficient alternative to traditional LIDAR systems.

## Key Features and Specifications

- **Microcontroller:** The system is powered by the MSP432E401Y microcontroller, featuring a Cortex M4 chip operating at 120MHz. This microcontroller manages data processing, storage, and execution of programmed tasks.
- **Time-of-Flight Sensor:** The VL53L1X ToF sensor measures the distance to nearby objects with high accuracy by emitting infrared laser pulses and calculating the time it takes for the pulses to return.
- **Stepper Motor & Driver:** A rotary device mechanism is employed to control parameters such as speed, angle, and direction, allowing for precise movement of the sensor.
- **Time-of-Flight Stepper Motor Mount:** A 3D-printed mount attaches the ToF sensor to the stepper motor, facilitating stable and accurate sensor positioning.
- **Push Buttons:** Onboard microcontroller push buttons (Reset and PJ1) enable reset, start, and stop functionalities.
- **Programming and Communication:** The system uses Keil software for programming in C and Python IDLE with Open3D for data visualization. Data communication between the microcontroller and PC is managed via UART (115200 bps).
- **Data Analysis:** AD2 + Waveform Software provides a circuit device analyzer for measuring frequency and bus speed. Python libraries (PySerial, Open3D) are utilized for data extraction and 3D rendering.

## Operation

Upon activation, the ToF sensor rotates 360 degrees, collecting distance measurements at fixed intervals. The data acquisition is controlled by the microcontroller, which uses I2C for sensor communication and UART for transmitting data to a PC. The collected measurements are processed and converted into 3D coordinates, which are then visualized using Python and Open3D. The visualization provides a detailed rendering of the scanned space, offering valuable insights into the environment's structure.

## System Components

- **Microcontroller Bus Speed:** 96MHz
- **Operating Voltages:** 3.3V for the ToF sensor, 5V for the stepper motor
- **ADC Sampling Rate:** Greater than or equal to signal frequency
- **LED Indicators:** Status tracking via LED D4
- **Cost Estimate:** $200 CAD (excluding AD2 device)
  
## Detailed Functionality

- **Distance Measurement:** The ToF sensor emits infrared pulses and measures the return time to determine distance. Data points are collected at regular intervals as the sensor rotates, and the microcontroller controls this process.
- **Data Transmission:** Data is transferred from the microcontroller to the PC via UART. Python scripts process and visualize the data, converting distance measurements into 3D coordinates.
- **Visualization:** The Python code utilizes Open3D to load coordinates, create a point cloud, and generate a 3D model of the scanned environment. This model provides a visual representation of the spatial layout, enhancing navigation and exploration.

## Summary

The Embedded Spatial Measurement System integrates advanced hardware and software to deliver an efficient and precise 3D scanning solution. By combining a high-performance microcontroller with a sophisticated ToF sensor and stepper motor, this project offers an affordable alternative to commercial LIDAR systems, making it ideal for indoor mapping and spatial analysis.
