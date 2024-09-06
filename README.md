# 3D-Spatial-Scanning-and-Data-Visualization-Project
## Description
The embedded spatial measurement system utilizes a time-of-flight (ToF) sensor and a stepper motor to create detailed maps of indoor environments. Controlled by an MSP432E401Y microcontroller operating at 96MHz, the system collects distance measurements within a single vertical plane. User interaction is facilitated through digital I/O inputs, with PC ‘Enter’ key for system activation and LED indicators for real-time status feedback. Once activated, the ToF sensor rotates 360 degrees, gathering distance samples at fixed intervals.

Data processing and communication are handled by the microcontroller, which sends the acquired measurements to a PC application via UART serial communication. Python scripts, in conjunction with Open3D, visualize the data, generating a 3D rendering of the scanned space. This integrated solution offers a cost-effective and compact alternative to commercial LIDAR systems, providing valuable insights for indoor exploration and navigation.

## Features
- **Microcontroller:** MSP432E401Y with a Cortex M4 chip, operating at 120MHz, utilized for processing, storing, and executing instructions for programmed tasks.
- **Time of Flight Sensor:** VL53L1X, enabling accurate distance measurement of nearby objects.
- **Stepper Motor & Driver:** Rotary device mechanism allowing control over parameters such as speed, angle, and direction.
- **Time of Flight Stepper Motor Mount:** 3D printed mount facilitating the attachment of the Time-of-Flight sensor to the stepper motor.
- **Onboard Microcontroller Push Buttons:** Reset and PJ1 buttons integrated into the MSP432E401Y microcontroller for reset, start, and stop functionalities.
- **Keil Microcontroller Project Software:** Programming software utilized for coding instructions in the C programming language for the MSP432E401Y Microcontroller.
- **AD2 + Waveform Software:** Circuit device analyzer accompanied by software for displaying results of measured frequency and bus speed of the system.
- **Python + Open3D:** PC Application facilitating data extraction from the Time-of-Flight sensor via serial communication (UART & Pyserial). Open3D utilized for graphical mapping of distance coordinates on the PC.
- **Universal Asynchronous Receiver/Transmitter (UART) Communication:** Communication protocol employed by the MSP432E401Y Microcontroller for data exchange between the Time-of-Flight sensor and PC.
- **Bus Speed:** 96 MHz
- **ADC Sampling Rate:** Ensured to be greater than or equal to the signal frequency.
- **Operating Voltage:** 3.3V for Time-of-Flight sensor and 5V for stepper motor
- **Baud Rate:** 115200 bps.
- **LED D4 Status Tracker:** LED indicator providing status tracking.
