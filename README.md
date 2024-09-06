# 3D-Spatial-Scanning-and-Data-Visualization-Project
## Description
The embedded spatial measurement system utilizes a time-of-flight (ToF) sensor and a stepper motor to create detailed maps of indoor environments. Controlled by an MSP432E401Y microcontroller operating at 96MHz, the system collects distance measurements within a single vertical plane. User interaction is facilitated through digital I/O inputs, with PC ‘Enter’ key for system activation and LED indicators for real-time status feedback. Once activated, the ToF sensor rotates 360 degrees, gathering distance samples at fixed intervals.

Data processing and communication are handled by the microcontroller, which sends the acquired measurements to a PC application via UART serial communication. Python scripts, in conjunction with Open3D, visualize the data, generating a 3D rendering of the scanned space. This integrated solution offers a cost-effective and compact alternative to commercial LIDAR systems, providing valuable insights for indoor exploration and navigation.

