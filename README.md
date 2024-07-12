# Elevator-Control-System
## Overview
The project report details the development of an elevator control system utilizing a dual Proportional-Integral-Derivative (PID) controller to regulate both position and velocity of a DC motor. The system aims to achieve smooth and accurate elevator movement, thereby enhancing user experience and safety. The report covers various aspects including theoretical background, design components, schematics, implementation procedures, and performance evaluation using MATLAB and Arduino.
## Introduction
Elevators are essential vertical transportation systems in modern buildings, requiring precise control for safe and efficient operation. The project focuses on implementing PID controllers to manage elevator car positioning and speed, ensuring passengers experience smooth rides between floors.
## Background Theory
The report begins with an overview of elevator components and control systems. It highlights the importance of PID control in achieving accurate positioning and smooth motion. Theoretical foundations explain how PID controllers manage errors and adjust control signals based on proportional, integral, and derivative terms.
## Mechanical and Electrical Design
Detailed mechanical designs of the elevator structure, including 3D models and schematics, are presented. This includes the elevator cell, body assembly, and H-Bridge layout for motor control. The electrical design covers components such as Arduino Nano, sensors, and motor interface circuits required for system operation.
## Control Design
The PID controller design process is outlined, starting with system identification using MATLAB's System Identification Toolbox to derive transfer functions. The report includes the tuning process for both position and velocity PID controllers, utilizing MATLAB's PID Tuner for optimal parameter adjustment. Performance evaluation through step response, Bode plot, and root locus analysis validates controller effectiveness.
## Implementation
Detailed procedures for connecting components, collecting sensor data, and integrating PID control algorithms into Arduino code are provided. The Arduino code snippet illustrates how position and velocity setpoints are managed using PID outputs to control motor speed and elevator position.
## Conclusion
The project report includes references to software tools (Autodesk Inventor, MATLAB, Arduino IDE), components used, and theoretical sources related to PID control and elevator systems.
