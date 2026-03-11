Here is a draft for your project's `README.md` file based on the details from your design documents:

# Improved Etch-a-Sketch

**Developed by:** Nathan Goldstein and James Gammie 
<img width="704" height="932" alt="Screenshot 2026-03-11 at 3 38 48 PM" src="https://github.com/user-attachments/assets/033cb6a2-a60a-4f80-b5c5-23d6cb9dbcea" />

## Project Introduction

This project is an enhanced version of the classic Etch-a-Sketch entertainment system, designed to address several inherent design limitations of the original toy.



<img width="459" height="819" alt="Screenshot 2026-03-11 at 3 38 59 PM" src="https://github.com/user-attachments/assets/5bffbfeb-6d3a-4c4b-805f-55f464553398" />



## Project Goal

The primary goal of this project is to create a more versatile and accessible drawing experience. Key improvements include:

* **Controllable Drawing:** The ability to stop a line from being drawn by "picking up" the pen.


* **Precision Erasing:** A dedicated eraser tool that allows for more precise corrections than the traditional "shake-to-erase" method.


* **Improved Accessibility:** Eliminating the need for physical shaking, making the device easier to use for everyone.

<img width="1422" height="1077" alt="Screenshot 2026-03-11 at 3 39 11 PM" src="https://github.com/user-attachments/assets/45fc1350-67f4-435e-9260-71708ce8945e" />


## Software Overview

The software is structured around an input-driven state machine with four main tasks: reading potentiometers, setting motor positions, calibration, and tool control.

* **Potentiometer Mapping:** The system uses two one-shot ADCs to constantly read the values from the X and Y potentiometers. These values are mapped directly to motor positions, where the minimum potentiometer value corresponds to the 0 position and the maximum value corresponds to the maximum reach of the gantry.


* **Calibration (Homing Sequence):** On startup, the system performs an automatic calibration sequence using limit switches. The gantry moves along each axis until it triggers a switch, establishing the precise (0,0) home position and the maximum boundaries for accurate and repeatable drawing.


* **Callback Functions & Control:** The code utilizes non-blocking tasks to manage motor updates and check for tool change inputs. When a tool change is detected, a dedicated task updates the servo position to toggle between the pen and the eraser.

<img width="803" height="1000" alt="Screenshot 2026-03-11 at 3 39 35 PM" src="https://github.com/user-attachments/assets/6e56f734-99e7-4ff8-9c7e-e2098c6a792e" />


## Hardware Overview

The hardware system is built on an **XY gantry** design that utilizes:

* **Movement:** Two stepper motors drive a rack-and-pinion system to move a carriage precisely across the drawing surface in both X and Y directions.


* **User Interface:** Two potentiometers (knobs) provide intuitive control for movement, mimicking the classic Etch-a-Sketch experience.


* **Tool Head:** A small servo motor is mounted on the carriage, featuring a dual attachment that holds both a marker and an eraser. This allows for automated switching between drawing and erasing modes.


* **Precision:** Hardware limit switches are installed at the axis boundaries to facilitate the automated homing and calibration process.
