# Wireless ESP32 Robot Arm Controller

This project is an extension of the robot arm project shown in this video:
https://youtu.be/mH10h8SrDmM?si=zd_d3eXHVhMORJaU

The original robot arm was modified with a wireless ESP32 control system. I added two ESP32 boards: one ESP32 is used as the **master/controller**, and the other ESP32 is used as the **slave/receiver** on the robot arm. This makes it possible to control the robot arm without any cable between the controller and the robot thanks to esp now.

The controller can control all servos of the robot arm wirelessly. It also includes a rotary encoder that allows the user to adjust the movement speed/sensitivity depending on what is needed. The wireless range is much more than required for normal use, and the controller has its own separate battery pack.

## Project Overview

The system consists of two main parts:

### 1. Master ESP32 / Wireless Controller

The master ESP32 is inside the handheld controller. It reads all input devices and sends the control data wirelessly to the slave ESP32 using ESP-NOW.

The controller includes:

* 2 analog joysticks
* Joystick push buttons
* 5 additional push buttons
* 1 rotary encoder with push button
* 4 sensitivity indicator LEDs
* 1 connection/status LED
* Battery pack
* MP1584 step-down module
* Battery indicator

### 2. Slave ESP32 / Robot Arm Receiver

The slave ESP32 is connected to the robot arm electronics. It receives the wireless control data from the master ESP32 and controls the servos of the robot arm.

## Wireless Communication

The communication between the controller and the robot arm is done with **ESP-NOW**.

ESP-NOW is useful for this project because:

* it does not require a Wi-Fi router
* it has low latency
* it works well for remote control projects
* it is simple to use between two ESP32 boards

## Controller Features

* Wireless robot arm control
* Two joysticks for multi-axis movement
* Extra push buttons for additional robot arm functions
* Rotary encoder for changing speed/sensitivity
* LED bar for sensitivity level indication
* Separate status LED for connection feedback
* Portable controller with its own battery pack
* MP1584 step-down module for power regulation
* Battery indicator connected to the controller power output

## Controller Pinout

The controller pinout is based on the master ESP32 code.

| Function                         | ESP32 GPIO |
| -------------------------------- | ---------- |
| Joystick 1 X-axis                | GPIO34     |
| Joystick 1 Y-axis                | GPIO35     |
| Joystick 1 button                | GPIO27     |
| Joystick 2 X-axis                | GPIO32     |
| Joystick 2 Y-axis                | GPIO33     |
| Joystick 2 button                | GPIO26     |
| Rotate clockwise button          | GPIO12     |
| Reset button                     | GPIO13     |
| Rotate counter-clockwise button  | GPIO14     |
| Servo 0 clockwise button         | GPIO22     |
| Servo 0 counter-clockwise button | GPIO23     |
| Sensitivity LED 1                | GPIO18     |
| Sensitivity LED 2                | GPIO19     |
| Sensitivity LED 3                | GPIO21     |
| Sensitivity LED 4                | GPIO25     |
| Rotary encoder CLK               | GPIO4      |
| Rotary encoder DT                | GPIO16     |
| Rotary encoder SW                | GPIO17     |
| Status LED                       | GPIO2      |

## Controller Wiring

The controller wiring was created in Cirkit Designer.

Controller wiring link:
https://app.cirkitdesigner.com/project/ed343159-9c17-45b6-8086-4fa38f00c40d

The controller uses a battery pack connected to an MP1584 step-down module. The output of the MP1584 module is connected to the ESP32 VIN and GND pins. The battery indicator is connected to the output side of the MP1584 module.

### Controller Power Wiring

| From                     | To                |
| ------------------------ | ----------------- |
| Battery pack positive    | Main power switch |
| Main power switch output | MP1584 IN+        |
| Battery pack negative    | MP1584 IN-        |
| MP1584 OUT+              | ESP32 VIN         |
| MP1584 OUT-              | ESP32 GND         |
| Battery indicator +      | MP1584 OUT+       |
| Battery indicator -      | MP1584 OUT-       |

## Buttons

All push buttons are connected between the ESP32 GPIO pin and GND.

The code uses `INPUT_PULLUP`, so the buttons are active-low. This means:

* button not pressed = HIGH
* button pressed = LOW

## Sensitivity LEDs

The controller has four LEDs to show the current sensitivity level.

LED order:

| LED  | Position    | Color | GPIO   |
| ---- | ----------- | ----- | ------ |
| LED1 | bottom      | blue  | GPIO18 |
| LED2 | middle-low  | blue  | GPIO19 |
| LED3 | middle-high | blue  | GPIO21 |
| LED4 | top         | red   | GPIO25 |

There is also one separate status LED between the buttons.

| LED        | Function          | GPIO  |
| ---------- | ----------------- | ----- |
| Status LED | connection status | GPIO2 |

## Rotary Encoder

The rotary encoder is used to change the sensitivity/speed level while using the controller.

| Encoder Pin | ESP32 GPIO |
| ----------- | ---------- |
| CLK         | GPIO4      |
| DT          | GPIO16     |
| SW          | GPIO17     |

The encoder button resets the sensitivity level back to the start value.

## Code Structure

The project uses two Arduino files:

| File         | Purpose                                   |
| ------------ | ----------------------------------------- |
| `master.ino` | Code for the handheld wireless controller |
| `slave.ino`  | Code for the ESP32 on the robot arm       |

The master ESP32 reads the joysticks, buttons, encoder, and LEDs. It sends a data structure wirelessly to the slave ESP32.

The slave ESP32 receives the data and controls the servos on the robot arm.

## Notes

This project is still based on the original robot arm concept from the YouTube video linked above. The main improvement is the custom wireless controller with two ESP32 boards.

The controller was designed to make the robot arm easier and more comfortable to use without needing a cable connection.

## Credits

Original robot arm project inspiration:
https://youtu.be/mH10h8SrDmM?si=zd_d3eXHVhMORJaU

Controller wiring created with Cirkit Designer:
https://app.cirkitdesigner.com/project/ed343159-9c17-45b6-8086-4fa38f00c40d
