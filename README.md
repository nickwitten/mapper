# 4180_team_project_2D_mapping_bot

This is a final team based project for Georgia Tech ECE4180 Embedded System Design Course.

## Team Members:

    Juntao Wang: jwang3046@gatech.edu
    Sicheng Zou: Sicheng.zou@gatech.edu
    Nick Witten: nick.witten@gatech.edu
    
## Overview:

The project is to create a room mapping robot. The purpose of the robot is to autonomously roam a room and plot the 2D coordinates of walls and obstacles while expanding its knowledge in a frontier-based fashion.

## Design Description:

The shadow bot chassis is used as the frame of the robot; 2 DC motors with a dual H-Bridge driver module are used to maneuver the robot; 3 LIDAR sensors are used to measure distances while the robot is moving. The whole system is based on the Mbed LPC1768 running Mbed RTOS. In addition, a raspberry pi is used for data manipulation in Python. The robot is also able to flash code wirelessly using wireless connection on raspberry pi and communication between raspberry pi and Mbed. 

## Software:

The virtual COM port is used to control the Mapper through the Raspberry Pi.  Make sure the Pi has internet connection, use SSH to obtain a terminal.  Clone this repository, and use gcc-arm-none-eabi compiler to build the MBED source code.  A USB micro type B to a USB mini type B cable should be used to connect the Raspberry Pi to the MBED.  To flash the binary file, copy it to /media/pi/MBED.  The two python scripts are to be run on the Pi and a personal PC, not connected to the Pi.

## Usage:

To control the robot, open the serial port to the MBED from the Pi.  This can be done with the following commands:

    pip install pyserial
    python -m serial.tools.miniterm /media/pi/MBED 9600

The following commands can be issued over the serial port

    Arrow Up:    Speed Up
    Arrow Downs: Slow Down
    Arrow Left:  Turn Counterclockwise 90 Degrees
    Arrow Right: Turn Clockwise 90 Degrees
    r:           Reset State and Map
    a:           Toggle Autonomous Mode
    Enter:       Print Mapper State
    
The two python scripts can be used to enable plotting.  pi_saver.py should be run on the Pi and pc_plotter.py should be run on a PC.  Commands
can still be issued while running the pi_saver.py script, but they should typed into the script STDIN and enter must be issued after every
keypress so they can be passed through to the MBED.  Make sure the Pi's IP address is updated in the pc_plotter.py script.

## Part List:

   [mbed LPC1768](https://www.sparkfun.com/products/9564)  
   [Shadow Bot Chassis](https://www.sparkfun.com/products/13301)  
   [SparkFun Motor Driver - Dual TB6612FNG (with Headers)](https://www.sparkfun.com/products/14450)  
   [Hobby Gearmotor - 140 RPM (Pair)](https://www.sparkfun.com/products/13302)  
   [2x Wheels](https://www.sparkfun.com/products/13259)  
   [3x VL53L0X ToF Sensor](https://www.adafruit.com/product/3317)  
   [2x Wheel Encoder Kit ](https://www.sparkfun.com/products/12629)  
   [5V Battery Pack](https://www.sparkfun.com/products/9835)  
   [DC Barrel Jack Adapter](https://www.sparkfun.com/products/10811)  
   [Raspberry Pi Zero W](https://www.sparkfun.com/products/14277)  
    
## Schematic:

### Motor Drivers and Dual Motors:

   | Mbed LPC1768  | Motor Driver |  Motor_Left | Motor_Right |
   | :---: | :---: | :---: | :---: |
   | VOUT         |  VCC |
   | GND          |  GND |
   |              | VM (to Power) |
   |            |  A01     |       RED |
   |            |   A02     |       BLACK |
   |            |   B01      |          |       RED |
   |            |    B02      |          |       BLACK |
   | p21        |    PWMA |
   | p22        |    PMWB |
   | p6         |    AI1  |
   | p7         |    BI1  |
   | p5         |    AI2  |
   | p8         |    BI2  |

### LIDAR Sensors:

   | Mbed LPC1768 | LIDAR_Right  | LIDAR_Left  | LIDAR_Center |
   | :---: | :---: | :---: | :---: |
   | VOUT         | VIN          | VIN          | VIN |
   | GND          |  GND         |  GND        |  GND |
   | p28          |  SDA         |  SDA        |  SDA|
   | p27          |  SCL       |    SCL       |   SCL|
   | p24          |  SHDN      |
   | p25          |             |   SHDN |
   | p26          |             |            | SHDN |

### Encoders:

   | Mbed LPC1768 |   Encoder_Left |  Encoder_Right |
   | :---: | :---: | :---: |
   | VOUT      |     RED         |   RED |
   | GND        |    BLACK       |   BLACK |
   | p11        |    WHITE |
   | p12         | |                  WHITE |

## Controller State Diagram:
![flow_chart_mapper](https://user-images.githubusercontent.com/64867842/166290131-8ed56b9a-3980-4f2d-981d-875d3332afb2.jpg)

## Autonomous Mode State Diagram:
![Automonus_Mode_Flow_Chart](https://user-images.githubusercontent.com/64867842/166316527-de09a802-df6c-48b6-8c22-c4eb831cb8b8.jpg)

## [Source Codes](https://github.com/Ericjuntao/4180_team_project_2D_mapping_bot/tree/main/src)

## Photos:
### Robot Front View:

![image](https://user-images.githubusercontent.com/103451305/166268167-431d8e7a-00c7-478a-a439-cd10e1d252b3.jpeg)

### Robot Top View:

![image](https://user-images.githubusercontent.com/103451305/166268255-cac2833a-e02a-4056-81dd-a8b20729ecb8.jpeg)

### Robot Motor and Encoder:
![image](https://user-images.githubusercontent.com/103451305/166268281-64000073-e4c9-41c7-bfc8-bb1114eec288.jpeg)

## Videos:

### Automatic Mapping Demo:
[![5d426e33601fc5a9d46645a86f8db33](https://user-images.githubusercontent.com/64867842/166330112-a8ef231e-3e2b-4b0d-a220-d8afc62608e4.png)](https://www.youtube.com/watch?v=H9ovxw3yx7E&ab_channel=ZouSicheng)

### Remote Control Mapping Demo:
[![0ea3cb7534f60492225665e88f2c3e7](https://user-images.githubusercontent.com/64867842/166330598-6b51ec51-b8f4-447f-ba78-00ca33d45f72.png)](https://www.youtube.com/watch?v=4T7IN5BH2GQ&ab_channel=ZouSicheng)





