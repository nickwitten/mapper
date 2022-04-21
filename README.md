# 4180_team_project_2D_mapping_bot

This is a final team based project for Georgia Tech ECE4180 Embedded System Design Course.

Team Members:

    Juntao Wang: jwang3046@gatech.edu
    Sicheng Zou: Sicheng.zou@gatech.edu
    Nick Witten: nick.witten@gatech.edu
    
Overview:

The project is to create a room mapping robot. The purpose of the robot is to autonomously roam a room and plot the 2D coordinates of walls and obstacles while expanding its knowledge in a frontier-based fashion.

Design Description:

The shadow bot chassis is used as the frame of the robot; 2 DC motors with a dual H-Bridge driver module are used to maneuver the robot; 3 LIDAR sensors are used to measure distances while the robot is moving. The whole system is based on the Mbed LPC1768 running Mbed RTOS. In addition, a raspberry pi is used for data manipulation in Python. The robot is also able to flash code wirelessly using wireless connection on raspberry pi and communication between raspberry pi and Mbed. 

Part List:

    mbed LPC1768 (https://www.sparkfun.com/products/9564)
    Shadow Bot Chassis: (https://www.sparkfun.com/products/13301)
    SparkFun Motor Driver - Dual TB6612FNG (with Headers) (https://www.sparkfun.com/products/14450)
    Hobby Gearmotor - 140 RPM (Pair) (https://www.sparkfun.com/products/13302)
    2x Wheels (https://www.sparkfun.com/products/13259)
    3x VL53L0X ToF Sensor (https://www.adafruit.com/product/3317)
    2x Wheel Encoder Kit (https://www.sparkfun.com/products/12629)
    5V Battery Pack (https://www.sparkfun.com/products/9835)
    DC Barrel Jack Adapter (https://www.sparkfun.com/products/10811)
    Raspberry Pi Zero W (https://www.sparkfun.com/products/14277)
    
Schematic:

Motor Drivers and Dual Motors:

    Mbed LPC1768   Motor Driver   Motor_Left  Motor_Right
    VOUT           VCC
    GND            GND
                   VM (to Power)
                   A01            RED
                   A02            BLACK
                   B01                        RED
                   B02                        BLACK
    p21            PWMA
    p22            PMWB
    p6             AI1
    p7             BI1
    p5             AI2
    p8             BI2

LIDAR Sensors:

    Mbed LPC1768   LIDAR_Right   LIDAR_Left   LIDAR_Center
    VOUT           VIN           VIN          VIN
    GND            GND           GND          GND
    p28            SDA           SDA          SDA
    p27            SCL           SCL          SCL
    p24            SHDN      
    p25                          SHDN
    p26                                      SHDN

Encoders:

    Mbed LPC1768   Encoder_Left   Encoder_Right
    VOUT           RED            RED
    GND            BLACK          BLACK
    p11            WHITE
    p12                           WHITE

Source Codes:

Photos/Videos:


Lowest EndGoal:

To make the robot walk straight along a wall and measure the distance between the robot and the wall

Solve a simple maze and walk through one path.
