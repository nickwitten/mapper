Our team is creating a room mapping robot. The purpose of the robot is to autonomously roam a room and plot the 2D coordinates of walls and obstacles while expanding its knowledge in a frontier-based fashion.

Our proposed design will include a raspberry pi as a system controller to implement overarching algorithms and data manipulation in a high-level language such as Python.  We will also use an LPC1768 MBED board as a microcontroller for motor control and data acquisition.  2 – 4 DC motors will be needed to drive wheel rotation and at least one LIDAR sensor to measure distances to close objects.  A servo motor will be considered for movement of the LIDAR sensor.

Related projects (URLs) and describe how your project will be different or improved:

https://os.mbed.com/users/swebster9/notebook/multiple-scanning-lidar-robot/#

https://os.mbed.com/users/abh15/notebook/robotic-cartographer/

https://os.mbed.com/users/arogliero3/notebook/semi-autonomous-robot---ece-4180/#

https://os.mbed.com/users/boyanmirov/code/RobotControlviaBluetooth/wiki/Homepage

Our project will be built on a similar robot platform listed above. However, we will be focusing more on the process of 2D mapping. We want our robot to be able to detect objects around it and process the data get from sensor to generate a 2D map stored in the SD card(maybe). We might want to use multiple sensors so that the detection  process is more accurate. Like the project above about the cartographer, we want to build something similar but with more functions and easier to control.
