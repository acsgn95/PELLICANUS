# PELLICANUS
https://www.crowdsupply.com/g-fusion/pellicanus

![crowd-supply-logo-dark@2x](https://user-images.githubusercontent.com/78763530/150209087-3343a52b-b8f7-4014-8455-4775039dd88b.png)

![image](https://user-images.githubusercontent.com/78763530/149827798-d9480e51-b02b-4c99-bfbe-de5ced77979b.png)

![20211224_154711](https://user-images.githubusercontent.com/78763530/149828149-ae5b037b-489c-4200-96d9-f780288ae33e.jpg)


Pellicanus is an open source INS/GPS integrated navigation system designed to be a handy, accessible development board. It consists of 10 DOF IMUs, a GNSS receiver and a RP2040 processor. There are GPIO pins for sensor data input. By default, it includes the loosely coupled extended kalman filter and odometer. It gives you the system’s 3d vector velocity, 3d vector position, and orientation angles.

Pellicanus is not only a tactical level INS/GPS, it also comes with completely open source mathematical models, software and schematics. Also, the system sensitivity is at a level to compete with military systems on the market.

Pellicanus can be configured with only a few changes to fit where you want to install it. For example, it can be used in drones, unmanned land vehicles, agricultural systems, or even robotics.

![GFusion](https://user-images.githubusercontent.com/78763530/149828035-9aa356ae-d224-4f8a-8faf-f423bed22c54.png)

# Navigation Algorithm
PELLICANUS includes loosely coupled EKF/NCF with discrete architecture. The architecture is as follows.

![defaultalgortihm](https://user-images.githubusercontent.com/78763530/150411685-8fd9774f-e29a-49a3-9d9a-58852d81ff50.png)

# GUI
We are working on a GUI where you can pull raw IMU data from PELLICANUS and edit them in various AHRS and Navigation programs.

![DisplayImage](https://user-images.githubusercontent.com/78763530/150698839-64f83870-bc07-46b9-8f1c-7df0c6f1497c.jpg)

# C Code
In the future, we will share the C driver of the Default Navigation Algorithm and the C code of the sensor drivers.

![PUTTY_result](https://user-images.githubusercontent.com/78763530/150804931-6b9ff653-b2a6-47cf-9c71-cb9c94e06441.png)

