# HADRON
HADRON is my first iteration of a 16 DOF bipedal humanoid robot. The focus of this project is on the controls and programming for bipedal locomotion and multiple DOF balance compensation.

The 3D design of HADRON is not my design. It was found from Thingiverse here: https://www.thingiverse.com/thing:356398.

<img src="https://github.com/rithvikpillai/HADRON/blob/main/cadmodel.png" width="400" />

However, the sensor & actuator selection, supporting electronics and programming are all my original design. The documentation for HADRON is not available anymore and the Thingiverse post is not in English, so my goal is to design this robot from the reference photos and STL's provided and document my progress along the way. Without further ado, let's get into it!

<b> Hardware </b>

<i> 3D Printed Parts </i>

- x2 Chest Plates
- x2 Feet
- x2 Hands
- x10 Assembly Brackets
- x14 Servo Brackets
- x14 Servo Joints

All of these parts can be printed from the '3D Printed Parts' folder in the repository with PLA filament and about 15-20 % infill density. They came out really nice and I was able to use a cool split color (blue/red) filament for the frontal chest piece. See pictures below.

<img src="https://github.com/rithvikpillai/HADRON/blob/main/hadron3.jpeg" width="400" />

<i> Electronics </i>

- x1 Arduino UNO
- x1 Breadboard
- Jumper Wires
- x1 12 V Li-Ion Battery, 3000 mAh (from AndyMark - am-4347) - Pending to be changed to a 6V
- x1 LM2596 DC-DC Buck (Step-Down) Voltage Converter - Currently using to make the 12V compatible (DON'T DO THIS)
- x16 MG996R 180 degree metal gear servo motors (stall torque ~ 11 kg/cm)
- x1 SG90 9g Micro-Servo
- x1 MPU6050 3-Axis Accelerometer & Gyroscope (I2C Module)
- x1 PCA9685 16 Channel 12-bit PWM Servo Motor Driver (I2C Module)

<b> Software </b>
TBD

