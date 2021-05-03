# 04_05_01
CG1112 Alex Project: This repository contains the code (under the master branch) for the design of Alex, a robotic vehicle that simulates a Search-and-Rescue (SAR) Robot. Alex will be remotely operated through our laptops, and it contains a Raspberry Pi (RPi), an Arduino and a LiDAR sensor to scan and map out its surrounding environment. Instructions (e.g. go forward, get colour etc.) will be sent to the RPi through our laptops, which will then be sent to the Arduino via serial communications protocols to be executed by Alex. 

The main functionalities of Alex include: mobility (forward, backward, turn left, turn right), colour detection of "victims", teleoperation, environment mapping using Hector SLAM algorithm and Robot Operating System (ROS), light and sound display (additional enhancement) as well as power saving (on both RPi and Arduino). 

Under the w7s2pi folder, alex-pi.cpp is the main C++ code executed on the RPi, while Alex.ino (further under the Alex/Alex folder) is the main code executed on the Arduino. 

Done by: Teng Yi Shiong, Tan Le Yi, Jared Cheang and Wong Zi Xin, Avellin


