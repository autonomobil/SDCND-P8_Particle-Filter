[//]: # (Image References)

[img1]: ./images/particle_filter.png "finished.png"
[img2]: ./images/Meas_vs_Kalmanfilt.png "measvskalman.png"
[img3]: ./images/NIS_process_noise_bad.png "finished.png"
[img4]: ./images/NIS_process_noise_good.png "finished.png"

___
# SDCND Term 2 Project 8: Particle Filter
## Kidnapped Vehicle Project for Udacity's Self-Driving Car Engineer Nanodegree Program
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This repository contains all the C++ code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree. Its objective is to use a 2D Particle Filter to determine a car's location using a map with landmarks, initialized with rough GPS coordinates.

##### The overall structure of this filter looks like this (source: Udacity):
![img1]


##### The results can be viewed here(Youtube):
[![result1](https://img.youtube.com/vi/y2gCHllvhw0/0.jpg)](https://www.youtube.com/watch?v=y2gCHllvhw0)

##### Udacity's project basis can be found [here](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).

## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* Udacity's simulator

## Setup and Running
These are the suggested steps for Windows setup:

* Follow these [instructions](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) for setting up Ubuntu BASH.
* Download Windows simulator [here](https://github.com/udacity/self-driving-car-sim/releases).
* Open Ubuntu Bash (write following commands to Ubuntu Bash command window)
* ``sudo apt-get update``
* ``sudo apt-get install git``
* ``sudo apt-get install cmake``
* ``sudo apt-get install openssl``
* ``sudo apt-get install libssl-dev``
* navigate to where you want to clone this repository to, for example:
 ``cd /mnt/c/Users/Bob``
* ``git clone https://github.com/autonomobil/SDCND-P8_Particle-Filter``
* ``sudo rm /usr/lib/libuWS.so``
* navigate to project folder: ``cd SDCND-P8_Particle-Filter``
* ``./install-ubuntu.sh``
* ``./build.sh``
* Launch the **term2_sim.exe** from Windows simulator folder
* Execute ``./run.sh``
* If you see ``Listening to port 4567 Connected!!!``, it is working
* Press **Start**


These files were modified compared to the [original repository](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project):  
* src/particle_filter.cpp
* src/particle_filter.h
