# Lecture 6 object detection and grasping

All the assignments were completed following the instructions in the lecture [slides](OMTP_LEC_6_OBJECT_DETECTION_AND_GRASPING.pdf).

## Assignment 1a
Included two logical cameras into gazebo environment lecture6_assignment1.world. 

## Assignment 1b
The two logical cameras were configured with Link name and Sensor name.

## Assignment 2
The simulation was run with the two cameras and tf tree was extracted as a pdf.
![Frames](frames.svg)

## Assignment 3
An object was spawned near Robot2, the observed coordinates for the object are transformed into gripper frame. The code can be found [here](scripts/lecture6_assignment3.py).

## Assignment 4 - robot1
A python script is written to pick up the objects coming from the conveyor belt and drop it in the nearest bin. The script was written for Robot2. The code can be found [here](scripts/lecture6_assignment4_robot1.py).

![Robot movement commanded from python](assignment4_r1.gif)
## Assignment 4 - robot2
A python script is written to pick up the spawned object from the floor and drop it in the nearest bin. The script was written for Robot2. The code can be found [here](scripts/lecture6_assignment4_robot2.py).
