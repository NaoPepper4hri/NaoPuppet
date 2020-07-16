# NaoPuppet
Using Kinects V1 and copying Human arm movements on Nao. This works by translating the angle between lower-arm/upper-arm/human-torso to Nao Elbow and Shoulder roll/pitch/yaw interpolations. 

This works with both Nao and Pepper. We never used this directly in experiments. This was a starting point for experimentation with Nao/Pepper's capabilities and how to interface it with other devices we had available.

## Requirements
Nao or Pepper

python 2.7 installed as instructed by [Aldebaran](http://doc.aldebaran.com/2-5/dev/python/install_guide.html)

Kinect V1 and related drivers

The following python packaged:

pynaoqi
pykinects
pygame

## To run
Connect the kinects to the computer. Set the ip to the ip of the robot in naoKinect.py.

use: 
python my_nao_track.py

## Note
Be careful with sudden arm movements Nao can fall over as we are not taking care of balance here.

Lesson learnt: There is difference between mechanically copying joint angles and copying movement intentions.


