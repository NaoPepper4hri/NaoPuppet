# NaoPuppet
Using Kinects V1 and Copying Human hand movement to Nao. This works by translating the angle between lower-arm/upper-arm/human-torso to Nao Elbow and Shoulder roll/pitch/yaw interpolations. 

This works with both Nao and Pepper. We never used this directly in experiments. This was a starting point for experimentation with Nao/Pepper's capabilities and how to interface it with other devices.

## Requirements
Nao or Pepper

python 2.7 installed as instructed by [Aldebaran](http://doc.aldebaran.com/2-5/dev/python/install_guide.html)

Kinect V1 and related drivers

The following python packaged:

pynaoqi
pykinects
pygame

