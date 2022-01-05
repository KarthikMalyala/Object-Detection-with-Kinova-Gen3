# Object-Detection-with-Kinova-Gen3
Object Detection and Computer Vision for the Adaptive Robotic Nursing Assistant (ARNA) using AR Track Alvar and Kinova Gen3 Robotic Arm

PICK & PLACE
1) roslaunch ar_track_alvar kinova_indiv.launch
2) roslaunch kortex_driver kortex_driver.launch
3) roslaunch kinova_vision kinova_vision.launch device:=10.0.0.30
4) rosrun kinova_vision kinovaTarget.py

Video Demonstration: https://youtu.be/9KQqGKGLyec
