# Automatic_Robotic_Arm_System

This system is too big to upload to github, about 4GB. I just upload some essential files and images of file directory. This system consists of three major modules and an accessary module of Arduino. The first major module is UI moduel created with QtCreator. The second major module is the RoboticArm moduel which is designed to control the movement of the robotic arm. It is created with Visual Studio C++ and the SDK of the robotic arm, "aubo i5". The last one is the PatternRecognition module which is responsible for object recognition, object location and posture analysis. It is created with Python, Tensorflow and OpenCV. The mentioned three major modules communicate with each other by TCP protocal. Finally, the Arduino module is designed to control the gripper of the robotic arm. It communicates with other modules by serial communication.
