# Go1-Gesture-Command

This repository contains two ROS2 packages for using hand gestures to send motion commands to the Unitree Go1:
- ros2_hgr
    * A ROS2 Python package with code forked from [kinivi's hand-gesture-recognition-using-mediapipe repository](https://github.com/kinivi/hand-gesture-recognition-mediapipe) that uses computer vision, Mediapipe, and a machine learning model to detect 8 different hand gestures.
- go1_cmd
    * A ROS2 C++ package that receives hand gesture data and utilizes services to send commands to the Go1 accordingly. 

https://user-images.githubusercontent.com/39091881/226065629-1f51c5ee-e34e-41f8-ba0c-dff40174ed21.mp4

## How It Works
Google's MediaPipe is an open-source framework with machine learning solutions, including hand detection, face detection, and more. Here, I use MediaPipe to detect my hands, and I use a TensorFlow model kinivi's repository to classify the gesture I am making. 

## Dependencies
You can import the necessary repositories listed in `go1_hgr.repos` into your workspace using vcs. To do so, clone this repository into the src directory of your workspace. Then in the root of your workspace, run the following:  
`vcs import < src/go1-gesture-command/go1_hgr.repos`  

Other dependencies include:  
* [OpenCV-Python for Ubuntu](https://docs.opencv.org/4.5.4/d2/de6/tutorial_py_setup_in_ubuntu.html)
* [NumPy](https://numpy.org/install/)
* [MediaPipe](https://google.github.io/mediapipe/getting_started/python.html)
* [TensorFlow](https://www.tensorflow.org/install)

## Launch
`ros2 launch ros2_hgr hgr.launch.xml`  
* The ros2_hgr launch file has defaults set to run the hgr_node with your computer's built-in webcam. 
* The `use_realsense` launch argument defaults to false and can be used for a similar but separate hgr_node specifically for the RealSense.

To launch using an external RealSense camera instead of a built-in webcam, use  
`ros2 launch ros2_hgr hgr.launch.xml use_realsense:=true`

## Gestures Guide
0. Open - stop
1. Close - look forward (normal 0&deg; yaw)
2. Pointer - recover stand up
3. OK - look up
4. Peace - look down
5. Thumbs Up - walk forward
6. Thumbs Down - walk backward
7. Quiet Coyote - lay down

https://user-images.githubusercontent.com/39091881/226063735-aaa51c78-5556-4b36-a2dd-72c2382e86d3.mp4

## Notes
This project would not have been possible without the help of Katie Hughes, Nick Morales, and Marno Nel, who I worked with throughout the Go1 disassembly and upgrades that were necessary to work with the Go1 and make this project and their projects possible. 
