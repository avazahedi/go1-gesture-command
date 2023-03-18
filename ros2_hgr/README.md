# ros2_hgr Package
The ros2_hgr package is based on [kinivi's hand-gesture-recognition-mediapipe repository](https://github.com/kinivi/hand-gesture-recognition-mediapipe). I restructured it into a ROS2 package and added more features.  
My modifications include adding new gestures and data to the existing model. To learn more about the general functionality, see the original repository.  

## Nodes
This package contains two nodes:
* hgr_node
    - This node uses your computer's built-in webcam to generate an image stream. From there, Mediapipe is used to detect hands and a TensorFlow model is used to label different hand gestures. An integer equivalent of the identified gesture is sent over /hgr_topic, which is subscribed to in the hgr_com node in the [go1_cmd](https://github.com/avazahedi/go1-gesture-command/tree/main/go1_cmd) package. 
* hgr_node_cam
    - This node is very similar to hgr_node, except that instead of using your computer's built-in webcam, it uses an external RealSense camera. 

<br>

### Notes
If you wish to do additional data training and logging, you will have to change the path in line 25 of hgr_node.py or line 29 of hgr_node_cam.py, depending on if you are using the built-in webcam or RealSense, respectively. You will also have to change the path_prefix in the keypoint_classification_EN.ipynb notebook for retraining the model.