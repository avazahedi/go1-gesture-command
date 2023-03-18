# go1_cmd Package
The go1_cmd package is a ROS2 C++ package that receives hand gesture data and calls Go1 motion services from the unitree_nav package accordingly.  

## Nodes
* hgr_com
    - This node subscribes to /hgr_topic, which receives hand gesture data from hgr_node in the [ros2_hgr](https://github.com/avazahedi/go1-gesture-command/tree/main/ros2_hgr) package. Based on the detected hand gesture, this node calls services from the [unitree_nav](https://github.com/ngmor/unitree_nav) package which commands the Go1 to move. 
