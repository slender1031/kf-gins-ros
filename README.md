# kf-gins-ros
kf-gins-ros

The main works are as follows:
1. Convert the datasets to rosbag
2. A ros version of kf-gins (based on C++)

### Configuration
* Ubuntu18.04
* ros-melodic
* Eigen3


### Program Compilation and Execution

**Attention:**
you need to check File **PATH** and ROS **Topics**.

Execute the following commands to compile the project:
```shell
cd  ~/gins_ws/src
git clone https://github.com/slender1031/kf-gins-ros.git
cd ../
catkin_make
```

Run commands:
```shell
source devel/setup.bash
rosrun data_convert data_convert_node
rosrun gins gins_node [path to YAML]
```

Rviz for visualization:
```shell
roslaunch gins gins_rviz.launch
```

<img src="https://github.com/slender1031/kf-gins-ros/blob/main/rviz.png" />


Thanks for the team of Prof. Xiaoji Niu of the Integrated and Intelligent Navigation (i2Nav) group from GNSS Research Center of Wuhan University for providing the open-source KF-GINS software.

感谢武汉大学卫星导航定位技术研究中心多源智能导航实验室(i2Nav)牛小骥教授团队开源的KF-GINS软件平台
(https://github.com/i2Nav-WHU/KF-GINS)

Thanks for the github user **zzzzyp-sgg** who open-source the tools for data format conversion.
(https://github.com/zzzzyp-sgg/SLAM-Tool)
