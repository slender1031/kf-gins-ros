# kf-gins-ros
kf-gins-ros

C++ code for kf-gins (based on ROS), you need to check File PATH and ROS Topics.

The main works are as follows:
1. Convert the datasets to rosbag
2. 

基于kf-gins改进了ros版本，将原始文件转为rosbag格式，能够读取ros数据流下的imu和gnss数据进行解算

Configuration：
* Ubuntu18.04
* ros-melodic
* Eigen3


I would like to acknowledge the team of Prof. Xiaoji Niu of the Integrated and Intelligent Navigation (i2Nav) group from GNSS Research Center of Wuhan University for providing the open-source KF-GINS software.
感谢武汉大学卫星导航定位技术研究中心多源智能导航实验室(i2Nav)牛小骥教授团队开源的KF-GINS软件平台(
https://github.com/i2Nav-WHU/KF-GINS)
