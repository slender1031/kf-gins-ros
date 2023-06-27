#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_


#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <eigen3/Eigen/Dense>

#include <fstream>

#include "gins_engine.h"


extern ros::Publisher pub_ins_odometry;
extern ros::Publisher pub_ins_path;

extern ros::Publisher pub_gins_blh;
extern ros::Publisher pub_gins_ned;

void pubINSMech(const GINSEngine &gins_engine, const std_msgs::Header &header);
void pubGINS(const GINSEngine &gins_engine, const std_msgs::Header &header);


#endif 