#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <fstream>
#include <sstream>

double dt=1.0/200;  //imu数据采样间隔

/* imu数据转bag，bag包以IMU数据为基础(因为通常imu数据是时间最长的) */
void imu2bag(rosbag::Bag &bag, const std::string imuFile, const std::string outBag, int gpsWeek)
{
    std::ifstream file(imuFile);
    if (!file.is_open())
    {
        ROS_ERROR_STREAM("Failed to open file!");
        return;
    }

    bag.open(outBag, rosbag::bagmode::Write);
    std::string line;

    while (std::getline(file, line))
    {
        // 将每行数据分割为各个字段
        std::istringstream iss(line);
        double time, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
        if (!(iss >> time >> gyro_x >> gyro_y >> gyro_z >> accel_x >> accel_y >> accel_z))
        {
            ROS_WARN_STREAM("Failed to parse line: " << line);
            continue;
        }

        // 创建IMU消息
        sensor_msgs::Imu imu_msg;
        // 315964800是GPS起始时间和计算机起始时间的一个固定差值
        time = time + 315964800 + 604800 * gpsWeek - 8 * 3600;
        imu_msg.header.stamp = ros::Time(time);
        imu_msg.angular_velocity.x = gyro_x/dt;
        imu_msg.angular_velocity.y = gyro_y/dt;
        imu_msg.angular_velocity.z = gyro_z/dt;
        imu_msg.linear_acceleration.x = accel_x/dt;
        imu_msg.linear_acceleration.y = accel_y/dt;
        imu_msg.linear_acceleration.z = accel_z/dt;

        // 写入ROSbag文件
        bag.write("/imu/data", ros::Time(time), imu_msg);
    }

    bag.close();
    file.close();
    std::cout << "imu data convert finished!" << std::endl;
}


/* gnss数据转bag */
void gnss2bag(rosbag::Bag &bag, const std::string gnssFile, const std::string outBag, int gpsWeek)
{
     std::ifstream file(gnssFile);
    if (!file.is_open())
    {
        ROS_ERROR_STREAM("Failed to open file!");
        return;
    }

    bag.open(outBag, rosbag::bagmode::Append);

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        double time, lat, lon, h, vn, ve, vd;
        if (!(iss >> time >> lat >>lon >>h >>vn >>ve >>vd))
        {
            ROS_WARN_STREAM("Failed to parse line: " << line);
            continue;
        }

        // 创建gnss消息
        sensor_msgs::NavSatFix gnss_msg;
        // 315964800是GPS起始时间和计算机起始时间的一个固定差值
        time = time + 315964800 + 604800 * gpsWeek - 8 * 3600;
        gnss_msg.header.stamp = ros::Time(time);
        gnss_msg.latitude=lat;
        gnss_msg.longitude=lon;
        gnss_msg.altitude=h;

        gnss_msg.position_covariance[0]=0.005*0.005;
        gnss_msg.position_covariance[3]=0.004*0.004;
        gnss_msg.position_covariance[6]=0.008*0.008;

        // 写入ROSbag文件
        bag.write("/gnss", ros::Time(time), gnss_msg);
    }

    bag.close();
    file.close();
    std::cout << "gnss data convert finished!" << std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_to_rosbag");
    ros::NodeHandle nh;

    // 创建rosbag文件
    rosbag::Bag bag;
    int gpsWeek = 2017;
    std::string imuFile = "src/data2bag/Leador-A15.txt";                 //imu数据文件
    std::string gnssFile = "src/data2bag/GNSS-RTK.txt";                  //gnss数据文件

    std::string outBag ="src/data2bag/output.bag";                         //生成的结果文件
    
    imu2bag(bag, imuFile, outBag, gpsWeek);         // imu转bag
    gnss2bag(bag, gnssFile, outBag, gpsWeek);      // gnss转bag


    return 0;
}