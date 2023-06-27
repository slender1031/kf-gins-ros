#include <iostream>
#include <string>
#include <chrono>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <stdio.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "gins_engine.h"
#include "visualization.h"

using namespace std;

std::condition_variable con;
std::queue<sensor_msgs::ImuConstPtr> imu_buf;
std::queue<sensor_msgs::NavSatFixConstPtr> gnss_buf;
std::mutex m_buf;

ros::Publisher pub_ins_odometry,  pub_ins_path;
ros::Publisher pub_gins_blh,  pub_gins_ned;


double current_time = -1;

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();
}

void gnss_callback(const sensor_msgs::NavSatFixConstPtr &gnss_msg)
{
    m_buf.lock();  
    gnss_buf.push(gnss_msg);
    m_buf.unlock();
    con.notify_one();
}


//IMU与GNSS的数据流做同步，以GNSS的时间间隔，确定一组imu_msg
bool getMeasurements(std::vector<sensor_msgs::ImuConstPtr> &imu_msg, sensor_msgs::NavSatFixConstPtr &gnss_msg)
{
    //当前imu和gnss数据都为空
    if(imu_buf.empty() || gnss_buf.empty()){
        return false;
    }

    //imu最新的时刻仍然小于gnss最早时刻
    if(imu_buf.back()->header.stamp.toSec()<gnss_buf.front()->header.stamp.toSec()){
        return false;
    }
        
    double front_imu_ts=imu_buf.front()->header.stamp.toSec();
    double front_gnss_ts=gnss_buf.front()->header.stamp.toSec();

    //第一个imu时间戳大于gnss时间戳，就把gnss数据丢掉
    while(!gnss_buf.empty() && front_imu_ts>front_gnss_ts )
    {
        ROS_WARN("throw gnss_msg, only should happen at the beginning");
        gnss_buf.pop();

        if(gnss_buf.empty()) return false;
        front_gnss_ts=gnss_buf.front()->header.stamp.toSec();
    }
    
    gnss_msg=gnss_buf.front();
    gnss_buf.pop();

    //截取两个gnss时刻中间的imu，放入imu_buf，作为一组imu数据
    while (!imu_buf.empty() && imu_buf.front()->header.stamp.toSec() < gnss_msg->header.stamp.toSec())
    {
        imu_msg.emplace_back(imu_buf.front());
        imu_buf.pop();
    }

    //多取出一个gnss时刻之后的imu数据，用来做内插，对齐到gnss时刻
    if(!imu_buf.empty()){
        imu_msg.emplace_back(imu_buf.front());
        imu_buf.pop();
    }

    if (imu_msg.empty()){
        ROS_WARN("no imu between two GNSS");
    }
    
    return true;
}


void process()
{
    GINSEngine gins_engine;

    while(true)
    {
        std::vector<sensor_msgs::ImuConstPtr> imu_msg;
        sensor_msgs::NavSatFixConstPtr gnss_msg;

        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
                    return getMeasurements(imu_msg,  gnss_msg);
                 });
        lk.unlock();

        for(int i=0; i<imu_msg.size()-1; i++)
        {
            sensor_msgs::ImuConstPtr imu_data=imu_msg[i];
            double t = imu_data->header.stamp.toSec();
            if(current_time<0)      //第一个历元 current_time=-1
                current_time=t;
            
            double dt=t-current_time;
            ROS_ASSERT(dt >= 0); //判断imu数据时间戳有没有错乱
            current_time = t;

            gins_engine.processIMU(dt, imu_data);   //IMU递推
            pubINSMech(gins_engine, imu_data->header);
        }   

        //利用最后两个imu时刻对gnss时刻做内插，计算内插时刻IMU递推结果
        sensor_msgs::ImuConstPtr imu_data_back=imu_msg.back();

        double gnss_t=gnss_msg->header.stamp.toSec();
        gins_engine.IMUInterpolate(gnss_t, imu_data_back);
        gins_engine.processGNSS(gnss_msg);      //gnss量测更新
        pubGINS(gins_engine, gnss_msg->header);

        //计算gnss时刻到最后一个imu时刻的递推结果
        double dt=imu_data_back->header.stamp.toSec()-gnss_t;
        gins_engine.processIMU(dt, imu_data_back);
        pubINSMech(gins_engine, imu_data_back->header);
    
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "gins");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun gins gins_node [config file] \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    ROS_WARN("waiting for gnss and imu...");

    ros::Subscriber sub_imu = n.subscribe(GNSS_TOPIC, 100, gnss_callback);
    ros::Subscriber sub_gnss = n.subscribe(IMU_TOPIC, 1000, imu_callback);

    pub_ins_odometry = n.advertise<nav_msgs::Odometry>("/insmech_odom", 1000);
    pub_ins_path = n.advertise<nav_msgs::Path>("/insmech_path", 1000);
    pub_gins_blh = n.advertise<sensor_msgs::NavSatFix>("/gins_blh", 1000);
    pub_gins_ned = n.advertise<nav_msgs::Path>("/gins_ned_path", 1000);

    std::thread measurement_process{process};
    
    ros::spin();
    return 0;
}
