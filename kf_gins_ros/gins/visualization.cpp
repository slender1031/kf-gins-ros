#include "visualization.h"

ros::Publisher pub_ins_odometry,  pub_ins_path;
ros::Publisher pub_gins_blh, pub_gins_ned;

nav_msgs::Path gins_ned_path;
nav_msgs::Path ins_blh_path;

bool first_gins=false;
Vector3d ned_first;

void pubINSMech(const GINSEngine &gins_engine, const std_msgs::Header &header)
{
    nav_msgs::Odometry odometry;
    odometry.header=header;
    odometry.header.frame_id="world";

    Vector3d P=gins_engine.pvacur.blh;
    odometry.pose.pose.position.x=P[0];
    odometry.pose.pose.position.y=P[1];
    odometry.pose.pose.position.z=P[2];

    Vector3d V=gins_engine.pvacur.vel;
    
    Quaterniond Q=gins_engine.pvacur.qnb;
    odometry.pose.pose.orientation.x=Q.x();
    odometry.pose.pose.orientation.y=Q.y();
    odometry.pose.pose.orientation.z=Q.z(); 
    odometry.pose.pose.orientation.w=Q.w();
    pub_ins_odometry.publish(odometry);


    Vector3d vec=Q.toRotationMatrix().eulerAngles(2,1,0)*180/M_PI;   // [yaw, pitch, roll]
    Vector3d euler (vec[2], vec[1], vec[0]);

    //Vector3d vec=quaternion2euler(Q)*180/M_PI;

    printf("IMU  time: %f, t: %f %f %f q: %f %f %f \n", header.stamp.toSec(), P.x()*180/M_PI, P.y()*180/M_PI, P.z(),  euler(0), euler(1), euler(2));

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose = odometry.pose.pose;
    ins_blh_path.header = header;
    ins_blh_path.header.frame_id = "world";
    ins_blh_path.poses.push_back(pose_stamped);
    pub_ins_path.publish(ins_blh_path);

    // write result to file
    ofstream foutC(GINS_RESULT_PATH, ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << header.stamp.toSec() * 1e9 << " ";
    foutC.precision(5);
    foutC << P.x() << " "<< P.y() << " "<< P.z() << " "
                << V.x() << " "<< V.y() << " "<< V.z() << " "
                << Q.w() << " " << Q.x() << " " << Q.y() << " " << Q.z() << endl;
    foutC.close();
}

void pubGINS(const GINSEngine &gins_engine, const std_msgs::Header &header)
{
    sensor_msgs::NavSatFix gps_position;
	gps_position.header=header;
    gps_position.header.frame_id = "world";

    Vector3d blh = gins_engine.pvacur.blh;
	gps_position.latitude  = blh[0];
	gps_position.longitude = blh[1];
	gps_position.altitude  = blh[2];	
    for(int i=0; i<9; i++){
        gps_position.position_covariance[i]=gins_engine.Cov_(i/3, i%3);
    }
        
	pub_gins_blh.publish(gps_position);  //发布更新后的BLH坐标

    //BLH转NED
    if(!first_gins){
        first_gins=true;
        ned_first=blh;
    }

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.header.frame_id = "world";
    
    Vector3d ned=Earth::global2local(ned_first, blh);
    pose_stamped.pose.position.x=ned(0);
    pose_stamped.pose.position.y=ned(1);
    pose_stamped.pose.position.z=ned(2);

    Quaterniond Q=gins_engine.pvacur.qnb;
    pose_stamped.pose.orientation.x=Q.x();
    pose_stamped.pose.orientation.y=Q.y();
    pose_stamped.pose.orientation.z=Q.z();
    pose_stamped.pose.orientation.w=Q.w();

    Vector3d vec=Q.toRotationMatrix().eulerAngles(2,1,0)*180/M_PI;   // [yaw, pitch, roll]
    Vector3d euler (vec[2], vec[1], vec[0]);

    //Vector3d vec=quaternion2euler(Q)*180/M_PI;

    printf("GINS  time: %f, t: %f %f %f q: %f %f %f  \n", header.stamp.toSec(), blh[0]*180/M_PI, blh[1]*180/M_PI, blh[2],  euler(0), euler(1), euler(2));

    gins_ned_path.header = header;
    gins_ned_path.header.frame_id = "world";
    gins_ned_path.poses.push_back(pose_stamped);
    pub_gins_ned.publish(gins_ned_path);    //发布更新后的ned坐标
}