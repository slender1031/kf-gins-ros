#ifndef GINS_ENGINE_H_
#define GINS_ENGINE_H_

#include <vector>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "INSMechan.h"
#include "earth.h"
#include "parameters.h"



class GINSEngine
{
    public:
        GINSEngine();
        ~GINSEngine()=default;

        void processIMU(double dt,  sensor_msgs::ImuConstPtr &imu_msg);
        void processGNSS(sensor_msgs::NavSatFixConstPtr &gnss_msg);

        void INSInitialization(double timestamp);
        void INSPropagation(IMU &imupre, IMU &imucur, PVA &pvapre, PVA &pvacur); //imu递推
        void IMUInterpolate(const double timestamp, const sensor_msgs::ImuConstPtr imu_data);

        void EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd);
        void EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H, Eigen::MatrixXd &R);
        void GNSSUpdate(GNSS &gnssdata);
        void StateFeedback();
       

        //imu相邻两个历元的输出
        IMU imupre;
        IMU imucur;

        //gnss定位历元结果
        GNSS gnssdata;

        //两个历元的递推结果
        PVA pvapre;
        PVA pvacur;
        
        //imu Bias和尺度误差
        Vector3d gyrbias;
        Vector3d accbias;
        Vector3d gyrscale;
        Vector3d accscale;

        // Kalman滤波相关
        Eigen::MatrixXd Cov_;
        Eigen::MatrixXd Qc_;   //系统状态噪声方差阵
        Eigen::MatrixXd dx_;    //


};

#endif // !GINS_ENGINE_H_

