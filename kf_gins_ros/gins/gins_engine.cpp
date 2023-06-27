
#include <vector>
#include "gins_engine.h"

bool first_imu=false;

//构造函数，定义协方差
GINSEngine::GINSEngine() {

    // 设置协方差矩阵，系统噪声阵和系统误差状态矩阵大小
    Cov_.resize(21, 21);     //EKF状态量协方差（pos vel att bg ba sg sa）
    Qc_.resize(18, 18);       //噪声方差(vrw arw bg ba sg sa )
    dx_.resize(21, 1);          //EKF状态误差
    Cov_.setZero();
    Qc_.setZero();
    dx_.setZero();

    // 初始化系统噪声阵
    Vector3d arw=ACC_W*M_PI/180/60;                                          // deg/sqrt(hr) -> rad/s
    Vector3d vrw=GYR_W*M_PI/180/60;                                          // m/s/sqrt(hr) -> m/s
    Vector3d gyrbias_std=GYR_BIAS_STD*M_PI/180/3600;    // deg/hr -> rad/s
    Vector3d accbias_std=ACC_BIAS_STD*1e-5;                 // mGal -> m/s^2     10e+5 mGal = 1 m/s^2
    Vector3d gyrscale_std=GYR_SCALE_STD*1e-6;
    Vector3d accscale_std=ACC_SCALE_STD*1e-6;
    

    Qc_.block(0, 0, 3, 3) = vrw.cwiseProduct(vrw).asDiagonal();
    Qc_.block(3, 3, 3, 3) = arw.cwiseProduct(arw).asDiagonal();
    Qc_.block(6, 6, 3, 3) = 2 / CORRTIME *gyrbias_std.cwiseProduct(gyrbias_std).asDiagonal();
    Qc_.block(9, 9, 3, 3) = 2 / CORRTIME * accbias_std.cwiseProduct(accbias_std).asDiagonal();
    Qc_.block(12, 12, 3, 3) = 2 / CORRTIME * gyrscale_std.cwiseProduct(gyrscale_std).asDiagonal();
    Qc_.block(15, 15, 3, 3) = 2 / CORRTIME * accscale_std.cwiseProduct(accscale_std).asDiagonal();

    // 设置系统状态(位置、速度、姿态和IMU误差)初值和初始协方差

    // 初始化协方差
    Vector3d initpos_std=POS_STD;                            //m
    Vector3d initvel_std=VEL_STD;                               // m/s
    Vector3d initatt_std=ATT_STD*M_PI/180;          //deg -> rad

    Vector3d initgyrbias_std=gyrbias_std;
    Vector3d initaccbias_std=accbias_std;
    Vector3d initgyrscale_std=gyrbias_std;
    Vector3d initaccscale_std=accbias_std;

    Cov_.block(0, 0, 3, 3) = initpos_std.cwiseProduct(initpos_std).asDiagonal();
    Cov_.block(3, 3, 3, 3) = initvel_std.cwiseProduct(initvel_std).asDiagonal();
    Cov_.block(6, 6, 3, 3) = initatt_std.cwiseProduct(initatt_std).asDiagonal();
    Cov_.block(9, 9, 3, 3)   = initgyrbias_std.cwiseProduct(initgyrbias_std).asDiagonal();
    Cov_.block(12, 12, 3, 3)   = initaccbias_std.cwiseProduct(initaccbias_std).asDiagonal();
    Cov_.block(15, 15, 3, 3)   =initgyrscale_std.cwiseProduct(initgyrscale_std).asDiagonal();
    Cov_.block(18, 18, 3, 3)   = initaccscale_std.cwiseProduct(initaccscale_std).asDiagonal();

}


//第1个历元状态初始化
void GINSEngine::INSInitialization(double timestamp)
{
    pvacur.time=timestamp;
    pvacur.blh << INITPOS[0]*M_PI/180, INITPOS[1]*M_PI/180, INITPOS[2];
    pvacur.vel<<INITVEL;
    pvacur.qnb=Rotation::euler2quaternion(INITATT*M_PI/180);
}


void GINSEngine::processIMU(double dt,  sensor_msgs::ImuConstPtr &imu_msg)
{ 
    double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
    dx = imu_msg->linear_acceleration.x;
    dy = imu_msg->linear_acceleration.y;
    dz = imu_msg->linear_acceleration.z;
    rx = imu_msg->angular_velocity.x;
    ry = imu_msg->angular_velocity.y;
    rz = imu_msg->angular_velocity.z;

    imucur.dt=dt;
    imucur.time=imu_msg->header.stamp.toSec();
    imucur.acc<<dx, dy, dz;
    imucur.gyro<<rx, ry, rz;

    if(!first_imu)
    {
        first_imu=true;

        //计算加速度和角速度增量，第一个历元直接积分，不取中值
        imucur.dvel=imucur.acc*dt;
        imucur.dtheta= imucur.gyro*dt;
        INSInitialization(imucur.time); //第一个历元做状态初始化
    }
    else
    {
        imucur.dvel=0.5*(imupre.acc+imucur.acc)*dt;
        imucur.dtheta=0.5*(imupre.gyro+imucur.gyro)*dt;

        INSPropagation(imupre, imucur, pvapre, pvacur); //INS递推
    }

    //更新时刻状态
    imupre=imucur;
    pvapre=pvacur;
}


void GINSEngine::IMUInterpolate(const double timestamp, const sensor_msgs::ImuConstPtr imu_msg)
{
    double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
    dx = imu_msg->linear_acceleration.x;
    dy = imu_msg->linear_acceleration.y;
    dz = imu_msg->linear_acceleration.z;
    rx = imu_msg->angular_velocity.x;
    ry = imu_msg->angular_velocity.y;
    rz = imu_msg->angular_velocity.z;

    IMU imu0=imupre;
    IMU imu1, imu_mid;
    imu1.time = imu_msg->header.stamp.toSec();
    imu1.gyro << rx, ry, rz;
    imu1.acc << dx, dy, dz;

    //加权平均值计算内插时刻
    double lamda1=(timestamp-imu0.time) / (imu1.time-imu0.time);
    double lamda2=(imu1.time - timestamp) / (imu1.time-imu0.time);

    imu_mid.time=timestamp;
    imu_mid.dt=timestamp-imu0.time;
    imu_mid.acc = lamda1*imu0.acc+lamda2*imu1.acc;
    imu_mid.gyro = lamda1*imu0.gyro+lamda2*imu1.gyro;
    imu_mid.dvel=0.5*(imu_mid.acc+imu0.acc)*imu_mid.dt;
    imu_mid.dtheta=0.5*(imu_mid.gyro+imu0.gyro)*imu_mid.dt;

    INSPropagation(imu0, imu_mid, pvapre, pvacur);

    //更新时刻状态
    imupre=imu_mid;
    pvapre=pvacur;
}



void GINSEngine::processGNSS(sensor_msgs::NavSatFixConstPtr &gnss_msg)
{
    
    double lat=gnss_msg->latitude;
    double lon=gnss_msg->longitude;
    double h=gnss_msg->altitude;
    gnssdata.blh<< lat*M_PI/180, lon*M_PI/180, h;
    gnssdata.time=gnss_msg->header.stamp.toSec();

    gnssdata.poscov(0, 0)=gnss_msg->position_covariance[0];     //N
    gnssdata.poscov(1, 1)=gnss_msg->position_covariance[3];     //E
    gnssdata.poscov(2, 2)=gnss_msg->position_covariance[6];     //D


    GNSSUpdate(gnssdata);
    StateFeedback(); //GNSS做完量测更新后，做一次误差状态反馈
    
    pvapre=pvacur;//更新上一时刻的状态
}


void GINSEngine:: INSPropagation(IMU &imupre, IMU &imucur, PVA &pvapre, PVA &pvacur)
{ 
    //INS状态递推
    INSMechan::insMechan(imupre, imucur, pvapre, pvacur);

    // 系统噪声传播，姿态误差采用phi角误差模型
    Eigen::MatrixXd Phi, F, Qd, G;

    // 初始化Phi阵(状态转移矩阵)，F阵，Qd阵(传播噪声阵)，G阵(噪声驱动阵)
    Phi.resizeLike(Cov_);
    F.resizeLike(Cov_);
    Qd.resizeLike(Cov_);
    G.resize(21, 18);                      
    Phi.setIdentity();
    F.setZero();
    Qd.setZero();
    G.setZero();

    // 使用上一历元状态计算状态转移矩阵Phi
    double rm,rn;
    Vector3d w_n_ie, w_n_en;
    rm=Earth::RM(pvapre.blh[0]);
    rn=Earth::RN(pvapre.blh[0]);
    w_n_ie=Earth::w_n_ie(pvapre.blh[0]);
    w_n_en=Earth::w_n_en(pvapre.blh, pvapre.vel);
    double gravity=Earth::gravity(pvapre.blh);

    Eigen::Matrix3d temp;
    Eigen::Vector3d accel, omega;
    double rmh, rnh;

    rmh   = rm + pvapre.blh[2];
    rnh   = rn + pvapre.blh[2];
    accel = imucur.acc;
    omega = imucur.gyro;


    //计算F矩阵
    // 1. 位置误差 position error
    temp.setZero();
    temp(0, 0) = -pvapre.vel[2] / rmh;
    temp(0, 2) = pvapre.vel[0] / rmh;
    temp(1, 0) = pvapre.vel[1] * tan(pvapre.blh[0]) / rnh;
    temp(1, 1) = -(pvapre.vel[2] + pvapre.vel[0] * tan(pvapre.blh[0])) / rnh;
    temp(1, 2) = pvapre.vel[1] / rnh;
    F.block(0, 0, 3, 3) = temp;    //Frr
    F.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();  //Frv

    // 2. 速度误差 velocity error
    temp.setZero();
    temp(0, 0) = -2 * pvapre.vel[1] * WGS84_WIE * cos(pvapre.blh[0]) / rmh -
                 pow(pvapre.vel[1], 2) / rmh / rnh / pow(cos(pvapre.blh[0]), 2);
    temp(0, 2) = pvapre.vel[0] * pvapre.vel[2] / rmh / rmh - pow(pvapre.vel[1], 2) * tan(pvapre.blh[0]) / rnh / rnh;
    temp(1, 0) = 2 * WGS84_WIE * (pvapre.vel[0] * cos(pvapre.blh[0]) - pvapre.vel[2] * sin(pvapre.blh[0])) / rmh +
                 pvapre.vel[0] * pvapre.vel[1] / rmh / rnh / pow(cos(pvapre.blh[0]), 2);
    temp(1, 2) = (pvapre.vel[1] * pvapre.vel[2] + pvapre.vel[0] * pvapre.vel[1] * tan(pvapre.blh[0])) / rnh / rnh;
    temp(2, 0) = 2 * WGS84_WIE * pvapre.vel[1] * sin(pvapre.blh[0]) / rmh;
    temp(2, 2) = -pow(pvapre.vel[1], 2) / rnh / rnh - pow(pvapre.vel[0], 2) / rmh / rmh +
                 2 * gravity / (sqrt(rm * rn) + pvapre.blh[2]);
    F.block(3, 0, 3, 3) = temp;  //Fvr

    temp.setZero();
    temp(0, 0)                  = pvapre.vel[2] / rmh;
    temp(0, 1)                  = -2 * (WGS84_WIE * sin(pvapre.blh[0]) + pvapre.vel[1] * tan(pvapre.blh[0]) / rnh);
    temp(0, 2)                  = pvapre.vel[0] / rmh;
    temp(1, 0)                  = 2 * WGS84_WIE * sin(pvapre.blh[0]) + pvapre.vel[1] * tan(pvapre.blh[0]) / rnh;
    temp(1, 1)                  = (pvapre.vel[2] + pvapre.vel[0] * tan(pvapre.blh[0])) / rnh;
    temp(1, 2)                  = 2 * WGS84_WIE * cos(pvapre.blh[0]) + pvapre.vel[1] / rnh;
    temp(2, 0)                  = -2 * pvapre.vel[0] / rmh;
    temp(2, 1)                  = -2 * (WGS84_WIE * cos(pvapre.blh(0)) + pvapre.vel[1] / rnh);
    F.block(3, 3, 3, 3)   = temp; //Fvv

    
    F.block(3, 6, 3, 3) = Rotation::skewSymmetric(pvapre.qnb.toRotationMatrix()* accel);
    F.block(3, 12, 3, 3)  =pvapre.qnb.toRotationMatrix();
    F.block(3, 18, 3, 3)  = pvapre.qnb.toRotationMatrix() * (accel.asDiagonal());
 
    // 3. 姿态误差
    // attitude error
    temp.setZero();
    temp(0, 0) = -WGS84_WIE * sin(pvapre.blh[0]) / rmh;
    temp(0, 2) = pvapre.vel[1] / rnh / rnh;
    temp(1, 2) = -pvapre.vel[0] / rmh / rmh;
    temp(2, 0) = -WGS84_WIE * cos(pvapre.blh[0]) / rmh - pvapre.vel[1] / rmh / rnh / pow(cos(pvapre.blh[0]), 2);
    temp(2, 2) = -pvapre.vel[1] * tan(pvapre.blh[0]) / rnh / rnh;
    F.block(6, 0, 3, 3) = temp;
    temp.setZero();
    temp(0, 1)                    = 1 / rnh;
    temp(1, 0)                    = -1 / rmh;
    temp(2, 1)                    = -tan(pvapre.blh[0]) / rnh;
    F.block(6, 3, 3, 3)   = temp;
    F.block(6, 6, 3, 3) = -Rotation::skewSymmetric(w_n_en+w_n_ie);
    F.block(6, 9, 3, 3)  = -pvapre.qnb.toRotationMatrix();
    F.block(6, 15, 3, 3)  = -pvapre.qnb.toRotationMatrix() * (omega.asDiagonal());

    // IMU零偏误差和比例因子误差，建模成一阶高斯-马尔科夫过程
    F.block(9, 9, 3, 3) = -1 / CORRTIME * Eigen::Matrix3d::Identity();
    F.block(12, 12, 3, 3) = -1 / CORRTIME * Eigen::Matrix3d::Identity();
    F.block(15, 15, 3, 3) = -1 / CORRTIME * Eigen::Matrix3d::Identity();
    F.block(18, 18, 3, 3) = -1 / CORRTIME * Eigen::Matrix3d::Identity();

    // 系统噪声驱动矩阵
    G.block(3, 0, 3, 3)    = pvapre.qnb.toRotationMatrix();
    G.block(6, 3, 3, 3)  = pvapre.qnb.toRotationMatrix();
    G.block(9, 6, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(12, 9, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(15, 12, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(18, 15, 3, 3) = Eigen::Matrix3d::Identity();

    // 计算状态转移矩阵Phi
    //一步转移矩阵式   Phi(tk/k-1)=I+F(tk-1)*dt
    Phi.setIdentity();
    Phi = Phi + F * imucur.dt;

    // 计算系统传播噪声
    Qd = G * Qc_ * G.transpose() ;
    Qd = (Phi * Qd * Phi.transpose() + Qd) * imucur.dt/ 2;

    // EKF预测传播系统协方差和系统误差状态
    EKFPredict(Phi, Qd);
}


void GINSEngine::StateFeedback()
{
    Eigen::Vector3d vectemp;

    // 1. 位置误差反馈
    Eigen::Vector3d delta_r = dx_.block(0, 0, 3, 1);
    Eigen::Matrix3d Dr_inv  = Earth::DRi(pvacur.blh);
    pvacur.blh -= Dr_inv * delta_r;

    // 2. 速度误差反馈
    vectemp = dx_.block(3, 0, 3, 1);
    pvacur.vel -= vectemp;

    // 3. 姿态误差反馈
    vectemp                = dx_.block(6, 0, 3, 1);
    Eigen::Quaterniond qnp = Rotation::Vector2Quaternion(vectemp);
    pvacur.qnb=qnp*pvacur.qnb;

    // 4. IMU零偏误差反馈
    vectemp = dx_.block(9, 0, 3, 1);
    gyrbias += vectemp;
    vectemp = dx_.block(12, 0, 3, 1);
    accbias += vectemp;

    // 5. IMU比例因子误差反馈
    vectemp = dx_.block(15, 0, 3, 1);
    gyrscale += vectemp;
    vectemp = dx_.block(18, 0, 3, 1);
    accscale += vectemp;

    // 误差状态反馈到系统状态后,将误差状态清零
    dx_.setZero();

}



void GINSEngine::GNSSUpdate(GNSS &gnssdata)
{
    // IMU位置转到GNSS天线相位中心位置
    // convert IMU position to GNSS antenna phase center position
    Eigen::Vector3d antenna_pos;
    Eigen::Matrix3d Dr, Dr_inv;
    Dr_inv      = Earth::DRi(pvacur.blh);
    Dr          = Earth::DR(pvacur.blh);
    
    // 安装参数(就是天线杆臂)
    Eigen::Vector3d antlever = ANTLEVER;
    antenna_pos = pvacur.blh + Dr_inv * pvacur.qnb.toRotationMatrix() * antlever;

    // GNSS位置测量新息 delta_zr
    Eigen::MatrixXd dz;
    dz = Dr * (antenna_pos - gnssdata.blh);

    // 构造GNSS位置观测矩阵Hr
    Eigen::MatrixXd H_gnsspos;
    H_gnsspos.resize(3, 21);
    H_gnsspos.setZero();
    H_gnsspos.block(0, 0, 3, 3)   = Eigen::Matrix3d::Identity();
    H_gnsspos.block(0, 6, 3, 3) = Rotation::skewSymmetric(pvacur.qnb.toRotationMatrix() * antlever);

    // 位置观测噪声阵
    Eigen::MatrixXd R_gnsspos=gnssdata.poscov;

    // EKF更新协方差和误差状态
    EKFUpdate(dz, H_gnsspos, R_gnsspos);

    // GNSS更新之后设置为不可用
    gnssdata.isvalid = false;
}


void GINSEngine::EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd) 
{

    assert(Phi.rows() == Cov_.rows());
    assert(Qd.rows() == Cov_.rows());

    // 传播系统协方差和误差状态
    Cov_ = Phi * Cov_ * Phi.transpose() + Qd;
    dx_  = Phi * dx_;
}


void GINSEngine::  EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H, Eigen::MatrixXd &R) 
{
    // 计算Kalman增益
    auto temp         = H * Cov_ * H.transpose() + R;
    Eigen::MatrixXd K = Cov_ * H.transpose() * temp.inverse();

    // 更新系统误差状态和协方差
    Eigen::MatrixXd I;
    I.resizeLike(Cov_);
    I.setIdentity();
    I = I - K * H;
    // 如果每次更新后都进行状态反馈，则更新前dx_一直为0，下式可以简化为：dx_ = K * dz;
    dx_  = dx_ + K * (dz - H * dx_);
    Cov_ = I * Cov_ * I.transpose() + K * R * K.transpose();
    
}