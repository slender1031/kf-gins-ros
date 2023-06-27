#include "parameters.h"

string IMU_TOPIC, GNSS_TOPIC;
Vector3d GYR_BIAS, GYR_BIAS_STD;
Vector3d ACC_BIAS, ACC_BIAS_STD;
Vector3d GYR_SCALE, GYR_SCALE_STD;
Vector3d ACC_SCALE, ACC_SCALE_STD;

Vector3d INITPOS, INITVEL, INITATT;
Vector3d POS_STD, VEL_STD, ATT_STD;

Vector3d ACC_W, ACC_N,  GYR_W, GYR_N;
double CORRTIME;
Vector3d ANTLEVER;
std::string GINS_RESULT_PATH;

void readParameters(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    
    fsSettings["imu_topic"]>>IMU_TOPIC;
    fsSettings["gnss_topic"]>>GNSS_TOPIC;

    fsSettings["output_path"] >> GINS_RESULT_PATH;
    std::ofstream fout(GINS_RESULT_PATH, std::ios::out);
    fout.close();
    
    cv::Vec3d cv_T;
    fsSettings["gyrbias"]>>cv_T;           cv::cv2eigen(cv_T, GYR_BIAS);    
    fsSettings["accbias"]>>cv_T;           cv::cv2eigen(cv_T, ACC_BIAS);
    fsSettings["gyrscale"]>>cv_T;         cv::cv2eigen(cv_T, GYR_SCALE);
    fsSettings["accscale"]>>cv_T;         cv::cv2eigen(cv_T, ACC_SCALE);

    
    fsSettings["initpos"]>>cv_T;      cv::cv2eigen(cv_T, INITPOS);
    fsSettings["initvel"]>>cv_T;        cv::cv2eigen(cv_T, INITVEL);
    fsSettings["initatt"]>>cv_T;        cv::cv2eigen(cv_T, INITATT);
    

    fsSettings["initposstd"]>>cv_T;      cv::cv2eigen(cv_T, POS_STD);
    fsSettings["initvelstd"]>>cv_T;        cv::cv2eigen(cv_T, VEL_STD);
    fsSettings["initattstd"]>>cv_T;        cv::cv2eigen(cv_T, ATT_STD);

    fsSettings["arw"]>>cv_T;                   cv::cv2eigen(cv_T, ACC_W);
    fsSettings["vrw"]>>cv_T;                   cv::cv2eigen(cv_T, GYR_W);

    fsSettings["gbstd"]>>cv_T;               cv::cv2eigen(cv_T, GYR_BIAS_STD);
    fsSettings["abstd"]>>cv_T;               cv::cv2eigen(cv_T, ACC_BIAS_STD);

    fsSettings["gsstd"]>>cv_T;                cv::cv2eigen(cv_T, GYR_SCALE_STD);
    fsSettings["asstd"]>>cv_T;                cv::cv2eigen(cv_T, ACC_SCALE_STD);
    CORRTIME=fsSettings["corrtime"];

    fsSettings["antlever"]>>cv_T;          cv::cv2eigen(cv_T, ANTLEVER);

    fsSettings.release();
    

}