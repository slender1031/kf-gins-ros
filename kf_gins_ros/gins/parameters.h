#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <vector>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <map>

using namespace std;
using Eigen::Vector3d;


extern string IMU_TOPIC;
extern string GNSS_TOPIC;

extern Vector3d GYR_BIAS, GYR_BIAS_STD;
extern Vector3d ACC_BIAS, ACC_BIAS_STD;

extern Vector3d GYR_SCALE, GYR_SCALE_STD;
extern Vector3d ACC_SCALE, ACC_SCALE_STD;


extern Vector3d INITPOS;
extern Vector3d INITVEL;
extern Vector3d INITATT;

extern Vector3d POS_STD;
extern Vector3d VEL_STD;
extern Vector3d ATT_STD;

extern Vector3d ACC_W, ACC_N;
extern Vector3d GYR_W, GYR_N;
extern double CORRTIME;

extern Vector3d ANTLEVER;
extern std::string GINS_RESULT_PATH;

void readParameters(std::string config_file);


#endif // ! PARAMETERS_H_