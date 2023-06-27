#ifndef BASICTYPE_H_
#define BASICTYPE_H_


#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;


struct PVA
{
    double time;
    Vector3d blh;
    Vector3d vel;
    Quaterniond qnb;
};

struct GNSS {
    double time;
    Vector3d blh;
    Matrix3d poscov;

    bool isvalid;
} ;

struct IMU 
{
        double time;
        double dt;
        Vector3d acc;
        Vector3d gyro;
        Vector3d dvel;
        Vector3d dtheta;
} ;


#endif //BASICTYPE_H_

