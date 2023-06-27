#ifndef ROTATION_H_
#define ROTATION_H_

#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

class Rotation
{
    public:
        static Matrix3d skewSymmetric(const Vector3d &vector) 
        {
            Matrix3d mat;
            mat << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1), vector(0), 0;
            return mat;
        }

        static Vector3d Quaternion2Vector(const Quaterniond &quaternion) 
        {
            Eigen::AngleAxisd axisd(quaternion);
            return axisd.angle() * axisd.axis();
        }

        static Quaterniond Vector2Quaternion(const Vector3d &rotvec) 
        {
            double angle = rotvec.norm();
            Vector3d vec = rotvec.normalized();
            return Quaterniond(Eigen::AngleAxisd(angle, vec));
        }

        static Quaterniond euler2quaternion(const Vector3d &euler) 
        {
                return Quaterniond(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                                Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
        }


        // ZYX旋转顺序, 前右下的IMU, 输出RPY
        static Vector3d matrix2euler(const Eigen::Matrix3d &dcm) 
        {
                Vector3d euler;

                euler[1] = atan(-dcm(2, 0) / sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));

                if (dcm(2, 0) <= -0.999) {
                    euler[0] = 0;
                    euler[2] = atan2((dcm(1, 2) - dcm(0, 1)), (dcm(0, 2) + dcm(1, 1)));
                    //std::cout << "[WARNING] Rotation::matrix2euler: Singular Euler Angle! Set the roll angle to 0!" << std::endl;
                } else if (dcm(2, 0) >= 0.999) {
                    euler[0] = 0;
                    euler[2] = M_PI + atan2((dcm(1, 2) + dcm(0, 1)), (dcm(0, 2) - dcm(1, 1)));
                    //std::cout << "[WARNING] Rotation::matrix2euler: Singular Euler Angle! Set the roll angle to 0!" << std::endl;
                } else {
                    euler[0] = atan2(dcm(2, 1), dcm(2, 2));
                    euler[2] = atan2(dcm(1, 0), dcm(0, 0));
                }

                // heading 0~2PI
                if (euler[2] < 0) {
                    euler[2] = M_PI * 2 + euler[2];
                }

                return euler;
        }

        static Vector3d quaternion2euler(const Quaterniond &quaternion) 
        {
                return matrix2euler(quaternion.toRotationMatrix());
        }

};



#endif // !ROTATION_H_

