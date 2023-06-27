#ifndef EARTH_H_
#define EARTH_H_

#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

// WGS84椭球模型参数
const double WGS84_WIE = 7.2921151467E-5;       /* 地球自转角速度*/
const double WGS84_F   = 0.0033528106647474805; /* 扁率 */
const double WGS84_RA  = 6378137.0000000000;    /* 长半轴a */
const double WGS84_RB  = 6356752.3142451793;    /* 短半轴b */
const double WGS84_GM0 = 398600441800000.00;    /* 地球引力常数 */
const double WGS84_E1  = 0.0066943799901413156; /* 第一偏心率平方 */
const double WGS84_E2  = 0.0067394967422764341; /* 第二偏心率平方 */


class Earth
{
    public:

        //RM子午圈曲率半径
        static double RM(const double lat)
        {
            return double(WGS84_RA*(1-WGS84_E1)/pow(1-WGS84_E1*sin(lat)*sin(lat)  ,3/2));
        }

        //RN卯酉圈曲率半径
        static double RN(const double lat)
        {
            return double(WGS84_RA/sqrt(1-WGS84_E1*sin(lat)*sin(lat)));
        }

        static Vector3d w_n_ie(const double lat)
        {
            return Vector3d(WGS84_WIE*cos(lat), 0, -WGS84_WIE*sin(lat));
        }

        static Vector3d w_n_en(const Vector3d blh, const Vector3d vel)
        {
            double rn=RN(blh[0]);
            double rm=RM(blh[0]);
            return Vector3d(vel[1]/(rn+blh[2]), -vel[0]/(rm+blh[2]), -vel[1]*tan(blh[0])/(rn+blh[2]) );
        }
        
        static double gravity(const Vector3d blh) 
        {
            double sin2 = sin(blh[0]);
            sin2 *= sin2;
            
            return 9.7803267715 * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin2 * sin2) +
               blh[2] * (0.0000000043977311 * sin2 - 0.0000030876910891) + 0.0000000000007211 * blh[2] * blh[2];
        }

        //n系转e系
        static Quaterniond qen(const Vector3d &blh) {
            Quaterniond quat;

            double coslon, sinlon, coslat, sinlat;

            coslon = cos(blh[1] * 0.5);
            sinlon = sin(blh[1] * 0.5);
            coslat = cos(-M_PI * 0.25 - blh[0] * 0.5);
            sinlat = sin(-M_PI * 0.25 - blh[0] * 0.5);

            quat.w() = coslat * coslon;
            quat.x() = -sinlat * sinlon;
            quat.y() = sinlat * coslon;
            quat.z() = coslat * sinlon;

            return quat;
        }

        /* 从n系到e系转换四元数得到纬度和经度 */
        static Vector3d blh(const Quaterniond &qne, double height) {
            return {-2 * atan(qne.y() / qne.w()) - M_PI * 0.5, 2 * atan2(qne.z(), qne.w()), height};
        }

        /* n系相对位置转大地坐标相对位置 */
        static Matrix3d DRi(const Vector3d &blh) {
            Matrix3d dri = Matrix3d::Zero();

            double rm=RM(blh[0]);
            double rn=RN(blh[0]);
            dri(0, 0) = 1.0 / (rm + blh[2]);
            dri(1, 1) = 1.0 / ((rn + blh[2]) * cos(blh[0]));
            dri(2, 2) = -1;
            return dri;
        }

        /* 大地坐标相对位置转n系相对位置 */
        static Matrix3d DR(const Vector3d &blh) {
            Matrix3d dr = Matrix3d::Zero();

            double rm=RM(blh[0]);
            double rn=RN(blh[0]);

            dr(0, 0) = rm + blh[2];
            dr(1, 1) = (rn + blh[2]) * cos(blh[0]);
            dr(2, 2) = -1;
            return dr;
        }


        /* 大地坐标(纬度、经度和高程)转地心地固坐标 */
        static Vector3d blh2ecef(const Vector3d &blh) {
            double coslat, sinlat, coslon, sinlon;
            double rnh, rn;

            coslat = cos(blh[0]);
            sinlat = sin(blh[0]);
            coslon = cos(blh[1]);
            sinlon = sin(blh[1]);

            rn  = RN(blh[0]);
            rnh = rn + blh[2];

            return {rnh * coslat * coslon, rnh * coslat * sinlon, (rnh - rn * WGS84_E1) * sinlat};
        }
        

        /* n系(导航坐标系)到e系(地心地固坐标系)转换矩阵 */
        static Matrix3d Ren(const Vector3d &blh) {
            double coslon, sinlon, coslat, sinlat;
            sinlat = sin(blh[0]);
            sinlon = sin(blh[1]);
            coslat = cos(blh[0]);
            coslon = cos(blh[1]);

            Matrix3d dcm;
            dcm <<-sinlat * coslon,  -sinlon,   -coslat * coslon,
                           -sinlat *  sinlon,   coslon,   -coslat * sinlon,
                                            coslat,               0,                      -sinlat;

            return dcm;
        }


        /* 大地坐标转局部坐标(在origin处展开) */
        static Vector3d global2local(const Vector3d &origin, const Vector3d &global) {
            Vector3d ecef0 = blh2ecef(origin);
            Matrix3d Ren0  = Ren(origin);

            Vector3d ecef1 = blh2ecef(global);

            return Ren0.transpose() * (ecef1 - ecef0);
    }

};




#endif // !EARTH_H_




