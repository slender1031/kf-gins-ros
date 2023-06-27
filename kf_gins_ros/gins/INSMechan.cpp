#include "INSMechan.h"
#include "earth.h"

void INSMechan::insMechan(IMU &imupre, IMU &imucur, PVA &pvapre, PVA &pvacur)
{

    VelocityUpdate(imupre, imucur, pvapre, pvacur);
    PositionUpdate(imupre, imucur, pvapre, pvacur);
    AttitudeUpdate(imupre, imucur, pvapre, pvacur);
}



//导航系NED速度推导
void INSMechan::VelocityUpdate(IMU &imupre, IMU &imucur, PVA &pvapre, PVA &pvacur)
{
    double RM, RN;
    Vector3d w_n_ie, w_n_en;
    RM=Earth::RM(pvapre.blh[0]);
    RN=Earth::RN(pvapre.blh[0]);
    w_n_ie=Earth::w_n_ie(pvapre.blh[0]);
    w_n_en=Earth::w_n_en(pvapre.blh, pvapre.vel);

    // v_nk=v_nk-1+delta_v_nf+delta_v_ng
    //三项分别对应速度积分初值、比力f积分项、重力g积分项

    //1、计算比力积分项delta_v_nf
    //旋转补偿项(rotation compensation term)
    //划桨效应补偿项(sculling compensation term)
    Vector3d RotationTerm=imucur.dtheta.cross(imucur.dvel)/2;
    Vector3d ScullingTerm=(imupre.dtheta.cross(imucur.dvel)+imupre.dvel.cross(imucur.dtheta))/12;
    Vector3d d_vbf=imucur.dvel+RotationTerm+ScullingTerm;

    //b系转n系
    Vector3d zeta=(w_n_ie+w_n_en)*imucur.dt/2;
    Matrix3d Cnb=pvapre.qnb.matrix();
    Matrix3d Cnn=Matrix3d::Identity()-0.5*Rotation::skewSymmetric(zeta);
    Vector3d d_vnf=Cnn*Cnb*d_vbf;


    //2、计算重力积分项delta_v_ng（缓慢变化项，取tk-1/2，中间时刻）
    double gravity=Earth::gravity(pvapre.blh);
    Vector3d gnl(0,0,gravity);
    Vector3d d_vng=(gnl-(2*w_n_ie+w_n_en).cross(pvapre.vel))*imucur.dt;

    //3、计算中间时刻的位置和速度
    Vector3d midvel=pvapre.vel+(d_vnf+d_vng)/2;

    //计算中间时刻的位置（因为位置还没更新，所以还没有当前时刻的位置，只能通过外推）
    Quaterniond qnn=Rotation::Vector2Quaternion(zeta);
    Vector3d zeta2(0, 0, -WGS84_WIE * imucur.dt / 2);
    Quaterniond qee=Rotation::Vector2Quaternion(zeta2);
    Quaterniond qen=Earth::qen(pvapre.blh);
    qen=qee*qen*qnn;

    Vector3d midpos;
    midpos[2]=pvapre.blh[2] - midpos[2]*imucur.dt/2;
    midpos = Earth::blh(qen, midpos[2]);

    //4、重新计算速度各项分量
    // 重新计算中间时刻tk-1/2的wie_e, wen_n
    w_n_ie=Earth::w_n_ie(midpos[0]);
    w_n_en=Earth::w_n_en(midpos, midvel);

    // 重新计算n系比力积分项
    zeta=(w_n_ie+w_n_en)*imucur.dt/2;
    Cnn=Matrix3d::Identity()-0.5*Rotation::skewSymmetric(zeta);
    d_vnf=Cnn*Cnb*d_vbf;

    //重新计算重力积分项d_vng
    gravity=Earth::gravity(midpos);
    gnl<<0, 0, gravity;
    d_vng=(gnl-(2*w_n_ie+w_n_en).cross(midvel))*imucur.dt;

    //5、更新速度
    pvacur.vel=pvapre.vel+d_vnf+d_vng;
}


//导航系NED位置推导
void INSMechan::PositionUpdate(IMU &imupre, IMU &imucur, PVA &pvapre, PVA &pvacur)
{
    //计算中间时刻tk-1/2速度和位置
    Vector3d midvel=(pvapre.vel+pvacur.vel)/2;
    Vector3d midpos=pvapre.blh+Earth::DRi(pvapre.blh) * midvel * imucur.dt / 2;

    // 计算中间时刻地理参数
    double RM, RN;
    Vector3d w_n_ie, w_n_en;
    RM=Earth::RM(midpos[0]);
    RN=Earth::RN(midpos[0]);
    w_n_ie=Earth::w_n_ie(midpos[0]);
    w_n_en=Earth::w_n_en(midpos, midvel);

    //计算tk-1到tk时刻旋转矢量
    Vector3d zeta=(w_n_ie+w_n_en)*imucur.dt;
    Quaterniond qnn=Rotation::Vector2Quaternion(zeta);
    Vector3d zeta2(0, 0, -WGS84_WIE * imucur.dt);
    Quaterniond qee=Rotation::Vector2Quaternion(zeta2);
    
    Quaterniond qen=Earth::qen(pvapre.blh);
    qen=qee*qen*qnn;

    //更新位置
    pvacur.blh[2]=pvapre.blh[2]-midvel[2]*imucur.dt;
    pvacur.blh    = Earth::blh(qen, pvacur.blh[2]);
}


//导航系NED姿态推导
void INSMechan::AttitudeUpdate(IMU &imupre, IMU &imucur, PVA &pvapre, PVA &pvacur)
{
    //计算中间时刻tk-1/2的位置和速度
    Vector3d midvel=(pvapre.vel+pvacur.vel)/2;
    Quaterniond qen_pre=Earth::qen(pvapre.blh);
    Quaterniond qen_cur=Earth::qen(pvacur.blh);
    Vector3d zeta=Rotation::Quaternion2Vector(qen_cur.inverse()*qen_pre);

    Quaterniond qen_mid(qen_pre*Rotation::Vector2Quaternion(zeta/2).inverse());
    Vector3d midpos;
    midpos[2] = (pvacur.blh[2] + pvapre.blh[2]) / 2;
    midpos    = Earth::blh(qen_mid, midpos[2]);

    //计算中间时刻地理参数
    double RM, RN;
    Vector3d w_n_ie, w_n_en;
    RM=Earth::RM(midpos[0]);
    RN=Earth::RN(midpos[0]);
    w_n_ie=Earth::w_n_ie(midpos[0]);
    w_n_en=Earth::w_n_en(midpos, midvel);

     //计算n系tk-1到tk时刻旋转矢量
    Quaterniond qnn=Rotation::Vector2Quaternion(-(w_n_ie+w_n_en)*imucur.dt);

    // 计算b系旋转四元数 补偿二阶圆锥误差
    Quaterniond qbb=Rotation::Vector2Quaternion(imucur.dtheta + imupre.dtheta.cross(imucur.dtheta) / 12);

    //姿态更新
    pvacur.qnb=qnn*pvapre.qnb*qbb;
}

