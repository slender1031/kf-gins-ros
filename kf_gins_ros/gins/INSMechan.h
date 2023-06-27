#ifndef INSMECHAN_H_
#define INSMECHAN_H_

#include "basictype.h"
#include "rotation.h"

class INSMechan
{
    public:

        static void insMechan(IMU &imupre, IMU &imucur, PVA &pvapre, PVA &pvacur);

        static void VelocityUpdate(IMU &imupre, IMU &imucur, PVA &pvapre, PVA &pvacur);
        static void PositionUpdate(IMU &imupre, IMU &imucur, PVA &pvapre, PVA &pvacur);
        static void AttitudeUpdate(IMU &imupre, IMU &imucur, PVA &pvapre, PVA &pvacur);

};




#endif // !INSMECHAN_H_
