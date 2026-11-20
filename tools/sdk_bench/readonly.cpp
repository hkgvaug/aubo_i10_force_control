// READ-ONLY health check. Does not enter canbus mode, does not command motion.
#include "aubo_driver/serviceinterface.h"
#include <cstdio>
#include <cmath>
int main(){
    setvbuf(stdout,NULL,_IONBF,0);
    ServiceInterface s;
    int r=s.robotServiceLogin("192.168.100.11",8899,"aubo","123456");
    printf("login=%d\n", r);
    if(r!=0) return 1;

    aubo_robot_namespace::RobotState st;
    int rs=s.robotServiceGetRobotCurrentState(st);
    printf("robotState=%d (0=stopped 1=running 2=paused 3=resumed)\n", (int)st);

    aubo_robot_namespace::RobotDiagnosis d;
    int rd=s.robotServiceGetRobotDiagnosisInfo(d);
    printf("diagnosis=%d  power=%d softEmg=%d remoteEmg=%d orpe=%d ribData=%u\n",
        rd,(int)d.armPowerStatus,(int)d.softEmergency,(int)d.remoteEmergency,
        (int)d.orpeStatus,(unsigned)d.macTargetPosDataSize);
    printf("singularityOverSpeedAlarm=%d  jointErrorStatus=%d  softEmergency=%d\n",
        (int)d.singularityOverSpeedAlarm,(int)d.jointErrorStatus,(int)d.softEmergency);

    aubo_robot_namespace::wayPoint_S w; ServiceInterface::initWayPointDataType(w);
    if(s.robotServiceGetCurrentWaypointInfo(w)==0){
        printf("joints(deg):");
        for(int i=0;i<6;i++) printf(" %8.2f", w.jointpos[i]*180.0/M_PI);
        printf("\nflange(m):  x=%.4f y=%.4f z=%.4f\n", w.cartPos.position.x, w.cartPos.position.y, w.cartPos.position.z);
    }
    s.robotServiceLogout();
    printf("read-only check done (control untouched)\n");
    return 0;
}
