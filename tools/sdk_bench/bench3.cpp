// (1) dump controller state, (2) test the BULK waypoint overload of
// robotServiceSetRobotPosData2Canbus, which is the documented batch API.
#include "aubo_driver/serviceinterface.h"
#include <chrono>
#include <cstdio>
#include <vector>
#include <cmath>
#include <thread>
using namespace std::chrono;
static double ms(steady_clock::time_point a, steady_clock::time_point b){
    return duration_cast<duration<double,std::milli>>(b-a).count(); }
int main(){
    setvbuf(stdout,NULL,_IONBF,0);
    ServiceInterface send, recv;
    if(send.robotServiceLogin("192.168.100.11",8899,"aubo","123456")!=0) return 1;
    recv.robotServiceLogin("192.168.100.11",8899,"aubo","123456");
    int rc=send.robotServiceEnterTcp2CanbusMode();
    if(rc==10023){ send.robotServiceLeaveTcp2CanbusMode(); rc=send.robotServiceEnterTcp2CanbusMode(); }
    printf("enterTcp2CanbusMode=%d\n", rc);

    aubo_robot_namespace::RobotDiagnosis d;
    recv.robotServiceGetRobotDiagnosisInfo(d);
    printf("state: armPowerStatus=%d softEmergency=%d remoteEmergency=%d orpeStatus=%d ribData=%u\n",
        (int)d.armPowerStatus,(int)d.softEmergency,(int)d.remoteEmergency,(int)d.orpeStatus,(unsigned)d.macTargetPosDataSize);

    aubo_robot_namespace::wayPoint_S w0; ServiceInterface::initWayPointDataType(w0);
    recv.robotServiceGetCurrentWaypointInfo(w0);
    printf("start wrist3=%.3f deg\n", w0.jointpos[5]*57.29578);

    // ---- single-point pushes for reference ----
    double q[8]={0}; for(int i=0;i<6;i++) q[i]=w0.jointpos[i];
    std::vector<double> lat;
    for(int i=0;i<20;i++){ auto a=steady_clock::now(); send.robotServiceSetRobotPosData2Canbus(q);
        lat.push_back(ms(a,steady_clock::now())); }
    double s=0; for(double x:lat) s+=x;
    printf("single-point push: mean=%.1f ms over %zu calls\n", s/lat.size(), lat.size());

    // ---- bulk push: 100 waypoints, wrist3 +0.5 deg ramp and back ----
    std::vector<aubo_robot_namespace::wayPoint_S> pts;
    const double amp = 0.5/57.29578;
    for(int i=0;i<100;i++){
        aubo_robot_namespace::wayPoint_S p = w0;
        p.jointpos[5] = w0.jointpos[5] + amp*sin(2*M_PI*i/99.0);
        pts.push_back(p);
    }
    auto t0=steady_clock::now();
    int rv = send.robotServiceSetRobotPosData2Canbus(pts);
    printf("BULK push of %zu waypoints: ret=%d  took %.1f ms\n", pts.size(), rv, ms(t0,steady_clock::now()));

    // sample actual motion for 3 s
    printf("sampling motion:");
    for(int i=0;i<12;i++){
        std::this_thread::sleep_for(milliseconds(250));
        aubo_robot_namespace::wayPoint_S w; ServiceInterface::initWayPointDataType(w);
        recv.robotServiceGetCurrentWaypointInfo(w);
        printf(" %.3f", w.jointpos[5]*57.29578);
    }
    printf("  (deg)\n");

    send.robotServiceLeaveTcp2CanbusMode();
    send.robotServiceLogout(); recv.robotServiceLogout();
    printf("done\n");
    return 0;
}
