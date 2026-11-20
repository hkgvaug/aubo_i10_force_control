// Measure how fast the controller DRAINS its RIB buffer (macTargetPosDataSize).
// This is the hard ceiling for any streaming servo, independent of ROS.
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
    printf("enterTcp2CanbusMode=%d\n",rc);

    aubo_robot_namespace::wayPoint_S w0; ServiceInterface::initWayPointDataType(w0);
    recv.robotServiceGetCurrentWaypointInfo(w0);

    // preload a closed oscillation so the arm ends where it started
    std::vector<aubo_robot_namespace::wayPoint_S> pts;
    const double amp=1.0/57.29578;   // +-1 deg on wrist3
    const int N=80;
    for(int i=0;i<N;i++){
        aubo_robot_namespace::wayPoint_S p=w0;
        p.jointpos[5]=w0.jointpos[5]+amp*sin(2*M_PI*i/(N-1.0));
        pts.push_back(p);
    }
    auto t0=steady_clock::now();
    int rv=send.robotServiceSetRobotPosData2Canbus(pts);
    printf("bulk load %d pts: ret=%d %.1f ms\n", N, rv, ms(t0,steady_clock::now()));

    aubo_robot_namespace::RobotDiagnosis d;
    printf("\n t(s)   ribData\n");
    auto start=steady_clock::now(); int last=-1;
    while(ms(start,steady_clock::now())<12000){
        recv.robotServiceGetRobotDiagnosisInfo(d);
        int cur=(int)d.macTargetPosDataSize;
        double t=ms(start,steady_clock::now())/1000.0;
        if(cur!=last){ printf("%6.2f  %3d\n", t, cur); last=cur; }
        if(cur==0 && t>0.5) break;
        std::this_thread::sleep_for(milliseconds(20));
    }
    double total=ms(start,steady_clock::now())/1000.0;
    printf("\ntotal %.1f s for %d points -> %.1f points/s consumed\n", total, N, N/total);

    send.robotServiceLeaveTcp2CanbusMode();
    send.robotServiceLogout(); recv.robotServiceLogout();
    printf("done\n");
    return 0;
}
