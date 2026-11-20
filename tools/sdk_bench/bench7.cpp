// Sweep chunk size / refill threshold for the bulk streaming path.
#include "aubo_driver/serviceinterface.h"
#include <chrono>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <thread>
using namespace std::chrono;
static double s_now(){ static auto t0=steady_clock::now();
    return duration_cast<duration<double>>(steady_clock::now()-t0).count(); }

static void trial(ServiceInterface& send, ServiceInterface& recv,
                  const aubo_robot_namespace::wayPoint_S& w0, double j0,
                  int chunk, double low_ms)
{
    aubo_robot_namespace::RobotDiagnosis d;
    const double STEP_MS=3.6, DUR=2.0, AMP=2.0/57.29578;
    const int TOTAL=(int)(DUR*1000/STEP_MS);
    int idx=0,calls=0,rib_min=100000,rib_max=0; double sum=0,mx=0;

    for(int i=0;i<200;i++){ recv.robotServiceGetRobotDiagnosisInfo(d);
        if(d.macTargetPosDataSize==0) break; std::this_thread::sleep_for(milliseconds(20)); }

    double t0=s_now();
    while(idx<TOTAL){
        recv.robotServiceGetRobotDiagnosisInfo(d);
        int rib=(int)d.macTargetPosDataSize;
        rib_min=std::min(rib_min,rib); rib_max=std::max(rib_max,rib);
        if(rib>low_ms){ std::this_thread::sleep_for(milliseconds(4)); continue; }
        std::vector<aubo_robot_namespace::wayPoint_S> c;
        for(int k=0;k<chunk && idx<TOTAL;k++,idx++){
            aubo_robot_namespace::wayPoint_S p=w0;
            p.jointpos[5]=j0+AMP*(idx/(double)TOTAL);
            c.push_back(p);
        }
        auto a=steady_clock::now();
        send.robotServiceSetRobotPosData2Canbus(c);
        double lat=duration_cast<duration<double,std::milli>>(steady_clock::now()-a).count();
        sum+=lat; mx=std::max(mx,lat); calls++;
    }
    for(int i=0;i<100;i++){ recv.robotServiceGetRobotDiagnosisInfo(d);
        if(d.macTargetPosDataSize==0) break; std::this_thread::sleep_for(milliseconds(20)); }
    double el=s_now()-t0;
    printf("chunk=%3d low=%5.0fms | calls=%2d avg=%6.1f max=%6.1f | rib %3d..%3d | %.1f Hz equivalent\n",
           chunk, low_ms, calls, sum/calls, mx, rib_min, rib_max, TOTAL/el);
    fflush(stdout);

    std::vector<aubo_robot_namespace::wayPoint_S> back(60, w0);
    send.robotServiceSetRobotPosData2Canbus(back);
    std::this_thread::sleep_for(milliseconds(1200));
}

int main(){
    setvbuf(stdout,NULL,_IONBF,0);
    ServiceInterface send, recv;
    if(send.robotServiceLogin("192.168.100.11",8899,"aubo","123456")!=0) return 1;
    recv.robotServiceLogin("192.168.100.11",8899,"aubo","123456");
    int rc=send.robotServiceEnterTcp2CanbusMode();
    if(rc==10023){ send.robotServiceLeaveTcp2CanbusMode(); rc=send.robotServiceEnterTcp2CanbusMode(); }
    printf("enter=%d\n",rc);
    aubo_robot_namespace::wayPoint_S w0; ServiceInterface::initWayPointDataType(w0);
    recv.robotServiceGetCurrentWaypointInfo(w0);
    double j0=w0.jointpos[5];
    printf("start wrist3=%.3f deg\n\n", j0*57.29578);
    int chunks[]={24,48,72,120};
    double lows[]={80,150,220,300};
    for(int ci=0;ci<4;ci++)
        for(int li=0;li<4;li++)
            if((ci+li)%2==0||ci==li) trial(send,recv,w0,j0,chunks[ci],lows[li]);
    send.robotServiceLeaveTcp2CanbusMode();
    send.robotServiceLogout(); recv.robotServiceLogout();
    printf("done\n"); return 0;
}
