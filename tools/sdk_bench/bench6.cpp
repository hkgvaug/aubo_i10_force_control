// Chunked streaming: keep the controller RIB topped up with BULK waypoint
// pushes instead of single points.  This is the intended high-rate path.
#include "aubo_driver/serviceinterface.h"
#include <chrono>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <thread>
#include <cmath>
using namespace std::chrono;
static double s_now(){ static auto t0=steady_clock::now();
    return duration_cast<duration<double>>(steady_clock::now()-t0).count(); }
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
    const double j0=w0.jointpos[5];
    printf("start wrist3=%.3f deg\n", j0*57.29578);

    // reference: smooth 3 s ramp of +3 deg on wrist3, sampled at 278 Hz (3.6 ms/pt)
    const double STEP_MS=3.6, AMP=3.0/57.29578, DUR=3.0;
    const int TOTAL=(int)(DUR*1000/STEP_MS);
    const int CHUNK=24;                 // ~86 ms of motion per bulk call
    const double LOW_MS=60.0;           // refill when the RIB drops below this

    aubo_robot_namespace::RobotDiagnosis d;
    int idx=0, calls=0; double max_lat=0, sum_lat=0;
    int rib_min=100000, rib_max=0;
    printf("\nstreaming %d points in chunks of %d ...\n", TOTAL, CHUNK);

    double t_start=s_now();
    while(idx<TOTAL){
        recv.robotServiceGetRobotDiagnosisInfo(d);
        int rib=(int)d.macTargetPosDataSize;
        rib_min=std::min(rib_min,rib); rib_max=std::max(rib_max,rib);
        if(rib>LOW_MS){ std::this_thread::sleep_for(milliseconds(5)); continue; }

        std::vector<aubo_robot_namespace::wayPoint_S> chunk;
        for(int k=0;k<CHUNK && idx<TOTAL;k++,idx++){
            aubo_robot_namespace::wayPoint_S p=w0;
            p.jointpos[5]=j0+AMP*(idx/(double)TOTAL);
            chunk.push_back(p);
        }
        auto a=steady_clock::now();
        send.robotServiceSetRobotPosData2Canbus(chunk);
        double lat=duration_cast<duration<double,std::milli>>(steady_clock::now()-a).count();
        max_lat=std::max(max_lat,lat); sum_lat+=lat; calls++;
    }
    // let it finish
    for(int i=0;i<100;i++){ recv.robotServiceGetRobotDiagnosisInfo(d);
        if(d.macTargetPosDataSize==0) break; std::this_thread::sleep_for(milliseconds(20)); }
    double elapsed=s_now()-t_start;

    printf("\nbulk calls=%d  avg=%.1f ms  max=%.1f ms\n", calls, sum_lat/calls, max_lat);
    printf("rib band: %d .. %d ms\n", rib_min, rib_max);
    printf("motion: %d pts in %.2f s -> %.1f Hz equivalent\n", TOTAL, elapsed, TOTAL/elapsed);

    aubo_robot_namespace::wayPoint_S w1; ServiceInterface::initWayPointDataType(w1);
    recv.robotServiceGetCurrentWaypointInfo(w1);
    printf("end wrist3=%.3f deg (commanded %.3f)\n", w1.jointpos[5]*57.29578, (j0+AMP)*57.29578);

    // park back
    std::vector<aubo_robot_namespace::wayPoint_S> back;
    for(int i=0;i<40;i++){ aubo_robot_namespace::wayPoint_S p=w0; back.push_back(p); }
    send.robotServiceSetRobotPosData2Canbus(back);
    std::this_thread::sleep_for(milliseconds(1500));

    send.robotServiceLeaveTcp2CanbusMode();
    send.robotServiceLogout(); recv.robotServiceLogout();
    printf("done\n"); return 0;
}
