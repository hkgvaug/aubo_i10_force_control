// Calibrate dt_ctrl: how long the i10 controller takes to play back one queued
// waypoint.  Moves wrist3 in a smooth triangle (out 2 deg, back 2 deg) so the
// net displacement is zero and there is no return-jump.  Every point is checked
// with the same joint step fuse used by aubo_driver.
#include "aubo_driver/serviceinterface.h"
#include "aubo_driver/joint_step_fuse.h"
#include <chrono>
#include <cstdio>
#include <cmath>
#include <vector>
#include <thread>
using namespace std::chrono;
static double s_now(){ static auto t0=steady_clock::now();
    return duration_cast<duration<double>>(steady_clock::now()-t0).count(); }

int main(int argc, char** argv)
{
    setvbuf(stdout,NULL,_IONBF,0);
    const int    N        = (argc>1) ? atoi(argv[1]) : 600;   // points
    const double AMP_DEG  = (argc>2) ? atof(argv[2]) : 2.0;   // half amplitude
    const double LIMIT    = 0.002;                            // driver default fuse
    const int    CHUNK    = 48;
    const double LOW_MS   = 120.0;

    ServiceInterface send, recv;
    if(send.robotServiceLogin("192.168.100.11",8899,"aubo","123456")!=0) return 1;
    recv.robotServiceLogin("192.168.100.11",8899,"aubo","123456");
    int rc=send.robotServiceEnterTcp2CanbusMode();
    if(rc==10023){ send.robotServiceLeaveTcp2CanbusMode(); rc=send.robotServiceEnterTcp2CanbusMode(); }
    printf("enterTcp2CanbusMode=%d\n", rc);
    if(rc!=0) return 1;

    aubo_robot_namespace::wayPoint_S w0; ServiceInterface::initWayPointDataType(w0);
    recv.robotServiceGetCurrentWaypointInfo(w0);
    const double j0 = w0.jointpos[5];
    printf("wrist3 start = %.4f deg, points=%d, amplitude=+-%.2f deg\n",
           j0*180.0/M_PI, N, AMP_DEG);

    const double amp = AMP_DEG*M_PI/180.0;
    std::vector<aubo_robot_namespace::wayPoint_S> pts;
    pts.reserve(N);
    double prev[6]; for(int i=0;i<6;i++) prev[i]=w0.jointpos[i];
    double max_step=0.0; int fuse_hits=0;
    for(int i=0;i<N;i++){
        double phase = (double)i/(N-1);                 // 0..1
        double tri   = 1.0 - std::fabs(2.0*phase - 1.0); // 0 -> 1 -> 0
        aubo_robot_namespace::wayPoint_S p = w0;
        p.jointpos[5] = j0 + amp*tri;
        int bad=-1; double d=0.0;
        if(!aubo_driver::jointStepWithin(prev, p.jointpos, 6, LIMIT, bad, d)){
            fuse_hits++; printf("FUSE at point %d joint %d delta %.5f\n", i, bad, d); break;
        }
        double step_here = std::fabs(p.jointpos[5] - prev[5]);
        max_step = std::max(max_step, step_here);
        for(int j=0;j<6;j++) prev[j]=p.jointpos[j];
        pts.push_back(p);
    }
    printf("built %zu points, max per-point step = %.6f rad (%.4f deg), fuse rejects = %d\n",
           pts.size(), max_step, max_step*180.0/M_PI, fuse_hits);
    if(pts.size()!=N || fuse_hits){ printf("ABORT: point stream not clean\n");
        send.robotServiceLeaveTcp2CanbusMode(); return 1; }

    aubo_robot_namespace::RobotDiagnosis d;
    for(int i=0;i<300;i++){ recv.robotServiceGetRobotDiagnosisInfo(d);
        if(d.macTargetPosDataSize==0) break; std::this_thread::sleep_for(milliseconds(20)); }
    printf("controller buffer empty, starting playback...\n");

    int idx=0, calls=0; double t_first=-1.0;
    while(idx<(int)pts.size()){
        recv.robotServiceGetRobotDiagnosisInfo(d);
        int rib=(int)d.macTargetPosDataSize;
        if(rib>(int)LOW_MS){ std::this_thread::sleep_for(milliseconds(5)); continue; }
        std::vector<aubo_robot_namespace::wayPoint_S> c;
        for(int k=0;k<CHUNK && idx<(int)pts.size();k++,idx++) c.push_back(pts[idx]);
        if(t_first<0.0) t_first=s_now();
        send.robotServiceSetRobotPosData2Canbus(c);
        calls++;
    }
    // wait for the controller to finish the last buffered point
    int polls=0;
    while(polls++<600){
        recv.robotServiceGetRobotDiagnosisInfo(d);
        if(d.macTargetPosDataSize==0) break;
        std::this_thread::sleep_for(milliseconds(10));
    }
    double t_total = s_now()-t_first;

    printf("\nbulk calls=%d over %.3f s\n", calls, t_total);
    printf("dt_ctrl = %.4f ms/point  (%.1f Hz)\n", 1000.0*t_total/N, N/t_total);

    aubo_robot_namespace::wayPoint_S w1; ServiceInterface::initWayPointDataType(w1);
    recv.robotServiceGetCurrentWaypointInfo(w1);
    printf("wrist3 end   = %.4f deg  (should equal start %.4f, delta %.4f deg)\n",
           w1.jointpos[5]*180.0/M_PI, j0*180.0/M_PI,
           (w1.jointpos[5]-j0)*180.0/M_PI);

    send.robotServiceLeaveTcp2CanbusMode();
    send.robotServiceLogout(); recv.robotServiceLogout();
    printf("left canbus mode, logged out\n");
    return 0;
}
