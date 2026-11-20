// Is the canbus push slow because of backpressure (target not changing), or is
// it a hard hardware limit?  Compare fixed cadences and a moving target.
#include "aubo_driver/serviceinterface.h"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>
#include <thread>
using namespace std::chrono;
static double ms(steady_clock::time_point a, steady_clock::time_point b){
    return duration_cast<duration<double,std::milli>>(b-a).count(); }
static void st(const char* n, std::vector<double>& v){
    if(v.empty()){ printf("%-38s no samples\n", n); return; }
    std::sort(v.begin(), v.end()); double s=0; for(double x:v) s+=x;
    printf("%-38s n=%3zu p50=%7.2f p90=%7.2f max=%7.2f mean=%7.2f ms\n",
        n, v.size(), v[v.size()/2], v[v.size()*9/10], v.back(), s/v.size());
    v.clear();
}

static void bench(ServiceInterface& s, const char* label, int n, int period_ms, double* q, double* q_start)
{
    std::vector<double> lat, gap; auto prev = steady_clock::now();
    for(int i=0;i<n;i++){
        auto a = steady_clock::now();
        s.robotServiceSetRobotPosData2Canbus(q);
        auto b = steady_clock::now();
        lat.push_back(ms(a,b));
        gap.push_back(ms(prev,b)); prev=b;
        (void)q_start;
        if(period_ms>0) std::this_thread::sleep_for(milliseconds(period_ms));
    }
    st(label, lat);
    char g[64]; snprintf(g,sizeof(g),"    -> achieved interval");
    st(g, gap);
}

int main(){
    setvbuf(stdout,NULL,_IONBF,0);
    ServiceInterface send, recv;
    if(send.robotServiceLogin("192.168.100.11",8899,"aubo","123456")!=0) return 1;
    recv.robotServiceLogin("192.168.100.11",8899,"aubo","123456");

    int rc = send.robotServiceEnterTcp2CanbusMode();
    if(rc==10023){ send.robotServiceLeaveTcp2CanbusMode(); rc=send.robotServiceEnterTcp2CanbusMode(); }
    printf("enterTcp2CanbusMode=%d\n\n", rc);
    if(rc!=0) return 1;

    aubo_robot_namespace::wayPoint_S wp; ServiceInterface::initWayPointDataType(wp);
    recv.robotServiceGetCurrentWaypointInfo(wp);
    double q0[8]={0}, q[8]={0};
    for(int i=0;i<6;i++){ q0[i]=wp.jointpos[i]; q[i]=q0[i]; }
    printf("hold q(deg) = %.2f %.2f %.2f %.2f %.2f %.2f\n\n",
        q0[0]*57.29578,q0[1]*57.29578,q0[2]*57.29578,q0[3]*57.29578,q0[4]*57.29578,q0[5]*57.29578);

    // A: hold position, free-running (as fast as the SDK allows)
    bench(send, "[E] hold, free-running", 200, 0, q, q0);

    // B: hold position, paced 20 Hz
    bench(send, "[F] hold, paced 20 Hz (50ms)", 60, 50, q, q0);

    // C: hold position, paced 100 Hz (10ms)
    bench(send, "[G] hold, paced 100 Hz (10ms)", 200, 10, q, q0);

    // D: MOVING target: wrist3 (joint 6, index 5) +-0.5 deg oscillation at 100 Hz
    const double amp = 0.5/57.29578;
    std::vector<double> lat, gap; auto prev=steady_clock::now();
    for(int i=0;i<200;i++){
        q[5] = q0[5] + amp*sin(2*M_PI*i/100.0);
        auto a=steady_clock::now();
        send.robotServiceSetRobotPosData2Canbus(q);
        auto b=steady_clock::now();
        lat.push_back(ms(a,b));
        gap.push_back(ms(prev,b)); prev=b;
        std::this_thread::sleep_for(milliseconds(10));
    }
    q[5]=q0[5];
    for(int i=0;i<20;i++){ send.robotServiceSetRobotPosData2Canbus(q); std::this_thread::sleep_for(milliseconds(50)); }
    st("[H] MOVING +-0.5deg wrist3, paced 100 Hz", lat);
    st("    -> achieved interval", gap);

    aubo_robot_namespace::wayPoint_S w2; ServiceInterface::initWayPointDataType(w2);
    recv.robotServiceGetCurrentWaypointInfo(w2);
    printf("\nfinal wrist3 = %.3f deg (start %.3f)\n", w2.jointpos[5]*57.29578, q0[5]*57.29578);

    send.robotServiceLeaveTcp2CanbusMode();
    send.robotServiceLogout(); recv.robotServiceLogout();
    printf("done\n");
    return 0;
}
