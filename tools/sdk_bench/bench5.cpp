// Sustained single-point streaming from an EMPTY buffer, with a slowly moving
// target. Answers: can we stream at 50/100 Hz through the single-point API?
#include "aubo_driver/serviceinterface.h"
#include <chrono>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <thread>
using namespace std::chrono;
static double ms(steady_clock::time_point a, steady_clock::time_point b){
    return duration_cast<duration<double,std::milli>>(b-a).count(); }
static void st(const char* n, std::vector<double>& v){
    if(v.empty()){printf("%-30s no data\n",n);return;}
    std::sort(v.begin(),v.end()); double s=0; for(double x:v)s+=x;
    printf("%-30s n=%3zu p50=%7.2f p90=%7.2f max=%8.2f mean=%7.2f ms\n",
        n,v.size(),v[v.size()/2],v[v.size()*9/10],v.back(),s/v.size()); v.clear(); }

static void run(ServiceInterface& send, ServiceInterface& recv, const char* label,
                int period_ms, int n, double* q0)
{
    // wait for an empty controller buffer
    aubo_robot_namespace::RobotDiagnosis d;
    for(int i=0;i<200;i++){ recv.robotServiceGetRobotDiagnosisInfo(d);
        if(d.macTargetPosDataSize==0) break; std::this_thread::sleep_for(milliseconds(20)); }

    double q[8]={0}; for(int i=0;i<6;i++) q[i]=q0[i];
    std::vector<double> lat, gap; auto prev=steady_clock::now();
    int rib_max=0;
    for(int i=0;i<n;i++){
        q[5]=q0[5]+(1.0/57.29578)*(i/(double)n);        // slow ramp, +1 deg total
        auto a=steady_clock::now();
        send.robotServiceSetRobotPosData2Canbus(q);
        auto b=steady_clock::now();
        lat.push_back(ms(a,b)); gap.push_back(ms(prev,b)); prev=b;
        if(i%20==0){ recv.robotServiceGetRobotDiagnosisInfo(d);
                     rib_max=std::max(rib_max,(int)d.macTargetPosDataSize); }
        if(period_ms>0) std::this_thread::sleep_for(milliseconds(period_ms));
    }
    st(label, lat); st("   achieved interval", gap);
    printf("   max ribData seen = %d\n", rib_max);
    // park back at start
    for(int i=0;i<30;i++){ send.robotServiceSetRobotPosData2Canbus(q0); std::this_thread::sleep_for(milliseconds(50)); }
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
    double q0[8]={0}; for(int i=0;i<6;i++) q0[i]=w0.jointpos[i];
    printf("start wrist3=%.3f\n\n", q0[5]*57.29578);

    run(send,recv,"[50 Hz pacing, moving]",20,200,q0); printf("\n");
    run(send,recv,"[100 Hz pacing, moving]",10,200,q0); printf("\n");
    run(send,recv,"[200 Hz pacing, moving]",5,300,q0); printf("\n");

    send.robotServiceLeaveTcp2CanbusMode();
    send.robotServiceLogout(); recv.robotServiceLogout();
    printf("done\n"); return 0;
}
