// Is every controller RPC slow, or only the canbus push?
#include "aubo_driver/serviceinterface.h"
#include <chrono>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <thread>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
using namespace std::chrono;
static double ms(steady_clock::time_point a, steady_clock::time_point b){
    return duration_cast<duration<double,std::milli>>(b-a).count(); }
static void st(const char* n, std::vector<double>& v){
    std::sort(v.begin(), v.end()); double s=0; for(double x:v) s+=x;
    printf("%-40s n=%3zu p50=%7.2f p90=%7.2f p99=%7.2f max=%7.2f mean=%7.2f ms\n",
        n, v.size(), v[v.size()/2], v[v.size()*9/10], v[v.size()*99/100], v.back(), s/v.size());
}
int main(){
    setvbuf(stdout,NULL,_IONBF,0);

    // --- raw TCP connect + tiny exchange, bypassing the SDK ---
    std::vector<double> conn, rtt;
    for(int i=0;i<30;i++){
        int fd=socket(AF_INET,SOCK_STREAM,0);
        sockaddr_in a{}; a.sin_family=AF_INET; a.sin_port=htons(8899);
        inet_pton(AF_INET,"192.168.100.11",&a.sin_addr);
        int one=1; setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&one,sizeof(one));
        auto t=steady_clock::now();
        if(connect(fd,(sockaddr*)&a,sizeof(a))==0){
            conn.push_back(ms(t,steady_clock::now()));
            // send a harmless 4-byte header and wait briefly for any reply
            unsigned char pkt[4]={0,0,0,0};
            auto t2=steady_clock::now();
            write(fd,pkt,4);
            unsigned char buf[64];
            struct timeval tv{0,200000}; setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv));
            int n=read(fd,buf,sizeof(buf));
            rtt.push_back(ms(t2,steady_clock::now()));
            (void)n;
        }
        close(fd);
        std::this_thread::sleep_for(milliseconds(100));
    }
    st("raw TCP connect (port 8899)", conn);
    st("raw TCP 4B write+read timeout", rtt);

    // --- trivial SDK queries ---
    ServiceInterface s;
    printf("login=%d\n", s.robotServiceLogin("192.168.100.11",8899,"aubo","123456"));
    aubo_robot_namespace::wayPoint_S wp; ServiceInterface::initWayPointDataType(wp);
    aubo_robot_namespace::RobotDiagnosis d;
    std::vector<double> wpv, diagv;
    for(int i=0;i<60;i++){
        auto t=steady_clock::now(); s.robotServiceGetCurrentWaypointInfo(wp);
        wpv.push_back(ms(t,steady_clock::now()));
        t=steady_clock::now(); s.robotServiceGetRobotDiagnosisInfo(d);
        diagv.push_back(ms(t,steady_clock::now()));
    }
    st("SDK GetCurrentWaypointInfo", wpv);
    st("SDK GetRobotDiagnosisInfo", diagv);
    s.robotServiceLogout();
    printf("done\n");
    return 0;
}
