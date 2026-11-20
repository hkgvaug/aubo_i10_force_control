// Release the controller back to the teach pendant: leave Tcp2CanbusMode + logout.
#include "aubo_driver/serviceinterface.h"
#include <cstdio>
int main(){
    setvbuf(stdout,NULL,_IONBF,0);
    ServiceInterface s;
    int r=s.robotServiceLogin("192.168.100.11",8899,"aubo","123456");
    printf("login=%d\n", r);
    if(r!=0) return 1;
    int rl=s.robotServiceLeaveTcp2CanbusMode();
    printf("leaveTcp2CanbusMode=%d  (0=left, 10023=was not in canbus mode)\n", rl);
    s.robotServiceLogout();
    printf("logout done\n");

    // confirm: a fresh session should now be able to log in without the
    // controller complaining about another controller holding the arm
    ServiceInterface s2;
    int r2=s2.robotServiceLogin("192.168.100.11",8899,"aubo","123456");
    bool conn=false; s2.robotServiceGetConnectStatus(conn);
    printf("re-login=%d connect=%d\n", r2, (int)conn);
    aubo_robot_namespace::RobotState st;
    int rs=s2.robotServiceGetRobotCurrentState(st);
    printf("getCurrentState=%d state=%d (0=stopped)\n", rs, (int)st);
    s2.robotServiceLogout();
    printf("controller released\n");
    return 0;
}
