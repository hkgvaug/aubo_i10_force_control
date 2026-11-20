// Minimal AUBO-i10 SDK example: OFFLINE TRACK (robotServiceOfflineTrack*).
//
// The offline-track API hands the *controller* a whole point list and lets it
// plan/interpolate internally (unlike Tcp2Canbus passthrough, which needs a new
// point every 5 ms).  It therefore does NOT work while the controller is in
// Tcp2CanbusMode - the driver must have released it first.
//
// SAFETY DEFAULTS
//   * dry run: appends and clears the trajectory, never starts motion
//   * the generated path is CLOSED (starts and ends at the current position)
//   * every consecutive pair is checked against --max-step-rad before appending
//
// Build:  see CMakeLists.txt
// Usage:  ./offline_track [--host 192.168.100.11] [--deg 2] [--points 200]
//                         [--vel-deg 10] [--max-step-rad 0.002] [--execute]

#include "aubo_driver/serviceinterface.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

static double now_s()
{
    using namespace std::chrono;
    static const steady_clock::time_point t0 = steady_clock::now();
    return duration_cast<duration<double>>(steady_clock::now() - t0).count();
}

struct Options
{
    std::string host = "192.168.100.11";
    int         port = 8899;
    double      deg  = 2.0;        // half amplitude of the wrist3 triangle
    int         points = 200;
    double      vel_deg = 10.0;    // joint max velocity handed to the controller
    double      max_step_rad = 0.002;
    bool        execute = false;   // dry run unless --execute
};

static Options parse(int argc, char **argv)
{
    Options o;
    for(int i = 1; i < argc; i++)
    {
        std::string a = argv[i];
        auto next = [&](const char *name) -> const char * {
            if(i + 1 >= argc) { printf("%s needs a value\n", name); exit(2); }
            return argv[++i];
        };
        if(a == "--host")             o.host = next("--host");
        else if(a == "--port")        o.port = atoi(next("--port"));
        else if(a == "--deg")         o.deg = atof(next("--deg"));
        else if(a == "--points")      o.points = atoi(next("--points"));
        else if(a == "--vel-deg")     o.vel_deg = atof(next("--vel-deg"));
        else if(a == "--max-step-rad")o.max_step_rad = atof(next("--max-step-rad"));
        else if(a == "--execute")     o.execute = true;
        else if(a == "-h" || a == "--help") {
            printf("usage: %s [--host IP] [--port P] [--deg A] [--points N]\n"
                   "          [--vel-deg V] [--max-step-rad R] [--execute]\n", argv[0]);
            exit(0);
        }
        else { printf("unknown option %s\n", a.c_str()); exit(2); }
    }
    return o;
}

// The vendor SDK smashes its own stack during static destruction after logout
// (visible as "*** stack smashing detected ***").  Leaving through _Exit skips
// the destructors so the example exits cleanly.
static void finish(ServiceInterface &svc, int code)
{
    svc.robotServiceLogout();
    printf("done (exit %d)\n", code);
    std::_Exit(code);
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    Options opt = parse(argc, argv);

    ServiceInterface svc;                       // send + status share one session here
    int ret = svc.robotServiceLogin(opt.host.c_str(), opt.port, "aubo", "123456");
    printf("login(%s:%d) = %d\n", opt.host.c_str(), opt.port, ret);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        return 1;

    // Offline track needs the robot-controller (not the passthrough).  Leaving
    // canbus mode is idempotent enough: it returns 10023 if we were not in it.
    int leave = svc.robotServiceLeaveTcp2CanbusMode();
    printf("leaveTcp2CanbusMode = %d%s\n", leave,
           leave == aubo_robot_namespace::ErrCode_ResponseReturnError
               ? "  (was not in passthrough mode - good)" : "");

    bool       real = false;
    svc.robotServiceGetIsRealRobotExist(real);
    aubo_robot_namespace::RobotDiagnosis diag;
    svc.robotServiceGetRobotDiagnosisInfo(diag);
    printf("realRobot=%d power=%d softEmg=%d remoteEmg=%d jointErr=%d\n",
           (int)real, (int)diag.armPowerStatus, (int)diag.softEmergency,
           (int)diag.remoteEmergency, (int)diag.jointErrorStatus);
    if(!diag.armPowerStatus || diag.softEmergency || diag.remoteEmergency || diag.jointErrorStatus)
    {
        printf("ABORT: robot not ready for motion\n");
        finish(svc, 1);
    }

    aubo_robot_namespace::wayPoint_S start;
    ServiceInterface::initWayPointDataType(start);
    if(svc.robotServiceGetCurrentWaypointInfo(start) != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        printf("ABORT: cannot read current waypoint\n");
        finish(svc, 1);
    }
    printf("start joints(deg):");
    for(int i = 0; i < 6; i++) printf(" %8.3f", start.jointpos[i] * 180.0 / M_PI);
    printf("\n");

    // ---- build a CLOSED triangle on the last joint: 0 -> +A -> 0 ----
    const int    JOINT = 5;
    const int    N     = (opt.points < 20) ? 20 : opt.points;
    const double amp   = opt.deg * M_PI / 180.0;

    std::vector<aubo_robot_namespace::wayPoint_S> pts;
    pts.reserve(N);
    double prev[6];
    for(int i = 0; i < 6; i++) prev[i] = start.jointpos[i];

    double max_step = 0.0;
    for(int i = 0; i < N; i++)
    {
        double phase = (double)i / (N - 1);              // 0..1
        double tri   = 1.0 - std::fabs(2.0 * phase - 1.0); // 0 -> 1 -> 0
        aubo_robot_namespace::wayPoint_S p = start;
        p.jointpos[JOINT] = start.jointpos[JOINT] + amp * tri;

        double step = std::fabs(p.jointpos[JOINT] - prev[JOINT]);
        if(opt.max_step_rad > 0.0 && step > opt.max_step_rad + 1e-9)
        {
            printf("FUSE: point %d jumps %.4f rad > %.4f rad, aborting\n",
                   i, step, opt.max_step_rad);
            finish(svc, 1);
        }
        if(step > max_step) max_step = step;
        for(int j = 0; j < 6; j++) prev[j] = p.jointpos[j];
        pts.push_back(p);
    }
    printf("built %d points on joint %d, +-%.2f deg, max step %.5f rad (%.4f deg)\n",
           N, JOINT, opt.deg, max_step, max_step * 180.0 / M_PI);
    printf("path is CLOSED: last point == start point\n");

    // ---- global move profile: this is what limits speed for the offline track ----
    svc.robotServiceInitGlobalMoveProfile();
    aubo_robot_namespace::JointVelcAccParam maxVel, maxAcc;
    for(int i = 0; i < 6; i++)
    {
        maxVel.jointPara[i] = opt.vel_deg * M_PI / 180.0;
        maxAcc.jointPara[i] = opt.vel_deg * M_PI / 180.0;
    }
    printf("set joint max vel/acc = %.1f deg/s -> ret=%d,%d\n",
           opt.vel_deg,
           svc.robotServiceSetGlobalMoveJointMaxVelc(maxVel),
           svc.robotServiceSetGlobalMoveJointMaxAcc(maxAcc));

    // ---- append ----
    int c = svc.robotServiceOfflineTrackWaypointClear();
    int a = svc.robotServiceOfflineTrackWaypointAppend(pts);
    printf("offlineTrack: clear=%d append=%d\n", c, a);
    if(a != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        printf("ABORT: append rejected, nothing will move\n");
        svc.robotServiceOfflineTrackWaypointClear();
        finish(svc, 1);
    }

    if(!opt.execute)
    {
        svc.robotServiceOfflineTrackWaypointClear();
        printf("\nDRY RUN: trajectory appended then cleared, robot did NOT move.\n"
               "          re-run with --execute to actually play it.\n");
        finish(svc, 0);
    }

    // ---- execute (blocking) ----
    printf("\nstarting offline track (blocking)...\n");
    double t0 = now_s();
    int s = svc.robotServiceOfflineTrackMoveStartup(true);
    double dt = now_s() - t0;
    printf("offlineTrackMoveStartup(true) = %d, wall time %.3f s (%.1f ms/point)\n",
           s, dt, 1000.0 * dt / N);

    aubo_robot_namespace::wayPoint_S end;
    ServiceInterface::initWayPointDataType(end);
    svc.robotServiceGetCurrentWaypointInfo(end);
    printf("end joints(deg):  ");
    for(int i = 0; i < 6; i++) printf(" %8.3f", end.jointpos[i] * 180.0 / M_PI);
    printf("\ndelta(deg):       ");
    for(int i = 0; i < 6; i++)
        printf(" %8.3f", (end.jointpos[i] - start.jointpos[i]) * 180.0 / M_PI);
    printf("\n");

    svc.robotServiceOfflineTrackWaypointClear();
    finish(svc, 0);
    return 0;
}
