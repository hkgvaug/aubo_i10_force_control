// Bare AUBO SDK benchmark - no ROS, no aubo_driver.
// Measures per-call cost of robotServiceSetRobotPosData2Canbus() alone and
// while the other SDK queries the driver issues are running concurrently.
#include "aubo_driver/serviceinterface.h"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <atomic>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std::chrono;
static const char* HOST = "192.168.100.11";

static double ms(steady_clock::time_point a, steady_clock::time_point b) {
    return duration_cast<duration<double, std::milli>>(b - a).count();
}

static void stats(const char* name, std::vector<double>& v) {
    if (v.empty()) return;
    std::sort(v.begin(), v.end());
    double sum = 0; for (double x : v) sum += x;
    printf("%-34s n=%3zu  min=%7.2f  p50=%7.2f  p90=%7.2f  p99=%7.2f  max=%7.2f  mean=%7.2f ms\n",
           name, v.size(), v.front(), v[v.size()/2], v[v.size()*9/10], v[v.size()*99/100], v.back(), sum/v.size());
    v.clear();
}

int main() {
    setvbuf(stdout, NULL, _IONBF, 0);
    ServiceInterface send, recv;

    auto t0 = steady_clock::now();
    int r1 = send.robotServiceLogin(HOST, 8899, "aubo", "123456");
    int r2 = recv.robotServiceLogin(HOST, 8899, "aubo", "123456");
    printf("login send=%d recv=%d  (%.1f ms)\n", r1, r2, ms(t0, steady_clock::now()));
    if (r1 != 0) { printf("login failed, abort\n"); return 1; }

    t0 = steady_clock::now();
    int rc = send.robotServiceEnterTcp2CanbusMode();
    printf("enterTcp2CanbusMode = %d  (%.1f ms)\n", rc, ms(t0, steady_clock::now()));
    if (rc == 10023) {   // ErrCode_ResponseReturnError: already in canbus mode
        printf("already in canbus mode, leave+re-enter\n");
        send.robotServiceLeaveTcp2CanbusMode();
        rc = send.robotServiceEnterTcp2CanbusMode();
        printf("re-enter = %d\n", rc);
    }
    if (rc != 0) { printf("cannot enter canbus mode, abort\n"); return 1; }

    aubo_robot_namespace::wayPoint_S wp; ServiceInterface::initWayPointDataType(wp);
    t0 = steady_clock::now();
    recv.robotServiceGetCurrentWaypointInfo(wp);
    printf("getCurrentWaypoint (%.1f ms) q_deg =", ms(t0, steady_clock::now()));
    for (int i = 0; i < 6; ++i) printf(" %.2f", wp.jointpos[i] * 180.0 / M_PI);
    printf("\n\n");

    double q[8] = {0};
    for (int i = 0; i < 6; ++i) q[i] = wp.jointpos[i];   // hold position: no motion

    std::atomic<bool> stop(false);
    std::vector<double> push_ms, diag_ms, io_ms;

    // ---------- Phase A: push only ----------
    for (int i = 0; i < 300; ++i) {
        auto a = steady_clock::now();
        send.robotServiceSetRobotPosData2Canbus(q);
        push_ms.push_back(ms(a, steady_clock::now()));
        std::this_thread::sleep_for(microseconds(2000));
    }
    stats("[A] push alone", push_ms);

    // ---------- Phase B: push + diagnostics poll (driver's timerCallback) ----------
    stop = false;
    std::thread diag_th([&]{
        aubo_robot_namespace::RobotDiagnosis d;
        while (!stop) {
            auto a = steady_clock::now();
            recv.robotServiceGetRobotDiagnosisInfo(d);
            diag_ms.push_back(ms(a, steady_clock::now()));
            std::this_thread::sleep_for(milliseconds(200));   // 5 Hz, driver's current default
        }
    });
    for (int i = 0; i < 300; ++i) {
        auto a = steady_clock::now();
        send.robotServiceSetRobotPosData2Canbus(q);
        push_ms.push_back(ms(a, steady_clock::now()));
        std::this_thread::sleep_for(microseconds(2000));
    }
    stop = true; diag_th.join();
    stats("[B] push + diag@5Hz", push_ms);
    stats("    diag call", diag_ms);

    // ---------- Phase C: push + IO status poll (driver's publishIOMsg) ----------
    stop = false;
    std::thread io_th([&]{
        std::vector<aubo_robot_namespace::RobotIoType> in, out;
        in.push_back(aubo_robot_namespace::RobotBoardUserDI);
        out.push_back(aubo_robot_namespace::RobotBoardUserDO);
        while (!stop) {
            std::vector<aubo_robot_namespace::RobotIoDesc> sv;
            auto a = steady_clock::now();
            recv.robotServiceGetBoardIOStatus(in, sv);
            io_ms.push_back(ms(a, steady_clock::now()));
            sv.clear();
            auto b = steady_clock::now();
            recv.robotServiceGetBoardIOStatus(out, sv);
            io_ms.push_back(ms(b, steady_clock::now()));
            std::this_thread::sleep_for(milliseconds(500));   // 2 Hz, driver's io_publish_rate
        }
    });
    for (int i = 0; i < 300; ++i) {
        auto a = steady_clock::now();
        send.robotServiceSetRobotPosData2Canbus(q);
        push_ms.push_back(ms(a, steady_clock::now()));
        std::this_thread::sleep_for(microseconds(2000));
    }
    stop = true; io_th.join();
    stats("[C] push + IO@2Hz", push_ms);
    stats("    boardIO call", io_ms);

    // ---------- Phase D: push with diag at driver's OLD 50 Hz rate ----------
    stop = false;
    std::thread diag50([&]{
        aubo_robot_namespace::RobotDiagnosis d;
        while (!stop) {
            auto a = steady_clock::now();
            recv.robotServiceGetRobotDiagnosisInfo(d);
            diag_ms.push_back(ms(a, steady_clock::now()));
            std::this_thread::sleep_for(milliseconds(20));    // 50 Hz
        }
    });
    for (int i = 0; i < 300; ++i) {
        auto a = steady_clock::now();
        send.robotServiceSetRobotPosData2Canbus(q);
        push_ms.push_back(ms(a, steady_clock::now()));
        std::this_thread::sleep_for(microseconds(2000));
    }
    stop = true; diag50.join();
    stats("[D] push + diag@50Hz", push_ms);
    stats("    diag call", diag_ms);

    send.robotServiceLeaveTcp2CanbusMode();
    send.robotServiceLogout();
    recv.robotServiceLogout();
    printf("\ndone (left canbus mode, logged out)\n");
    return 0;
}
