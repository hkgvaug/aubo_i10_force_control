// Unit test for the shipped joint step fuse (no ROS, no robot).
//   g++ -O2 -std=c++11 -I <aubo_driver>/include test_joint_step_fuse.cpp -o /tmp/t
#include "aubo_driver/joint_step_fuse.h"
#include <cstdio>
#include <cstring>
#include <cmath>

using aubo_driver::jointStepWithin;

static int failures = 0;
static void expect(const char *name, bool got, bool want)
{
    if(got == want) printf("  ok    %s\n", name);
    else { printf("  FAIL  %s (got %d, want %d)\n", name, (int)got, (int)want); failures++; }
}

int main()
{
    const int N = 6;
    const double LIM = 0.002;
    double ref[N] = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
    double cand[N];
    int bad = -1; double delta = 0.0;

    printf("joint step fuse test (limit %.4f rad)\n", LIM);

    memcpy(cand, ref, sizeof(ref));
    expect("identical point accepted", jointStepWithin(ref, cand, N, LIM, bad, delta), true);

    memcpy(cand, ref, sizeof(ref));
    cand[2] += LIM;
    expect("delta == limit accepted", jointStepWithin(ref, cand, N, LIM, bad, delta), true);

    memcpy(cand, ref, sizeof(ref));
    cand[2] += LIM * 1.001;
    expect("delta == limit*1.001 rejected", jointStepWithin(ref, cand, N, LIM, bad, delta), false);
    printf("        reported joint=%d delta=%.6f rad (%.3f deg)\n", bad, delta, delta * 180.0 / M_PI);

    memcpy(cand, ref, sizeof(ref));
    cand[1] += 30.0 * M_PI / 180.0;
    expect("30 deg single-point jump rejected", jointStepWithin(ref, cand, N, LIM, bad, delta), false);
    printf("        reported joint=%d delta=%.4f rad (%.2f deg)\n", bad, delta, delta * 180.0 / M_PI);

    memcpy(cand, ref, sizeof(ref));
    cand[6] = 99.0;
    expect("joints beyond n are ignored", jointStepWithin(ref, cand, N, LIM, bad, delta), true);

    memcpy(cand, ref, sizeof(ref));
    cand[1] += 1.0;
    expect("limit 0 disables the fuse", jointStepWithin(ref, cand, N, 0.0, bad, delta), true);

    expect("null reference accepted", jointStepWithin(NULL, cand, N, LIM, bad, delta), true);

    {
        double prev[N]; memcpy(prev, ref, sizeof(ref));
        double next[N];
        bool allOk = true;
        for(int k = 0; k < 48; k++)
        {
            memcpy(next, prev, sizeof(next));
            next[0] += 0.0019;
            if(!jointStepWithin(prev, next, N, LIM, bad, delta)) { allOk = false; break; }
            memcpy(prev, next, sizeof(next));
        }
        expect("48-point ramp (0.0019 rad/pt) accepted", allOk, true);
    }

    printf(failures == 0 ? "\nALL PASS\n" : "\n%d FAILURE(S)\n", failures);
    return failures == 0 ? 0 : 1;
}
