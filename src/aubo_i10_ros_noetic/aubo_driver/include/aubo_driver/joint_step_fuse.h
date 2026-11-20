#ifndef AUBO_DRIVER_JOINT_STEP_FUSE_H_
#define AUBO_DRIVER_JOINT_STEP_FUSE_H_

#include <cmath>

namespace aubo_driver {

/** Hard software fuse applied before every canbus position push.
 *
 *  The controller replays queued waypoints at a fixed ~3.6 ms per point, so a
 *  large difference between two consecutive points is turned into a step
 *  impulse (servo slam + harmonic-drive backlash knock).  This check rejects
 *  such a point instead of letting it reach the controller.
 *
 *  @param reference  previously accepted (or measured) joint positions
 *  @param candidate  point about to be sent
 *  @param n          number of joints to compare
 *  @param limit      max allowed |delta| per joint, radians; <= 0 disables
 *  @param bad_joint  out: index of the worst offending joint (or -1)
 *  @param bad_delta  out: its absolute delta
 *  @return true when the point is safe to send
 */
inline bool jointStepWithin(const double *reference, const double *candidate,
                            int n, double limit,
                            int &bad_joint, double &bad_delta)
{
    if(limit <= 0.0 || reference == NULL || candidate == NULL)
        return true;

    /** Tolerance so a delta that equals the limit is not rejected by float
     *  rounding (e.g. (0.3 + 0.002) - 0.3 evaluates slightly above 0.002).
     *  1e-9 rad is ~6e-8 deg, far below anything mechanically meaningful. **/
    const double tol = 1e-9;
    const double threshold = limit + tol;

    for(int i = 0; i < n; i++)
    {
        double delta = std::fabs(candidate[i] - reference[i]);
        if(delta > threshold)
        {
            bad_joint = i;
            bad_delta = delta;
            return false;
        }
    }
    return true;
}

} // namespace aubo_driver

#endif
