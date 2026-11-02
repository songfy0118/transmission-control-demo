#include "pid.h"
#include <math.h>
#include <stddef.h>

static double clamp(double value, double low, double high) {
    return fmax(low, fmin(high, value));
}

int pid_init(SpeedPid *pid, double kp, double ki, double kd, double tau) {
    if (!pid || !isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        !isfinite(tau) || kp < 0 || ki < 0 || kd < 0 || tau < 0) return 0;
    *pid = (SpeedPid){kp, ki, kd, tau, 0, 0, 0, 0};
    return 1;
}

int pid_step(SpeedPid *pid, double target, double measured, double dt, double *output) {
    if (!output) return 0;
    *output = 0;
    if (!pid || !isfinite(target) || !isfinite(measured) ||
        !isfinite(dt) || dt <= 0) return 0;
    const double error = target - measured;
    /* Derivative on measurement avoids a derivative kick at a target-speed step. */
    const double raw_d = pid->initialized ? -(measured - pid->previous_measurement) / dt : 0;
    const double alpha = dt / (pid->derivative_tau + dt);
    const double filtered_d = pid->derivative + alpha * (raw_d - pid->derivative);
    const double candidate_i = pid->integral + pid->ki * error * dt;
    const double trial = pid->kp * error + candidate_i + pid->kd * filtered_d;
    if (!isfinite(trial) || !isfinite(filtered_d)) return 0;
    /* Integrate only when unsaturated, or when error drives the output out of saturation. */
    if ((trial >= -1 && trial <= 1) || (trial > 1 && error < 0) || (trial < -1 && error > 0)) {
        pid->integral = candidate_i;
    }
    pid->derivative = filtered_d;
    pid->previous_measurement = measured;
    pid->initialized = 1;
    *output = clamp(pid->kp * error + pid->integral + pid->kd * filtered_d, -1, 1);
    return 1;
}
