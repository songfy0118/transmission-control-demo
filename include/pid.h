#ifndef SPEED_PID_H
#define SPEED_PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double kp, ki, kd, derivative_tau;
    double integral, previous_measurement, derivative;
    int initialized;
} SpeedPid;

/* Signed output: positive throttle, negative brake; saturated to [-1, 1]. */
int pid_init(SpeedPid *pid, double kp, double ki, double kd, double tau);
int pid_step(SpeedPid *pid, double target, double measured, double dt, double *output);

#ifdef __cplusplus
}
#endif
#endif
