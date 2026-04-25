#ifndef PID_H
#define PID_H

class PID {
private:
    double kp, ki, kd;
    double prevError;
    double integral;

public:
    PID(double kp_, double ki_, double kd_)
        : kp(kp_), ki(ki_), kd(kd_), prevError(0.0), integral(0.0) {}

    double Compute(double target, double actual, double dt) {
        double error = target - actual;
        integral += error * dt;
        double p = kp * error;
        double i = ki * integral;
        double d = kd * (error - prevError) / dt;
        prevError = error;
        return p + i + d;
    }
};

#endif