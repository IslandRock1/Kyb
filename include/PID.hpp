
#ifndef TESTRIG_PID_HPP
#define TESTRIG_PID_HPP


class PID {
public:
    PID(double kp, double ki, double kd);
    int calculate(double currentAngle);

    void incrementTarget(double angle);
    void setTarget(double angle);
    double getTarget();

private:
    double _kp;
    double _ki;
    double _kd;

    double _alpha = 0.001;
    double _prevPower = 0.0;
    double _integral = 0.0;
    double _prevError = 0.0;
    double _targetAngle = 0.0;
    unsigned long _prevTime = 0.0;
};


#endif //TESTRIG_PID_HPP