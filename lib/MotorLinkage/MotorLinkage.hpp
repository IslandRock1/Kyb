
#ifndef MOTORLINKAGE_HPP
#define MOTORLINKAGE_HPP

#include "DCMotor.hpp"
#include "PID.hpp"
#include "SensorAS5600.hpp"

struct LinkageConfigPins {
    int MOTOR_PWM_PIN;
    int MOTOR_IN0_PIN;
    int MOTOR_IN1_PIN;
    int MOTOR_PWM_CHANNEL;
    double gearing;

    double kp;
    double ki;
    double kd;

    int I2C_SDA_PIN;
    int I2C_SCL_PIN;
    int I2C_BUS_NUM;
};

class MotorLinkage {
public:
    MotorLinkage(const LinkageConfigPins &pins);
    void begin();
    void updatePosition(double degrees);
    void updatePower(int power);

    int getPower() const;
    double getDegrees() const;

private:
    DCMotor _motor;
    PID _pid;
    SensorAS5600 _sensor;

    double _currentAngle = 0.0;
    double _gearing = 0.0;
    int _power = 0;
};



#endif //MOTORLINKAGE_HPP
