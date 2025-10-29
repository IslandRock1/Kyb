
#include "MotorLinkage.hpp"

MotorLinkage::MotorLinkage(const LinkageConfigPins &pins)
    : _motor(
        pins.MOTOR_PWM_PIN,
        pins.MOTOR_IN0_PIN,
        pins.MOTOR_IN1_PIN,
        pins.MOTOR_PWM_CHANNEL
    ),
    _pid(
        pins.kp,
        pins.ki,
        pins.kd
    ),
    _sensor(
        pins.I2C_SDA_PIN,
        pins.I2C_SCL_PIN,
        pins.I2C_BUS_NUM
    ),
    _gearing(pins.gearing){}

void MotorLinkage::begin() {
    _sensor.begin();
    _sensor.resetCumulativePosition();

    _motor.begin();
}

void MotorLinkage::updatePosition(const double degrees) {
    _pid.setTarget(degrees * _gearing);

    _currentAngle = _sensor.getCumulativePosition() * SensorAS5600::RAW_TO_DEG;
    _power = _pid.calculate(_currentAngle);
    _motor.move(_power);
}

void MotorLinkage::updatePower(int power) {
    _currentAngle = _sensor.getCumulativePosition() * SensorAS5600::RAW_TO_DEG;
    _power = power;
    _motor.move(power);
}

void MotorLinkage::setZeroPosition() {
    _sensor.resetCumulativePosition();
}


double MotorLinkage::getDegrees() const {
    return _currentAngle / _gearing;
}

int MotorLinkage::getPower() const {
    return _power;
}

