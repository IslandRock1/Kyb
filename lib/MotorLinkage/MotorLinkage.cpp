
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
        pins.I2C_SDA_PIN,
        pins.I2C_BUS_NUM
    ){}

void MotorLinkage::begin() {
    _sensor.begin();
    _sensor.resetCumulativePosition();

    _motor.begin();
}

void MotorLinkage::update(const double degrees) {
    _pid.setTarget(degrees);

    _currentAngle = _sensor.getCumulativePosition() * SensorAS5600::RAW_TO_DEG;
    const auto motorPower = _pid.calculate(_currentAngle);
    _motor.move(static_cast<int>(motorPower));
}

double MotorLinkage::getDegrees() const {
    return _currentAngle;
}
