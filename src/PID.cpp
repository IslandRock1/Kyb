
#include <cmath>
#include <Arduino.h>

#include "PID.hpp"

#include "AS5600.h"

PID::PID(double kp, double ki, double kd)
    : _kp(kp), _ki(ki), _kd(kd) {}

void PID::incrementTarget(double angle) {
    _targetAngle += angle;
}


void PID::setTarget(double angle) {
    _targetAngle = angle;
}

double PID::getTarget() {
    return _targetAngle;
}

double PID::calculate(double currentAngle) {
    // Regner ut feilen
    double error = _targetAngle - currentAngle;
    _integral += error;

    auto deltaTime = micros() - _prevTime;
    double dt;
    _prevTime = micros();
    if (deltaTime == 0) {
        dt = 0;
    } else {
        dt = (error - _prevError) / static_cast<double>(deltaTime);
    }
    _prevError = error;

    double power;
    // Dødsone for å unngå vibrering
    if (std::abs(error) < 1.0) {
        power = 0.0;
    } else {
        power = _kp * error + _ki * _integral + _kd * dt;
    }

    // Begrenser pådraget
    if (power > 255) power = 255;
    if (power < -255) power = -255;

    power = (_prevPower * (1 - _alpha)) + (power * _alpha);

    /*Serial.print("Prev: ");
    Serial.print(_prevPower);
    Serial.print(" | New: ");
    Serial.print(power);
    Serial.print(" | Floated: ");
    Serial.println(static_cast<int>(power));*/

    _prevPower = power;

    // pådraget som heltall
    return power;
}