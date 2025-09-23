#include <Arduino.h>

#include "PID.hpp"
#include "DCMotor.hpp"
#include "SensorAS5600.hpp"
#include "SerialControl.hpp"
#include "Config.hpp"

DCMotor motor(MOTOR_PWM_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN, PWM_CHANNEL);
PID pid(2.5, 0.0, 0.0); // Kp 2.5
SensorAS5600 sensor{I2C_SDA0_PIN, I2C_SCL0_PIN};
SerialControl serial_control;

unsigned long startup_time;
void setup() {
    startup_time = millis();

    sensor.begin();
    motor.begin();
}

void loop() {
    serial_control.update();
    pid.setTarget(serial_control.inputData.targetPosition0);
    int32_t currentAngle_steps = sensor.getCumulativePosition();
    serial_control.outputData.currentPosition0 = currentAngle_steps;
    double currentAngle = static_cast<double>(currentAngle_steps) * SensorAS5600::RAW_TO_DEG;

    double motorPower = pid.calculate(currentAngle);
    motor.move(static_cast<int>(motorPower));

    /*Serial.print("Target: ");
    Serial.print(pid.getTarget());
    Serial.print(" | Current: ");
    Serial.print(currentAngle, 2);
    Serial.print(" | Error: ");
    Serial.print(pid.getTarget() - currentAngle, 2);
    Serial.print(" | Motor signal: ");
    Serial.println(motorPower / 255.0);*/
}
