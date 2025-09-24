#include <Arduino.h>

#include "PID.hpp"
#include "DCMotor.hpp"
#include "SensorAS5600.hpp"
#include "SerialControl.hpp"
#include "Config.hpp"

DCMotor motor0{MOTOR0_PWM_PIN, MOTOR0_IN1_PIN, MOTOR0_IN2_PIN, PWM_CHANNEL0};
DCMotor motor1{MOTOR1_PWM_PIN, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, PWM_CHANNEL1};
PID pid0(2.5, 0.0, 0.0); // Kp 2.5
PID pid1(2.5, 0.0, 0.0);
SensorAS5600 sensor0{I2C_SDA0_PIN, I2C_SCL0_PIN, 0};
SensorAS5600 sensor1{I2C_SDA1_PIN, I2C_SCL1_PIN, 1};
SerialControl serial_control;

unsigned long startup_time;
void setup() {
    startup_time = millis();

    sensor0.begin();
    sensor1.begin();

    sensor0.resetCumulativePosition();
    sensor1.resetCumulativePosition();

    motor0.begin();
    motor1.begin();
}

void loop() {
    serial_control.update();
    pid0.setTarget(serial_control.inputData.targetPosition0);
    pid1.setTarget(serial_control.inputData.targetPosition1);

    int32_t currentAngle_steps0 = sensor0.getCumulativePosition();
    int32_t currentAngle_steps1 = sensor1.getCumulativePosition();

    serial_control.outputData.currentPosition0 = currentAngle_steps0;
    serial_control.outputData.currentPosition1 = currentAngle_steps1;

    double currentAngle0 = static_cast<double>(currentAngle_steps0) * SensorAS5600::RAW_TO_DEG;
    double currentAngle1 = static_cast<double>(currentAngle_steps1) * SensorAS5600::RAW_TO_DEG;

    double motorPower0 = pid0.calculate(currentAngle0);
    double motorPower1 = pid1.calculate(currentAngle1);
    motor0.move(static_cast<int>(motorPower0));
    motor1.move(static_cast<int>(motorPower1));

    /*Serial.print("Target: ");
    Serial.print(pid.getTarget());
    Serial.print(" | Current: ");
    Serial.print(currentAngle, 2);
    Serial.print(" | Error: ");
    Serial.print(pid.getTarget() - currentAngle, 2);
    Serial.print(" | Motor signal: ");
    Serial.println(motorPower / 255.0);*/
}
