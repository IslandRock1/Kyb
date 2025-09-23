#include <Arduino.h>

#include "PID.hpp"
#include "DCMotor.hpp"
#include "SensorAS5600.hpp"
#include "Config.hpp"

DCMotor motor(MOTOR_PWM_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN, PWM_CHANNEL);
PID controller(2.5, 0.0, 0.0); // Kp 2.5
SensorAS5600 sensor{I2C_SDA0_PIN, I2C_SCL0_PIN};

unsigned long startup_time;
void setup() {
    Serial.begin(115200);
    Serial.println("Starting robot arm.");
    startup_time = millis();

    sensor.begin();
    motor.begin();
    controller.setTarget(0.0);
}

unsigned long motorIncrementTime = 20000;
auto prev_print_time = millis();
void loop() {
    int32_t currentAngle_steps = sensor.getCumulativePosition();
    double currentAngle = static_cast<double>(currentAngle_steps) * SensorAS5600::RAW_TO_DEG;

    double motorPower = controller.calculate(currentAngle);
    motor.move(static_cast<int>(motorPower));

    if (millis() - prev_print_time > 1) {
        prev_print_time = millis();

        Serial.print("Target: ");
        Serial.print(controller.getTarget());
        Serial.print(" | Current: ");
        Serial.print(currentAngle, 2);
        Serial.print(" | Error: ");
        Serial.print(controller.getTarget() - currentAngle, 2);
        Serial.print(" | Time: ");
        Serial.print(motorIncrementTime - (millis() - startup_time));
        Serial.print(" | Motor signal: ");
        Serial.println(motorPower / 255.0);
    }

    if (millis() - startup_time > motorIncrementTime) {
        Serial.println("Setting new target!");
        controller.incrementTarget(360.0 * 4.5); // About one rotation after gearing

        startup_time = millis();
    }
}
