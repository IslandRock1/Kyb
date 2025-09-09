#include <Wire.h>
#include <AS5600.h>
#include <Arduino.h>

#include "PID.hpp"
#include "DCMotor.hpp"

// Pinner
const int MOTOR_PWM_PIN = 25;
const int MOTOR_IN1_PIN = 16;
const int MOTOR_IN2_PIN = 17;
const int I2C_SDA_PIN = 19;
const int I2C_SCL_PIN = 18;
const int PWM_CHANNEL = 0;

DCMotor motor(MOTOR_PWM_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN, PWM_CHANNEL);
PID controller(2.5, 0.001, 0.0); // Kp 2.5
AS5600 sensor;

void setup() {
    Serial.begin(115200);
    Serial.println("Objektorientert testrigg starter...");

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    motor.begin();

    if (!sensor.isConnected()) {
        Serial.println("FEIL: Finner ikke sensor.");
        while(true);
    }
    Serial.println("Sensor funnet!");

    // startmål
    controller.setTarget(0.0);
}

auto t0 = millis();
auto prev_print_time = millis();
void loop() {
    // sensor
    double currentAngle = static_cast<double>(sensor.getCumulativePosition()) * AS5600_RAW_TO_DEGREES;

    // pådrag med regulatoren
    int motorPower = controller.calculate(currentAngle);

    // kjører motor
    motor.move(motorPower);

    if (millis() - prev_print_time > 100) {
        prev_print_time = millis();

        // Utskrift
        Serial.print("Mål: ");
        Serial.print(controller.getTarget());
        Serial.print(" | Nåværende: ");
        Serial.print(currentAngle, 2);
        Serial.print(" | Error: ");
        Serial.print(controller.getTarget() - currentAngle, 2);
        Serial.print(" | Pådrag: ");
        Serial.println(motorPower);
    }

    if (millis() - t0 > 1) {
        controller.incrementTarget(0.1);
        t0 = millis();
    }
}

