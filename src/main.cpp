#include <Wire.h>
#include <AS5600.h>
#include <Arduino.h>

#include "PID.hpp"
#include "DCMotor.hpp"

// Pinner
const int MOTOR_PWM_PIN = 25;
const int MOTOR_IN1_PIN = 26;
const int MOTOR_IN2_PIN = 14;
const int I2C_SDA0_PIN = 18;
const int I2C_SCL0_PIN = 19;
const int I2C_SDA1_PIN = 21;
const int I2C_SCL1_PIN = 17;
const int PWM_CHANNEL = 0;

DCMotor motor(MOTOR_PWM_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN, PWM_CHANNEL);
PID controller(2.5, 0.0, 0.0); // Kp 2.5
AS5600 sensor;

void setupSensor() {
    Wire.begin(I2C_SDA0_PIN, I2C_SCL0_PIN);

    auto t0 = millis();

    while (!sensor.isConnected()) {
        Serial.print("FEIL: Finner ikke sensor.");
        if (millis() - t0 > 1000) {
            t0 = millis();
            Serial.println(".");
        }

    }
    Serial.println();
    Serial.println("Sensor funnet!");
}

unsigned long startup_time;
void setup() {
    Serial.begin(115200);
    Serial.println("Objektorientert testrigg starter...");
    startup_time = millis();

    setupSensor();
    motor.begin();
    controller.setTarget(0.0);
}

auto prev_print_time = micros();
void loop() {
    // sensor
    int currentAngle_steps = sensor.getCumulativePosition();
    double currentAngle = static_cast<double>(currentAngle_steps) * AS5600_RAW_TO_DEGREES;

    // pådrag med regulatoren
    double motorPower = controller.calculate(currentAngle);

    // kjører motor
    motor.move(static_cast<int>(motorPower));
    unsigned long motorIncrementTime = 20000;

    if (micros() - prev_print_time > 10000) {
        prev_print_time = micros();

        // Utskrift
        Serial.print("Mål: ");
        Serial.print(controller.getTarget());
        Serial.print(" | Nåværende: ");
        Serial.print(currentAngle, 2);
        Serial.print(" | Error: ");
        Serial.print(controller.getTarget() - currentAngle, 2);
        Serial.print(" | Time: ");
        Serial.print(motorIncrementTime - (millis() - startup_time));
        Serial.print(" | Pådrag: ");
        Serial.println(motorPower / 255.0);
    }

    if (millis() - startup_time > motorIncrementTime) {
        Serial.println("Setting new target!");
        controller.incrementTarget(360.0 * 4.5);

        startup_time = millis();
    }
}
