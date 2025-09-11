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
PID controller(2.5, 0.0, 0.0); // Kp 2.5
AS5600 sensor;

auto null_time_signal = millis();
int get_motor_power() {
    auto dt = millis() - null_time_signal;
    int base_time = 10000;
    int offset_time = 1000;

    if (dt < base_time) {
        return 0;
    }

    double partitions = 10.0;
    for (int i = 0; i < (partitions + 1); i++) {
        if (dt < (base_time + offset_time * i)) {
            return static_cast<int>(i * 255.0 / partitions);
        }
    }
    return 255;
}

void setup() {
    Serial.begin(1152000);
    Serial.println("Objektorientert testrigg starter...");

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    motor.begin();

    if (!sensor.isConnected()) {
        Serial.println("FEIL: Finner ikke sensor.");
        while(true);
    }
    Serial.println("Sensor funnet!");

    controller.setTarget(0.0);
}

auto prev_print_time = micros();
void loop() {
    // sensor
    int currentAngle_steps = sensor.getCumulativePosition();

    // pådrag med regulatoren
    double motorPower = controller.calculate(static_cast<double>(currentAngle_steps) * AS5600_RAW_TO_DEGREES);
    motorPower = get_motor_power();

    // kjører motor
    motor.move(static_cast<int>(motorPower));

    if (micros() - prev_print_time > 10000) {
        prev_print_time = micros();

        // Utskrift
        /*Serial.print("Mål: ");
        Serial.print(controller.getTarget());
        Serial.print(" | Nåværende: ");
        Serial.print(currentAngle, 2);
        Serial.print(" | Error: ");
        Serial.print(controller.getTarget() - currentAngle, 2);
        Serial.print(" | Pådrag: ");
        Serial.println(motorPower / 255.0);*/

        // Utskrift for komunikasjon med pc
        Serial.print(currentAngle_steps);
        Serial.print(",");
        Serial.print(motorPower / 255.0, 6);
        Serial.print(",");
        Serial.println(micros());
    }

    /*if (millis() - t0 > 1) {
        controller.incrementTarget(0.1);
        t0 = millis();
    }*/
}

