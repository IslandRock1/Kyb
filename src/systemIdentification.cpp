#include <Wire.h>
#include <AS5600.h>
#include <Arduino.h>
#include <iostream>
#include <string>

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

// if python is ready, send "Ready"
bool checkReadyFromSerial() {
    if (Serial.available()) {
        String msg = Serial.readStringUntil('\n');
        msg.trim();
        if (msg == "Ready") {
            return true;
        }
    }
    return false;
}

bool ready_received = false;

// if readMotorData != "Ready" return 0 (hold motor), else 255(start motor);
int get_motor_power() {
    // check if Python sent "Ready"
    if (!ready_recieved) {
        if (checkReadyFromSerial()) {
            ready_recieved = true;
            null_time_signal = millis(); // reset start time
        } else {
            return 0; // not ready, hold motor
        }
    }

    auto dt = millis() - null_time_signal;
    int base_time = 10000;
    int offset_time = 1000;

    double partitions = 10.0;
    for (int i = 0; i < (partitions + 1); i++) {
        if (dt < (base_time + offset_time * i)) {
            return static_cast<int>(i * 255.0 / partitions);
        }
    }
    return 255; // motor ready, start
}

void setupSensor() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

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

void setup() {
    Serial.begin(1152000);
    Serial.println("Objektorientert testrigg starter...");

    // setupSensor();
    motor.begin();
    controller.setTarget(0.0);


}

auto prev_print_time = micros();
void loop() {
    int currentAngle_steps = 0;

    auto motorPower = get_motor_power();
    motor.move(motorPower);

    if (micros() - prev_print_time > 10000) {
        prev_print_time = micros();

        // Utskrift for komunikasjon med pc
        Serial.print(currentAngle_steps);
        Serial.print(",");
        Serial.print(motorPower / 255.0, 6);
        Serial.print(",");
        Serial.println(micros());
    }
}

