#include <Arduino.h>

#include "PID.hpp"
#include "DCMotor.hpp"
#include "SensorAS5600.hpp"

// Pinner
const int MOTOR_PWM_PIN = 25;
const int MOTOR_IN1_PIN = 26;
const int MOTOR_IN2_PIN = 14;
const int I2C_SDA0_PIN = 18;
const int I2C_SCL0_PIN = 19;
const int I2C_SDA1_PIN = 21;
const int I2C_SCL1_PIN = 17;
const int PWM_CHANNEL = 0;

DCMotor motor{MOTOR_PWM_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN, PWM_CHANNEL};
PID controller{2.5, 0.0, 0.0}; // Kp 2.5
SensorAS5600 sensor{I2C_SDA0_PIN, I2C_SCL0_PIN};

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
    if (!ready_received) {
        if (checkReadyFromSerial()) {
            ready_received = true;
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

void setup() {
    Serial.begin(1152000);
    Serial.println("Starting system identification logging.");

    sensor.begin();
    motor.begin();
    controller.setTarget(0.0);

    sensor.resetCumulativePosition();
}

auto prev_print_time = micros();
void loop() {
    int32_t currentAngle_steps = sensor.getCumulativePosition();
    auto motorPower = get_motor_power();
    motor.move(motorPower);

    if (micros() - prev_print_time > 100000) {
        prev_print_time = micros();

        // Utskrift for komunikasjon med pc
        Serial.print(currentAngle_steps);
        Serial.print(",");
        Serial.print(motorPower / 255.0, 6);
        Serial.print(",");
        Serial.println(micros());
    }
}

