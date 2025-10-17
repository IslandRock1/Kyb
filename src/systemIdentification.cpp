#include <Arduino.h>

#include "DCMotor.hpp"
#include "SensorAS5600.hpp"
#include "Config.hpp"

DCMotor motor{MOTOR0_PWM_PIN, MOTOR0_IN1_PIN, MOTOR0_IN2_PIN, PWM_CHANNEL0};
SensorAS5600 sensor{I2C_SDA0_PIN, I2C_SCL0_PIN, 0};

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
    int base_time = 1000;
    int offset_time = 1000;
    int ending_time = 3000;

    double partitions = 10.0;
    for (int i = 0; i < (partitions + 1); i++) {
        if (dt < (base_time + offset_time * i)) {
            return static_cast<int>(i * 255.0 / partitions);
        }
    }

    if (millis() - null_time_signal < (base_time + offset_time * partitions + ending_time)) {
        return 255;
    }

    ready_received = false;
    return 0; // motor ready, start
}

void setup() {
    Serial.begin(1152000);
    Serial.println("Starting system identification logging.");

    sensor.begin();
    motor.begin();

    sensor.resetCumulativePosition();
}

auto prev_print_time = micros();
void loop() {
    int32_t currentAngle_steps = sensor.getCumulativePosition();
    auto motorPower = get_motor_power();
    motor.move(motorPower);


    // Utskrift for komunikasjon med pc
    Serial.print(currentAngle_steps);
    Serial.print(",");
    Serial.print(motorPower / 255.0, 6);
    Serial.print(",");
    Serial.println(micros());
}

