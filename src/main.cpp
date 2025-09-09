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
PID controller(2.5, 0.01, 0.0); // Kp 2.5
AS5600 sensor;

void setup() {
    Serial.begin(115200);
    Serial.println("Objektorientert testrigg starter...");

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    //
    motor.begin();

    if (!sensor.isConnected()) {
        Serial.println("FEIL: Finner ikke sensor.");
        while(1);
    }
    Serial.println("Sensor funnet!");

    // startmål
    controller.setTarget(0.0);
}

auto t0 = micros();
void loop() {
    // sensor
    float currentAngle = sensor.getCumulativePosition() * AS5600_RAW_TO_DEGREES;

    // pådrag med regulatoren
    int motorPower = controller.calculate(currentAngle);

    // kjører motor
    motor.move(motorPower);

    // Utskrift
    Serial.print("Mål: ");
    Serial.print(controller.getTarget());
    Serial.print(" | Nåværende: ");
    Serial.print(currentAngle, 2);
    Serial.print(" | Pådrag: ");
    Serial.println(motorPower);

    delay(20); //ikke bra praksis ifølge Øystein, stopper alt

    if (millis() - t0 > 1000) {
        controller.incrementTarget(90.0);
        t0 = millis();
    }
}

