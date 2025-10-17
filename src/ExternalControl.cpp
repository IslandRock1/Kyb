#include <Arduino.h>

#include "SerialControl.hpp"
#include "Config.hpp"
#include "MotorLinkage.hpp"

SerialControl serial_control;

LinkageConfigPins pinsWrist = {
    MOTOR0_PWM_PIN,
    MOTOR0_IN1_PIN,
    MOTOR0_IN2_PIN,
    PWM_CHANNEL0,
    4.4,

    2.5, 0, 0.01,
    I2C_SDA0_PIN, I2C_SCL0_PIN, I2C_BUSNUM0
};
MotorLinkage motorLinkageWrist{pinsWrist};

LinkageConfigPins pinsShoulder = {
    MOTOR1_PWM_PIN,
    MOTOR1_IN1_PIN,
    MOTOR1_IN2_PIN,
    PWM_CHANNEL1,
    -1.0,

    2.5, 0, 0.01,
    I2C_SDA1_PIN, I2C_SCL1_PIN, I2C_BUSNUM1
};
MotorLinkage motorLinkageShoulder{pinsShoulder};

unsigned long startup_time;
void setup() {
    startup_time = millis();

    motorLinkageWrist.begin();
    motorLinkageShoulder.begin();
}

void loop() {
    serial_control.update();

    if (serial_control.inputData.positionMode0) {
        motorLinkageWrist.updatePosition(serial_control.inputData.position0);

    } else {
        motorLinkageWrist.updatePower(serial_control.inputData.power0);
    }

    if (serial_control.inputData.positionMode1) {
        motorLinkageShoulder.updatePosition(serial_control.inputData.position1);
    } else {
        motorLinkageShoulder.updatePower(serial_control.inputData.power1);
    }

    serial_control.outputData.position0 = motorLinkageWrist.getDegrees();
    serial_control.outputData.power0 = motorLinkageWrist.getPower();

    serial_control.outputData.position1 = motorLinkageShoulder.getDegrees();
    serial_control.outputData.power1 = motorLinkageShoulder.getPower();
}
