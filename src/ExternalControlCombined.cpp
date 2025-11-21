#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <vector>

/*************************************************************
 * CONFIG
 *************************************************************/
constexpr int MOTOR0_PWM_PIN = 25;
constexpr int MOTOR0_IN1_PIN = 26;
constexpr int MOTOR0_IN2_PIN = 14;

constexpr int MOTOR1_PWM_PIN = 33;
constexpr int MOTOR1_IN1_PIN = 32;
constexpr int MOTOR1_IN2_PIN = 13;

constexpr int I2C_SDA0_PIN = 5;
constexpr int I2C_SCL0_PIN = 17;
constexpr int I2C_BUSNUM0 = 0;

constexpr int I2C_SDA1_PIN = 19;
constexpr int I2C_SCL1_PIN = 18;
constexpr int I2C_BUSNUM1 = 1;

constexpr int PWM_CHANNEL0 = 0;
constexpr int PWM_CHANNEL1 = 1;


/*************************************************************
 * DCMotor CLASS
 *************************************************************/
class DCMotor {
public:
    DCMotor(int PWMPin, int in1Pin, int in2Pin, int channel)
        : _pwmPin(PWMPin), _in1Pin(in1Pin), _in2Pin(in2Pin), _channel(channel) {}

    void begin() {
        pinMode(_in1Pin, OUTPUT);
        pinMode(_in2Pin, OUTPUT);
        ledcSetup(_channel, 5000, 8);
        ledcAttachPin(_pwmPin, _channel);
    }

    void move(int power) {
        if (power > 0) {
            digitalWrite(_in1Pin, HIGH);
            digitalWrite(_in2Pin, LOW);
        } else {
            digitalWrite(_in1Pin, LOW);
            digitalWrite(_in2Pin, HIGH);
        }
        ledcWrite(_channel, abs(power));
    }

private:
    int _pwmPin;
    int _in1Pin;
    int _in2Pin;
    int _channel;
};


/*************************************************************
 * PID CLASS
 *************************************************************/
class PID {
public:
    PID(double kp, double ki, double kd)
        : _kp(kp), _ki(ki), _kd(kd) {}

    void incrementTarget(double angle) { _targetAngle += angle; }
    void setTarget(double angle) { _targetAngle = angle; }
    double getTarget() { return _targetAngle; }

    double calculate(double currentAngle) {
        double error = _targetAngle - currentAngle;
        _integral += error;

        unsigned long now = micros();
        double dt = (now - _prevTime);
        _prevTime = now;

        double derivative = dt > 0 ? (error - _prevError) / dt : 0;
        _prevError = error;

        if (fabs(error) < 1.0) return 0;

        double power = _kp * error + _ki * _integral + _kd * derivative;
        power = constrain(power, -255, 255);
        return power;
    }

private:
    double _kp, _ki, _kd;
    double _integral = 0.0;
    double _prevError = 0.0;
    double _targetAngle = 0.0;
    unsigned long _prevTime = 0;
};


/*************************************************************
 * AS5600 SENSOR CLASS
 *************************************************************/
class SensorAS5600 {
public:
    SensorAS5600(int SDA, int SCL, int bus_num)
        : _sda(SDA), _scl(SCL), _wire(bus_num) {}

    void begin() {
        _wire.begin(_sda, _scl);
        _sensor = AS5600(&_wire);

        unsigned long t0 = millis();
        while (!_sensor.isConnected()) {
            if (millis() - t0 > 1000) {
                Serial.println("Waiting for AS5600...");
                t0 = millis();
            }
        }
        Serial.println("AS5600 OK");
    }

    int32_t getCumulativePosition() {
        return _sensor.getCumulativePosition();
    }

    void resetCumulativePosition() {
        _sensor.resetCumulativePosition();
    }

    static constexpr double RAW_TO_DEG = 360.0 / 4096.0;

private:
    AS5600 _sensor;
    int _sda, _scl;
    TwoWire _wire;
};


/*************************************************************
 * MotorLinkage CLASS
 *************************************************************/
struct LinkageConfigPins {
    int MOTOR_PWM_PIN;
    int MOTOR_IN0_PIN;
    int MOTOR_IN1_PIN;
    int MOTOR_PWM_CHANNEL;
    double gearing;

    double kp, ki, kd;

    int I2C_SDA_PIN;
    int I2C_SCL_PIN;
    int I2C_BUS_NUM;
};

class MotorLinkage {
public:
    MotorLinkage(const LinkageConfigPins &pins)
        : _motor(pins.MOTOR_PWM_PIN, pins.MOTOR_IN0_PIN, pins.MOTOR_IN1_PIN, pins.MOTOR_PWM_CHANNEL),
          _pid(pins.kp, pins.ki, pins.kd),
          _sensor(pins.I2C_SDA_PIN, pins.I2C_SCL_PIN, pins.I2C_BUS_NUM),
          _gearing(pins.gearing) {}

    void begin() {
        _sensor.begin();
        _sensor.resetCumulativePosition();
        _motor.begin();
    }

    void updatePosition(double degrees) {
        _pid.setTarget(degrees * _gearing);
        _currentAngle = _sensor.getCumulativePosition() * SensorAS5600::RAW_TO_DEG;
        _power = _pid.calculate(_currentAngle);
        _motor.move(_power);
    }

    void updatePower(int power) {
        _currentAngle = _sensor.getCumulativePosition() * SensorAS5600::RAW_TO_DEG;
        _power = power;
        _motor.move(power);
    }

    void setZeroPosition() {
        _sensor.resetCumulativePosition();
    }

    double getDegrees() const { return _currentAngle / _gearing; }
    int getPower() const { return _power; }

private:
    DCMotor _motor;
    PID _pid;
    SensorAS5600 _sensor;

    double _currentAngle = 0.0;
    double _gearing = 0.0;
    int _power = 0;
};


/*************************************************************
 * SerialControl CLASS
 *************************************************************/
struct SerialData {
    double position0;
    double position1;

    int power0;
    int power1;

    bool positionMode0;
    bool positionMode1;
    bool doResetPosition;
};

class SerialControl {
public:
    SerialControl() { Serial.begin(115200); }

    void update() {
        readData();
        sendData();
    }

    SerialData inputData;
    SerialData outputData;

private:
    void parseDATA(const String &input) {
        std::vector<String> tokens;
        int start = 0;
        int end = input.indexOf(',');

        while (end != -1) {
            tokens.push_back(input.substring(start, end));
            start = end + 1;
            end = input.indexOf(',', start);
        }
        if (start < input.length()) tokens.push_back(input.substring(start));

        if (tokens.size() == 7) {
            inputData.position0 = tokens[0].toDouble();
            inputData.position1 = tokens[1].toDouble();
            inputData.power0 = tokens[2].toInt();
            inputData.power1 = tokens[3].toInt();
            inputData.positionMode0 = tokens[4].toInt();
            inputData.positionMode1 = tokens[5].toInt();
            inputData.doResetPosition = tokens[6].toInt();
        }
    }

    void readData() {
        if (Serial.available()) {
            String msg = Serial.readStringUntil('\n');
            parseDATA(msg);
        }
    }

    void sendData() {
        Serial.print(outputData.position0); Serial.print(",");
        Serial.print(outputData.position1); Serial.print(",");
        Serial.print(outputData.power0); Serial.print(",");
        Serial.print(outputData.power1); Serial.print(",");
        Serial.print(outputData.positionMode0); Serial.print(",");
        Serial.println(outputData.positionMode1);
    }
};


/*************************************************************
 * MAIN APPLICATION (Your ExternalControl.cpp)
 *************************************************************/
SerialControl serial_control;

LinkageConfigPins pinsWrist = {
    MOTOR0_PWM_PIN, MOTOR0_IN1_PIN, MOTOR0_IN2_PIN, PWM_CHANNEL0,
    4.4, 2.5, 0, 0.01, I2C_SDA0_PIN, I2C_SCL0_PIN, I2C_BUSNUM0
};
MotorLinkage motorLinkageWrist(pinsWrist);

LinkageConfigPins pinsShoulder = {
    MOTOR1_PWM_PIN, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, PWM_CHANNEL1,
    -1.0, 2.5, 0, 0.01, I2C_SDA1_PIN, I2C_SCL1_PIN, I2C_BUSNUM1
};
MotorLinkage motorLinkageShoulder(pinsShoulder);


void setup() {
    motorLinkageWrist.begin();
    motorLinkageShoulder.begin();
}

void loop() {
    serial_control.update();

    if (serial_control.inputData.positionMode0)
        motorLinkageWrist.updatePosition(serial_control.inputData.position0);
    else
        motorLinkageWrist.updatePower(serial_control.inputData.power0);

    if (serial_control.inputData.positionMode1)
        motorLinkageShoulder.updatePosition(serial_control.inputData.position1);
    else
        motorLinkageShoulder.updatePower(serial_control.inputData.power1);

    if (serial_control.inputData.doResetPosition) {
        motorLinkageShoulder.setZeroPosition();
        motorLinkageWrist.setZeroPosition();
    }

    serial_control.outputData.position0 = motorLinkageWrist.getDegrees();
    serial_control.outputData.position1 = motorLinkageShoulder.getDegrees();
    serial_control.outputData.power0 = motorLinkageWrist.getPower();
    serial_control.outputData.power1 = motorLinkageShoulder.getPower();
}
