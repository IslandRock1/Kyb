#include <Wire.h>
#include <AS5600.h>
#include <Arduino.h>

class DCMotor {
public:
    DCMotor(int PWMPin, int in1Pin, int in2Pin, int channel);

    void begin();
    void move(int power);

private:
    int _pwmPin;
    int _in1Pin;
    int _in2Pin;
    int _channel;
};

DCMotor::DCMotor(int pwmPin, int in1Pin, int in2Pin, int channel) {
    _pwmPin = pwmPin;
    _in1Pin = in1Pin;
    _in2Pin = in2Pin;
    _channel = channel;
}

void DCMotor::begin() {
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);
    ledcSetup(_channel, 5000, 8);
    ledcAttachPin(_pwmPin, _channel);
}

void DCMotor::move(int power) {
    // retning basert på fortegnet
    if (power > 0) {
        digitalWrite(_in1Pin, HIGH);
        digitalWrite(_in2Pin, LOW);
    } else {
        digitalWrite(_in1Pin, LOW);
        digitalWrite(_in2Pin, HIGH);
    }

    // hastighet basert på absoluttverdien
    ledcWrite(_channel, abs(power));
}



class PositionController {
public:
    PositionController(float kp);

    void setTarget(float angle);

    int calculate(float currentAngle);
    float targetAngle;

private:
    float _kp;
};

PositionController::PositionController(float kp) {
    _kp = kp;
    targetAngle = 0.0; //målet
}

void PositionController::setTarget(float angle) {
    targetAngle = angle;
}

int PositionController::calculate(float currentAngle) {
    // Regner ut feilen
    float error = targetAngle - currentAngle;

    // Dødsone for å unngå vibrering
    if (abs(error) < 1.0) {
        return 0;
    }

    // pådraget
    float power = _kp * error;

    // Begrenser pådraget
    if (power > 255) power = 255;
    if (power < -255) power = -255;

    // pådraget som heltall
    return (int)power;
}

// Pinner
const int MOTOR_PWM_PIN = 25;
const int MOTOR_IN1_PIN = 16;
const int MOTOR_IN2_PIN = 17;
const int I2C_SDA_PIN = 19;
const int I2C_SCL_PIN = 18;
const int PWM_CHANNEL = 0;


DCMotor motor(MOTOR_PWM_PIN, MOTOR_IN1_PIN, MOTOR_IN2_PIN, PWM_CHANNEL);
PositionController controller(2.5); // Kp 2.5
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
    Serial.print(controller.targetAngle);
    Serial.print(" | Nåværende: ");
    Serial.print(currentAngle, 2);
    Serial.print(" | Pådrag: ");
    Serial.println(motorPower);

    delay(20); //ikke bra praksis ifølge Øystein, stopper alt

    if (millis() - t0 > 1000) {
        controller.setTarget(controller.targetAngle + 90.0);
        t0 = millis();
    }
}

