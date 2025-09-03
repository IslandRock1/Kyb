#include <Wire.h>
#include <AS5600.h>
#include <Arduino.h>

//Objekt
AS5600 as5600;

//Pinner motordriveren
const int MOTOR_PWM_PIN = 25; //Pinne for motorhastighet
const int MOTOR_IN1_PIN = 16;
const int MOTOR_IN2_PIN = 17;

//Pinner sensorkommunikasjon (I2C)
const int I2C_SDA_PIN = 19;
const int I2C_SCL_PIN = 18;

//Inst for PWM
const int PWM_CHANNEL = 0; //første kanal (0-15)
const int PWM_RESOLUTION = 8; //8-bit hastighet (0-255)
const int PWM_FREQUENCY = 5000;

//variabler
float targetAngle = 0.0;
float Kp = 2.5;

void setup() {
    // put your setup code here, to run once:

    //starter kommunikasjon.
    Serial.begin(115200);
    Serial.println("Testrigg start :-)");

    //starter I2C-kom. sensor
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    //sjekker kontakt med sensor
    if (as5600.isConnected()) {
        Serial.println("Hall-effekt sensor funnet!");
    }
    else{
        Serial.println("Error: Finner ikke Hall-effekt sensor.");
        while(1); //Stopper programmet (så lenge 1 er 1, ikke kontakt med sensor)
    }

    //motorpinner utgnag
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN, OUTPUT);

    // Konfigurerer PWM-kanal for motorhastighet
    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PWM_PIN, PWM_CHANNEL);


}


int spd = 255;
void loop() {
    // put your main code here, to run repeatedly:

    //posisjon sensor

    float currentAngle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;

    //regner ut feilen
    float error = targetAngle - currentAngle;

    //motorpådrag
    float motorPower = Kp*error;

    // setter motor retning
    if(motorPower > 0){
        digitalWrite(MOTOR_IN1_PIN, HIGH);
        digitalWrite(MOTOR_IN2_PIN, LOW);
    }
    else {
        digitalWrite(MOTOR_IN1_PIN, LOW);
        digitalWrite(MOTOR_IN2_PIN, HIGH);
    }

    //setter hastigheten
    int motorSpeed = abs(motorPower); //absoluttverdi for positiv verdi
    if (motorSpeed > 250) {
        motorSpeed = 250;               //begrenser hastigheten til max verid
    }

    //dødsone for motvirke vibrering
    if (abs(error) < 1.0) {
        motorSpeed = 5; //stopper motoren for små avvik
        Serial.println("I dødsone.");
    }

    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
    ledcWrite(PWM_CHANNEL, spd);
    delay(5000);
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    ledcWrite(PWM_CHANNEL, spd);


    //utskrift
    Serial.print("Mål: ");
    Serial.print(targetAngle);
    Serial.print(" | Nåverende vinkel: ");
    Serial.print(currentAngle, 2);
    Serial.print (" | Motorpådrag: ");
    Serial.print(motorSpeed);
    Serial.print(" | Dir: ");
    Serial.println(motorPower > 0);

    delay(200); //ikke bra praksis ifølge Øystein, stopper alt men la det til
    targetAngle += 5;

    if (targetAngle > 360) {
        targetAngle = -360;
    }



}
