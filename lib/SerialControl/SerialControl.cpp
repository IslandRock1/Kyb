
#include <Arduino.h>
#include <vector>
#include "SerialControl.hpp"

SerialControl::SerialControl(): inputData(), outputData() {
    Serial.begin(115200);
}

void SerialControl::update() {
    readData();
    sendData();
}

void SerialControl::parseDATA(const String &input) {
    std::vector<String> tokens;
    int start = 0;
    int end = input.indexOf(',');

    while (end != -1) {
        tokens.push_back(input.substring(start, end));
        start = end + 1;
        end = input.indexOf(',', start);
    }

    // last token
    if (start < input.length()) {
        tokens.push_back(input.substring(start));
    }

    if (tokens.size() == 6) {
        inputData.position0 = tokens[0].toDouble();
        inputData.position1 = tokens[1].toDouble();

        inputData.power0 = tokens[2].toInt();
        inputData.power1 = tokens[3].toInt();

        inputData.positionMode0 = tokens[4].toInt() != 0;
        inputData.positionMode1 = tokens[5].toInt() != 0;
    } else {
        Serial.println("ERROR");
    }
}


void SerialControl::readData() {
    if (Serial.available()) {
        String msg = Serial.readStringUntil('\n');
        parseDATA(msg);
    }
}

void SerialControl::sendData() {
    String out;
    out += outputData.position0;
    out += ",";
    out += outputData.position1;
    out += ",";
    out += outputData.power0;
    out += ",";
    out += outputData.power1;
    out += ",";
    out += outputData.positionMode0;
    out += ",";
    out += outputData.positionMode1;
    Serial.println(out);
}
