
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
    std::vector<int32_t> values;
    int start = 0;
    int end = input.indexOf(',');

    while (end != -1) {
        values.push_back(input.substring(start, end).toInt());
        start = end + 1;
        end = input.indexOf(',', start);
    }

    // last number
    if (start < input.length()) {
        values.push_back(input.substring(start).toInt());
    }

    if (!values.empty()) {
        inputData.targetPosition0 = values[0];
        inputData.targetPosition1 = values[1];
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
    out += outputData.currentPosition0;
    out += ",";
    out += outputData.currentPosition1;
    Serial.println(out);
}
