
#ifndef TESTRIG_SERIALCONTROL_HPP
#define TESTRIG_SERIALCONTROL_HPP

#include <Arduino.h>

struct InputData {
    double targetPosition0;
    double targetPosition1;
};

struct OutputData {
    double currentPosition0;
    double currentPosition1;
};

class SerialControl {
public:
    SerialControl();

    void update();
    InputData inputData;
    OutputData outputData;

private:
    void parseDATA(const String &input);
    void readData();
    void sendData();
};


#endif //TESTRIG_SERIALCONTROL_HPP