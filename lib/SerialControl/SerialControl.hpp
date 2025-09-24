
#ifndef TESTRIG_SERIALCONTROL_HPP
#define TESTRIG_SERIALCONTROL_HPP

#include <Arduino.h>

struct InputData {
    int32_t targetPosition0;
    int32_t targetPosition1;
};

struct OutputData {
    int32_t currentPosition0;
    int32_t currentPosition1;
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