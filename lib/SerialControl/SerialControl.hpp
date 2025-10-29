
#ifndef TESTRIG_SERIALCONTROL_HPP
#define TESTRIG_SERIALCONTROL_HPP

#include <Arduino.h>

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
    SerialControl();

    void update();
    SerialData inputData;
    SerialData outputData;

private:
    void parseDATA(const String &input);
    void readData();
    void sendData();
};


#endif //TESTRIG_SERIALCONTROL_HPP