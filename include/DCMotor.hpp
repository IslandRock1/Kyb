//
// Created by Øystein Bringsli.
//

#ifndef TESTRIG_DCMOTOR_HPP
#define TESTRIG_DCMOTOR_HPP


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


#endif //TESTRIG_DCMOTOR_HPP