
#include "SensorAS5600.hpp"

SensorAS5600::SensorAS5600(const int sda, const int scl)
	: _sda(sda), _scl(scl) {}


void SensorAS5600::begin() {
	Wire.begin(_sda, _scl);

	auto t0 = millis();

	if (!_sensor.isConnected()) {Serial.print("ERROR: Unable to connect to sensor.");}
	while (!_sensor.isConnected()) {
		if (millis() - t0 > 1000) {
			t0 = millis();
			Serial.println(".");
		}
	}
	Serial.println();
	Serial.println("Sensor found!");
}

int32_t SensorAS5600::getCumulativePosition() {
	return _sensor.getCumulativePosition();
}

void SensorAS5600::resetCumulativePosition() {
	_sensor.resetCumulativePosition();
}
