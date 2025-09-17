
#ifndef KYB_SENSORAS5600_HPP
#define KYB_SENSORAS5600_HPP

#include <AS5600.h>

class SensorAS5600 {
public:
	SensorAS5600(int SDA, int SCL);
	void begin();
	uint32_t getCumulativePosition();

	static constexpr double RAW_TO_RAD = 2.0 * 3.1415926 / 4096.0;
	static constexpr double RAW_TO_DEG = 360.0 / 4096.0;
private:
	AS5600 _sensor;
	int _sda;
	int _scl;
};


#endif //KYB_SENSORAS5600_HPP