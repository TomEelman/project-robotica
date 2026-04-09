#include "LIDAR.h"
#include <cstring>
#include <cmath>
 
LIDAR::LIDAR(uart_inst_t* uartPort, int baudRate)
    : uartPort(uartPort),
      baudRate(baudRate),
      maxRange(4000),
      minRange(0),
      rotationSpeed(0),
      currentAngle(0),
      nextAngle(0),
      updated(false)
{
    uart_init(uartPort, baudRate);
    memset(scanData, 0, sizeof(scanData));
}

bool LIDAR::Update() {
	return true;
}

bool LIDAR::IsObjectInRange(int minAngle, int maxAngle, int threshold) const {
	return true;

}

void LIDAR::ApplyMotionCorrection(float currentYaw) {
	return;
}
