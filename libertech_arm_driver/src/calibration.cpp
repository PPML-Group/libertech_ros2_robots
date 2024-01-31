#include "libertech_arm_driver/ArmDriver.hpp"

int main()
{
	ArmDriver armDriver;
	sleep(2);

    // 零位
	armDriver.setZero();

    armDriver.save();

	return 0;
}
