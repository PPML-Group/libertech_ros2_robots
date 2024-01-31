#include "libertech_arm_driver/ArmDriver.hpp"

int main()
{
	ArmDriver armDriver;

	// armDriver.setZero();

	armDriver.runSendReceiveThread();

	Vec14 q;
	// q[0] = 15 * M_PI / 180;
	// q[1] = 15 * M_PI / 180;
	// q[2] = 15 * M_PI / 180;
	// q[3] = 15 * M_PI / 180;
	// q[4] = 20 * M_PI / 180;
	// q[5] = 20 * M_PI / 180;
	// q[6] = 20 * M_PI / 180;

	q[0] = 10 * M_PI / 180;
	q[1] = 0 * M_PI / 180;
	q[2] = 0 * M_PI / 180;
	q[3] = 20 * M_PI / 180;
	q[4] = 0 * M_PI / 180;
	q[5] = 0 * M_PI / 180;
	q[6] = 0 * M_PI / 180;

	for (int i=0;i<2;i++) {
		q[i+7] = 0;
	}

	int dir = 0;
	while (1)
	{
		if (dir % 2 == 0) {

			q[0] = 10 * M_PI / 180;
			q[1] = 0 * M_PI / 180;
			q[2] = 0 * M_PI / 180;
			q[3] = 30 * M_PI / 180;
			q[4] = 0 * M_PI / 180;
			q[5] = 0 * M_PI / 180;
			q[6] = 0 * M_PI / 180;

			armDriver.setQ(q);
		}
		else {

			q[0] = 0 * M_PI / 180;
			q[1] = 0 * M_PI / 180;
			q[2] = 0 * M_PI / 180;
			q[3] = 0 * M_PI / 180;
			q[4] = 0 * M_PI / 180;
			q[5] = 0 * M_PI / 180;
			q[6] = 0 * M_PI / 180;

			armDriver.setQ(q);
		}
		dir++;
		sleep(1);
	}
	

// 	armDriver.setMode("velocity");
//     // armDriver.setId(1,2);
// //armDriver.getOffset();
// Vec14 qd;
// qd(6)=1;// 
// armDriver.setQd(qd);

// 	armDriver.runSendReceiveThread();

// Vec14 max;
// Vec14 min;
// Vec14 kp; 
// Vec14 kd;
   
// max(7)=6.28;//+360
// min(7)=-6.28;//-360
// kp(7)=800;
// kd(7)=2000;//0x7d0//i+1可能需要该以下

// std::cout<<"赋值1 min="<<min(7)<<std::endl;
    
// armDriver.setMinMaxPos(min,max);//最小数值不正确a
// armDriver.setKp(kp);
// armDriver.setKd(kd);
// armDriver.getKp();
// armDriver.getKd();

// std::cout<<"1"<<std::endl;

// std::cout<<"2"<<std::endl;

// std::cout<<"3"<<std::endl;


// sleep(10);
// std::cout<<"sleep(10);"<<std::endl;

// armDriver.stop();
// 	sleep(5);
// std::cout<<"armDriver.stop();"<<std::endl;
//  while (1)
//  {
// 	/* code */
//  }
 
	// 设置位置指令
	// Vec14 q;
	// q[0] = 8;
	// q[1] = 8;
	// armDriver.setQ(q);

	
	// Vec14 qd;
	// armDriver.setMode("velocity");
	// armDriver.runSendReceiveThread();
	// armDriver.setQd(qd);

	


	while (1)
	{
		sleep(1);
	}

	armDriver.pause();
	return 0;
}
