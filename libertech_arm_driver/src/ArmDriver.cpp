#include "libertech_arm_driver/ArmDriver.hpp"

ArmDriver::ArmDriver() {	
	init();
}

ArmDriver::~ArmDriver() {
	delete cmd_;  // 释放 cmd_ 指向的内存
	delete data_; // 释放 data_ 指向的内存

	usleep(100000);//延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
	usleep(100000);//延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
	usleep(100000);//延时100ms。
	VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
	//除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
}

int ArmDriver::init() {
    // 初始化USBCAN设备
    vciNum_=VCI_FindUsbDevice2(vciPInfoAll_);
    printf(">>USBCAN DEVICE NUM:");printf("%d", vciNum_);printf(" PCS");printf("\n");

    for(int i=0;i<vciNum_;i++)
    {
		printf("Device:");printf("%d", i);printf("\n");
                printf(">>Get VCI_ReadBoardInfo success!\n");
		
		printf(">>Serial_Num:%c", vciPInfoAll_[i].str_Serial_Num[0]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[1]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[2]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[3]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[4]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[5]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[6]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[7]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[8]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[9]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[10]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[11]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[12]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[13]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[14]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[15]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[16]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[17]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[18]);
		printf("%c", vciPInfoAll_[i].str_Serial_Num[19]);printf("\n");

		printf(">>hw_Type:%c", vciPInfoAll_[i].str_hw_Type[0]);
		printf("%c", vciPInfoAll_[i].str_hw_Type[1]);
		printf("%c", vciPInfoAll_[i].str_hw_Type[2]);
		printf("%c", vciPInfoAll_[i].str_hw_Type[3]);
		printf("%c", vciPInfoAll_[i].str_hw_Type[4]);
		printf("%c", vciPInfoAll_[i].str_hw_Type[5]);
		printf("%c", vciPInfoAll_[i].str_hw_Type[6]);
		printf("%c", vciPInfoAll_[i].str_hw_Type[7]);
		printf("%c", vciPInfoAll_[i].str_hw_Type[8]);
		printf("%c", vciPInfoAll_[i].str_hw_Type[9]);printf("\n");	

		printf(">>Firmware Version:V");
		printf("%x", (vciPInfoAll_[i].fw_Version&0xF00)>>8);
		printf(".");
		printf("%x", (vciPInfoAll_[i].fw_Version&0xF0)>>4);
		printf("%x", vciPInfoAll_[i].fw_Version&0xF);
		printf("\n");
	}
	
    // 打开第一个USBCAN，理论上只有一个
    if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		printf(">>open deivce success!\n");//打开设备成功
	}else
	{
		printf(">>open deivce error!\n");
		exit(1);
	}
    if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&vciPInfo_)==1)//读取设备序列号、版本等信息。
	{
        printf(">>Get VCI_ReadBoardInfo success!\n");

		printf(">>Serial_Num:%c", vciPInfo_.str_Serial_Num[0]);
		printf("%c", vciPInfo_.str_Serial_Num[1]);
		printf("%c", vciPInfo_.str_Serial_Num[2]);
		printf("%c", vciPInfo_.str_Serial_Num[3]);
		printf("%c", vciPInfo_.str_Serial_Num[4]);
		printf("%c", vciPInfo_.str_Serial_Num[5]);
		printf("%c", vciPInfo_.str_Serial_Num[6]);
		printf("%c", vciPInfo_.str_Serial_Num[7]);
		printf("%c", vciPInfo_.str_Serial_Num[8]);
		printf("%c", vciPInfo_.str_Serial_Num[9]);
		printf("%c", vciPInfo_.str_Serial_Num[10]);
		printf("%c", vciPInfo_.str_Serial_Num[11]);
		printf("%c", vciPInfo_.str_Serial_Num[12]);
		printf("%c", vciPInfo_.str_Serial_Num[13]);
		printf("%c", vciPInfo_.str_Serial_Num[14]);
		printf("%c", vciPInfo_.str_Serial_Num[15]);
		printf("%c", vciPInfo_.str_Serial_Num[16]);
		printf("%c", vciPInfo_.str_Serial_Num[17]);
		printf("%c", vciPInfo_.str_Serial_Num[18]);
		printf("%c", vciPInfo_.str_Serial_Num[19]);printf("\n");

		printf(">>hw_Type:%c", vciPInfo_.str_hw_Type[0]);
		printf("%c", vciPInfo_.str_hw_Type[1]);
		printf("%c", vciPInfo_.str_hw_Type[2]);
		printf("%c", vciPInfo_.str_hw_Type[3]);
		printf("%c", vciPInfo_.str_hw_Type[4]);
		printf("%c", vciPInfo_.str_hw_Type[5]);
		printf("%c", vciPInfo_.str_hw_Type[6]);
		printf("%c", vciPInfo_.str_hw_Type[7]);
		printf("%c", vciPInfo_.str_hw_Type[8]);
		printf("%c", vciPInfo_.str_hw_Type[9]);printf("\n");

		printf(">>Firmware Version:V");
		printf("%x", (vciPInfo_.fw_Version&0xF00)>>8);
		printf(".");
		printf("%x", (vciPInfo_.fw_Version&0xF0)>>4);
		printf("%x", vciPInfo_.fw_Version&0xF);
		printf("\n");	
	}else
	{
		printf(">>Get VCI_ReadBoardInfo error!\n");
		exit(1);
	}

    vciConfig_.AccCode=0;
	vciConfig_.AccMask=0xFFFFFFFF;
	vciConfig_.Filter=1;//接收所有帧
	vciConfig_.Timing0=0x00;/*波特率1000k*/
	vciConfig_.Timing1=0x14;
	vciConfig_.Mode=0;//正常模式		
	
    // 后续两路机械臂各占一个通道
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&vciConfig_)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_InitCAN(VCI_USBCAN2,0,1,&vciConfig_)!=1)
	{
		printf(">>Init CAN2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
	{
		printf(">>Start CAN2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

    // TODO: 确定调试电机个数 可以调试双臂 7 * 2
    numJoints_ = 7;
	numCAN_ = 2;
	std::cout<<"numJoints_: "<<numJoints_<<std::endl;

	cmd_ = new Command(); // 初始化为 Command 类型的默认对象
	data_ = new Data();   // 初始化为 Data 类型的默认对象
	clearCmd();
	clearData();
	clearError();

	setMode("position");
    return 1;
}

int ArmDriver::doSendReceive() {
	int ret;
	// TODO: 线程锁

	if (mode_ == "position") {
		// 先设置kp,kdstd
		// ret = sendKp();
		// ret = sendKd();
		ret = sendQ();
		std::cout<<"sendQ"<<std::endl;
	} else if (mode_ == "velocity") {
		ret = sendQd();
		std::cout<<"sendQd"<<std::endl;
	} else if (mode_ == "force") {
		ret = sendTau();
	} else {
		// 处理未知模式的情况
	}

	// std::cout<<"ret: "<<ret<<std::endl;
	usleep(500);
	
	VCI_CAN_OBJ rec[3000];
	int reclen=0;
	int ind=0;
	// 总是获取电流速度位置，在使用线程时
	if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
	{
		for(int j=0;j<reclen;j++)
		{
			printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
			if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
			if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
			if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
			if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
			printf("DLC:0x%02X",rec[j].DataLen);//帧长度
			printf(" data:0x");	//数据
			for(int i = 0; i < rec[j].DataLen; i++)
			{
				printf(" %02X", rec[j].Data[i]);
			}
			// TODO: 对应的储存在Data
			


			printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
			printf("\n");
		}
	}

	std::cout<<"reclen: "<<reclen<<std::endl;

    return ret; 
}

void ArmDriver::runSendReceive() {
    // TODO: 只有位置模式
    while(isRunning_) {
		int ret = doSendReceive();
		// usleep(1000);
    }
}

void ArmDriver::runSendReceiveThread() {
    isRunning_ = true;
	thread_ = std::thread([this] { runSendReceive(); }); // 使用 Lambda 表达式捕获 this

}

int ArmDriver::pause() {
    isRunning_ = false;
	if (thread_.joinable()) {
		thread_.join();
	}
	usleep(100000);
}

int ArmDriver::stop() {
	for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		VCI_CAN_OBJ send[numJoints_];
		for(int i=0;i<numJoints_;i++) {
			int id = i+1;
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=1;
			send[i].Data[0]=0x02;
		}	
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
	}
	return 1;
}

int ArmDriver::setId(int id,int setID) {
	// 暂时不用
	VCI_CAN_OBJ send[1];
	send[0].SendType=0;
	send[0].RemoteFlag=0;
	send[0].ExternFlag=0;
	send[0].ID=id;
	send[0].DataLen=5;
	send[0].Data[0]=0x2e;
	
	uint8_t u8[4];
	int32_t int_x = static_cast<int32_t>(setID);
    for (int i = 0; i < 4; ++i) {
        u8[i] = (int_x >> (i * 8)) & 0xFF;
    }
	for (size_t j = 1; j <5 ; j++)
	{
		send[0].Data[j]=u8[j-1];
	}
	int ret=VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);

	send[0].ID=setID;
	send[0].Data[0]=0x0e; 
	// ret=VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);
	// ret=VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);

	printf("Set ID from %d to %d\n", id, setID);
    return ret;
}

int ArmDriver::sendKp() {	
	for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		VCI_CAN_OBJ send[numJoints_];
		for(int i=0;i<numJoints_;i++) {
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=5;
			send[i].Data[0]=0x2b;

			uint8_t u8[4];

			{
				std::lock_guard<std::mutex> lock(mtx_);
				convertFloatToHexBytes(cmd_->kpDes[i+7*idCAN],u8); //
			}
			
			// swapEndianness(u8);
			for (size_t j = 1; j <5 ; j++)
			{
				send[i].Data[j]=u8[j-1];
			}
		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
	}
    return 1;
}

int ArmDriver::sendKd() {
	for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		VCI_CAN_OBJ send[numJoints_];
		for(int i=0;i<numJoints_;i++) {
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=5;
			send[i].Data[0]=0x2d;

			uint8_t u8[4];
			{
				std::lock_guard<std::mutex> lock(mtx_);
				convertFloatToHexBytes(cmd_->kdDes[i+7*idCAN],u8); //
			}
			
			// swapEndianness(u8);
			for (size_t j = 1; j <5 ; j++)
			{
				send[i].Data[j]=u8[j-1];
			}
		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
	}
    return 1;
}

int ArmDriver::sendQ() {
	for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		VCI_CAN_OBJ send[numJoints_];
		for(int i=0;i<numJoints_;i++) {
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=5;
			send[i].Data[0]=0x44;  // 44好像不需要大小端转换

			uint8_t u8[4];
			
			{
				std::lock_guard<std::mutex> lock(mtx_);
				convertPositionToHexBytes(cmd_->qDes[i+7*idCAN],u8); 
				// convertPositionToHexBytes(-M_PI,u8); // cmd是弧度
			} 
			for (size_t j = 1; j <5 ; j++)
			{
				send[i].Data[j]=u8[j-1];
			}
		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
	}
    return 1;
}

int ArmDriver::sendQd() {
	for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		VCI_CAN_OBJ send[numJoints_];
		for(int i=0;i<numJoints_;i++) {
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=5;
			send[i].Data[0]=0x43; //

			uint8_t u8[4];
			{
				std::lock_guard<std::mutex> lock(mtx_);
				convertVelocityToHexBytes(cmd_->qdDes[i+7*idCAN],u8); // 弧度制
			}
			
			// swapEndianness(u8);
			for (size_t j = 1; j <5 ; j++)
			{
				send[i].Data[j]=u8[j-1];
			}
		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
	}
    return 1;
}

int ArmDriver::sendTau() {
	for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		VCI_CAN_OBJ send[numJoints_];
		for(int i=0;i<numJoints_;i++) {
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=5;
			send[i].Data[0]=0x42;

			uint8_t u8[4];
			
			{
				std::lock_guard<std::mutex> lock(mtx_);
				convertTorqueToHexBytes(cmd_->qDes[i+7*idCAN],u8); // TODO：力，底层写转电流
			}

			// swapEndianness(u8);
			for (size_t j = 1; j <5 ; j++)
			{
				send[i].Data[j]=u8[j-1];
			}
		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
	}
    return 1;
}

int ArmDriver::setModePosition() {
	printf("Set Mode: Position\n");
	for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		// 获取
		VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
		for(int i=0;i<numJoints_;i++) { 
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=1;
			send[i].Data[0]=0x08;
		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
		// std::cout<<"ret: "<<ret<<std::endl;

		VCI_CAN_OBJ rec[3000]; //接收缓存，设为3000为佳。
		int reclen=0;

		int32_t int_q[numJoints_] = {0};

		// while (!reclen) {
			if((reclen=VCI_Receive(VCI_USBCAN2,0,idCAN,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
			{
				for(int j=0;j<reclen;j++)
				{
					printf("CAN%d RX ID:0x%08X", idCAN, rec[j].ID);//ID
					if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
					if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
					if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
					if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
					printf("DLC:0x%02X",rec[j].DataLen);//帧长度
					printf(" data:0x");	//数据
					for(int i = 0; i < rec[j].DataLen; i++)
					{
						printf(" %02X", rec[j].Data[i]);
					}

					int_q[rec[j].ID-1] = (int32_t(rec[j].Data[1]))  |  (int32_t(rec[j].Data[2]) << 8)  |   (int32_t(rec[j].Data[3])<< 16) | (int32_t(rec[j].Data[4])<<24);

					printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
					printf("\n");
				}
			}

			std::cout<<"reclen: "<<reclen<<std::endl;
		// }

		// 设置电机运行模式为位置模式 ，并设置目标位置
		for(int i=0;i<numJoints_;i++) {
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=5;
			send[i].Data[0]=0x1e;

			std::cout<<"int_q:" << int_q[i] << std::endl;

			uint8_t u8[4];
			for (int j = 0; j < 4; ++j) {
				u8[j] = (int_q[i] >> (j * 8)) & 0xFF;
			}
			printHexBytes(u8);
			swapEndianness(u8);
			for (size_t j = 1; j <5 ; j++)
			{
				send[i].Data[j]=u8[j-1];
				// send[i].Data[j]=0;
			}

			// !!!! 避免位置跳变，将cmd的值赋值为当前值
			// cmd_->qDes[i] = convertHexBytesToPosition(u8);
		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
	}
	usleep(100000);
	printf("Set Mode Success\n");
	return 1;
}

int ArmDriver::setModeVelocity() {
	// 与设置位置不同，不用读取，直接设为0
	printf("Set Mode: Velocity\n");
	for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
		for(int i=0;i<numJoints_;i++) {
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=5;
			send[i].Data[0]=0x1d;
		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
	}
	usleep(100000);
	printf("Set Mode Success\n");
	return 1;
}

int ArmDriver::setModeTorque() {
	// 与设置位置不同，不用读取，直接设为0
	printf("Set Mode: Torque\n");
    for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
		for(int i=0;i<numJoints_;i++) {
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=5;
			send[i].Data[0]=0x1c;
		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
	}
	usleep(100000);
	printf("Set Mode Success\n"); // TODO： 逻辑修改
	return 1;
}

int ArmDriver::setMode(const std::string& mode) {
	// TODO: 目前只有位置控制
	// stop(); // TODO： 切换是否需要停止
	pause(); // TODO: 等待发送结束
	mode_ = mode;

	int ret;
	if (mode == "position") {
        ret = setModePosition();
    } else if (mode == "velocity") {
        ret = setModeVelocity();
    } else if (mode == "force") {
        ret = setModeTorque();
    } else {
        // 处理未知模式的情况
    }
    return ret;
}

int ArmDriver::resetToDefault() {
	for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
		for(int i=0;i<numJoints_;i++) {
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=1;
			send[i].Data[0]=0x0f;
		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
	}
	usleep(100000);
	return 1;
}

int ArmDriver::clearError() {
	for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
		for(int i=0;i<numJoints_;i++) {
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=1;
			send[i].Data[0]=0x0b;
		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
	}
	usleep(100000);
	return 1;
}

int ArmDriver::setMinMaxPos(const Vec14& qmin, const Vec14& qmax) {
    for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send

		for(int i=0;i<numJoints_;i++) { 
		
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=5;
			send[i].Data[0]=0x27;

			uint8_t u8[4];
			convertFloatToHexBytes(convertRadiansToDegrees((qmin(i+7*idCAN))/360)*51*65536,u8);//忘记角度转换下发位置
			printHexBytes(u8);
			//	swapEndianness(u8);//此处不需要转换大小端
		
			for (size_t b = 0; b < 4; b++)
			{
				send[i].Data[b+1]=u8[b];
			}
		}
		
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
		////////////////////////////////////////////////////////////////////////////////////////
		for(int i=0;i<numJoints_;i++) { 
		
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=5;
			send[i].Data[0]=0x26;

			uint8_t u8[4];
			convertFloatToHexBytes(convertRadiansToDegrees((qmax(i+7*idCAN))/360)*51*65536,u8);//忘记角度转换下发位置
			printHexBytes(u8);
			//	swapEndianness(u8);//此处不需要转换大小端
			for (size_t b = 0; b < 4; b++)
			{
				send[i].Data[b+1]=u8[b];
			}

		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
	}
	return 1;
}

int ArmDriver::setMinMaxVel(const Vec14& vmin, const Vec14& vmax) {
    return 0;
}

int ArmDriver::setMinMaxAcc(const Vec14& amin, const Vec14& amax) {
    return 0;
}

int ArmDriver::clearCmd() {
    if (cmd_) {
        for (int i = 0; i < NUM; ++i) {
            cmd_->qDes[i] = 0.0f;
            cmd_->qdDes[i] = 0.0f;
            cmd_->tauDes[i] = 0.0f;
            cmd_->kpDes[i] = 10.0f;
            cmd_->kdDes[i] = 1.0f;
        }
    }
	return 1;
}

int ArmDriver::clearData() {
    if (data_) {
        for (int i = 0; i < NUM; ++i) {
            data_->q[i] = 0.0f;
            data_->qd[i] = 0.0f;
            data_->tau[i] = 0.0f;
        }
    }
	return 1;
}

int ArmDriver::setQ(const Vec14& q) {
	std::lock_guard<std::mutex> lock(mtx_);

	for(int i=0;i<NUM;i++) {
		cmd_->qDes[i] = q[i];
	}
    return 1;
}

int ArmDriver::setQd(const Vec14& qd) {
	std::lock_guard<std::mutex> lock(mtx_);

	for(int i=0;i<NUM;i++) {
		cmd_->qdDes[i] = qd[i];
	}
    return 1;
}

int ArmDriver::setTau(const Vec14& tau) {
    return 0;
}

int ArmDriver::setKp(const Vec14& kp) {
	std::lock_guard<std::mutex> lock(mtx_);

	for(int i=0;i<NUM;i++) {
		cmd_->qdDes[i] = kp[i];
	}
	// for (int idCAN=0; idCAN<numCAN_; idCAN++) {
	// VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send

    // for(int i=0;i<numJoints_;i++) { 
	 
    //     int id = i+1; // TODO: 双臂id得换
	// 	send[i].SendType=0;
    //     send[i].RemoteFlag=0;
    //     send[i].ExternFlag=0;
    //     send[i].ID=id;
    //     send[i].DataLen=5;
    //     send[i].Data[0]=0x2b;

	// 	uint8_t u8[4];
	// 	convertFloatToHexBytes(kp(i+1),u8);//忘记角度转换下发位置
	// 	printHexBytes(u8);
	// 	//	swapEndianness(u8);//此处不需要转换大小端
	
	// 	for (size_t b = 0; b < 4; b++)
	// 	{
	// 		send[i].Data[b+1]=u8[b];
	// 	}

	// }
    // int ret=VCI_Transmit(VCI_USBCAN2, 0, 0, send, numJoints_);
	// std::cout<<"ret: "<<ret<<std::endl;
	// }
    return 1;
}

int ArmDriver::setKd(const Vec14& kd) {
	std::lock_guard<std::mutex> lock(mtx_);

	for(int i=0;i<NUM;i++) {
		cmd_->qdDes[i] = kd[i];
	}
	// VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
    // for(int i=0;i<numJoints_;i++) { 
	 
    //     int id = i+1; // TODO: 双臂id得换
	// 	send[i].SendType=0;
    //     send[i].RemoteFlag=0;
    //     send[i].ExternFlag=0;
    //     send[i].ID=id;
    //     send[i].DataLen=5;
    //     send[i].Data[0]=0x2d;
	// 	uint8_t u8[4];
		
	// 	convertFloatToHexBytes(kd(i+1),u8);//忘记角度转换下发位置
	// 	printHexBytes(u8);
	// 	//	swapEndianness(u8);//此处不需要转换大小端
	
	// 	for (size_t b = 0; b < 4; b++)
	// 	{
    
	// 		send[i].Data[b+1]=u8[b];
	// 		/* code */
	// 	}

	// }
	
    // int ret=VCI_Transmit(VCI_USBCAN2, 0, 0, send, numJoints_);
	// std::cout<<"ret: "<<ret<<std::endl;
    return 1;
}

int ArmDriver::getQ(Vec14& qm) {
	
	if (isRunning_)	{
		// 返回DATA中的值
	}
	else {
		// 如果当前没有使用线程，则通过发08实现获取
	}
	
    return 1; // 返回空的 vector
}

int ArmDriver::getQd(Vec14& qdm) {
	
	if (isRunning_)	{
		// 返回DATA中的值
	}
	else {
		// 如果当前没有使用线程，则通过发xx实现获取
	}
	
    return 1; // 返回空的 vector
}

int ArmDriver::getTau(Vec14& qm) {
	
	if (isRunning_)	{
		// 返回DATA中的值
	}
	else {
		// 如果当前没有使用线程，则通过发xx实现获取
	}
	
    return 1; // 返回空的 vector
}


int ArmDriver::getKp(Vec14& kpm) {
	// 线程里是不随时获取Kp的，因此只能手动单次获取
	for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
		for(int i=0;i<numJoints_;i++) { 
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=1;
			send[i].Data[0]=0x12;
		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
		////////////////////////////////////////////////////////////////////////////////////////////
		VCI_CAN_OBJ rec[3000]; //接收缓存，设为3000为佳。
		int reclen=0;

		int32_t int_q[numJoints_] = {0};
		if((reclen=VCI_Receive(VCI_USBCAN2,0,idCAN,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(int j=0;j<reclen;j++)
			{
				printf("CAN%d RX ID:0x%08X", idCAN, rec[j].ID);//ID
				if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
				if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
				if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
				if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
				printf("DLC:0x%02X",rec[j].DataLen);//帧长度
				printf(" data:0x");	//数据
				for(int i = 0; i < rec[j].DataLen; i++)
				{
					printf(" %02X", rec[j].Data[i]);
				}
				
				int_q[rec[j].ID-1] = (int32_t(rec[j].Data[1]))  |  (int32_t(rec[j].Data[2]) << 8)  |   (int32_t(rec[j].Data[3])<< 16) | (int32_t(rec[j].Data[4])<<24);

				printf(" !TimeStamp:0x%08X\n",rec[j].TimeStamp);//时间标识。
				printf("位置环kp%08X\n",int_q[rec[j].ID-1]);//时间标识。
				
				printf("\n");
			}
		}
	
	}
	return 1;
}

int ArmDriver::getKd(Vec14& kdm) {
	// 线程里是不随时获取Kd的，因此只能手动单次获取
	for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
		for(int i=0;i<numJoints_;i++) { 
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=1;
			send[i].Data[0]=0x13;
		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }

		////////////////////////////////////////////////////////////////////////////////////////////
		VCI_CAN_OBJ rec[3000]; //接收缓存，设为3000为佳。
		int reclen=0;
		int32_t int_q[numJoints_] = {0};
		if((reclen=VCI_Receive(VCI_USBCAN2,0,idCAN,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(int j=0;j<reclen;j++)
			{
				printf("CAN%d RX ID:0x%08X", idCAN, rec[j].ID);//ID
				if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
				if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
				if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
				if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
				printf("DLC:0x%02X",rec[j].DataLen);//帧长度
				printf(" data:0x");	//数据
				for(int i = 0; i < rec[j].DataLen; i++)
				{
					printf(" %02X", rec[j].Data[i]);
				}
				
				int_q[rec[j].ID-1] = (int32_t(rec[j].Data[1]))  |  (int32_t(rec[j].Data[2]) << 8)  |   (int32_t(rec[j].Data[3])<< 16) | (int32_t(rec[j].Data[4])<<24);

				printf(" !TimeStamp:0x%08X\n",rec[j].TimeStamp);//时间标识。
				printf("位置环kd%08X\n",int_q[rec[j].ID-1]);//时间标识。
				
				printf("\n");
			}
		}
	}
	return 1;
}

void ArmDriver::convertPositionToHexBytes(float p, uint8_t (&bytes)[4]) {
	float p_convert = convertRadiansToDegrees(p)/360*51*65536;
	convertFloatToHexBytes(p_convert,bytes);
}

void ArmDriver::convertVelocityToHexBytes(float v, uint8_t (&bytes)[4]) {
	convertFloatToHexBytes((convertRadiansToDegrees(v)*51*100)/360,bytes);


}

void ArmDriver::convertTorqueToHexBytes(float t, uint8_t (&bytes)[4]) {

}

float ArmDriver::convertHexBytesToPosition(const uint8_t (&bytes)[4]) {
	float p_convert = convertHexBytesToFloat(bytes)/(360*51*65536);
	return convertDegreesToRadians(p_convert);
}

float ArmDriver::convertHexBytesToVelocity(const uint8_t (&bytes)[4]) {

}

float ArmDriver::convertHexBytesToTorque(const uint8_t (&bytes)[4]) {

}

void ArmDriver::convertFloatToHexBytes(float x, uint8_t (&bytes)[4]) {
	int32_t int_x = static_cast<int32_t>(x);
    for (int i = 0; i < 4; ++i) {
        bytes[i] = (int_x >> (i * 8)) & 0xFF;
    }
	// swapEndianness(bytes); 
}

float ArmDriver::convertHexBytesToFloat(const uint8_t (&bytes)[4]) {
    uint8_t swappedBytes[4];
    std::copy(std::begin(bytes), std::end(bytes), std::begin(swappedBytes)); // 复制数组

    // swapEndianness(swappedBytes); // 执行大小端转换

    int32_t int_x = 0;
    for (int i = 0; i < 4; ++i) {
        int_x |= static_cast<int32_t>(swappedBytes[i]) << (i * 8);
    }
    return *reinterpret_cast<float*>(&int_x);
}

double ArmDriver::convertDegreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

double ArmDriver::convertRadiansToDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

void ArmDriver::swapEndianness(uint8_t (&bytes)[4]) {
    std::swap(bytes[0], bytes[3]);
    std::swap(bytes[1], bytes[2]);
}

void ArmDriver::printHexBytes(const uint8_t (&bytes)[4]) {
    std::cout << "Hex representation: ";
    for (int i = 3; i >= 0; --i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytes[i]) << " ";
    }
    std::cout << std::endl;
}

int ArmDriver::setZero() {
	for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		int32_t buff_ff[numJoints_];
		int32_t buff[numJoints_];
		int32_t int_q[numJoints_] = {0};

		printf("获取位置\n");
		// 获取
		VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
		for(int i=0;i<numJoints_;i++) { 
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=1;
			send[i].Data[0]=0x08;
		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }

		VCI_CAN_OBJ rec[3000]; //接收缓存，设为3000为佳。
		int reclen=0;
		
		// while (!reclen) {
		if((reclen=VCI_Receive(VCI_USBCAN2,0,idCAN,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(int j=0;j<reclen;j++)
			{
				printf("CAN%d RX ID:0x%08X", idCAN, rec[j].ID);//ID
				if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
				if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
				if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
				if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
				printf("DLC:0x%02X",rec[j].DataLen);//帧长度
				printf(" data:0x");	//数据
				for(int i = 0; i < rec[j].DataLen; i++)
				{
					printf(" %02X", rec[j].Data[i]);
				}

				int_q[rec[j].ID-1] = (int32_t(rec[j].Data[1]))  |  (int32_t(rec[j].Data[2]) << 8)  |   (int32_t(rec[j].Data[3])<< 16) | (int32_t(rec[j].Data[4])<<24);

				printf(" !TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
				printf("\n");
			}
		}

		for (size_t i = 0; i < numJoints_; i++)
		{
			printf("位置!!!!!!!!!!!%08x\n",int_q[i]);
			buff[i]=int_q[i];
		}
		///////////////////////////////////////////////////////////////////////////////////////////////////////
		
		for(int i=0;i<numJoints_;i++) { 
			int id = i+1; // TODO: 双臂id得换
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=1;
			send[i].Data[0]=0x54;
		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }

		if((reclen=VCI_Receive(VCI_USBCAN2,0,idCAN,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
			{
				for(int j=0;j<reclen;j++)
				{
					printf("CAN%d RX ID:0x%08X", idCAN, rec[j].ID);//ID
					if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
					if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
					if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
					if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
					printf("DLC:0x%02X",rec[j].DataLen);//帧长度
					printf(" data:0x");	//数据
					for(int i = 0; i < rec[j].DataLen; i++)
					{
						printf(" %02X", rec[j].Data[i]);
					}

					int_q[rec[j].ID-1] = (int32_t(rec[j].Data[1]))  |  (int32_t(rec[j].Data[2]) << 8)  |   (int32_t(rec[j].Data[3])<< 16) | (int32_t(rec[j].Data[4])<<24);

					printf(" !TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
					printf("\n");
				}
			}
		for (size_t i = 0; i < numJoints_; i++)
		{
			printf("偏移!!!!!!!!!!!%08x\n",int_q[i]);
			fflush(stdout);	
		}
		fflush(stdout);	


		for (size_t x = 0; x < numJoints_; x++)
		{	
			buff_ff[x]=buff[x]+int_q[x];
			printf("新偏移!!!!!!!!!!!!!!%08x",buff_ff[x]);
			fflush(stdout);	
		}

		////////////////////////////////////////////////////////////////////////////////////
		for(int s=0;s<numJoints_;s++) { 
			int id = s+1; // TODO: 双臂id得换
			send[s].SendType=0;
			send[s].RemoteFlag=0;
			send[s].ExternFlag=0;
			send[s].ID=id;
			send[s].DataLen=5;
			send[s].Data[0]=0x53;
			uint8_t u8[4];
			convertFloatToHexBytes(buff_ff[s],u8);
			//swapEndianness(u8);//加上大小端反了
			for (size_t t = 0; t < 4; t++)
			{
				send[s].Data[t+1]=u8[t];
			}

		}
		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
	}
	return 1;
}

int ArmDriver::save() {
	for (int idCAN=0; idCAN<numCAN_; idCAN++) {
		VCI_CAN_OBJ send[numJoints_];
		for(int i=0;i<numJoints_;i++) {
			int id = i+1; 
			send[i].SendType=0;
			send[i].RemoteFlag=0;
			send[i].ExternFlag=0;
			send[i].ID=id;
			send[i].DataLen=1;
			send[i].Data[0]=0x0e;
		}	

		if (VCI_Transmit(VCI_USBCAN2, 0, idCAN, send, numJoints_) != 1) {
            return 0;  // 发送失败，返回0
        }
	}
    return 1;
}
