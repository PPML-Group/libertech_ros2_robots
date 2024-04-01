#include "libertech_arm_driver/ArmSDK.hpp"
#include <chrono>

ArmSDK::ArmSDK(int canID_)
{
  canID_ = canID_;
}

ArmSDK::~ArmSDK()
{
  delete cmd_;  // 释放 cmd_ 指向的内存
  delete data_; // 释放 data_ 指向的内存

  // 除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
  usleep(100000);                       // 延时100ms。
  VCI_ResetCAN(VCI_USBCAN2, 0, canID_); // 复位CAN1/CAN2通道。
  usleep(100000);                       // 延时100ms。
  // VCI_CloseDevice(VCI_USBCAN2, 0); // 关闭设备。
}

int ArmSDK::init()
{
  // 初始化USBCAN设备
  vciNum_ = VCI_FindUsbDevice2(vciPInfoAll_);
  printf(">>USBCAN DEVICE NUM:");
  printf("%d", vciNum_);
  printf(" PCS");
  printf("\n");

  for (int i = 0; i < vciNum_; i++)
  {
    printf("Device:");
    printf("%d", i);
    printf("\n");
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
    printf("%c", vciPInfoAll_[i].str_Serial_Num[19]);
    printf("\n");

    printf(">>hw_Type:%c", vciPInfoAll_[i].str_hw_Type[0]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[1]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[2]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[3]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[4]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[5]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[6]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[7]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[8]);
    printf("%c", vciPInfoAll_[i].str_hw_Type[9]);
    printf("\n");

    printf(">>Firmware Version:V");
    printf("%x", (vciPInfoAll_[i].fw_Version & 0xF00) >> 8);
    printf(".");
    printf("%x", (vciPInfoAll_[i].fw_Version & 0xF0) >> 4);
    printf("%x", vciPInfoAll_[i].fw_Version & 0xF);
    printf("\n");
  }

  // 打开第一个USBCAN，理论上只有一个
  if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1) // 打开设备
  {
    printf(">>open deivce success!\n"); // 打开设备成功
  }
  else
  {
    printf(">>open deivce error!\n");
    exit(1);
  }
  if (VCI_ReadBoardInfo(VCI_USBCAN2, 0, &vciPInfo_) == 1) // 读取设备序列号、版本等信息。
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
    printf("%c", vciPInfo_.str_Serial_Num[19]);
    printf("\n");

    printf(">>hw_Type:%c", vciPInfo_.str_hw_Type[0]);
    printf("%c", vciPInfo_.str_hw_Type[1]);
    printf("%c", vciPInfo_.str_hw_Type[2]);
    printf("%c", vciPInfo_.str_hw_Type[3]);
    printf("%c", vciPInfo_.str_hw_Type[4]);
    printf("%c", vciPInfo_.str_hw_Type[5]);
    printf("%c", vciPInfo_.str_hw_Type[6]);
    printf("%c", vciPInfo_.str_hw_Type[7]);
    printf("%c", vciPInfo_.str_hw_Type[8]);
    printf("%c", vciPInfo_.str_hw_Type[9]);
    printf("\n");

    printf(">>Firmware Version:V");
    printf("%x", (vciPInfo_.fw_Version & 0xF00) >> 8);
    printf(".");
    printf("%x", (vciPInfo_.fw_Version & 0xF0) >> 4);
    printf("%x", vciPInfo_.fw_Version & 0xF);
    printf("\n");
  }
  else
  {
    printf(">>Get VCI_ReadBoardInfo error!\n");
    exit(1);
  }

  vciConfig_.AccCode = 0;
  vciConfig_.AccMask = 0xFFFFFFFF;
  vciConfig_.Filter = 1;     // 接收所有帧
  vciConfig_.Timing0 = 0x00; /*波特率1000k*/
  vciConfig_.Timing1 = 0x14;
  vciConfig_.Mode = 0; // 正常模式

  // 后续两路机械臂各占一个通道
  if (VCI_InitCAN(VCI_USBCAN2, 0, canID_, &vciConfig_) != 1)
  {
    printf(">>Init CAN%d error\n", canID_ + 1);
    VCI_CloseDevice(VCI_USBCAN2, 0);
  }

  if (VCI_StartCAN(VCI_USBCAN2, 0, canID_) != 1)
  {
    printf(">>Start CAN%d error\n", canID_ + 1);
    VCI_CloseDevice(VCI_USBCAN2, 0);
  }

  // TODO: 确定调试电机个数 可以调试双臂 7 * 2
  numJoints_ = 7;
  std::cout << "numJoints_: " << numJoints_ << std::endl;

  cmd_ = new Command(); // 初始化为 Command 类型的默认对象
  data_ = new Data();   // 初始化为 Data 类型的默认对象
  clearCmd();
  clearData();
  clearError();

  setMode("position");
  return 1;
}

int ArmSDK::doSendReceive()
{
  auto start = std::chrono::high_resolution_clock::now();
  int ret;
  // TODO: 线程锁

  if (mode_ == "position")
  {
    // 先设置kp,kdstd
    ret = sendKp();
    // ret = sendKd();
    ret = sendQ();
    // std::cout<<"sendQ"<<std::endl;
  }
  else if (mode_ == "velocity")
  {
    ret = sendQd();
    // std::cout<<"sendQd"<<std::endl;
  }
  else if (mode_ == "force")
  {
    ret = sendTau();
  }
  else
  {
    // 处理未知模式的情况
  }

  // std::cout<<"ret: "<<ret<<std::endl;
  // usleep(500);

  // VCI_CAN_OBJ rec[3000];
  // int reclen = 0;
  // int ind = 0;
  // // 总是获取电流速度位置，在使用线程时
  // if ((reclen = VCI_Receive(VCI_USBCAN2, 0, ind, rec, 3000, 100)) > 0) // 调用接收函数，如果有数据，进行数据处理显示。
  // {
  //   for (int j = 0; j < reclen; j++)
  //   {
  //     // printf("CAN%d RX ID:0x%08X", ind+1, rec[j].ID);//ID
  //     // if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
  //     // if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
  //     // if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
  //     // if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
  //     // printf("DLC:0x%02X",rec[j].DataLen);//帧长度
  //     // printf(" data:0x");	//数据
  //     // for(int i = 0; i < rec[j].DataLen; i++)
  //     // {
  //     // 	printf(" %02X", rec[j].Data[i]);
  //     // }
  //     printf("CAN%d RX ID:0x%08X ", ind + 1, rec[j].ID); // ID

  //     // 解析电流（前两个字节）
  //     int32_t cur_int =
  //         static_cast<int32_t>(rec[j].Data[1]) << 8 | static_cast<int32_t>(rec[j].Data[0]); // TODO: 2字节怎么转
  //     // data_->tau[rec[j].ID-1];
  //     // 解析速度（中间两个字节）
  //     int32_t qd_int =
  //         static_cast<int32_t>(rec[j].Data[3]) << 8 | static_cast<int32_t>(rec[j].Data[2]); // TODO: 2字节怎么转
  //     // data_->tau[rec[j].ID-1];
  //     // 解析位置（最后四个字节）
  //     uint8_t bytes[4];
  //     std::copy(rec[j].Data + 4, rec[j].Data + 8, bytes);

  //     {
  //       std::lock_guard<std::mutex> lock(mtx_);
  //       data_->q[rec[j].ID - 1] = convertHexBytesToPosition(bytes);
  //       // data_->qd

  //       std::cout << "qm[" << rec[j].ID - 1 << "]: " << data_->q[rec[j].ID - 1] << std::endl;
  //     }

  //     // printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
  //     // printf("\n");
  //   }
  // }

  // //   std::cout << "reclen: " << reclen << std::endl;
  // std::cout<<"Debug: doSendReceive -- ret"<<ret<<std::endl;
  // std::cout<<"ret: "<<ret<<std::endl;
  usleep(200);

  VCI_CAN_OBJ rec[3000];
  int reclen = 0;
  // 总是获取电流速度位置，在使用线程时
  if ((reclen = VCI_Receive(VCI_USBCAN2, 0, canID_, rec, 3000, 100)) > 0) // 调用接收函数，如果有数据，进行数据处理显示。
  {
    for (int j = 0; j < reclen; j++)
    {
      // printf("CAN%d RX ID:0x%08X ", idCAN + 1, rec[j].ID);  // ID

      // 解析电流（前两个字节）
      int32_t cur_int =
          static_cast<int32_t>(rec[j].Data[1]) << 8 | static_cast<int32_t>(rec[j].Data[0]); // TODO: 2字节怎么转
      // data_->tau[rec[j].ID-1];
      // 解析速度（中间两个字节）
      int32_t qd_int =
          static_cast<int32_t>(rec[j].Data[3]) << 8 | static_cast<int32_t>(rec[j].Data[2]); // TODO: 2字节怎么转
      // data_->tau[rec[j].ID-1];
      // 解析位置（最后四个字节）
      uint8_t bytes[4];
      std::copy(rec[j].Data + 4, rec[j].Data + 8, bytes);
      // swapEndianness(bytes);

      {
        std::lock_guard<std::mutex> lock(mtx_);
        data_->q[rec[j].ID - 1] = convertHexBytesToPosition(bytes);
        // data_->qd

        // std::cout << "qm[" << rec[j].ID - 1 << "]: " << data_->q[rec[j].ID - 1] << std::endl;
      }
    }
  }

  auto finish = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start);
  // std::cout << "time: " << elapsed.count() << "ms" << std::endl;

  std::cout << "qm: ";
  {
    for (auto q : data_->q)
    {
      std::cout << q << " ";
    }
  }
  std::cout << std::endl;

  return ret;
}

void ArmSDK::runSendReceive()
{
  // TODO: 只有位置模式
  while (isRunning_)
  {
    int ret = doSendReceive();
    // usleep(1000);
  }
}

void ArmSDK::runSendReceiveThread()
{
  isRunning_ = true;
  thread_ = std::thread([this]
                        { runSendReceive(); }); // 使用 Lambda 表达式捕获 this
}

int ArmSDK::pause()
{
  isRunning_ = false;
  if (thread_.joinable())
  {
    thread_.join();
  }
  usleep(100000);
}

int ArmSDK::stop()
{
  VCI_CAN_OBJ send[numJoints_];
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 1;
    send[i].Data[0] = 0x02;
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }
  return 1;
}

int ArmSDK::setId(int id, int setID)
{
  // 暂时不用
  VCI_CAN_OBJ send[1];
  send[0].SendType = 0;
  send[0].RemoteFlag = 0;
  send[0].ExternFlag = 0;
  send[0].ID = id;
  send[0].DataLen = 5;
  send[0].Data[0] = 0x2e;

  uint8_t u8[4];
  int32_t int_x = static_cast<int32_t>(setID);
  for (int i = 0; i < 4; ++i)
  {
    u8[i] = (int_x >> (i * 8)) & 0xFF;
  }
  for (size_t j = 1; j < 5; j++)
  {
    send[0].Data[j] = u8[j - 1];
  }
  int ret = VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);

  send[0].ID = setID;
  send[0].Data[0] = 0x0e;
  // ret=VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);
  // ret=VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1);

  printf("Set ID from %d to %d\n", id, setID);

  save();
  return ret;
}

int ArmSDK::sendKp()
{
  VCI_CAN_OBJ send[numJoints_];
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 5;
    send[i].Data[0] = 0x2b;

    uint8_t u8[4];

    {
      std::lock_guard<std::mutex> lock(mtx_);
      convertFloatToHexBytes(cmd_->kpDes[i], u8); //
    }

    // swapEndianness(u8);
    for (size_t j = 1; j < 5; j++)
    {
      send[i].Data[j] = u8[j - 1];
    }
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }
  return 1;
}

int ArmSDK::sendKd()
{
  VCI_CAN_OBJ send[numJoints_];
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 5;
    send[i].Data[0] = 0x2d;

    uint8_t u8[4];
    {
      std::lock_guard<std::mutex> lock(mtx_);
      convertFloatToHexBytes(cmd_->kdDes[i], u8); //
    }

    // swapEndianness(u8);
    for (size_t j = 1; j < 5; j++)
    {
      send[i].Data[j] = u8[j - 1];
    }
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }
  return 1;
}

int ArmSDK::sendQ()
{
  VCI_CAN_OBJ send[numJoints_];
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 5;
    send[i].Data[0] = 0x44;

    uint8_t u8[4];

    {
      std::lock_guard<std::mutex> lock(mtx_);
      convertPositionToHexBytes(cmd_->qDes[i], u8);
      // convertPositionToHexBytes(-M_PI,u8); // cmd是弧度
    }
    for (size_t j = 1; j < 5; j++)
    {
      send[i].Data[j] = u8[j - 1];
    }
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }
  return 1;
}

int ArmSDK::sendQd()
{
  VCI_CAN_OBJ send[numJoints_];
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 5;
    send[i].Data[0] = 0x43; //

    uint8_t u8[4];
    {
      std::lock_guard<std::mutex> lock(mtx_);
      convertVelocityToHexBytes(cmd_->qdDes[i], u8); // 弧度制
    }

    // swapEndianness(u8);
    for (size_t j = 1; j < 5; j++)
    {
      send[i].Data[j] = u8[j - 1];
    }
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }
  return 1;
}

int ArmSDK::sendTau()
{
  VCI_CAN_OBJ send[numJoints_];
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 5;
    send[i].Data[0] = 0x42;

    uint8_t u8[4];

    {
      std::lock_guard<std::mutex> lock(mtx_);
      convertTorqueToHexBytes(cmd_->qDes[i], u8); // TODO：力，底层写转电流
    }

    // swapEndianness(u8);
    for (size_t j = 1; j < 5; j++)
    {
      send[i].Data[j] = u8[j - 1];
    }
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }
  return 1;
}

int ArmSDK::setModePosition()
{
  printf("Set Mode: Position\n");

  // 获取
  VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 1;
    send[i].Data[0] = 0x08;
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }
  usleep(500);
  // std::cout<<"ret: "<<ret<<std::endl;

  VCI_CAN_OBJ rec[3000]; // 接收缓存，设为3000为佳。
  int reclen = 0;
  // int32_t int_q[numJoints_] = { 0 };
  std::map<int, std::array<uint8_t, 4>> u8All;
  // 补全键值对
  for (int i = 0; i < numJoints_; ++i)
  {
    if (u8All.find(i) == u8All.end())
    { // 如果键不存在
      std::array<uint8_t, 4> u8Default = {0, 0, 0, 0};
      u8All[i] = u8Default;
    }
  }

  // while (!reclen) {
  if ((reclen = VCI_Receive(VCI_USBCAN2, 0, canID_, rec, 3000, 100)) >
      0) // 调用接收函数，如果有数据，进行数据处理显示。
  {
    for (int j = 0; j < reclen; j++)
    {
      printf("CAN%d RX ID:0x%08X", canID_, rec[j].ID); // ID
      if (rec[j].ExternFlag == 0)
        printf(" Standard "); // 帧格式：标准帧
      if (rec[j].ExternFlag == 1)
        printf(" Extend   "); // 帧格式：扩展帧
      if (rec[j].RemoteFlag == 0)
        printf(" Data   "); // 帧类型：数据帧
      if (rec[j].RemoteFlag == 1)
        printf(" Remote ");                 // 帧类型：远程帧
      printf("DLC:0x%02X", rec[j].DataLen); // 帧长度
      printf(" data:0x");                   // 数据
      for (int i = 0; i < rec[j].DataLen; i++)
      {
        printf(" %02X", rec[j].Data[i]);
      }

      if (rec[j].DataLen == 5)
      {
        std::array<uint8_t, 4> u8;
        std::memcpy(u8.data(), rec[j].Data + 1, 4);
        u8All[rec[j].ID - 1] = u8;
      }

      // 这句话是低位在后，因为电机协议收到的第一个是低位
      // int_q[rec[j].ID - 1] = (int32_t(rec[j].Data[1])) | (int32_t(rec[j].Data[2]) << 8) |
      // 					   (int32_t(rec[j].Data[3]) << 16) | (int32_t(rec[j].Data[4]) << 24);

      // 这句话是低位在前，是协议默认，后面就不用交换大小端
      // int_q[rec[j].ID - 1] = (int32_t(rec[j].Data[1]) << 24) | (int32_t(rec[j].Data[2]) << 16) |
      // 					   (int32_t(rec[j].Data[3]) << 8) | (int32_t(rec[j].Data[4]));

      printf(" TimeStamp:0x%08X", rec[j].TimeStamp); // 时间标识。
      printf("\n");
    }
  }

  // std::cout << "reclen: " << reclen << std::endl;
  // }

  // 设置电机运行模式为位置模式 ，并设置目标位置
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 5;
    send[i].Data[0] = 0x1e;

    //   std::cout << "int_q[" << i << "]:" << int_q[i] << std::endl;

    uint8_t u8[4];
    std::copy(u8All[i].data(), u8All[i].data() + u8All[i].size(), u8);

    //   for (int j = 0; j < 4; ++j)
    //   {
    // 	u8[j] = (int_q[i] >> (j * 8)) & 0xFF;
    //   }
    printHexBytes(u8);
    //   swapEndianness(u8);

    // data_->q[rec[j].ID - 1] = convertHexBytesToPosition(bytes);

    // printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
    // printf("\n");

    std::cout << "Initial q[" << i << "]  =  " << convertHexBytesToPosition(u8) << std::endl;

    for (size_t j = 1; j < 5; j++)
    {
      send[i].Data[j] = u8[j - 1]; // 0
    }

    // !!!! 避免位置跳变，将cmd的值赋值为当前值
    // cmd_->qDes[i] = convertHexBytesToPosition(u8);
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }

  usleep(100000);
  printf("Set Mode Success\n");
  return 1;
}

int ArmSDK::setModeVelocity()
{
  // 与设置位置不同，不用读取，直接设为0
  printf("Set Mode: Velocity\n");

  VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 5;
    send[i].Data[0] = 0x1d;
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }

  usleep(100000);
  printf("Set Mode Success\n");
  return 1;
}

int ArmSDK::setModeTorque()
{
  // 与设置位置不同，不用读取，直接设为0
  printf("Set Mode: Torque\n");

  VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 5;
    send[i].Data[0] = 0x1c;
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }

  usleep(100000);
  printf("Set Mode Success\n"); // TODO： 逻辑修改
  return 1;
}

int ArmSDK::setMode(const std::string &mode)
{
  // TODO: 目前只有位置控制
  // stop(); // TODO： 切换是否需要停止
  pause(); // TODO: 等待发送结束
  mode_ = mode;

  int ret;
  if (mode == "position")
  {
    ret = setModePosition();
  }
  else if (mode == "velocity")
  {
    ret = setModeVelocity();
  }
  else if (mode == "force")
  {
    ret = setModeTorque();
  }
  else
  {
    // 处理未知模式的情况
  }
  return ret;
}

int ArmSDK::resetToDefault()
{

  VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 1;
    send[i].Data[0] = 0x0f;
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }

  usleep(100000);
  return 1;
}

int ArmSDK::clearError()
{

  VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 1;
    send[i].Data[0] = 0x0b;
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }

  usleep(100000);
  return 1;
}

int ArmSDK::setMinMaxPos(const Vec7 &qmin, const Vec7 &qmax)
{

  VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send

  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 5;
    send[i].Data[0] = 0x27;

    uint8_t u8[4];
    convertFloatToHexBytes(convertRadiansToDegrees((qmin(i)) / 360) * 51 * 65536,
                           u8); // 忘记角度转换下发位置
    printHexBytes(u8);
    //	swapEndianness(u8);//此处不需要转换大小端

    for (size_t b = 0; b < 4; b++)
    {
      send[i].Data[b + 1] = u8[b];
    }
  }

  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }
  ////////////////////////////////////////////////////////////////////////////////////////
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 5;
    send[i].Data[0] = 0x26;

    uint8_t u8[4];
    convertFloatToHexBytes(convertRadiansToDegrees((qmax(i)) / 360) * 51 * 65536,
                           u8); // 忘记角度转换下发位置
    printHexBytes(u8);
    //	swapEndianness(u8);//此处不需要转换大小端
    for (size_t b = 0; b < 4; b++)
    {
      send[i].Data[b + 1] = u8[b];
    }
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }
  return 1;
}

int ArmSDK::setMinMaxVel(const Vec7 &vmin, const Vec7 &vmax)
{
  // return 0;
}

int ArmSDK::setMinMaxAcc(const Vec7 &amin, const Vec7 &amax)
{
  // return 0;
}

int ArmSDK::clearCmd()
{
  if (cmd_)
  {
    for (int i = 0; i < NUM; ++i)
    {
      cmd_->qDes[i] = 0.0f;
      cmd_->qdDes[i] = 0.0f;
      cmd_->tauDes[i] = 0.0f;
      cmd_->kpDes[i] = 800.0f;
      cmd_->kdDes[i] = 1.0f;
    }
  }
  return 1;
}

int ArmSDK::clearData()
{
  if (data_)
  {
    for (int i = 0; i < NUM; ++i)
    {
      data_->q[i] = 0.0f;
      data_->qd[i] = 0.0f;
      data_->tau[i] = 0.0f;
    }
  }
  return 1;
}

int ArmSDK::setQ(const Vec7 &q)
{
  std::lock_guard<std::mutex> lock(mtx_);

  for (int i = 0; i < NUM; i++)
  {
    cmd_->qDes[i] = q[i];
  }
  return 1;
}

int ArmSDK::setQd(const Vec7 &qd)
{
  std::lock_guard<std::mutex> lock(mtx_);

  for (int i = 0; i < NUM; i++)
  {
    cmd_->qdDes[i] = qd[i];
  }
  return 1;
}

int ArmSDK::setTau(const Vec7 &tau)
{
  // return 0;
}

int ArmSDK::setKp(const Vec7 &kp)
{
  std::lock_guard<std::mutex> lock(mtx_);

  for (int i = 0; i < NUM; i++)
  {
    cmd_->qdDes[i] = kp[i];
  }
  return 1;
}

int ArmSDK::setKd(const Vec7 &kd)
{
  std::lock_guard<std::mutex> lock(mtx_);

  for (int i = 0; i < NUM; i++)
  {
    cmd_->qdDes[i] = kd[i];
  }
  return 1;
}

// int ArmSDK::getQ(double (&qm)[7])
int ArmSDK::getQ(Vec7 &qm)
{
  if (isRunning_)
  {
    // 返回DATA中的值
    // Eigen::Map<Vec7> vector(qm, 7);
    // qm = data_->q
    for (int i = 0; i < 7; ++i)
    {
      qm[i] = data_->q[i];
    }
  }
  else
  {
    // 如果当前没有使用线程，则通过发08实现获取
  }

  return 1; // 返回空的 vector
}

int ArmSDK::getQd(Vec7 &qdm)
{
  if (isRunning_)
  {
    // 返回DATA中的值
  }
  else
  {
    // 如果当前没有使用线程，则通过发xx实现获取
  }

  return 1; // 返回空的 vector
}

int ArmSDK::getTau(Vec7 &qm)
{
  if (isRunning_)
  {
    // 返回DATA中的值
  }
  else
  {
    // 如果当前没有使用线程，则通过发xx实现获取
  }

  return 1; // 返回空的 vector
}

int ArmSDK::getKp(Vec7 &kpm)
{
  // 线程里是不随时获取Kp的，因此只能手动单次获取

  VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 1;
    send[i].Data[0] = 0x12;
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }
  usleep(1000);
  ////////////////////////////////////////////////////////////////////////////////////////////
  VCI_CAN_OBJ rec[3000]; // 接收缓存，设为3000为佳。
  int reclen = 0;

  // int32_t int_q[numJoints_] = {0};
  if ((reclen = VCI_Receive(VCI_USBCAN2, 0, canID_, rec, 3000, 100)) >
      0) // 调用接收函数，如果有数据，进行数据处理显示。
  {
    for (int j = 0; j < reclen; j++)
    {
      printf("CAN%d RX ID:0x%08X", canID_, rec[j].ID); // ID
      if (rec[j].ExternFlag == 0)
        printf(" Standard "); // 帧格式：标准帧
      if (rec[j].ExternFlag == 1)
        printf(" Extend   "); // 帧格式：扩展帧
      if (rec[j].RemoteFlag == 0)
        printf(" Data   "); // 帧类型：数据帧
      if (rec[j].RemoteFlag == 1)
        printf(" Remote ");                 // 帧类型：远程帧
      printf("DLC:0x%02X", rec[j].DataLen); // 帧长度
      printf(" data:0x");                   // 数据
      for (int i = 0; i < rec[j].DataLen; i++)
      {
        printf(" %02X", rec[j].Data[i]);
      }

      printf(" !TimeStamp:0x%08X\n", rec[j].TimeStamp); // 时间标识。

      printf("\n");

      uint8_t bytes[4];
      std::copy(rec[j].Data + 1, rec[j].Data + 5, bytes);
      kpm[rec[j].ID - 1] = convertHexBytesToFloat(bytes);
    }
  }
  return 1;
}

int ArmSDK::getKd(Vec7 &kdm)
{
  // 线程里是不随时获取Kd的，因此只能手动单次获取

  VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 1;
    send[i].Data[0] = 0x13;
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  VCI_CAN_OBJ rec[3000]; // 接收缓存，设为3000为佳。
  int reclen = 0;
  int32_t int_q[numJoints_] = {0};
  if ((reclen = VCI_Receive(VCI_USBCAN2, 0, canID_, rec, 3000, 100)) >
      0) // 调用接收函数，如果有数据，进行数据处理显示。
  {
    for (int j = 0; j < reclen; j++)
    {
      printf("CAN%d RX ID:0x%08X", canID_, rec[j].ID); // ID
      if (rec[j].ExternFlag == 0)
        printf(" Standard "); // 帧格式：标准帧
      if (rec[j].ExternFlag == 1)
        printf(" Extend   "); // 帧格式：扩展帧
      if (rec[j].RemoteFlag == 0)
        printf(" Data   "); // 帧类型：数据帧
      if (rec[j].RemoteFlag == 1)
        printf(" Remote ");                 // 帧类型：远程帧
      printf("DLC:0x%02X", rec[j].DataLen); // 帧长度
      printf(" data:0x");                   // 数据
      for (int i = 0; i < rec[j].DataLen; i++)
      {
        printf(" %02X", rec[j].Data[i]);
      }

      int_q[rec[j].ID - 1] = (int32_t(rec[j].Data[1])) | (int32_t(rec[j].Data[2]) << 8) |
                             (int32_t(rec[j].Data[3]) << 16) | (int32_t(rec[j].Data[4]) << 24);

      printf(" !TimeStamp:0x%08X\n", rec[j].TimeStamp); // 时间标识。
      printf("位置环kd%08X\n", int_q[rec[j].ID - 1]);   // 时间标识。

      printf("\n");
    }
  }
  return 1;
}

void ArmSDK::convertPositionToHexBytes(float p, uint8_t (&bytes)[4])
{
  float p_convert = convertRadiansToDegrees(p) / 360 * 51 * 65536;
  convertFloatToHexBytes(p_convert, bytes);
}

void ArmSDK::convertVelocityToHexBytes(float v, uint8_t (&bytes)[4])
{
  convertFloatToHexBytes((convertRadiansToDegrees(v) * 51 * 100) / 360, bytes); // Check
}

void ArmSDK::convertTorqueToHexBytes(float t, uint8_t (&bytes)[4])
{
  // 力转电流
  float cur = t; // Check
  convertFloatToHexBytes(cur, bytes);
}

float ArmSDK::convertHexBytesToPosition(const uint8_t (&bytes)[4])
{
  float p_convert = convertHexBytesToFloat(bytes) * 360 / (51 * 65536);
  return convertDegreesToRadians(p_convert);
}

float ArmSDK::convertHexBytesToVelocity(const uint8_t (&bytes)[4])
{
  float p_convert = convertHexBytesToFloat(bytes) * 360 / (51 * 65536); // Check
}

float ArmSDK::convertHexBytesToTorque(const uint8_t (&bytes)[4])
{
  // Check
}

void ArmSDK::convertFloatToHexBytes(float x, uint8_t (&bytes)[4])
{
  // 默认Little-Endian, 转换后低位在前,4位
  int32_t int_x = static_cast<int32_t>(x);
  for (int i = 0; i < 4; ++i)
  {
    bytes[i] = (int_x >> (i * 8)) & 0xFF;
  }
  //   std::cout << "convertFloatToHexBytes int:" << int_x << std::endl;
  // swapEndianness(bytes);
}

float ArmSDK::convertHexBytesToFloat(const uint8_t (&bytes)[4])
{
  // 默认Little-Endian, 输入低位在前的数
  int32_t int_x = 0;
  for (int i = 0; i < 4; ++i)
  {
    int_x |= static_cast<int32_t>(bytes[i]) << (i * 8);
  }
  // return *reinterpret_cast<float*>(&int_x);
  // return (float)int_x;
  //   std::cout << std::dec << "convertHexBytesToFloat int:" << int_x << std::endl;
  //   std::cout << std::dec << "convertHexBytesToFloat float:" << static_cast<float>(int_x) << std::endl;
  return static_cast<float>(int_x);
}

double ArmSDK::convertDegreesToRadians(double degrees)
{
  return degrees * M_PI / 180.0;
}

double ArmSDK::convertRadiansToDegrees(double radians)
{
  return radians * 180.0 / M_PI;
}

void ArmSDK::swapEndianness(uint8_t (&bytes)[4])
{
  std::swap(bytes[0], bytes[3]);
  std::swap(bytes[1], bytes[2]);
}

void ArmSDK::printHexBytes(const uint8_t (&bytes)[4])
{
  std::cout << "Hex representation: ";
  // for (int i = 3; i >= 0; --i) {
  //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytes[i]) << " ";
  // }
  for (int i = 0; i <= 3; ++i)
  {
    std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytes[i]) << " ";
  }
  std::cout << std::endl;
}

int ArmSDK::setZero()
{
  // 获取position
  VCI_CAN_OBJ send[numJoints_]; // 使用局部变量send
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 1;
    send[i].Data[0] = 0x08;
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }
  usleep(500);

  VCI_CAN_OBJ rec[3000]; // 接收缓存，设为3000为佳。
  int reclen = 0;
  // int32_t int_q[numJoints_] = { 0 };
  // std::map<int, std::array<uint8_t, 4>> u8Position;
  // // 补全键值对
  // for (int i = 0; i < numJoints_; ++i)
  // {
  //   if (u8Position.find(i) == u8Position.end())
  //   { // 如果键不存在
  //     std::array<uint8_t, 4> u8Default = {0, 0, 0, 0};
  //     u8Position[i] = u8Default;
  //   }
  // }
  int32_t position[numJoints_] = {0};

  if ((reclen = VCI_Receive(VCI_USBCAN2, 0, canID_, rec, 3000, 100)) >
      0) // 调用接收函数，如果有数据，进行数据处理显示。
  {
    for (int j = 0; j < reclen; j++)
    {
      printf("CAN%d RX ID:0x%08X", canID_, rec[j].ID); // ID
      if (rec[j].ExternFlag == 0)
        printf(" Standard "); // 帧格式：标准帧
      if (rec[j].ExternFlag == 1)
        printf(" Extend   "); // 帧格式：扩展帧
      if (rec[j].RemoteFlag == 0)
        printf(" Data   "); // 帧类型：数据帧
      if (rec[j].RemoteFlag == 1)
        printf(" Remote ");                 // 帧类型：远程帧
      printf("DLC:0x%02X", rec[j].DataLen); // 帧长度
      printf(" data:0x");                   // 数据
      for (int i = 0; i < rec[j].DataLen; i++)
      {
        printf(" %02X", rec[j].Data[i]);
      }
      printf(" TimeStamp:0x%08X", rec[j].TimeStamp); // 时间标识。
      printf("\n");

      if (rec[j].DataLen == 5)
      {
        // std::array<uint8_t, 4> u8;
        // std::memcpy(u8.data(), rec[j].Data + 1, 4);
        // u8Position[rec[j].ID - 1] = u8;
        // std::cout << "q now [" << rec[j].ID - 1 << "]: " << convertHexBytesToPosition(reinterpret_cast<uint8_t(&)[4]>(u8)) << std::endl;
        uint8_t u8[4];
        std::copy(rec[j].Data + 1, rec[j].Data + 5, u8);
        position[rec[j].ID - 1] = (int32_t(rec[j].Data[1])) | (int32_t(rec[j].Data[2]) << 8) |
                                  (int32_t(rec[j].Data[3]) << 16) | (int32_t(rec[j].Data[4]) << 24);
        std::cout << "q now [" << rec[j].ID - 1 << "]: " << convertHexBytesToPosition(u8) << std::endl;
      }
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 1;
    send[i].Data[0] = 0x54;
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }
  usleep(500);

  // std::map<int, std::array<uint8_t, 4>> u8Bias;
  // // 补全键值对
  // for (int i = 0; i < numJoints_; ++i)
  // {
  //   if (u8Bias.find(i) == u8Bias.end())
  //   { // 如果键不存在
  //     std::array<uint8_t, 4> u8Default = {0, 0, 0, 0};
  //     u8Bias[i] = u8Default;
  //   }
  // }
  int32_t bias[numJoints_] = {0};

  if ((reclen = VCI_Receive(VCI_USBCAN2, 0, canID_, rec, 3000, 100)) >
      0) // 调用接收函数，如果有数据，进行数据处理显示。
  {
    for (int j = 0; j < reclen; j++)
    {
      printf("CAN%d RX ID:0x%08X", canID_, rec[j].ID); // ID
      if (rec[j].ExternFlag == 0)
        printf(" Standard "); // 帧格式：标准帧
      if (rec[j].ExternFlag == 1)
        printf(" Extend   "); // 帧格式：扩展帧
      if (rec[j].RemoteFlag == 0)
        printf(" Data   "); // 帧类型：数据帧
      if (rec[j].RemoteFlag == 1)
        printf(" Remote ");                 // 帧类型：远程帧
      printf("DLC:0x%02X", rec[j].DataLen); // 帧长度
      printf(" data:0x");                   // 数据
      for (int i = 0; i < rec[j].DataLen; i++)
      {
        printf(" %02X", rec[j].Data[i]);
      }
      printf(" TimeStamp:0x%08X", rec[j].TimeStamp); // 时间标识。
      printf("\n");

      if (rec[j].DataLen == 5)
      {
        // std::array<uint8_t, 4> u8;
        // std::memcpy(u8.data(), rec[j].Data + 1, 4);
        // u8Bias[rec[j].ID - 1] = u8;
        // std::cout << " bias now [" << rec[j].ID - 1 << "]: " << convertHexBytesToPosition(reinterpret_cast<uint8_t(&)[4]>(u8)) << std::endl;
        uint8_t u8[4];
        std::copy(rec[j].Data + 1, rec[j].Data + 5, u8);
        bias[rec[j].ID - 1] = (int32_t(rec[j].Data[1])) | (int32_t(rec[j].Data[2]) << 8) |
                              (int32_t(rec[j].Data[3]) << 16) | (int32_t(rec[j].Data[4]) << 24);
        std::cout << "bias now [" << rec[j].ID - 1 << "]: " << convertHexBytesToPosition(u8) << std::endl;
      }
    }
  }

  /////////////////////////////////////////////////////////////////////
  // std::map<int, std::array<uint8_t, 4>> u8BiasNew;
  int32_t biasNew[numJoints_] = {0};

  for (size_t i = 0; i < numJoints_; i++)
  {
    biasNew[i] = position[i] + bias[i];
    // for (size_t j = 0; j < 4; j++)
    // {
    //   int32_t intPosition = (int32_t(u8Position[i][0])) | (int32_t(u8Position[i][1]) << 8) |
    //                         (int32_t(u8Position[i][2]) << 16) | (int32_t(u8Position[i][3]) << 24);
    //   int32_t intBias = (int32_t(u8Bias[i][0])) | (int32_t(u8Bias[i][1]) << 8) |
    //                     (int32_t(u8Bias[i][2]) << 16) | (int32_t(u8Bias[i][3]) << 24);
    //   int32_t intBiasNew = intPosition + intBias;

    //   u8BiasNew[i][j] = u8Position[i][j] + u8Bias[i][j];
    // }
  }

  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 5;
    send[i].Data[0] = 0x53;

    uint8_t u8[4];
    convertFloatToHexBytes(biasNew[i], u8);
    // swapEndianness(u8);//加上大小端反了, 感觉得加上
    for (size_t t = 0; t < 4; t++)
    {
      send[i].Data[t + 1] = u8[t];
    }

    std::cout << " bias new [" << i << "]: " << convertHexBytesToPosition(u8) << std::endl;

    for (size_t j = 1; j < 5; j++)
    {
      send[i].Data[j] = u8[j - 1]; // 0
    }
  }
  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }

  usleep(100000);
  save();
  usleep(100000);
  printf("Set Bias Success\n");

  return 1;
}

int ArmSDK::save()
{

  VCI_CAN_OBJ send[numJoints_];
  for (int i = 0; i < numJoints_; i++)
  {
    int id = i + 1;
    send[i].SendType = 0;
    send[i].RemoteFlag = 0;
    send[i].ExternFlag = 0;
    send[i].ID = id;
    send[i].DataLen = 1;
    send[i].Data[0] = 0x0e;
  }

  if (VCI_Transmit(VCI_USBCAN2, 0, canID_, send, numJoints_) != 1)
  {
    // return 0; // 发送失败，返回0
  }
  return 1;
}
