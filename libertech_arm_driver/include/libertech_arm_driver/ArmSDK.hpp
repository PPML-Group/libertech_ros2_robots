#ifndef ARM_SDK_H
#define ARM_SDK_H

#include <vector>
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"
#include <thread>
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <string>
#include <iomanip>
#include "math/mathTypes.h"
#include <shared_mutex>
#include <mutex>
#include <map>

#define NUM 7

union to_uint32
{
  uint8_t u8[4];
  uint32_t locale_int;
};

struct Command
{
  float qDes[NUM];
  float qdDes[NUM];
  float tauDes[NUM];
  float kpDes[NUM];
  float kdDes[NUM];
  float kiDes[NUM];
  int mode[NUM]; // 用于切换模式
};

struct Data
{
  float q[NUM];
  float qd[NUM];
  float tau[NUM];
  int mode[NUM];
  int error[NUM];
};

class ArmSDK
{
public:
  // ArmSDK(int canID_);  // Constructor
  ArmSDK(int canIDRight, int canIDLeft, bool useGripper); // Constructor
  ~ArmSDK();

  // 运行
  int init();
  // VCI_CAN_OBJ* createCANFrames();
  int doSendReceive();
  void runSendReceive();
  void runSendReceiveThread();
  int pause();             // 只停止runSendReceiveThread
  int stop(const int cid); // 急停

  // 基于cmd进行发送
  int sendKp(const int cid);
  int sendKd(const int cid);
  int sendKi(const int cid);
  int sendQ(const int cid);
  int sendQd(const int cid);
  int sendTau(const int cid);

  // 设置模式
  int setModePosition(const int cid);
  int setModeVelocity(const int cid);
  int setModeTorque(const int cid);
  int setMode(const std::string &mode, const int cid);
  int setHome(const int cid);

  // 标定与辅助功能
  int setId(int id, int setID);
  int setZero(const int cid);
  int save(const int cid);

  // 恢复与清除错误
  int resetToDefault(const int cid);
  int clearError(const int cid);

  // 设置最大最小参数
  int setMinMaxPos(const Vec7 &qmin, const Vec7 &qmax, const int cid);
  int setMinMaxVel(const Vec7 &vmin, const Vec7 &vmax, const int cid);
  int setMinMaxAcc(const Vec7 &amin, const Vec7 &amax, const int cid);

  // 设置关节
  int clearCmd();
  int clearData();

  int setQ(const Vec7 &q, const int cid);
  int setQd(const Vec7 &qd, const int cid);
  int setTau(const Vec7 &tau, const int cid);
  int setKp(const Vec7 &kp, const int cid);
  int setKd(const Vec7 &kd, const int cid);
  int setKi(const Vec7 &ki, const int cid);
  int setCmd();

  // 读取关节
  int getQ(Vec7 &qm, const int cid);
  int getQd(Vec7 &qdm, const int cid);
  int getTau(Vec7 &taum, const int cid);
  int getKp(Vec7 &kpm, const int cid); // 位置环
  int getKd(Vec7 &kdm, const int cid); // 位置环
  int getKi(Vec7 &kim, const int cid); // 位置环
  int getKx(Vec7 &kxm, const BYTE code, const int cid); // 任意参数

  // 转换
  void convertPositionToHexBytes(float p, uint8_t (&bytes)[4]);
  void convertVelocityToHexBytes(float v, uint8_t (&bytes)[4]);
  void convertTorqueToHexBytes(float t, uint8_t (&bytes)[4]);
  float convertHexBytesToPosition(const uint8_t (&bytes)[4]);
  float convertHexBytesToVelocity(const uint8_t (&bytes)[4]);
  float convertHexBytesToTorque(const uint8_t (&bytes)[4]);

  void convertFloatToHexBytes(float x, uint8_t (&bytes)[4]);
  float convertHexBytesToFloat(const uint8_t (&bytes)[4]);
  void swapEndianness(uint8_t (&bytes)[4]);
  void printHexBytes(const uint8_t (&bytes)[4]);

  double convertDegreesToRadians(double degrees);
  double convertRadiansToDegrees(double radians);

private:
  // USB2CAN
  VCI_BOARD_INFO vciPInfo_;
  VCI_BOARD_INFO vciPInfoAll_[50];
  VCI_INIT_CONFIG vciConfig_;
  int vciNum_;

  // ARM
  std::thread thread_;
  std::atomic<bool> isRunning_;
  std::string mode_;

  std::mutex mtx_; // 创建一个共享互斥锁

  int numJoints_; // 电机数量
  // int numCAN_;     // 两条CAN
  bool hasRight_;
  bool hasLeft_;
  bool useGripper_;
  std::vector<std::pair<int, bool>> dictID_;
  // VCI_CAN_OBJ *send_;
  // VCI_CAN_OBJ rec_[3000];
  // Command* cmd_ = nullptr;
  // Data* data_ = nullptr;  // 存储关节角度的私有变量
  std::vector<Command *> cmd_;
  std::vector<Data *> data_;
};

#endif /* ARM_SDK_H */
