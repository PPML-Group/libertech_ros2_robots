#ifndef ARM_DRIVER_H
#define ARM_DRIVER_H

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

#define NUM 14

union to_uint32 {
    uint8_t u8[4];
    uint32_t locale_int;
};

struct Command {
  float qDes[NUM];
  float qdDes[NUM];
  float tauDes[NUM];
  float kpDes[NUM];
  float kdDes[NUM];
  int mode[NUM]; // 用于切换模式


};

struct Data {
  float q[NUM];
  float qd[NUM];
  float tau[NUM];
  int mode[NUM];
  int error[NUM];
};

class ArmDriver {
public:
    ArmDriver();  // Constructor
    ~ArmDriver();

    // 运行
    int init();
    // VCI_CAN_OBJ* createCANFrames();
    int doSendReceive();
    void runSendReceive();
    void runSendReceiveThread();
    int pause();// 只停止runSendReceiveThread
    int stop();// 急停

    // 基于cmd进行发送
    int sendKp();
    int sendKd();
    int sendQ();
    int sendQd();
    int sendTau();

    // 设置模式
    int setModePosition();
    int setModeVelocity();
    int setModeTorque();
    int setMode(const std::string& mode);
    int setHome();

    // 标定与辅助功能
    int setId(int id,int setID);
    int setZero();
    int save();
  //  int setOffset();
   
int test() ;

    // 恢复与清除错误
    int resetToDefault();
    int clearError();

    // 设置最大最小参数
    int setMinMaxPos(const Vec14& qmin, const Vec14& qmax);
    int setMinMaxVel(const Vec14& vmin, const Vec14& vmax);
    int setMinMaxAcc(const Vec14& amin, const Vec14& amax);
  
    // 设置关节
    int clearCmd();
    int clearData();

    int setQ(const Vec14& q);
    int setQd(const Vec14& qd);
    int setTau(const Vec14& tau);
    int setKp(const Vec14& kp);
    int setKd(const Vec14& kd);
    int setCmd();

    // 读取关节
    int getQ(Vec14& qm);
    int getQd(Vec14& qdm);
    int getTau(Vec14& taum);
    int getKp(Vec14& kpm);//位置环
    int getKd(Vec14& kpm);

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
    int numCAN_; // 两条CAN
    // VCI_CAN_OBJ *send_;
    // VCI_CAN_OBJ rec_[3000];
    Command *cmd_ = nullptr;
    Data *data_ = nullptr; // 存储关节角度的私有变量
};

#endif /* ARM_DRIVER_H */
