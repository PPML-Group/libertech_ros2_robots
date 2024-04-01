#include "libertech_arm_driver/ArmSDK.hpp"

int main()
{
  ArmSDK arm(0);
  arm.init();
  sleep(2);

  // 零位
  arm.setZero();

  arm.save();

  return 0;
}


// #include "libertech_arm_driver/arm.hpp"

// int main()
// {
//   arm arm;
//   arm.init();
//   sleep(2);

//   // 零位
//   arm.setZero();

//   arm.save();

//   return 0;
// }