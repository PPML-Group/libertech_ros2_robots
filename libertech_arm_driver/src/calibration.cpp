// #include "libertech_arm_driver/ArmSDK.hpp"

// int main()
// {
//   ArmSDK arm(0, -1, false);
//   arm.init();
//   sleep(2);

//   // 零位
//   arm.setZero(0);

//   arm.save(0);

//   return 0;
// }

#include "libertech_arm_driver/ArmSDK.hpp"

int main()
{
  ArmSDK arm(-1, 1, false);
  arm.init();
  sleep(2);

  // 零位
  arm.setZero(1);

  arm.save(1);

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