#include "libertech_arm_driver/ArmSDK.hpp"
#include <fstream>
#include <numeric>

std::vector<Vec6> planQuinticPolynomialTrajectory(const VecX &q0, const VecX &v0, const VecX &a0, const VecX &qf,
                                                  const VecX &vf, const VecX &af, double T)
{
  int DOF = q0.size(); // 关节自由度
  std::vector<Vec6> allCoeffs;

  for (int i = 0; i < DOF; ++i)
  {
    Mat6 M;
    Vec6 states, coeffs;

    M << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1, T, pow(T, 2), pow(T, 3), pow(T, 4), pow(T, 5), 0, 1,
        2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4), 0, 0, 2, 6 * T, 12 * pow(T, 2), 20 * pow(T, 3);

    states << q0(i), v0(i), a0(i), qf(i), vf(i), af(i);
    coeffs = M.inverse() * states;
    allCoeffs.push_back(coeffs);
  }
  return allCoeffs;
}

int main()
{
  int num_joints = 7;
  Vec7 q0, v0, a0, qf, vf, af;
  q0.setZero();
  v0.setZero();
  a0.setZero();

  qf << 0, -0, 0, 0, 0, 0, 0;
  // qf << 15, -15, 10, 30, 15, 0, 10;
  // qf <<-15, 15, -10, -30, 15, 0, 10;

  qf = qf * M_PI / 180;
  vf.setZero();
  af.setZero();
  double T = 1;
  double dt = 0.01; // 0.001 0.005

  std::vector<Vec6> coeffs = planQuinticPolynomialTrajectory(q0, v0, a0, qf, vf, af, T);

  std::vector<Vec7> q_traj;

  for (double t = 0; t <= T + 1e-6; t += dt)
  {
    // std::cout << t << std::endl;
    Vec7 q;
    for (int i = 0; i < num_joints; ++i)
    {
      q[i] = coeffs[i](0) + coeffs[i](1) * t + coeffs[i](2) * pow(t, 2) + coeffs[i](3) * pow(t, 3) +
             coeffs[i](4) * pow(t, 4) + coeffs[i](5) * pow(t, 5);
    }
    q_traj.push_back(q);
  }

  std::ofstream outfile("q_traj.txt");
  if (outfile.is_open())
  {
    outfile << std::fixed << std::setprecision(6); // 设置输出格式，保留2位小数
    // std::cout << q_traj.size() << std::endl;
    for (size_t i = 0; i < q_traj.size(); ++i)
    {
      for (int j = 0; j < num_joints; ++j)
      {
        outfile << std::setw(10) << q_traj[i][j]; // 设置字段宽度为8个字符
        if (j < num_joints - 1)
        {
          outfile << " ";
        }
      }
      outfile << std::endl;
    }
    outfile.close();
  }
  else
  {
    std::cerr << "Unable to open file for writing!" << std::endl;
    return 1;
  }
  ////////////////////////////////////////////////////////////////////
  // ArmSDK arm(0, -1, false);
  ArmSDK arm(0, 1, false);

  float pos = 360 * M_PI / 180;
  std::cout << "pos:" << pos << std::endl;

  uint8_t pos_u8[4];
  arm.convertPositionToHexBytes(pos, pos_u8);
  arm.printHexBytes(pos_u8);

  float pos_convert_inv = arm.convertHexBytesToPosition(pos_u8);
  std::cout << "pos_convert_inv:" << pos_convert_inv << std::endl;

  arm.init();
  usleep(1000);

  ///////////////////////////////////////

  // kp kd
  Vec7 kx;
  for (int i = 0; i < 2; i++)
  {
    arm.getKp(kx, 0);
    std::cout << "kpm[" << i << "]:" << kx.transpose() << std::endl;
    kx.setZero();
    arm.getKd(kx, 0);
    std::cout << "kdm[" << i << "]:" << kx.transpose() << std::endl;
    kx.setZero();
    arm.getKi(kx, 0);
    std::cout << "kim[" << i << "]:" << kx.transpose() << std::endl;
    kx.setZero();
  }
  ///////////////////////////////////////
  usleep(10000);

  arm.runSendReceiveThread();

  for (int i = 0; i < 2; i++)
  {
    Vec7 kp, kd, ki;
    kp << 800, 800, 800, 800, 800, 800, 800;
    kd << 1000, 1000, 1000, 1000, 1000, 1000, 1000;
    ki << 1, 1, 1, 1, 1, 1, 1;
    arm.setKp(kp, i);
    // arm.setKi(ki, i);
  }

  sleep(2);

  // std::cout << "............" << std::endl;

  // arm.pause();
  // for (int i = 0; i < 2; i++)
  // {
  //   arm.getKp(kx, 0);
  //   std::cout << "kpm[" << i << "]:" << kx.transpose() << std::endl;
  //   kx.setZero();
  //   arm.getKd(kx, 0);
  //   std::cout << "kdm[" << i << "]:" << kx.transpose() << std::endl;
  //   kx.setZero();
  //   arm.getKi(kx, 0);
  //   std::cout << "kim[" << i << "]:" << kx.transpose() << std::endl;
  //   kx.setZero();
  // }
  // while (1)
  // {
  //   sleep(1);
  // }

  // Vec7 q = Vec7::Zero();
  // // q[0] = 15 * M_PI / 180;
  // // q[7] = 15 * M_PI / 180;
  // q[3] = 40 * M_PI / 180;
  // // q[2] = 10 * M_PI / 180;

  // q[10] = -40 * M_PI / 180;
  // arm.setQ(q);

  std::vector<int> loop_indices(3);
  std::iota(loop_indices.begin(), loop_indices.end(), 0);
  std::vector<Vec7> q_traj_inv(q_traj.rbegin(), q_traj.rend());

  for (auto _ : loop_indices)
  {
    for (auto q : q_traj)
    {
      arm.setQ(q, 0);
      // arm.setQ(q, 1);
      usleep(dt * 1000000);
      // std::cout << "q:" << q.transpose() << std::endl;
    }

    for (auto q : q_traj_inv)
    {
      arm.setQ(q, 0);
      // arm.setQ(q, 1);
      usleep(dt * 1000000);
      // std::cout << "q:" << q.transpose() << std::endl;
    }
  }

  // for (double t = 0; t <= T + 1e-6; t += 0.1)
  // {
  //   Vec7 q;
  //   for (int i = 0; i < num_joints; ++i)
  //   {
  //     q[i] = coeffs[i](0) + coeffs[i](1) * t + coeffs[i](2) * pow(t, 2) + coeffs[i](3) * pow(t, 3) +
  //            coeffs[i](4) * pow(t, 4) + coeffs[i](5) * pow(t, 5);
  //   }
  //   arm.setQ(q);
  //   sleep(0.1);
  // }

  // for (double t = T; t >= 0 - 1e-6; t -= 0.1)
  // {
  //   Vec7 q;
  //   for (int i = 0; i < num_joints; ++i)
  //   {
  //     q[i] = coeffs[i](0) + coeffs[i](1) * t + coeffs[i](2) * pow(t, 2) + coeffs[i](3) * pow(t, 3) +
  //            coeffs[i](4) * pow(t, 4) + coeffs[i](5) * pow(t, 5);
  //   }
  //   arm.setQ(q);
  //   sleep(0.1);
  // }

  //   Vec7 q;
  //   // q[0] = 15 * M_PI / 180;
  //   // q[1] = 15 * M_PI / 180;
  //   // q[2] = 15 * M_PI / 180;
  //   // q[3] = 15 * M_PI / 180;
  //   // q[4] = 20 * M_PI / 180;
  //   // q[5] = 20 * M_PI / 180;
  //   // q[6] = 20 * M_PI / 180;

  //   q[0] = 10 * M_PI / 180;
  //   q[1] = 0 * M_PI / 180;
  //   q[2] = 0 * M_PI / 180;
  //   q[3] = 20 * M_PI / 180;
  //   q[4] = 0 * M_PI / 180;
  //   q[5] = 0 * M_PI / 180;
  //   q[6] = 0 * M_PI / 180;

  //   for (int i = 0; i < 2; i++)
  //   {
  // 	q[i + 7] = 0;
  //   }

  //   int dir = 0;
  //   while (1)
  //   {
  // 	if (dir % 2 == 0)
  // 	{
  // 	  q[0] = 10 * M_PI / 180;
  // 	  q[1] = 0 * M_PI / 180;
  // 	  q[2] = 0 * M_PI / 180;
  // 	  q[3] = 30 * M_PI / 180;
  // 	  q[4] = 0 * M_PI / 180;
  // 	  q[5] = 0 * M_PI / 180;
  // 	  q[6] = 0 * M_PI / 180;

  // 	  arm.setQ(q);
  // 	}
  // 	else
  // 	{
  // 	  q[0] = 0 * M_PI / 180;
  // 	  q[1] = 0 * M_PI / 180;
  // 	  q[2] = 0 * M_PI / 180;
  // 	  q[3] = 0 * M_PI / 180;
  // 	  q[4] = 0 * M_PI / 180;
  // 	  q[5] = 0 * M_PI / 180;
  // 	  q[6] = 0 * M_PI / 180;

  // 	  arm.setQ(q);
  // 	}
  // 	dir++;
  // 	sleep(1);
  //   }

  // 	arm.setMode("velocity");
  //     // arm.setId(1,2);
  // //arm.getOffset();
  // Vec7 qd;
  // qd(6)=1;//
  // arm.setQd(qd);

  // 	arm.runSendReceiveThread();

  // Vec7 max;
  // Vec7 min;
  // Vec7 kp;
  // Vec7 kd;

  // max(7)=6.28;//+360
  // min(7)=-6.28;//-360
  // kp(7)=800;
  // kd(7)=2000;//0x7d0//i+1可能需要该以下

  // std::cout<<"赋值1 min="<<min(7)<<std::endl;

  // arm.setMinMaxPos(min,max);//最小数值不正确a
  // arm.setKp(kp);
  // arm.setKd(kd);
  // arm.getKp();
  // arm.getKd();

  // std::cout<<"1"<<std::endl;

  // std::cout<<"2"<<std::endl;

  // std::cout<<"3"<<std::endl;

  // sleep(10);
  // std::cout<<"sleep(10);"<<std::endl;

  // arm.stop();
  // 	sleep(5);
  // std::cout<<"arm.stop();"<<std::endl;

  // 设置位置指令
  // Vec7 q;
  // q[0] = 8;
  // q[1] = 8;
  // arm.setQ(q);

  // Vec7 qd;
  // arm.setMode("velocity");
  // arm.runSendReceiveThread();
  // arm.setQd(qd);

  while (1)
  {
    sleep(1);
  }

  arm.pause();
  return 0;
}
