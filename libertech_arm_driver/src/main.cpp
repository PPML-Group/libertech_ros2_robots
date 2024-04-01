#include "libertech_arm_driver/ArmDriver.hpp"
#include <fstream>
#include <numeric>

std::vector<Vec6> planQuinticPolynomialTrajectory(const VecX& q0, const VecX& v0, const VecX& a0, const VecX& qf,
                                                  const VecX& vf, const VecX& af, double T)
{
  int DOF = q0.size();  // 关节自由度
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
  int num_joints = 14;
  Vec14 q0, v0, a0, qf, vf, af;
  q0.setZero();
  v0.setZero();
  a0.setZero();

  qf << 15, -15, 10, 30, 15, 0, 10, -15, 15, -10, -30, 15, 0, 10;

  qf = qf * M_PI / 180;
  vf.setZero();
  af.setZero();
  double T = 2;
  double dt = 0.001;

  std::vector<Vec6> coeffs = planQuinticPolynomialTrajectory(q0, v0, a0, qf, vf, af, T);

  std::vector<Vec14> q_traj;

  for (double t = 0; t <= T + 1e-6; t += dt)
  {
    // std::cout << t << std::endl;
    Vec14 q;
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
    outfile << std::fixed << std::setprecision(6);  // 设置输出格式，保留2位小数
    // std::cout << q_traj.size() << std::endl;
    for (size_t i = 0; i < q_traj.size(); ++i)
    {
      for (int j = 0; j < num_joints; ++j)
      {
        outfile << std::setw(10) << q_traj[i][j];  // 设置字段宽度为8个字符
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
  ArmDriver armDriver;

  float pos = 360 * M_PI / 180;
  std::cout << "pos:" << pos << std::endl;

  uint8_t pos_u8[4];
  armDriver.convertPositionToHexBytes(pos, pos_u8);
  armDriver.printHexBytes(pos_u8);

  float pos_convert_inv = armDriver.convertHexBytesToPosition(pos_u8);
  std::cout << "pos_convert_inv:" << pos_convert_inv << std::endl;

  armDriver.init();
  sleep(2);
  armDriver.runSendReceiveThread();
	sleep(2);

  // Vec14 q = Vec14::Zero();
  // // q[0] = 15 * M_PI / 180;
  // // q[7] = 15 * M_PI / 180;
  // q[3] = 40 * M_PI / 180;
  // // q[2] = 10 * M_PI / 180;

  // q[10] = -40 * M_PI / 180;
	// armDriver.setQ(q);

  std::vector<int> loop_indices(5);
  std::iota(loop_indices.begin(), loop_indices.end(), 0);
  std::vector<Vec14> q_traj_inv(q_traj.rbegin(),q_traj.rend());

  for (auto _ : loop_indices)
  {
    for (auto q : q_traj){
      armDriver.setQ(q);
      usleep(dt*1000000);
      std::cout << "q:" << q.transpose() << std::endl;
    }


    for (auto q : q_traj_inv){
      armDriver.setQ(q);
      usleep(dt*1000000);
      std::cout << "q:" << q.transpose() << std::endl;
    }
  }

    // for (double t = 0; t <= T + 1e-6; t += 0.1)
    // {
    //   Vec14 q;
    //   for (int i = 0; i < num_joints; ++i)
    //   {
    //     q[i] = coeffs[i](0) + coeffs[i](1) * t + coeffs[i](2) * pow(t, 2) + coeffs[i](3) * pow(t, 3) +
    //            coeffs[i](4) * pow(t, 4) + coeffs[i](5) * pow(t, 5);
    //   }
    //   armDriver.setQ(q);
    //   sleep(0.1);
    // }

    // for (double t = T; t >= 0 - 1e-6; t -= 0.1)
    // {
    //   Vec14 q;
    //   for (int i = 0; i < num_joints; ++i)
    //   {
    //     q[i] = coeffs[i](0) + coeffs[i](1) * t + coeffs[i](2) * pow(t, 2) + coeffs[i](3) * pow(t, 3) +
    //            coeffs[i](4) * pow(t, 4) + coeffs[i](5) * pow(t, 5);
    //   }
    //   armDriver.setQ(q);
    //   sleep(0.1);
    // }

  //   Vec14 q;
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

  // 	  armDriver.setQ(q);
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

  // 	  armDriver.setQ(q);
  // 	}
  // 	dir++;
  // 	sleep(1);
  //   }

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
