#include "libertech_arm_driver/ArmSDK.hpp"
#include <fstream>
#include <numeric>

std::vector<Vec6> getQuinticPolynomialCoeffs(const VecX &q0, const VecX &v0, const VecX &a0, const VecX &qf,
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

std::vector<Vec7> planQuinticPolynomialTrajectory(const VecX &q0, const VecX &v0, const VecX &a0, const VecX &qf,
                                                  const VecX &vf, const VecX &af, double T, double dt)
{
  std::vector<Vec6> coeffs = getQuinticPolynomialCoeffs(q0, v0, a0, qf, vf, af, T);
  int DOF = q0.size();
  std::vector<Vec7> q_traj;

  for (double t = 0; t <= T + 1e-6; t += dt)
  {
    // std::cout << t << std::endl;
    Vec7 q;
    for (int i = 0; i < DOF; ++i)
    {
      q[i] = coeffs[i](0) + coeffs[i](1) * t + coeffs[i](2) * pow(t, 2) + coeffs[i](3) * pow(t, 3) +
             coeffs[i](4) * pow(t, 4) + coeffs[i](5) * pow(t, 5);
    }
    q_traj.push_back(q);
  }

  return q_traj;
}

int main()
{
  int num_joints = 7;
  Vec7 v0, a0, vf, af;
  Vec7 q0_r, q0_l, qf_r, qf_l;
  double T = 1;
  double dt = 0.005;

  v0.setZero();
  a0.setZero();
  vf.setZero();
  af.setZero();

  ArmSDK arm(0, 1, false);
  arm.init();
  usleep(1000);

  // kp kd
  Vec7 kx;
  for (int i = 0; i < 1; i++)
  {
    arm.getKp(kx, i);
    std::cout << "kpm[" << i << "]:" << kx.transpose() << std::endl;
    kx.setZero();
    arm.getKd(kx, i);
    std::cout << "kdm[" << i << "]:" << kx.transpose() << std::endl;
    kx.setZero();
    arm.getKi(kx, i);
    std::cout << "kim[" << i << "]:" << kx.transpose() << std::endl;
    kx.setZero();
  }
  usleep(10000);

  arm.runSendReceiveThread();

  for (int i = 0; i < 1; i++)
  {
    Vec7 kp, kd, ki;
    // kp << 800, 800, 800, 800, 800, 800, 800;
    // kd << 1000, 1000, 1000, 1000, 1000, 1000, 1000;
    kd << 800, 800, 800, 800, 800, 800, 800;
    kp << 1000, 1000, 1000, 1000, 1000, 1000, 1000;
    ki << 0, 0, 0, 0, 0, 0, 0;
    arm.setKp(kp, 0);
    arm.setKi(ki, i);
  }
  sleep(1);

  // while (1)
  // {
  //   arm.getQ(q0_r, 0);
  //   arm.getQ(q0_l, 1);

  //   std::cout << "右臂当前位置:" << q0_r.transpose() << std::endl;
  //   std::cout << "左臂当前位置:" << q0_l.transpose() << std::endl;
  //   usleep(1000000);
  // }
  T = 3;
  std::vector<Vec7> q_traj;

  q0_l.setZero();
  qf_l.setZero();
  arm.getQ(q0_l, 1);
  qf_l << 0, 0, 0, 0, 0, 0, 0;
  qf_l = qf_l * M_PI / 180;
  q_traj = planQuinticPolynomialTrajectory(q0_l, v0, a0, qf_l, vf, af, T, dt);

  for (auto q : q_traj)
  {
    arm.setQ(q, 1);
    usleep(dt * 1000000);
  }

  // q0_l.setZero();
  // qf_l.setZero();
  // arm.getQ(q0_l, 1);
  // qf_l << 0, 0, 0, 0, 0, 0, 30;
  // qf_l = qf_l * M_PI / 180;
  // q_traj = planQuinticPolynomialTrajectory(q0_l, v0, a0, qf_l, vf, af, T, dt);

  // for (auto q : q_traj)
  // {
  //   arm.setQ(q, 1);
  //   usleep(dt * 1000000);
  // }

  // q0_l.setZero();
  // qf_l.setZero();
  // arm.getQ(q0_l, 1);
  // qf_l << 0, 0, 0, 0, 0, 0, 0;
  // qf_l = qf_l * M_PI / 180;
  // q_traj = planQuinticPolynomialTrajectory(q0_l, v0, a0, qf_l, vf, af, T, dt);

  // for (auto q : q_traj)
  // {
  //   arm.setQ(q, 1);
  //   usleep(dt * 1000000);
  // }

  // q0_l.setZero();
  // qf_l.setZero();
  // arm.getQ(q0_l, 1);
  // qf_l << 0, 0, 0, 0, 0, 0, 0;
  // qf_l = qf_l * M_PI / 180;
  // q_traj = planQuinticPolynomialTrajectory(q0_l, v0, a0, qf_l, vf, af, T, dt);

  // for (auto q : q_traj)
  // {
  //   arm.setQ(q, 1);
  //   // arm.setQ(q, 1);
  //   usleep(dt * 1000000);
  //   // std::cout << "q:" << q.transpose() << std::endl;
  // }

  q0_r.setZero();
  qf_r.setZero();
  arm.getQ(q0_r, 0);
  qf_r << 0, 0, 0, 0, 0, 0, 0;
  qf_r = qf_r * M_PI / 180;
  q_traj = planQuinticPolynomialTrajectory(q0_r, v0, a0, qf_r, vf, af, T, dt);

  for (auto q : q_traj)
  {
    arm.setQ(q, 0);
    usleep(dt * 1000000);
  }
  
  // q0_r.setZero();
  // qf_r.setZero();
  // arm.getQ(q0_r, 0);
  // qf_r << 0, 0, 30, 0, 0, 0, 0;
  // qf_r = qf_r * M_PI / 180;
  // q_traj = planQuinticPolynomialTrajectory(q0_r, v0, a0, qf_r, vf, af, T, dt);

  // for (auto q : q_traj)
  // {
  //   arm.setQ(q, 0);
  //   usleep(dt * 1000000);
  // }

  // q0_r.setZero();
  // qf_r.setZero();
  // arm.getQ(q0_r, 0);
  // qf_r << 0, 0, 0, 0, 0, 0, 0;
  // qf_r = qf_r * M_PI / 180;
  // q_traj = planQuinticPolynomialTrajectory(q0_r, v0, a0, qf_r, vf, af, T, dt);

  // for (auto q : q_traj)
  // {
  //   arm.setQ(q, 0);
  //   usleep(dt * 1000000);
  // }

  // q0_r.setZero();
  // qf_r.setZero();
  // arm.getQ(q0_r, 0);
  // qf_r << 30, 30, 0, 20, 0, 0, 0;
  // qf_r = qf_r * M_PI / 180;
  // q_traj = planQuinticPolynomialTrajectory(q0_r, v0, a0, qf_r, vf, af, T, dt);

  // for (auto q : q_traj)
  // {
  //   arm.setQ(q, 0);
  //   // arm.setQ(q, 1);
  //   usleep(dt * 1000000);
  //   // std::cout << "q:" << q.transpose() << std::endl;
  // }

  // q0_r.setZero();
  // qf_r.setZero();
  // arm.getQ(q0_r, 0);
  // qf_r << 0, 0, 0, 0, 0, 0, 0;
  // qf_r = qf_r * M_PI / 180;
  // q_traj = planQuinticPolynomialTrajectory(q0_r, v0, a0, qf_r, vf, af, T, dt);

  // for (auto q : q_traj)
  // {
  //   arm.setQ(q, 0);
  //   // arm.setQ(q, 1);
  //   usleep(dt * 1000000);
  //   // std::cout << "q:" << q.transpose() << std::endl;
  // }
  // sleep(1);

  // q0_r.setZero();
  // qf_r.setZero();
  // arm.getQ(q0_r, 0);
  // qf_r << 30, 30, 0, 20, 0, 0, 0;
  // qf_r = qf_r * M_PI / 180;
  // q_traj = planQuinticPolynomialTrajectory(q0_r, v0, a0, qf_r, vf, af, T, dt);

  // for (auto q : q_traj)
  // {
  //   arm.setQ(q, 0);
  //   // arm.setQ(q, 1);
  //   usleep(dt * 1000000);
  //   // std::cout << "q:" << q.transpose() << std::endl;
  // }
  // usleep(500000);

  // q0_r.setZero();
  // qf_r.setZero();
  // arm.getQ(q0_r, 0);
  // qf_r << 1.737492, -0.142930, -0.098426, -1.001563, 0.298998, -0.352930, 0.246161;
  // // qf_r << 1.437492, 0, 0, 0, 0, 0, 0;
  // q_traj = planQuinticPolynomialTrajectory(q0_r, v0, a0, qf_r, vf, af, T, dt);

  // for (auto q : q_traj)
  // {
  //   arm.setQ(q, 0);
  //   // arm.setQ(q, 1);
  //   usleep(dt * 1000000);
  //   // std::cout << "q:" << q.transpose() << std::endl;
  // }
  // usleep(500000);

  // q0_r.setZero();
  // qf_r.setZero();
  // arm.getQ(q0_r, 0);
  // qf_r << 1.437492, -0.142930, -0.098426, -1.001563, 0.348998, -0.352930, 0.246161;
  // // qf_r << 1.437492, 0, 0, 0, 0, 0, 0;
  // q_traj = planQuinticPolynomialTrajectory(q0_r, v0, a0, qf_r, vf, af, T, dt);

  // for (auto q : q_traj)
  // {
  //   arm.setQ(q, 0);
  //   // arm.setQ(q, 1);
  //   usleep(dt * 1000000);
  //   // std::cout << "q:" << q.transpose() << std::endl;
  // }
  // sleep(6);

  // q0_r.setZero();
  // qf_r.setZero();
  // arm.getQ(q0_r, 0);
  // qf_r << 1.737492, -0.142930, -0.098426, -1.001563, 0.348998, -0.352930, 0.246161;
  // // qf_r << 1.437492, 0, 0, 0, 0, 0, 0;
  // q_traj = planQuinticPolynomialTrajectory(q0_r, v0, a0, qf_r, vf, af, T, dt);

  // for (auto q : q_traj)
  // {
  //   arm.setQ(q, 0);
  //   // arm.setQ(q, 1);
  //   usleep(dt * 1000000);
  //   // std::cout << "q:" << q.transpose() << std::endl;
  // }
  // usleep(500000);

  

  // qf << 0, 0, 0, 0, 0, 0, 0;
  // // qf << 15, -15, 10, 30, 15, 0, 10;
  // // qf <<-15, 15, -10, -30, 15, 0, 10;

  // std::vector<Vec7> q_traj = planQuinticPolynomialTrajectory(q0, v0, a0, qf, vf, af, T, dt);

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

  // std::vector<int> loop_indices(3);
  // std::iota(loop_indices.begin(), loop_indices.end(), 0);
  // std::vector<Vec7> q_traj_inv(q_traj.rbegin(), q_traj.rend());

  // for (auto _ : loop_indices)
  // {
  //   for (auto q : q_traj)
  //   {
  //     arm.setQ(q, 0);
  //     // arm.setQ(q, 1);
  //     usleep(dt * 1000000);
  //     // std::cout << "q:" << q.transpose() << std::endl;
  //   }

  //   for (auto q : q_traj_inv)
  //   {
  //     arm.setQ(q, 0);
  //     // arm.setQ(q, 1);
  //     usleep(dt * 1000000);
  //     // std::cout << "q:" << q.transpose() << std::endl;
  //   }
  // }

  while (1)
  {
    sleep(1);
  }

  arm.pause();
  return 0;
}
