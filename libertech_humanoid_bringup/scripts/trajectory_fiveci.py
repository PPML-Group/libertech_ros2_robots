import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize']=(12.8, 7.2)
class ArcTrajectory:
    def __init__(self, t0, t1, theta0, theta1, r):
        coeffs = self.__ComputeQuinticCoeffs(t0, t1, theta0, theta1)
        self.theta_poly = np.poly1d(coeffs[::-1])
        self.theta_dot_poly = self.theta_poly.deriv(1)  # 一阶导数（速度）
        self.theta_ddot_poly = self.theta_poly.deriv(2)  # 二阶导数（加速度）
        self.r = r

    @classmethod
    def __ComputeQuinticCoeffs(cls, t0, t1, theta0, theta1, v0=0.0, v1=0.0, a0=0.0, a1=0.0):
        T = t1 - t0
        h = theta1 - theta0
        k0 = theta0
        k1 = v0
        k2 = 0.5 * a0
        k3 = (20. * h - (8. * v1 + 12. * v0) * T - (3 * a0 - a1) * T**2) / (2. * T**3)
        k4 = (-30. * h + (14*v1 + 16*v0)*T + (3*a0 - 2*a1)*T**2) / (2. * T**4)
        k5 = (12 * h - 6*(v1 + v0) * T + (a1 - a0) * T**2) / (2. * T**5)
        return (k0, k1, k2, k3, k4, k5)

    def generate_trajectory(self, t0, t1, num_points=100):
        ts = np.linspace(t0, t1, num_points)
        thetas = self.theta_poly(ts)
        theta_dots = self.theta_dot_poly(ts)
        theta_ddots = self.theta_ddot_poly(ts)
        xs = self.r * np.cos(thetas)
        ys = self.r * np.sin(thetas)
        constant_value = 0  # 你可以选择任何常数值
        zs = np.full_like(xs, constant_value)
        vx = -self.r * np.sin(thetas) * theta_dots
        vy = self.r * np.cos(thetas) * theta_dots
        ax = -self.r * np.cos(thetas) * theta_dots**2 - self.r * np.sin(thetas) * theta_ddots
        ay = -self.r * np.sin(thetas) * theta_dots**2 + self.r * np.cos(thetas) * theta_ddots
        return ts, xs, ys, zs, vx, vy, ax, ay

if __name__ == "__main__":
    t0, t1 = 0, 5.0
    theta0, theta1 = 0, np.pi/3 # 从0到90度
    r = 5  # 半径为5米
    arc = ArcTrajectory(t0, t1, theta0, theta1, r)
    ts, xs, ys, vx, vy, ax, ay = arc.generate_trajectory(t0, t1)
    plt.figure(figsize=(10, 8))  # 调整图形尺寸
    plt.plot(xs, ys, label='Arc Trajectory')
    plt.title("Arc Trajectory")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.axis("equal")
    plt.grid(True)
    plt.xlim(min(xs) - 1, max(xs) + 1)
    plt.ylim(min(ys) - 1, max(ys) + 1)
    plt.legend()
    plt.tight_layout()
    plt.show()
    # plt.figure( )
    # plt.subplot(411)
    # plt.plot(ts, xs, label='X Position')
    # plt.plot(ts, ys, label='Y Position')
    # plt.title("Position")
    # plt.legend()

    # plt.subplot(412)
    # plt.plot(ts, vx, label='X Velocity')
    # plt.plot(ts, vy, label='Y Velocity')
    # plt.title("Velocity")
    # plt.legend()

    # plt.subplot(413)
    # plt.plot(ts, ax, label='X Acceleration')
    # plt.plot(ts, ay, label='Y Acceleration')
    # plt.title("Acceleration")
    # plt.legend()
    
    # plt.subplot(414)
    # plt.plot(xs, ys)
    # plt.title("Arc Trajectory")
    # plt.xlabel("X Position")
    # plt.ylabel("Y Position")
    # plt.axis("equal")
    # plt.grid(True)
    # plt.legend()
    
    # plt.tight_layout()
    # plt.show()
    
    
