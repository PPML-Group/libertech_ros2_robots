from scipy.spatial.transform import Rotation as R

# 四元数 q = [x, y, z, w]
quat = [0.5, -0.5, 0.5, -0.5]

# 创建 Rotation 对象
rotation = R.from_quat(quat)

# 转换到欧拉角（默认ZYX顺序）
euler = rotation.as_euler('zyx', degrees=True)  # 如果需要弧度制，去掉 degrees=True

# 转换到旋转矩阵
rot_matrix = rotation.as_matrix()

print("Euler Angles (ZYX):", euler)
print("Rotation Matrix:\n", rot_matrix)
