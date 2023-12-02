import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
import math
# from math import sq
# print(math.sqrt(2))


# # 轴向量
# axis = np.array([1, 0, 0])
# # 角度（弧度）
# angle = 360

# # 创建 Rotation 对象
# rotation = Rotation.from_rotvec(angle * axis)

# # 获取四元数表示
# quaternion = rotation.as_quat() #w, x, y, z
# print("quaternion:",quaternion)
# print(quaternion)
# 示例四元数
quaternion = np.array([1,-math.sqrt(2)/2,math.sqrt(2)/2,0]) # xyzw 0 0 0 1

# Normalize the quaternion to unit length
quaternion = quaternion.astype(float)  # Convert to float data type
print("qua norm: ",np.linalg.norm(quaternion))
quaternion /= np.linalg.norm(quaternion)
print("qua: ",quaternion)
# 提取方向向量和旋转角度
direction_vector = quaternion[1:]
rotation_angle = 2 * np.arccos(quaternion[0])

# 创建一个坐标轴图
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制坐标轴
ax.quiver(0, 0, 0, 1, 0, 0, color='r')
ax.quiver(0, 0, 0, 0, 1, 0, color='g')
ax.quiver(0, 0, 0, 0, 0, 1, color='b')


# 绘制轴角表示的旋转轴
print(direction_vector)
ax.quiver(0, 0, 0, direction_vector[0], direction_vector[1], direction_vector[2], color='m')

# 设置坐标轴范围
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

# 设置图形标题
ax.set_title(f"Rotation Axis: {direction_vector}, Angle: {np.degrees(rotation_angle):.2f} degrees")

# 显示图形
plt.show()
