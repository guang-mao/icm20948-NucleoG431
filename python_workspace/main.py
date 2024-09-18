import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# 设定Arduino串口的参数
ser = serial.Serial('COM5', 115200)  # 修改为Arduino实际连接的COM口和波特率

# 初始化图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

max_points = 1000

# 用于保存数据的列表
x_data, y_data, z_data = [], [], []

# 更新函数，用于实时更新图形
def update(frame):
    line = ser.readline().decode('utf-8').strip()  # 读取Arduino的串口数据
    try:
        x, y, z = map(float, line.split(','))  # 将数据解析为浮点数

        # 限制数据列表的长度，只保留最新的 max_points 个数据
        if len(x_data) > max_points:
            x_data.pop(0)
            y_data.pop(0)
            z_data.pop(0)
        
        x_data.append(x)
        y_data.append(y)
        z_data.append(z)
        
        ax.clear()  # 清除上次绘制的图像
        ax.scatter(x_data, y_data, z_data, c='b', marker='o')  # 绘制新的点
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Real-time 3D Scatter Plot')
    except ValueError:
        print(f"无法解析的串口数据: {line}")

# 设置动画更新频率
ani = FuncAnimation(fig, update, interval=30, cache_frame_data=False)  # 每30毫秒更新一次图形

# 显示图形
plt.show()

# 关闭串口
ser.close()

