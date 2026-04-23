import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV（无表头）
df = pd.read_csv("interpolated_positions.csv", header=None)

# 取第1个电机（第0列）
motor1 = df.iloc[:, 0]

# 画图
plt.plot(motor1)
plt.xlabel("Time Step")
plt.ylabel("Position")
plt.title("Motor 1 Position")
plt.grid()

plt.show()