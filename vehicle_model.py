import numpy as np
import pandas as pd
from scipy.integrate import odeint

# import runge_kutta.py as rk
import matplotlib.pyplot as plt


def sim_vehicle_model(delta, v, time, beta, dot_phai):
    m = 2000
    I = 3000
    lf = 1.51
    lr = 1.49
    Kf = 10**5
    Kr = 10**5

    a11 = -Kf / m - Kr / m
    a12 = -Kf * lf / m + Kr * lr / m
    a13 = Kf / m
    a21 = -Kf * lf / I + Kr * lr / I
    a22 = -Kf * lf**2 / I - Kr * lr**2 / I
    a23 = Kf * lf / I

    dot_beta = (a11 / v) * beta + (-1 + (a12 / v**2)) * dot_phai + (a13 / v) * delta
    ddot_phai = a21 * beta + (a22 / v) * dot_phai + a23 * delta

    return dot_beta, ddot_phai


# #########
v = 10  # m/s
t_finish = 5
dt = 0.001  # シミュレーションの時間ステップ

# 初期条件
beta = 0.0
dot_phai = 0.0

# 時間点のリストを作成
steps = int(t_finish / dt)
t_list = np.linspace(0, t_finish, steps)

# 結果を保存するリスト
beta_list = []
dot_phai_list = []


def rk4_step(
    beta_current, dot_phai_current, delta_input, v_speed, current_time, h_step
):
    """
    4次ルンゲクッタ法を1ステップ実行します。

    引数:
        beta_current (float): beta の現在の値。
        dot_phai_current (float): dot_phai の現在の値。
        delta_input (float): 現在の時刻における操舵入力 (delta)。
        v_speed (float): 車速。
        current_time (float): 現在の時刻。
        h_step (float): 時間ステップ (dt)。

    戻り値:
        tuple: (beta_next, dot_phai_next) - RK4 ステップ後の値。
    """

    # sim_vehicle_model から導関数を取得するためのヘルパー関数
    def f(b, dp, t, delta_val, v_val):
        db, ddp = sim_vehicle_model(delta_val, v_val, t, b, dp)
        return np.array([db, ddp])

    # K1
    k1 = f(beta_current, dot_phai_current, current_time, delta_input, v_speed)

    # K2
    beta_k2 = beta_current + 0.5 * h_step * k1[0]
    dot_phai_k2 = dot_phai_current + 0.5 * h_step * k1[1]
    k2 = f(beta_k2, dot_phai_k2, current_time + 0.5 * h_step, delta_input, v_speed)

    # K3
    beta_k3 = beta_current + 0.5 * h_step * k2[0]
    dot_phai_k3 = dot_phai_current + 0.5 * h_step * k2[1]
    k3 = f(beta_k3, dot_phai_k3, current_time + 0.5 * h_step, delta_input, v_speed)

    # K4
    beta_k4 = beta_current + h_step * k3[0]
    dot_phai_k4 = dot_phai_current + h_step * k3[1]
    k4 = f(beta_k4, dot_phai_k4, current_time + h_step, delta_input, v_speed)

    # beta と dot_phai を更新
    beta_next = beta_current + (h_step / 6.0) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
    dot_phai_next = dot_phai_current + (h_step / 6.0) * (
        k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]
    )

    return beta_next, dot_phai_next


# シミュレーションループ
for i, t in enumerate(t_list):
    # 操舵入力の例 (変更可能)
    delta = 0.2 * np.sin(1 * np.pi * 2 * t)
    # delta = 0.2

    # 次のステップのために更新する前に現在の値を保存
    beta_list.append(beta)
    dot_phai_list.append(dot_phai)

    # RK4 を使って次のステップを計算
    beta, dot_phai = rk4_step(beta, dot_phai, delta, v, t, dt)

# 結果を Pandas DataFrame に保存
df = pd.DataFrame({"time": t_list, "beta": beta_list, "dot_phai": dot_phai_list})

# 結果の最初の数行を表示
print(df.head())

# 必要に応じて結果を CSV ファイルに保存することもできます
# df.to_csv('vehicle_sim_result.csv', index=False)
# ans = sim_vehicle_model(1,1,1,1,1)
# print(ans)

# Integrate dot_phai to get phai (yaw angle)
df["phai"] = df["dot_phai"].cumsum() * dt

# Initial position
x = 0.0
y = 0.0

# Lists to store position data
x_list = [x]
y_list = [y]
current_phai_list = [x]
current_beta_list = [x]
# Calculate x and y coordinates
for i in range(1, len(df)):
    current_phai = df["phai"].iloc[i - 1]
    current_beta = df["beta"].iloc[i - 1]

    current_phai_list.append(current_phai)
    current_beta_list.append(current_beta)

    # Kinematic equations for vehicle position
    # Assuming constant velocity 'v' for simplicity, as it was given as a constant 'v=17'
    dot_x = v * np.cos(current_phai + current_beta)
    dot_y = v * np.sin(current_phai + current_beta)

    x += dot_x * dt
    y += dot_y * dt

    x_list.append(x)
    y_list.append(y)

df["x"] = x_list
df["y"] = y_list
df["current_phai"] = current_phai_list
df["current_beta"] = current_beta_list
# Plotting the trajectory


plt.figure(figsize=(10, 8))
plt.plot(df["x"], df["y"], label="車両の軌道")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("車両の軌道シミュレーション")
plt.grid(True)
plt.axis("equal")  # Equal scaling for x and y axes
plt.legend()
plt.show()
