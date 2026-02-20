import numpy as np
import pandas as pd
from scipy.integrate import odeint
from vehicle_model import rk4_step
import matplotlib.pyplot as plt
from typing import Annotated
import math

Vector4 = Annotated[np.ndarray, "4次元ベクトル"]
Vector6 = Annotated[np.ndarray, "6次元ベクトル"]


def sim_vehicle_model_velocity(delta: float, tau: Vector4, x: Vector6, time  ):
   
    m = 1500
    I = 1500
    center2rightwheel = 0.7
    center2leftwheel = 0.7
    wheelbase_f = 1.5
    wheelbase_r = 1.8
    Kf = 10**4
    Kr = 10**4
    radius = 0.3
    Inertia_matrix = np.diag([m, m, I])

    x = x.reshape(-1, 1)  # 状態ベクトルを列ベクトルに変換
    velocity_x = x[0,0]  # 車両のx方向速度 (m/s)
    velocity_y = x[1,0]  # 車両のy方向速度 (m/s)
    dot_phai = x[2,0]    # ヨーレート (rad/s)
    # x[3], x[4] は X, Y 座標 (m)
    phi        = x[5]  # ヨー角 (rad)

    slip_angle_fl = delta - (velocity_y + wheelbase_f * dot_phai) / (velocity_x - center2leftwheel * dot_phai)
    slip_angle_fr = delta - (velocity_y + wheelbase_f * dot_phai) / (velocity_x + center2rightwheel * dot_phai)
    slip_angle_rl = -(velocity_y - wheelbase_r * dot_phai) / (velocity_x - center2leftwheel * dot_phai)
    slip_angle_rr = -(velocity_y - wheelbase_r * dot_phai) / (velocity_x + center2rightwheel * dot_phai)
    slip_angle_vector = np.array([slip_angle_fl, slip_angle_fr, slip_angle_rl, slip_angle_rr]).reshape(-1, 1)

    Fs_fl = Kf * slip_angle_fl
    Fs_fr = Kf * slip_angle_fr
    Fs_rl = Kr * slip_angle_rl
    Fs_rr = Kr * slip_angle_rr
    Fs_vector = np.array([Fs_fl, Fs_fr, Fs_rl, Fs_rr]).reshape(-1, 1)

    Ft_fl = tau[0] / radius
    Ft_fr = tau[1] / radius
    Ft_rl = tau[2] / radius
    Ft_rr = tau[3] / radius
    Ft_vector = np.array([Ft_fl, Ft_fr, Ft_rl, Ft_rr]).reshape(-1, 1)
    
    Fx = np.diag([math.cos(delta), math.cos(delta), 1, 1]) @ Ft_vector + np.diag([-math.sin(delta), -math.sin(delta), 0, 0]) @ Fs_vector
    Fy = np.diag([math.sin(delta), math.sin(delta), 0, 0]) @ Ft_vector + np.diag([math.cos(delta), math.cos(delta), 1, 1]) @ Fs_vector

    vector_1 = np.array([velocity_y * dot_phai, -velocity_x * dot_phai, 0]).reshape(-1, 1)
    matrix_1 = np.array([[1,1,1,1],[0,0,0,0],[-center2leftwheel,center2rightwheel,-center2leftwheel,center2rightwheel]])
    matrix_2 = np.array([[0,0,0,0],[1,1,1,1],[wheelbase_f,wheelbase_f,-wheelbase_r,-wheelbase_r]])

    v_dot = vector_1 + np.linalg.inv(Inertia_matrix) @ (matrix_1 @ Fx) + np.linalg.inv(Inertia_matrix) @ (matrix_2 @ Fy)

    dX = velocity_x * math.cos(phi) - velocity_y * math.sin(phi)
    dY = velocity_x * math.sin(phi) + velocity_y * math.cos(phi)
    dphi = dot_phai

    x_dot_full = np.array([
    v_dot[0, 0], 
    v_dot[1, 0], 
    v_dot[2, 0], 
    dX, 
    dY, 
    dphi
    ]).reshape(-1, 1)

    return Fx,Fy, Fs_vector, Ft_vector, slip_angle_vector, x_dot_full


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
        db, ddp = sim_vehicle_model_velocity(delta_val, v_val, t, b, dp)
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


if __name__ == "__main__":
    # Fxのテスト
    delta = 0.1  # ステアリング角度 (ラジアン)
    # tau = np.array([100.0, 100.0, 100.0, 100.0])  # 各輪のトルク (N*m)
    tau = np.array([0.0, 0.0, 0.0, 0.0])  # 各輪のトルク (N*m)

    velocity_x = 10.0  # 車両のx方向速度 (m/s)
    velocity_y = 0.0   # 車両のy方向速度 (m/s)
    dot_phai = 0.0     # ヨーレート (rad/s)
    X = 0.0            # 車両のX座標 (m)
    Y = 0.0            # 車両のY座標 (m)
    phi = 0.0          # ヨー角 (rad)
    x = np.array([velocity_x, velocity_y, dot_phai, X, Y, phi]).reshape(-1, 1)  # 状態ベクトル
    time = 0.0         # 時間 (使用されない)

    Fx, Fy, Fs_vector, Ft_vector, slip_angle_vector,x_dot = sim_vehicle_model_velocity(delta, tau, x, time)
    
    print("Fx (車両のx方向力):")
    print(Fx)
    print("\nFy (車両のy方向力):")
    print(Fy)
    print("\nFs_vector (各輪の横方向力):")
    print(Fs_vector)
    print("\nslip_angle_vector (各輪のスリップ角):")
    print(slip_angle_vector)
    print("\nFt_vector (各輪の縦方向力):")
    print(Ft_vector)
    print("\nx_dot (車両の状態変化率):")
    print(x_dot)
