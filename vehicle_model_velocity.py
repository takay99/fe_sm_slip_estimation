import numpy as np
import pandas as pd
from scipy.integrate import odeint
from vehicle_model import rk4_step
import matplotlib.pyplot as plt
from typing import Annotated
import math

Vector4 = Annotated[np.ndarray, "4次元ベクトル"]


def sim_vehicle_model_velocity(delta: float, tau: Vector4, velocity_x,velocity_y, time ,dot_phai ):
   
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

    x_dot = vector_1 + np.linalg.inv(Inertia_matrix) @ (matrix_1 @ Fx) + np.linalg.inv(Inertia_matrix) @ (matrix_2 @ Fy)

    return Fx,Fy, Fs_vector, Ft_vector, slip_angle_vector, x_dot


if __name__ == "__main__":
    # Fxのテスト
    delta = 0.1  # ステアリング角度 (ラジアン)
    # tau = np.array([100.0, 100.0, 100.0, 100.0])  # 各輪のトルク (N*m)
    tau = np.array([0.0, 0.0, 0.0, 0.0])  # 各輪のトルク (N*m)

    velocity_x = 10.0  # 車両のx方向速度 (m/s)
    velocity_y = 0.0   # 車両のy方向速度 (m/s)
    dot_phai = 0.0     # ヨーレート (rad/s)
    time = 0.0         # 時間 (使用されない)

    Fx, Fy, Fs_vector, Ft_vector, slip_angle_vector,x_dot = sim_vehicle_model_velocity(delta, tau, velocity_x, velocity_y, time, dot_phai)
    
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