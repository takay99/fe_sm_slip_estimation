import numpy as np
import pandas as pd
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from typing import Annotated
import math
import runge_kutta 

Vector4 = Annotated[np.ndarray, "4次元ベクトル"]
Vector6 = Annotated[np.ndarray, "6次元ベクトル"]
Vector5 = Annotated[np.ndarray, "ステアとトルクの5次元ベクトル"]


def sim_vehicle_model_velocity( x: Vector6 ,t,input: Vector5):
   
    delta = input[0]  # ステアリング角度 (ラジアン)
    tau = np.array(input[1:])  # 各輪のトルク (N*m

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
    phi = x[5]  # ヨー角 (rad)

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

    return x_dot_full



if __name__ == "__main__":
    # Fxのテスト
    delta = 0.1  # ステアリング角度 (ラジアン)
    # tau = np.array([100.0, 100.0, 100.0, 100.0])  # 各輪のトルク (N*m)
    tau = np.array([0.0, 0.0, 0.0, 0.0])  # 各輪のトルク (N*m)
    input_vector = np.concatenate(([delta], tau))  # 入力ベクトル (ステアリング角度とトルク)


    velocity_x = 10.0  # 車両のx方向速度 (m/s)
    velocity_y = 0.0   # 車両のy方向速度 (m/s)
    dot_phai = 0.0     # ヨーレート (rad/s)
    X = 0.0            # 車両のX座標 (m)
    Y = 0.0            # 車両のY座標 (m)
    phi = 0.0          # ヨー角 (rad)
    x = np.array([velocity_x, velocity_y, dot_phai, X, Y, phi]).reshape(-1, 1)  # 状態ベクトル
    time = 0.0         # 時間 (使用されない)

    x_dot = sim_vehicle_model_velocity(x, input_vector)
    
    runge_kutta_result = runge_kutta.rk4_step(sim_vehicle_model_velocity,np.array([0,0,0,0,0,0]).reshape(-1, 1),0,0.01,input_vector )  # sample = np.array([1,2,3,4,5]).reshape(-1, 1)


    print("\nx_dot (車両の状態変化率):")
    print(x_dot)
