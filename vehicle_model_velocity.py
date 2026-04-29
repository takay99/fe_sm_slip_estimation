import numpy as np
import pandas as pd
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from typing import Annotated
import math
import fe_sm_slip_estimation.runge_kutta as runge_kutta

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
    phi = x[5,0]  # ヨー角 (rad)

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

    finish_time = 50.0  # 終了時間 (秒)
    dt = 0.01          # 刻み時間 (秒)
    time_steps = np.arange(0, finish_time, dt)
    
    # 初期値の設定 [vx, vy, r, X, Y, phi]
    # 初期速度 10m/s (36km/h) で直進状態から開始
    initial_value = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(-1, 1)
    
    # 時系列入力データの作成 (例: 1秒後にステアリングを0.1rad切る)
    # input_vector = [delta, tau_fl, tau_fr, tau_rl, tau_rr]
    inputs = []
    for t in time_steps:
        if t < 1.0:
            delta = 0.0
        else:
            delta = 0.1 # 1秒後にハンドルを切る
        
        tau = np.zeros(4) # トルクは0でコースト走行
        inputs.append(np.concatenate(([delta], tau)))
    
    # --- シミュレーション実行 ---
    results = []
    current_x = initial_value
    
    print(f"Starting simulation: 0.0s to {finish_time}s...")
    
    for i, t in enumerate(time_steps):
        # 現在のステップの入力を取得
        u = inputs[i]
        
        # 結果を保存
        results.append(current_x.flatten())
        
        # RK4による状態更新
        current_x = runge_kutta.rk4_step(
            sim_vehicle_model_velocity,
            current_x,
            t,
            dt,
            variation=u
        )
    
    # 結果のデータフレーム化
    df = pd.DataFrame(results, columns=['vx', 'vy', 'r', 'X', 'Y', 'phi'])
    df['time'] = time_steps

    slipangle = np.arctan2(df['vy'], df['vx'])
    df['slip_angle'] = slipangle

    plt.figure(figsize=(12, 5))
    plt.plot(df['time'], df['slip_angle'], label='Slip Angle')
    plt.grid(True)

    plt.figure(figsize=(12, 5))
    plt.plot(df['time'], df['vx'], label='Velocity X')
    plt.plot(df['time'], df['vy'], label='Velocity Y')
    plt.grid(True)
    plt.legend()

    # --- 結果の可視化 ---
    plt.figure(figsize=(12, 5))

    # XY軌跡
    plt.subplot(1, 2, 1)
    plt.plot(df['X'], df['Y'], label='Trajectory')
    plt.title('Vehicle Trajectory (Global XY)')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()

    # ヨーレートの時間変化
    plt.subplot(1, 2, 2)
    plt.plot(df['time'], df['r'], label='Yaw Rate', color='red')
    plt.title('Yaw Rate over Time')
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw Rate [rad/s]')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

    print("Simulation finished.")