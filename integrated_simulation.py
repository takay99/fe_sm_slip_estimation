import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math

# --- ユーザのアップロードしたモジュール ---
import runge_kutta
from vehicle_model_velocity import sim_vehicle_model_velocity
from vehicle_state_observer import VelocityEstimator, VehicleStateObserver

def heuristic_schedule_simple(val, val_dot, sigma, sigma_dot):
    """
    heuristic_schedule.py の代用（簡易版ガウス関数）
    直進状態（値が0に近い）ほど1に近づき、変化が大きいほど0に近づく
    """
    f_val = math.exp(-(val**2) / (2 * sigma**2))
    f_dot = math.exp(-(val_dot**2) / (2 * sigma_dot**2))
    return f_val * f_dot

if __name__ == "__main__":
    finish_time = 10.0  # 終了時間 (秒)
    dt = 0.01          # 刻み時間 (秒)
    time_steps = np.arange(0, finish_time, dt)
    
    # 初期値の設定 [vx, vy, r, X, Y, phi]
    # 初期速度 10m/s (36km/h) で直進状態から開始
    initial_value = np.array([10.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(-1, 1)
    
    # 時系列入力データの作成 (1秒後にステアリングを0.1rad切る)
    inputs = []
    for t in time_steps:
        delta = 0.1 * math.sin(0.5 * math.pi * (t - 1.0)) if t >= 1.0 else 0.0
        tau = np.zeros(4) # トルクは0でコースト走行
        inputs.append(np.concatenate(([delta], tau)))
    
    # --- 推定器(Observer)の初期化 ---
    # Observer側のトレッドは 0.7 + 0.7 = 1.4m
    VelEst = VelocityEstimator(s_time=dt, s1=4.5, s2=1.0, track=1.4, ax_threshold=1.0)
    BetaEst = VehicleStateObserver(a0=10, a1=5, a2=10, dt=dt)
    
    # 微分計算用の前回値保持
    str_prev = 0.0
    omega_z_obs_prev = 0.0
    beta_dot_obs_prev = 0.0
    beta_ddot_obs_prev = 0.0  # ★追加

    results = []
    current_x = initial_value
    
    print(f"Starting integrated simulation: 0.0s to {finish_time}s...")
    
    for i, t in enumerate(time_steps):
        u = inputs[i]
        delta_sim = u[0]
        
        # ----------------------------------------------------
        # 1. シミュレータによる真の微分値・状態量の計算 (Simulator: 左正系)
        # ----------------------------------------------------
        x_dot = sim_vehicle_model_velocity(current_x, t, u)
        dvx_true = x_dot[0, 0]
        dvy_true = x_dot[1, 0]
        
        vx_true = current_x[0, 0]
        vy_true = current_x[1, 0]
        r_true  = current_x[2, 0]
        beta_true = math.atan2(vy_true, vx_true)
        
        # センサ観測値の疑似合成 (遠心力を加味したIMUの加速度)
        ax_sim_meas = dvx_true - vy_true * r_true
        ay_sim_meas = dvy_true + vx_true * r_true
        
        # 車輪速度センサの疑似合成 (左正系での計算)
        v_fl = (vx_true - 0.7 * r_true) / math.cos(delta_sim) if math.cos(delta_sim) != 0 else vx_true
        v_fr = (vx_true + 0.7 * r_true) / math.cos(delta_sim) if math.cos(delta_sim) != 0 else vx_true
        v_rl = vx_true - 0.7 * r_true
        v_rr = vx_true + 0.7 * r_true
        v_meas = (v_fl, v_fr, v_rl, v_rr)
        
        # ----------------------------------------------------
        # 2. 座標系の変換 (Simulator[左正] -> Observer[右正])
        # ----------------------------------------------------
        ax_obs     = ax_sim_meas
        ay_obs     = -ay_sim_meas      # 横方向の符号反転
        omega_z_obs= -r_true           # ヨーレートの符号反転
        delta_obs  = -delta_sim        # 操舵角の符号反転
        


        # ----------------------------------------------------
        # 3. オブザーバによる推定フェーズ (Observer: 右正系)
        # ----------------------------------------------------
        # (1) 車体速度の推定
        vx_est = VelEst.estimate(v_meas, delta_obs, omega_z_obs, ax_obs)
        
        # (2) ヒューリスティック関数の計算 (簡易版)
        str_dot = (delta_obs - str_prev) / dt
        omega_z_dot = (omega_z_obs - omega_z_obs_prev) / dt
        
        # ★ F_t の計算には「前回ステップ」の beta_dot, beta_ddot を使用する
        F_str = heuristic_schedule_simple(delta_obs, str_dot, 0.1, 0.1)
        F_omegaz = heuristic_schedule_simple(omega_z_obs, omega_z_dot, 0.18, 0.18)
        F_betadot = heuristic_schedule_simple(beta_dot_obs_prev, beta_ddot_obs_prev, 0.06, 0.3)
        F_t = F_str * F_omegaz * F_betadot
        
        # (3) 状態オブザーバの更新 (ここで self.dV_hat_dt が初めて計算される)
        beta_hat_obs_rad = BetaEst.update_state(ax_obs, ay_obs, omega_z_obs, vx_est, F_t=F_t)
        vx_hat, vy_hat_obs = BetaEst.get_estimated_velocity()

        # ★ 次回ステップのために、更新後の状態から beta_dot と beta_ddot を計算・取得
        beta_dot_obs = BetaEst.get_estimated_slip_angle_dot()
        beta_ddot_obs = (beta_dot_obs - beta_dot_obs_prev) / dt
        # ----------------------------------------------------
        # 4. 座標系の逆変換 (Observer[右正] -> Simulator[左正])
        # ----------------------------------------------------
        beta_hat_sim_rad = -beta_hat_obs_rad
        vy_hat_sim       = -vy_hat_obs
        
        # 結果の保存
        results.append([
            t, vx_true, vy_true, r_true, math.degrees(beta_true), 
            vx_hat, vy_hat_sim, math.degrees(beta_hat_sim_rad), F_t
        ])
        
        # ----------------------------------------------------
        # 5. 次のステップへの状態更新と前回値の保存
        # ----------------------------------------------------
        current_x = runge_kutta.rk4_step(
            sim_vehicle_model_velocity, current_x, t, dt, variation=u
        )
        
        str_prev = delta_obs
        omega_z_obs_prev = omega_z_obs
        beta_dot_obs_prev = beta_dot_obs

    # --- 結果のデータフレーム化とプロット ---
    df = pd.DataFrame(results, columns=[
        'time', 'vx_true', 'vy_true', 'r_true', 'beta_true_deg', 
        'vx_hat', 'vy_hat', 'beta_hat_deg', 'F_t'
    ])
    
    plt.figure(figsize=(12, 8))
    
    # 横滑り角の比較
    plt.subplot(2, 1, 1)
    plt.plot(df['time'], df['beta_true_deg'], label='True Slip Angle (Sim)', color='black', linestyle='--')
    plt.plot(df['time'], df['beta_hat_deg'], label='Estimated Slip Angle (Obs)', color='blue')
    plt.title('Sideslip Angle: True vs Estimated')
    plt.ylabel('Slip Angle [deg]')
    plt.grid(True)
    plt.legend()
    
    # 横速度とヒューリスティック重みの比較
    plt.subplot(2, 1, 2)
    plt.plot(df['time'], df['vy_true'], label='True Vy (Sim)', color='black', linestyle='--')
    plt.plot(df['time'], df['vy_hat'], label='Estimated Vy (Obs)', color='green')
    plt.plot(df['time'], df['F_t'], label='Heuristic F(t)', color='red', alpha=0.5)
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s] / F(t) Weight')
    plt.grid(True)
    plt.legend()
    
    plt.tight_layout()
    plt.show()

    print("Integrated Simulation finished.")