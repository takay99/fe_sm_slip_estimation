from math import exp
import math


def longitudinal_speed_estimation(
    velocity: float,
    velocity_delay: float,
    velocity_dot: float,
    Ax: float,
    sigma_1: float,
    sigma_2: float,
) -> float:
    W_i = exp(
        -(((Ax - velocity_dot) ** 2) / (2 * sigma_1**2))
        - (((velocity_delay - velocity) ** 2) / (2 * sigma_2**2))
    )

    return W_i


def meas2longitudinal_speed(
    V_meas_FL: float,
    V_meas_FR: float,
    V_meas_RL: float,
    V_meas_RR: float,
    steer: float,
    owmega_z: float,
    track: float,
) -> float:
    V_FL = V_meas_FL * math.cos(steer) - (owmega_z * track / 2)
    V_FR = V_meas_FR * math.cos(steer) + (owmega_z * track / 2)
    V_RL = V_meas_RL - (owmega_z * track / 2)
    V_RR = V_meas_RR + (owmega_z * track / 2)

    return (V_FL, V_FR, V_RL, V_RR)


if __name__ == "__main__":
    # テストコード

    s_time = 0.1  # 測定周期[s]
    
    # メンバ関数の引数
    owmega_z = 0.1
    v_FL_meas = 9.5
    v_FR_meas = 9.7
    v_RL_meas = 9.4
    v_RR_meas = 9.6
    steer = math.pi / 18
    a_x = -6
    
    # パラメータ (コンストラクタ引数)
    s1 = 4.5 # 定数
    s2 = 1.0 # 定数
    track = 1.5  # 車両トラック[m]
    Ax_threshold = 60.0

    # メンバ変数に保管する値
    omega_z_delay = 0.1
    v_delay_FL_meas = 10.0
    v_delay_FR_meas = 10.2
    v_delay_RL_meas = 9.8
    v_delay_RR_meas = 10.1

    (v_FL, v_FR, v_RL, v_RR) = meas2longitudinal_speed(
        v_FL_meas, v_FR_meas, v_RL_meas, v_RR_meas, steer, owmega_z, track
    )
    print(f"v_FL: {v_FL}, v_FR: {v_FR}, v_RL: {v_RL}, v_RR: {v_RR}")

    (v_delay_FL, v_delay_FR, v_delay_RL, v_delay_RR) = meas2longitudinal_speed(
        v_delay_FL_meas,
        v_delay_FR_meas,
        v_delay_RL_meas,
        v_delay_RR_meas,
        steer,
        omega_z_delay,
        track,
    )

    v_dot_FL = (v_FL - v_delay_FL) / s_time
    v_dot_FR = (v_FR - v_delay_FR) / s_time
    v_dot_RL = (v_RL - v_delay_RL) / s_time
    v_dot_RR = (v_RR - v_delay_RR) / s_time

    W_FL = longitudinal_speed_estimation(v_FL, v_delay_FL, v_dot_FL, a_x, s1, s2)
    W_FR = longitudinal_speed_estimation(v_FR, v_delay_FR, v_dot_FR, a_x, s1, s2)
    W_RL = longitudinal_speed_estimation(v_RL, v_delay_RL, v_dot_RL, a_x, s1, s2)
    W_RR = longitudinal_speed_estimation(v_RR, v_delay_RR, v_dot_RR, a_x, s1, s2)

    print(f"W_FL: {W_FL}, W_FR: {W_FR}, W_RL: {W_RL}, W_RR: {W_RR}")

    if abs(a_x) < Ax_threshold:
        # 加減速が小さい時は、重み付き平均（または単純平均）で算出
        W_sum = W_FL + W_FR + W_RL + W_RR
        V_est = (W_FL * v_FL + W_FR * v_FR + W_RL * v_RL + W_RR * v_RR) / W_sum

    elif a_x >= Ax_threshold:
        # 急加速時（駆動スリップの可能性）：車輪速度が車体速度より高くなるため、最小値を選択
        V_est = min(v_FL, v_FR, v_RL, v_RR)

    else:  # a_x <= -Ax_threshold の場合
        # 急減速・制動時（ロックの可能性）：車輪速度が車体速度より低くなるため、最大値を選択
        V_est = max(v_FL, v_FR, v_RL, v_RR)

    print(f"Estimated Longitudinal Speed: {V_est}")
