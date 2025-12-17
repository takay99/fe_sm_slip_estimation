from math import exp


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

if __name__ == "__main__":
    # テストコード
    # v_FL = 10.0
    # v_FR = 10.2
    # v_RL = 9.8
    # v_RR = 10.1
    
    # v_d_FL = 9.5
    # v_d_FR = 9.7
    # v_d_RL = 9.4
    # v_d_RR = 9.6

    s_time = 0.1  # センサの遅延時間[s]
    
    v_FL = 9.5
    v_FR = 9.7
    v_RL = 9.4
    v_RR = 9.6
    
    v_d_FL = 10.0
    v_d_FR = 10.2
    v_d_RL = 9.8
    v_d_RR = 10.1

    a_x = -6
    s1 = 4.5
    s2 = 1.0
    Ax_threshold = 60.0

    v_dot_FL = (v_FL - v_d_FL)/s_time
    v_dot_FR = (v_FR - v_d_FR)/s_time
    v_dot_RL = (v_RL - v_d_RL)/s_time
    v_dot_RR = (v_RR - v_d_RR)/s_time

    W_FL = longitudinal_speed_estimation(v_FL, v_d_FL, v_dot_FL, a_x, s1, s2)
    W_FR = longitudinal_speed_estimation(v_FR, v_d_FR, v_dot_FR, a_x, s1, s2)
    W_RL = longitudinal_speed_estimation(v_RL, v_d_RL, v_dot_RL, a_x, s1, s2)
    W_RR = longitudinal_speed_estimation(v_RR, v_d_RR, v_dot_RR, a_x, s1, s2)

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