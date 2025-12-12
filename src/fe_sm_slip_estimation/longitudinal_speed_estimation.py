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
    v = 10.0
    v_d = 9.5
    v_dot = 0.5
    a_x = 0.6
    s1 = 4.5
    s2 = 1.0

    result = longitudinal_speed_estimation(v, v_d, v_dot, a_x, s1, s2)
    print(f"Longitudinal Speed Estimation Result: {result}")
