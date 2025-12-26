import math
from math import exp
import numpy as np


class VelocityEstimator:
    def __init__(
        self, s_time: float, s1: float, s2: float, track: float, ax_threshold: float
    ):
        """
        コンストラクタ: 定数やパラメータを初期化
        """
        self.s_time = s_time
        self.s1 = s1
        self.s2 = s2
        self.track = track
        self.ax_threshold = ax_threshold

        # 前回値を保持する変数（初期値は0）
        self.prev_v_wheels = (0.0, 0.0, 0.0, 0.0)
        self.is_first_run = True

    def _calc_wheel_longitudinal_speeds(self, v_meas, steer, omega_z):
        """4輪の車輪速度を計算する内部関数"""
        v_fl_meas, v_fr_meas, v_rl_meas, v_rr_meas = v_meas

        v_fl = v_fl_meas * math.cos(steer) - (omega_z * self.track / 2)
        v_fr = v_fr_meas * math.cos(steer) + (omega_z * self.track / 2)
        v_rl = v_rl_meas - (omega_z * self.track / 2)
        v_rr = v_rr_meas + (omega_z * self.track / 2)
        return (v_fl, v_fr, v_rl, v_rr)

    def _calc_weight(self, v, v_delay, v_dot, ax):
        """重みW_iを計算する内部関数"""
        term1 = -((ax - v_dot) ** 2) / (2 * self.s1**2)
        term2 = -((v_delay - v) ** 2) / (2 * self.s2**2)
        return exp(term1 + term2)

    def estimate(self, v_meas: tuple, steer: float, omega_z: float, ax: float) -> float:
        """
        現在の観測値から推定車体速度 V_est を計算して返す
        v_meas: (v_FL_meas, v_FR_meas, v_RL_meas, v_RR_meas)
        """
        # 1. 現在の各輪速度を計算
        curr_v_wheels = self._calc_wheel_longitudinal_speeds(v_meas, steer, omega_z)

        # 初回実行時は前回値がないため、現在値を前回値として処理（加速度0）
        if self.is_first_run:
            self.prev_v_wheels = curr_v_wheels
            self.is_first_run = False

        # 2. 重み計算と推定
        v_dots = []
        weights = []
        for curr, prev in zip(curr_v_wheels, self.prev_v_wheels):
            v_dot = (curr - prev) / self.s_time
            w = self._calc_weight(curr, prev, v_dot, ax)
            v_dots.append(v_dot)
            weights.append(w)

        # 3. 加速度条件による分岐
        if abs(ax) < self.ax_threshold:
            w_sum = sum(weights)
            # ゼロ除算防止
            if w_sum == 0:
                v_est = sum(curr_v_wheels) / 4.0
            else:
                v_est = sum(w * v for w, v in zip(weights, curr_v_wheels)) / w_sum
        elif ax >= self.ax_threshold:
            v_est = min(curr_v_wheels)
        else:  # ax <= -self.ax_threshold
            v_est = max(curr_v_wheels)

        # 今回の値を次回のために保存
        self.prev_v_wheels = curr_v_wheels

        return v_est



class VehicleStateObserver:
    def __init__(self, a0, a1, a2, dt):
        self.a0 = a0
        self.a1 = a1
        self.a2 = a2
        self.dt = dt

        # 推定状態ベクトル [Vx, Vy]T を格納するメンバ変数。初期値はゼロ。
        self.V_hat = np.array([0.0, 0.0])

        # 推定された横滑り角（デバッグ用）
        self.beta_hat_deg = 0.0

    def calculate_dV_hat_dt(self, Ax, Ay, omega_z, Vx_meas, F_t):
        """
        メンバ関数1: 推定状態の変化率 d(V_hat)/dt を計算します (論文 式5の右辺)。

        :param Ax: 測定された縦加速度 (Ax(t))
        :param Ay: 測定された横加速度 (Ay(t))
        :param omega_z: 測定されたヨーレート (omega_z(t))
        :param Vx_meas: 測定された縦速度 (V_x) - ホイール速度から推定される値
        :param F_t: ヒューリスティック関数 F(t) の値
        :return: 推定状態の変化率ベクトル [dVx/dt, dVy/dt]T
        """
        Vx_hat = self.V_hat[0]
        Vy_hat = self.V_hat[1]

        # ----------------------------------------------------
        # 1. 状態行列 M (A-K(ωz)C) の計算
        # ----------------------------------------------------
        M_11 = -self.a0 - self.a1 * np.abs(omega_z)
        M_12 = omega_z 
        M_21 = -(self.a2 + 1) * omega_z
        M_22 = -F_t

        # M * V_hat (行列と状態ベクトルの積)
        dV_hat_dt_M = np.array(
            [
                M_11 * Vx_hat + M_12 * Vy_hat,  # dVx/dt の第1項
                M_21 * Vx_hat + M_22 * Vy_hat,  # dVy/dt の第1項
            ]
        )

        # ----------------------------------------------------
        # 2. 加速度入力項 (B * A_inputs) の計算
        # ----------------------------------------------------
        dV_hat_dt_B = np.array([Ax, Ay])

        # ----------------------------------------------------
        # 3. 速度フィードバック補正項 (K * V_x) の計算
        # ----------------------------------------------------
        K_1 = self.a0 + self.a1 * np.abs(omega_z)
        K_2 = self.a2 * omega_z

        K_Vx = np.array(
            [K_1 * Vx_meas, K_2 * Vx_meas]  # dVx/dt の補正項  # dVy/dt の補正項
        )

        dV_hat_dt = dV_hat_dt_M + dV_hat_dt_B + K_Vx

        return dV_hat_dt

    def update_state(self, Ax, Ay, omega_z, Vx_meas, F_t):
        """
        メンバ関数2: 状態の変化率を積分し、新しい推定状態 (V_hat) を更新します。
        この関数を実行すれば、オブザーバーが1ステップ進みます。

        :param Ax: 測定された縦加速度 (Ax(t))
        :param Ay: 測定された横加速度 (Ay(t))
        :param omega_z: 測定されたヨーレート (omega_z(t))
        :param Vx_meas: 測定された縦速度 (V_x) - ホイール速度から推定される値
        :param F_t: ヒューリスティック関数 F(t) の値
        :return: 更新された横滑り角 (beta_hat_deg)
        """

        self.dV_hat_dt = self.calculate_dV_hat_dt(Ax, Ay, omega_z, Vx_meas, F_t)

        # 離散化 (オイラー法): V_hat(t+dt) = V_hat(t) + dV_hat/dt * dt
        self.V_hat = self.V_hat + self.dV_hat_dt * self.dt

        # Vx_hatが負にならないように最小値を設定
        # self.V_hat[0] = max(self.V_hat[0], 0.01) 

        # 横滑り角の計算: beta = arctan(Vy / Vx) (論文 式3)
        beta_hat_rad = np.arctan2(self.V_hat[1], self.V_hat[0])
        self.beta_hat_deg = (beta_hat_rad)

        return self.beta_hat_deg

    def get_estimated_velocity(self):
        """推定された縦速度と横速度を返します。"""
        return self.V_hat[0], self.V_hat[1]

    def get_estimated_slip_angle_dot(self):
        """推定された横滑り角速度を返します。"""
        beta_hat_dot = np.arctan2(self.dV_hat_dt[0], -self.dV_hat_dt[1])
        return beta_hat_dot
# ----------------------------------------------------
# 使用例（簡易的なシミュレーション）
# ----------------------------------------------------
if __name__ == "__main__":
    # パラメータ設定 (論文 Table 1 の値を使用)
    a0_val = 10
    a1_val = 5
    a2_val = 10
    dt_val = 0.01  # 100 Hzで実行 (論文にもあるように)

    observer = vihicleStateObserver(a0_val, a1_val, a2_val, dt_val)

    # シミュレーションデータ (適当な値 - 直進から緩やかなカーブを想定)
    time_points = np.arange(0, 10, dt_val)

    # 初期状態: 直進
    Ax_data = np.zeros_like(time_points)
    Ay_data = np.zeros_like(time_points)
    omega_z_data = np.zeros_like(time_points)
    Vx_meas_data = np.full_like(time_points, 20.0)  # 72 km/h
    F_t_data = np.full_like(time_points, 1.0)  # 直進なのでFは大きめ

    # 2秒後からカーブを開始 (Ax=0, Ay=3m/s^2, omega_z=10deg/s)
    curve_start_index = int(2.0 / dt_val)
    curve_end_index = int(4.0 / dt_val)

    Ay_data[curve_start_index:curve_end_index] = 3.0
    omega_z_data[curve_start_index:curve_end_index] = np.deg2rad(10.0)
    F_t_data[curve_start_index:curve_end_index] = 0.0  # カーブ中はFはゼロ

    print("--- 簡易シミュレーション実行結果 ---")
    for i, t in enumerate(time_points):
        # 状態更新を実行
        beta_hat = observer.update_state(
            Ax=Ax_data[i],
            Ay=Ay_data[i],
            omega_z=omega_z_data[i],
            Vx_meas=Vx_meas_data[i],
            F_t=F_t_data[i],
        )
        Vx_hat, Vy_hat = observer.get_estimated_velocity()

        if i % 100 == 0:  # 1秒ごとに出力
            print(
                f"Time: {t:.2f}s | Ax: {Ax_data[i]:.1f}, Ay: {Ay_data[i]:.1f}, ωz: {np.rad2deg(omega_z_data[i]):.1f}°/s, F: {F_t_data[i]:.1f}"
            )
            print(
                f"  Vx_hat: {Vx_hat:.2f} m/s, Vy_hat: {Vy_hat:.2f} m/s | β_hat: {beta_hat:.2f}°"
            )

    print("------------------------------------")
    print(f"最終推定横滑り角: {beta_hat:.2f} 度")


# # --- 使用例 ---
# if __name__ == "__main__":
#     # クラスのインスタンス化（パラメータを設定）
#     estimator = VelocityEstimator(
#         s_time=0.1, s1=4.5, s2=1.0, track=1.5, ax_threshold=60.0
#     )

#     # 1ステップ目の入力（過去データがない場合の挙動確認）
#     # 本来はループの中で回します
#     v_meas_input = (9.5, 9.7, 9.4, 9.6)
#     steer_input = math.pi / 18
#     omega_z_input = 0.1
#     ax_input = -6.0

#     # ダミーで「前回値」をセットしたい場合は、内部変数を直接書き換えるか、
#     # 2回 estimate を呼ぶことでシミュレーション可能です
#     estimator.prev_v_wheels = (10.0, 10.2, 9.8, 10.1)  # テストコードのdelay相当
#     estimator.is_first_run = False

#     v_est = estimator.estimate(v_meas_input, steer_input, omega_z_input, ax_input)
#     print(f"Estimated Longitudinal Speed: {v_est:.4f}")
