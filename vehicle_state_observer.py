import numpy as np


class vihicleStateObserver:
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

        dV_hat_dt = self.calculate_dV_hat_dt(Ax, Ay, omega_z, Vx_meas, F_t)

        # 離散化 (オイラー法): V_hat(t+dt) = V_hat(t) + dV_hat/dt * dt
        self.V_hat = self.V_hat + dV_hat_dt * self.dt

        # Vx_hatが負にならないように最小値を設定
        self.V_hat[0] = max(self.V_hat[0], 0.01)

        # 横滑り角の計算: beta = arctan(Vy / Vx) (論文 式3)
        beta_hat_rad = np.arctan2(self.V_hat[1], self.V_hat[0])
        self.beta_hat_deg = np.rad2deg(beta_hat_rad)

        return self.beta_hat_deg

    def get_estimated_velocity(self):
        """推定された縦速度と横速度を返します。"""
        return self.V_hat[0], self.V_hat[1]


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
    curve_end_index = int(10.0 / dt_val)

    Ay_data[curve_start_index:300] = 20 * np.deg2rad(10.0) * 0.8
    Ay_data[301:curve_end_index] = 20 * np.deg2rad(10.0)

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
