import math
from math import exp


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


# --- 使用例 ---
if __name__ == "__main__":
    # クラスのインスタンス化（パラメータを設定）
    estimator = VelocityEstimator(
        s_time=0.1, s1=4.5, s2=1.0, track=1.5, ax_threshold=60.0
    )

    # 1ステップ目の入力（過去データがない場合の挙動確認）
    # 本来はループの中で回します
    v_meas_input = (9.5, 9.7, 9.4, 9.6)
    steer_input = math.pi / 18
    omega_z_input = 0.1
    ax_input = -6.0

    # ダミーで「前回値」をセットしたい場合は、内部変数を直接書き換えるか、
    # 2回 estimate を呼ぶことでシミュレーション可能です
    estimator.prev_v_wheels = (10.0, 10.2, 9.8, 10.1)  # テストコードのdelay相当
    estimator.is_first_run = False

    v_est = estimator.estimate(v_meas_input, steer_input, omega_z_input, ax_input)
    print(f"Estimated Longitudinal Speed: {v_est:.4f}")
