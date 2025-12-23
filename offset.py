import numpy as np
import lowpassfilter_onestep 

class GyroOffsetManager:
    """ジャイロオフセットの推定とホールドを管理するクラス"""
    def __init__(self, cutoff_period: float, T: float,V_threshold:float):
        # 1/周期 = カットオフ周波数
        self.offset_filter = lowpassfilter_onestep.LowPassFilterOnestep(1.0 / cutoff_period, T)
        self.current_offset = 0.0
        self.V_threshold = V_threshold
    def estimate(self, omega_z_raw: float, v_est: float) -> float:
        """
        車両状態に応じてオフセットを更新し、補正後の値を返す
        """
        # 1. アクティベーションロジック (例: 車速が0.01m/s未満なら停止とみなす)
        is_stopping = abs(v_est) < self.V_threshold

        if is_stopping:
            # 停止中：フィルタを回してオフセットを逐次更新（学習）
            self.current_offset = self.offset_filter.filter(omega_z_raw)
        else:
            # 走行中：フィルタを更新せず、current_offset は前回の値を保持（ホールド）
            pass

        # 2. 常に現在のオフセットで補正して返す
        return omega_z_raw - self.current_offset

    def get_current_offset(self) -> float:
        """現在の推定オフセット値を確認する用"""
        return self.current_offset
    

class AxOffsetManager:
    """縦加速度(Ax)のオフセット推定とホールドを管理"""
    def __init__(self, cutoff_period: float, T: float):
        # 推定条件: 1500秒（非常にゆっくりとした学習）
        self.offset_filter = lowpassfilter_onestep.LowPassFilterOnestep(1.0 / cutoff_period, T)
        self.current_offset = 0.0

    def estimate(self, ax_raw: float, dvx_dt: float, vx: float, omega_z: float) -> float:
        """
        :param ax_raw: 縦加速度センサ値 [m/s^2]
        :param dvx_dt: 車輪速から計算した速度微分 [m/s^2]
        :param vx: 推定車速 [m/s]
        :param omega_z: ヨーレート [rad/s] (または deg/s, 条件判定用)
        """
        # --- アクティベーションロジック (第4章 B項) ---
        # 1. 車両が動いている (Vx > 0)
        cond1 = vx > 0.1
        # 2. 加速度が小さい (|Ax| < 1 m/s^2)
        cond2 = abs(ax_raw) < 1.0
        # 3. 直進している (|omega_z| <= 5 deg/s) ※単位はシステムに合わせて調整してください
        cond3 = abs(np.degrees(omega_z)) <= 5.0

        if cond1 and cond2 and cond3:
            # 条件成立時: (センサ値 - 理論値) の不一致分をオフセットとして学習
            error = ax_raw - dvx_dt
            self.current_offset = self.offset_filter.filter(error)
        else:
            # 条件不成立時: フィルタを回さず、現在の値をホールド
            pass

        return ax_raw - self.current_offset
    
class AyOffsetManager:
    """横加速度(Ay)のオフセット推定とホールドを管理"""
    def __init__(self, cutoff_period: float, T: float):
        # 推定条件: 1500秒
        self.offset_filter = lowpassfilter_onestep.LowPassFilterOnestep(1.0 / cutoff_period, T)
        self.current_offset = 0.0

    def estimate(self, ay_raw: float, vx: float, omega_z: float) -> float:
        """
        :param ay_raw: 横加速度センサ値 [m/s^2]
        :param vx: 推定車速 [m/s]
        :param omega_z: ヨーレート [rad/s]
        """
        # --- アクティベーションロジック (第4章 C項) ---
        # Ayの場合、一般的には「走行中」であることが条件となります
        is_moving = vx > 0.5 

        if is_moving:
            # 推定式: ΔAy = Aoff_y - (omega_z * Vx)
            # 長期平均で Vy_dot がゼロになるという仮定に基づき、差分をフィルタリング
            error = ay_raw - (omega_z * vx)
            self.current_offset = self.offset_filter.filter(error)
        else:
            # 停止中などはホールド
            pass

        return ay_raw - self.current_offset