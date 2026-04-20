import numpy as np


def lowpass_filter(data, cutoff_freq, T):
    data = np.asarray(data)
    omega_c_T = cutoff_freq * 2 * np.pi * T
    alpha = omega_c_T / (1 + omega_c_T)
    D = 2 + cutoff_freq * T
    # a1 = (cutoff_freq * T - 2) / D
    # b1 = cutoff_freq * T / D
    # b0 = b1
    # 出力配列
    y = np.zeros(len(data))

    # 初期値: 最初の入力が数値ならそれを、NaNなら0を使う
    if len(data) == 0:
        return y
    if not np.isnan(data[0]):
        y[0] = data[0]
    else:
        y[0] = 0.0

    # 再帰処理中に入力が NaN の場合は直前の出力をそのまま継続して使う
    # これにより入力中の NaN が後続出力へ伝播するのを防ぐ
    for i in range(1, len(data)):
        if np.isnan(data[i]):
            y[i] = y[i - 1]
        else:
            y[i] = y[i - 1] + alpha * (data[i] - y[i - 1])
    return y
