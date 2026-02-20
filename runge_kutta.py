from typing import Callable
import numpy as np
import numpy.typing as npt


def rk4_step(
    f: (
        Callable[
            [npt.NDArray[np.floating], float, npt.NDArray[np.floating]],
            npt.NDArray[np.floating],
        ]
        | Callable[[npt.NDArray[np.floating], float], npt.NDArray[np.floating]]
    ),
    current_integrate_value: npt.NDArray[np.floating],
    current_time: float,
    h_step: float,
    variation: npt.NDArray[np.floating] | None = None,
) -> npt.NDArray[np.floating]:
    """
    4次ルンゲクッタ法を1ステップ実行します。

    引数:
        beta_current (float): beta の現在の値。
        dot_phai_current (float): dot_phai の現在の値。
        delta_input (float): 現在の時刻における操舵入力 (delta)。
        v_speed (float): 車速。
        current_time (float): 現在の時刻。
        h_step (float): 時間ステップ (dt)。

    戻り値:
        tuple: (beta_next, dot_phai_next) - RK4 ステップ後の値。
    """

    # K1
    if variation is None:
        k1 = f(current_integrate_value, current_time)
    else:
        k1 = f(current_integrate_value, current_time, variation)
    assert len(k1) == len(current_integrate_value), "長さ一緒にしろ"

    # K2
    arr = []
    for i in range(len(current_integrate_value)):
        tmp = current_integrate_value[i] + 0.5 * h_step * k1[i]
        arr.append(tmp)
    if variation is None:
        k2 = f(arr, current_time + 0.5 * h_step)
    else:
        k2 = f(arr, current_time + 0.5 * h_step, variation)
    assert len(k2) == len(current_integrate_value), "長さ一緒にしろ"

    # K3
    arr = []
    for i in range(len(current_integrate_value)):
        tmp = current_integrate_value[i] + 0.5 * h_step * k2[i]
        arr.append(tmp)
    if variation is None:
        k3 = f(arr, current_time + 0.5 * h_step)
    else:
        k3 = f(arr, current_time + 0.5 * h_step, variation)
    assert len(k3) == len(current_integrate_value), "長さ一緒にしろ"

    # K4
    arr = []
    for i in range(len(current_integrate_value)):
        tmp = current_integrate_value[i] + h_step * k3[i]
        arr.append(tmp)
    if variation is None:
        k4 = f(arr, current_time + h_step)
    else:
        k4 = f(arr, current_time + h_step, variation)
    assert len(k4) == len(current_integrate_value), "長さ一緒にしろ"

    # beta と dot_phai を更新

    next_value = []
    for i in range(len(current_integrate_value)):
        next_value.append(
            current_integrate_value[i]
            + (h_step / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])
        )

    return np.array(next_value)
