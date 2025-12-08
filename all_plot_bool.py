import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import CheckButtons


def all_plot_bool(output_data):
    time = output_data.iloc[:, 0] / float(1000)

    fig, ax = plt.subplots()

    # グラフウィンドウの作成
    # plt.figure(figsize=(10, 6))

    # 最初のプロット: poly_acc_1
    (l0,) = ax.plot(time, output_data.iloc[:, 1], label="acc x r")  # output_data(:,3)
    # 後続のプロット
    (l1,) = ax.plot(time, output_data.iloc[:, 2], label="acc y r")  # output_data(:,3)
    (l2,) = ax.plot(time, output_data.iloc[:, 3], label="gyr z r")  # output_data(:,4)
    (l3,) = ax.plot(time, output_data.iloc[:, 4], label="acc x f")  # output_data(:,3)
    (l4,) = ax.plot(time, output_data.iloc[:, 5], label="acc y f")  # output_data(:,6)
    (l5,) = ax.plot(time, output_data.iloc[:, 6], label="gyr z f")  # output_data(:,7)
    (l6,) = ax.plot(time, output_data.iloc[:, 7], label="m acc 1")  # output_data(:,8)
    (l7,) = ax.plot(time, -output_data.iloc[:, 8], label="m acc 2")  # -output_data(:,9)
    (l8,) = ax.plot(
        time, -output_data.iloc[:, 9], label="m vel 1"
    )  # -output_data(:,10)
    (l9,) = ax.plot(time, output_data.iloc[:, 10], label="m vel 2")  # output_data(:,11)
    (l10,) = ax.plot(time, output_data.iloc[:, 11], label="str")  # output_data(:,12)
    (l11,) = ax.plot(
        time, -output_data.iloc[:, 12] / 100, label="current1"
    )  # -output_data(:,13)/100
    (l12,) = ax.plot(
        time, output_data.iloc[:, 13] / 100, label="current2"
    )  # output_data(:,14)/100

    lines_by_label = {
        l.get_label(): l
        for l in [l0, l1, l2, l3, l4, l5, l6, l7, l8, l9, l10, l11, l12]
    }
    line_colors = [l.get_color() for l in lines_by_label.values()]

    rax = ax.inset_axes([0.0, 0.0, 0.08, 0.4])
    check = CheckButtons(
        ax=rax,
        labels=lines_by_label.keys(),
        actives=[l.get_visible() for l in lines_by_label.values()],
        label_props={"color": line_colors},
        frame_props={"edgecolor": line_colors},
        check_props={"facecolor": line_colors},
    )

    def callback(label):
        ln = lines_by_label[label]
        ln.set_visible(not ln.get_visible())
        ln.figure.canvas.draw_idle()

    check.on_clicked(callback)
    plt.show()

    return fig, ax
