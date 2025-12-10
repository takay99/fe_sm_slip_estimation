import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import all_plot_bool
from matplotlib.widgets import CheckButtons
import lowpassfilter


def main():
    print("Hello from fe-sm-data-analize!")

    output_data = pd.read_csv(
        "LOG00289.txt",
        header=None,
        delim_whitespace=False,
        # 古い引数の代わりに新しい引数を使う
        on_bad_lines="skip",
        # error_bad_lines=False  <- これは削除する
    )
    print(output_data)

    for col in output_data.columns:
        output_data[col] = pd.to_numeric(output_data[col], errors="coerce")

    # 時間軸（最初の列）に NaN がある行を削除する
    # 他のデータ列に NaN が残ってもプロットは可能ですが、時間軸は連続している必要があるため
    output_data = output_data.dropna(subset=[0]).reset_index(drop=True)

    figure, axe = all_plot_bool.all_plot_bool(output_data)

    print("end")


if __name__ == "__main__":

    main()
