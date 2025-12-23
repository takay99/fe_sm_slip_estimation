import lowpassfilter_onestep 
import longitudinal_speed_estimation_class
import offset 
import vehicle_state_observer
import pandas as pd

if __name__ == "__main__":
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

    for i in range(1, len(output_data.iloc[:,0])):
       print(f"i: {output_data.iloc[i,0]}")
       str = float(output_data.iloc[i,11])
       str_dot = float(output_data.iloc[i,11]-output_data.iloc[i-1,11])/(float(output_data.iloc[i,0])-float(output_data.iloc[i-1,0]))

       