import fe_sm_slip_estimation.lowpassfilter_onestep as lpfs
import fe_sm_slip_estimation.longitudinal_speed_estimation_class as lse
import fe_sm_slip_estimation.offset as offset 
import fe_sm_slip_estimation.vehicle_state_observer as vehicle_state_observer
import fe_sm_slip_estimation.heuristic_schedule as heuristic_schedule
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import fe_sm_slip_estimation.lowpassfilter as lowpassfilter


if __name__ == "__main__":
    print("Hello from fe-sm-data-analize!")

    output_data = pd.read_csv(
        "LOG00297.txt",
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
    # 【追加】他の列に紛れ込んでいる NaN を前後の値で線形補間し、それでも残る場合は0で埋める
    output_data = output_data.interpolate(method='linear', limit_direction='both').fillna(0.0)

    output_data.iloc[:, 0] = output_data.iloc[:, 0] - output_data.iloc[0, 0]
    output_data.iloc[:,1] = -output_data.iloc[:,1]
    output_data.iloc[:,2] = -output_data.iloc[:,2] 
    output_data.iloc[:,3] = -output_data.iloc[:,3]
    output_data.iloc[:,4] = -output_data.iloc[:,4] 
    output_data.iloc[:,5] = -output_data.iloc[:,5] 
    output_data.iloc[:,6] = -output_data.iloc[:,6] 
    output_data.iloc[:,9] = lowpassfilter.lowpass_filter(output_data.iloc[:,9], cutoff_freq=2, T=0.01)
    output_data.iloc[:,10] = lowpassfilter.lowpass_filter(output_data.iloc[:,10], cutoff_freq=2.0, T=0.01)
    beta_dot = float(0.0)
    beta_dot_prev = float(0.0)

    Ax_offset = float(0.0)
    Ay_offset = float(0.0)
    omega_z_offset = float(0.0)
    
    Vx_prev = float(0.0)

    StrDotFilter = lpfs.LowPassFilterOnestep(cutoff_freq=10.0, T=0.01)
    OmegaZDotFilter = lpfs.LowPassFilterOnestep(cutoff_freq=10.0, T=0.01)
    BetaDotFilter = lpfs.LowPassFilterOnestep(cutoff_freq=10.0, T=0.01)
    dVxDtFilter = lpfs.LowPassFilterOnestep(cutoff_freq=2.0, T=0.01)

    GyroOffset = offset.GyroOffsetManager(cutoff_period=150.0, T=0.01,V_threshold=0.01)
    AxOffset = offset.AxOffsetManager(cutoff_period=150.0, T=0.01)
    AyOffset = offset.AyOffsetManager(cutoff_period=150.0, T=0.01)

    VelEst = lse.VelocityEstimator(
        s_time=0.01, s1=4.5, s2=1.0, track=0.165, ax_threshold=1.0)

    BetaEst = vehicle_state_observer.VehicleStateObserver(10,5,10,0.01)
    beta_hat_storage = np.zeros(len(output_data))
    element_dV_hat_storage = np.zeros((len(output_data), 4))
    V_hat = np.zeros((len(output_data),2))  
    dot_V_hat = np.zeros((len(output_data),2))
    offsetedAxy_strage = np.zeros((len(output_data),2))
    V_est_array = np.zeros((len(output_data)))
    F_array = np.zeros((len(output_data)))

    test_Axoffset_storage = np.zeros((len(output_data),2)) 
    test_Ayoffset_storage = np.zeros((len(output_data),2)) 

    for i in range(1, len(output_data.iloc[:,0])):
        # print(f"i: {output_data.iloc[i,0]}")

        ################
        time = float(output_data.iloc[i,0]/1000.0)  # ms -> s
        time_prev = float(output_data.iloc[i-1,0]/1000.0)  # ms -> s
        
        str = float(output_data.iloc[i,11])
        str_dot = StrDotFilter.filter(float(output_data.iloc[i,11]-output_data.iloc[i-1,11])/(time-time_prev))

        omega_z = float(output_data.iloc[i,3])
        omega_z_dot = OmegaZDotFilter.filter(float((output_data.iloc[i,3])-(output_data.iloc[i-1,3]))/(time-time_prev))

        beta_dot =  float(beta_dot)
        beta_ddot = BetaDotFilter.filter(float((beta_dot - beta_dot_prev)/(time - time_prev)))

        # Ax_raw = float(output_data.iloc[i,2]+output_data.iloc[i,5])/2.0
        # Ax_raw = float(output_data.iloc[i,5] + output_data.iloc[i,2])/2.0
        Ax_raw = float(output_data.iloc[i,2])
        # Ay_raw = float(output_data.iloc[i,4] + output_data.iloc[i,1])/2.0
        Ay_raw = float(output_data.iloc[i,1])

        if i < 10:
            print("[i,5]", output_data.iloc[i,5], "[i,2]", output_data.iloc[i,2])
            print("[i,4]", output_data.iloc[i,4], "[i,1]", output_data.iloc[i,1])
        omega_z_raw = float(output_data.iloc[i,6])
        ################

        #######velocity estimation#######
        V_meas = (float(-output_data.iloc[i,9]),float(-output_data.iloc[i,9]),float(output_data.iloc[i,10]),float(output_data.iloc[i,10]))
        # print("V_meas:", V_meas)
        V_est = VelEst.estimate(V_meas,str,omega_z_offset, Ax_offset)

        #######offset#######
        omega_z_offset = GyroOffset.estimate(omega_z_raw,V_est)
        dvx_dt = dVxDtFilter.filter((V_est - Vx_prev)/(time - time_prev))
        Ax_offset,offset_value_x, flag_x = AxOffset.estimate(Ax_raw, dvx_dt, V_est, omega_z)
        Ay_offset,offset_value_y, flag_y = AyOffset.estimate(Ay_raw,V_est, omega_z)
        offsetedAxy_strage[i,0] = Ax_offset
        offsetedAxy_strage[i,1] = Ay_offset

        test_Axoffset_storage[i,0] = offset_value_x
        test_Axoffset_storage[i,1] = flag_x
        test_Ayoffset_storage[i,0] = offset_value_y
        test_Ayoffset_storage[i,1] = flag_y


        # Ax_offset = Ax_raw
        # Ay_offset = Ay_raw
        ####

        #######heuristic schedule#######        
        F_str = heuristic_schedule.heuristic_schedule(str, str_dot, 0.1, 0.1 )
        F_omegaz = heuristic_schedule.heuristic_schedule(omega_z, omega_z_dot, 0.18, 0.18 )
        F_betadot = heuristic_schedule.heuristic_schedule(beta_dot, beta_ddot, 0.06, 0.3 )
        F = F_str * F_omegaz * F_betadot
        F_array[i] = F
        ###########################

        ###########################
        # beta_hat_deg = BetaEst.update_state(Ay_offset, Ax_offset,  omega_z_offset,   V_est,  F_t=F)
        # 修正後のコード
        beta_hat_deg, element_dV_dat = BetaEst.update_state(Ax_offset, Ay_offset,  omega_z_offset,   V_est,  F_t=F)
        # beta_hat_deg = BetaEst.update_state(Ax_offset, Ay_offset,  omega_z_offset,   V_est,  F_t=F)

        beta_hat_storage[i] = beta_hat_deg
        element_dV_hat_storage[i,0:4] = element_dV_dat.T
        
        beta_dot = BetaEst.get_estimated_slip_angle_dot()
        V_hat[i] = BetaEst.get_estimated_velocity()
        dot_V_hat[i] = BetaEst.get_estimated_dotvelocity()

        print("dV_hat:", element_dV_dat, "V_hat", V_hat[i])
        if i < 10:
           print("V_hat:", V_hat[i])
        V_est_array[i] = V_est
        # print("Estimated Slip Angle: {:.4f} deg".format(beta_hat_deg))
        ###########################

        ###########################
        V_prev = V_est
        # print(f"V_est: {V_est}, Ax_offset: {Ax_offset}, Ay_offset: {Ay_offset}, omega_z_offset: {omega_z_offset}, F: {F} ")
        # 
        # 
        beta_dot_prev = beta_dot
        #######
        
    plt.figure(figsize=(10, 6))
    # 最初のプロット: poly_acc_1
    # plt.plot(output_data.iloc[:,0]/1000.0, output_data.iloc[:, 1], label='acc x r') # output_data(:,3)
    # # 後続のプロット
    # plt.plot(output_data.iloc[:,0]/1000.0, output_data.iloc[:, 2], label='acc y r') # output_data(:,3)
    # plt.plot(output_data.iloc[:,0]/1000.0, output_data.iloc[:, 3], label='gyr z r') # output_data(:,4)
    # plt.plot(output_data.iloc[:,0]/1000.0, output_data.iloc[:, 4], label='acc x f') # output_data(:,3)
    # plt.plot(output_data.iloc[:,0]/1000.0, output_data.iloc[:, 5], label='acc y f') # output_data(:,6)
    # plt.plot(output_data.iloc[:,0]/1000.0, output_data.iloc[:, 6], label='gyr z r') # output_data(:,7)
    # plt.plot(output_data.iloc[:,0]/1000.0, np.rad2deg(beta_hat_storage[:]), label='estimated slip angle') # output_data(:,7)
    plt.plot(output_data.iloc[:,0]/1000.0, V_hat[:,0], label='V_x') 
    plt.plot(output_data.iloc[:,0]/1000.0, V_hat[:,1], label='V_y')
    # plt.plot(output_data.iloc[:,0]/1000.0, test_dV_hat_storage[:,0], label='d_Vx/dt') 
    # plt.plot(output_data.iloc[:,0]/1000.0, test_dV_hat_storage[:,1], label='d_Vy/dt')
    # output_data(:,7)
    plt.plot(output_data.iloc[:,0]/1000.0, V_est_array[:], label='estimated V')
    plt.plot(output_data.iloc[:,0]/1000.0, (-output_data.iloc[:,9]), label='wheel speed f')
    # グリッドの表示
    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('Data Plot')
    plt.legend()
    # plt.show()
    
    plt.figure(figsize=(10, 6))
    plt.plot(output_data.iloc[:,0]/1000.0, element_dV_hat_storage[:,0], label='0') 
    plt.plot(output_data.iloc[:,0]/1000.0, element_dV_hat_storage[:,1], label='1') 
    plt.plot(output_data.iloc[:,0]/1000.0, element_dV_hat_storage[:,2], label='2')
    plt.plot(output_data.iloc[:,0]/1000.0, element_dV_hat_storage[:,3], label='3')

    plt.plot(output_data.iloc[:,0]/1000.0, dot_V_hat[:,0], label='2')
    plt.plot(output_data.iloc[:,0]/1000.0, dot_V_hat[:,1], label='3')
    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('Data Plot')
    plt.legend()
    

    plt.figure(figsize=(10, 6))
    plt.plot(output_data.iloc[:,0]/1000.0, output_data.iloc[:,2], label='x') 
    plt.plot(output_data.iloc[:,0]/1000.0, output_data.iloc[:,1], label='y') 

    plt.plot(output_data.iloc[:,0]/1000.0, offsetedAxy_strage[:,0], label='x_offset')
    plt.plot(output_data.iloc[:,0]/1000.0, offsetedAxy_strage[:,1], label='y_offset')
    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('Data Plot')
    plt.legend()
    
    plt.figure(figsize=(10, 6))
    plt.plot(output_data.iloc[:,0]/1000.0, test_Axoffset_storage[:,0], label='x_erorr') 
    plt.plot(output_data.iloc[:,0]/1000.0, test_Axoffset_storage[:,1], label='x_flag') 
    plt.plot(output_data.iloc[:,0]/1000.0, test_Ayoffset_storage[:,0], label='y_erorr') 
    plt.plot(output_data.iloc[:,0]/1000.0, test_Ayoffset_storage[:,1], label='y_flag') 
    plt.grid(True)
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('Data Plot')
    plt.legend()

    plt.show()


    # plt.figure(figsize=(10, 4))
    # plt.plot(output_data.iloc[:,0]/1000.0, F_array, label='F', color='magenta')
    # plt.grid(True)
    # plt.xlabel('Time (s)')
    # plt.ylabel('F')
    # plt.title('Heuristic F over Time')
    # plt.legend()
    # plt.show()

    # # 【確認用】後輪センサと前輪センサの波形が一致しているかプロット
    # plt.figure(figsize=(10, 4))
    # plt.plot(output_data.iloc[:, 2], label="Rear Lateral (Col 1)", alpha=0.7)
    # plt.plot(output_data.iloc[:, 5], label="Front Lateral? (Col 4)", alpha=0.7)
    # plt.title("Sensor Consistency Check")
    # plt.legend()
    # plt.show()
    #     # print(f"steer: {str}, steer_dot: {str_dot}")

    