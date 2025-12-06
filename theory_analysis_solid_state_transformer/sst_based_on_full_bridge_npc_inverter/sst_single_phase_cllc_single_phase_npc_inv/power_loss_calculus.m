% clear all
% close all
% clc
% 
% load sim_result_1.mat;
clc

N2 = Nc;
N1 = N2 - floor(0.5/tc);
figure; plot(time_tc_sim(N1:N2), inverter_device_data_modA_sim(N1:N2,6)); grid on

idc_grid = mean(current_battery_sim(N1:N2));
udc_grid = mean(voltage_battery_sim(N1:N2));
p_dc_grid = idc_grid*udc_grid

p_loss_dab1_Q1 = mean(inverter_1_dab_devices_data_modA_sim(N1:N2,1));
p_loss_dab1_Q2 = mean(inverter_1_dab_devices_data_modA_sim(N1:N2,6));
p_loss_dab2_Q1 = mean(inverter_2_dab_devices_data_modA_sim(N1:N2,1));
p_loss_dab2_Q2 = mean(inverter_2_dab_devices_data_modA_sim(N1:N2,6));

p_loss_afe_Q1 = mean(inverter_device_data_modA_sim(N1:N2,1));
p_loss_afe_Q2 = mean(inverter_device_data_modA_sim(N1:N2,6));

p_loss_dab = 2*(p_loss_dab1_Q1+p_loss_dab1_Q2) + 2*(p_loss_dab2_Q1+p_loss_dab2_Q2)
p_loss_inv = 4*(p_loss_afe_Q1+p_loss_afe_Q2)

ac_grid_voltage_rms = sqrt(mean(ac_grid_voltage_sim.^2));
ac_grid_current_rms = sqrt(mean(ac_grid_current_sim.^2));

p_ac_grid = mean(ac_grid_voltage_sim.*ac_grid_current_sim)

ploss_calc = 2*p_loss_dab + 2*p_loss_inv

efficiency1 = 1 - abs(ploss_calc/p_dc_grid)
efficiency2 = 1 - abs(ploss_calc/p_ac_grid)

p_dab_modA_input = mean(dab_voltage_input_modA_sim.*dab_current_input_modA_sim)

i1_tr = sqrt(mean(i1_dab_transformer_modA_sim.^2))
i2_tr = sqrt(mean(i2_dab_transformer_modA_sim.^2))
u1_tr = sqrt(mean(u1_dab_transformer_modA_sim.^2))
u2_tr = sqrt(mean(u2_dab_transformer_modA_sim.^2))
