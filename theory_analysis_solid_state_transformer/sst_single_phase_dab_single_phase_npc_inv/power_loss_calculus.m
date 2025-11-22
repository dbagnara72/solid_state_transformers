clear all
close all
clc

load sim_result_1.mat;
idc_grid = mean(current_battery_sim);
udc_grid = mean(voltage_battery_sim);
p_dc_grid = mean(current_battery_sim.*voltage_battery_sim)

p_loss_dab1_Q1 = mean(inverter_1_dab_devices_data_modA_sim(:,1));
p_loss_dab1_Q2 = mean(inverter_1_dab_devices_data_modA_sim(:,6));
p_loss_dab2_Q1 = mean(inverter_2_dab_devices_data_modA_sim(:,1));
p_loss_dab2_Q2 = mean(inverter_2_dab_devices_data_modA_sim(:,6));

p_loss_afe_Q1 = mean(inverter_device_data_modA_sim(:,1));
p_loss_afe_Q2 = mean(inverter_device_data_modA_sim(:,6));

p_loss_dab = 2*(p_loss_dab1_Q1+p_loss_dab1_Q2) + 2*(p_loss_dab2_Q1+p_loss_dab2_Q2)
p_loss_inv = 4*(p_loss_afe_Q1+p_loss_afe_Q2)

ac_grid_voltage_rms = sqrt(mean(ac_grid_voltage_sim.^2));
ac_grid_current_rms = sqrt(mean(ac_grid_current_sim.^2));

p_ac_grid = mean(ac_grid_voltage_sim.*ac_grid_current_sim)

p_dc_est = p_ac_grid - 2*p_loss_dab -2*p_loss_inv

ploss_calc = p_ac_grid + p_dc_grid

efficiency = 1 - ploss_calc/p_ac_grid

p_dab_modA_input = mean(dab_voltage_input_modA_sim.*dab_current_input_modA_sim)
