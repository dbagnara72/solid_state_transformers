%[text] ## Settings for simulink model initialization and data analysis
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
% simlength = 3.75;
simlength = 1;
transmission_delay = 125e-6*2;
s=tf('s');

model = 'sst_three_phase_dab_three_levels_inv';
rpi_enable = 0;
rpi_ccaller = 1;
%[text] ### Settings voltage application
application400 = 0;
application690 = 1;
application480 = 0;

n_modules = 2;
%[text] ## Settings and initialization
fPWM = 4e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM*6; % PWM frequency 
tPWM_DAB = 1/fPWM_DAB;
half_phase_pulses = 1/fPWM_DAB/2;

TRGO_double_update = 0;
if TRGO_double_update
    ts_afe = 1/fPWM_AFE/2;
    ts_inv = 1/fPWM_INV/2;
    ts_dab = 1/fPWM_DAB/2;
else
    ts_afe = 1/fPWM_AFE;
    ts_inv = 1/fPWM_INV;
    ts_dab = 1/fPWM_DAB;
end
ts_battery = ts_dab;
tc = ts_dab/1000*24;

z_dab=tf('z',ts_dab);
z_afe=tf('z',ts_afe);

t_misura = simlength - 0.2;
Nc = ceil(t_misura/tc);
Ns_battery = ceil(t_misura/ts_battery);
Ns_dab = ceil(t_misura/ts_dab);
Ns_afe = ceil(t_misura/ts_afe);
Ns_inv = ceil(t_misura/ts_inv);

Pnom = 275e3;
ubattery = 1250;
margin_factor = 1.25;
Vdab1_dc_nom = ubattery;
Idab1_dc_nom = Pnom/Vdab1_dc_nom;
Vdab2_dc_nom = 1500;
Idab2_dc_nom = Pnom/Vdab2_dc_nom;

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:1dd96712]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:174c5e41]
%[text] ### AFE simulation sampling time
dead_time_DAB = 0;
dead_time_AFE = 0;
dead_time_INV = 0;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;
%[text] ## Grid Emulator Settings
grid_emulator;
%[text] ### Nominal DClink voltage seting
if (application690 == 1)
    Vdc_bez = 1070; % DClink voltage reference
elseif (application480 == 1)
    Vdc_bez = 750; % DClink voltage reference
else
    Vdc_bez = 660; % DClink voltage reference
end
%[text] ### ADC quantizations
adc12_quantization = 1/2^11;
adc16_quantization = 1/2^15;
%[text] ### DAB dimensioning
LFi_dc = 400e-6;
RLFi_dc = 50e-3;
%[text] #### DClink, and dclink-brake parameters
Vdc_ref = Vdc_bez; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
Rbrake = 4;
CFi_dc1 = 900e-6*4;
RCFi_dc1_internal = 1e-3;
CFi_dc2 = 900e-6*4;
RCFi_dc2_internal = 1e-3;
%[text] #### Tank LC and HF-Transformer parameters
% three phase DAB
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/2) %[output:6e4cc00d]
f0 = fPWM_DAB/5;
Cs = 1/Ls/(2*pi*f0)^2 %[output:01f62bda]

m1 = 12;
m2 = 12;
m12 = m1/m2;

Ls1 = Ls/2;
Cs1 = Cs*2;
Cs2 = Cs1/m12^2;
Ls2 = Ls1*m12^2;

lm_trafo = 1e-3;
rfe_trafo = 1e3;
rd1_trafo = 5e-3;
ld1_trafo = Ls1;
rd2_trafo = rd1_trafo/m12^2;
ld2_trafo = ld1_trafo/m12^2;
%[text] #### DClink Lstray model
Lstray_module = 100e-9;
RLstray_dclink = 10e-3;
C_HF_Lstray_dclink = 15e-6;
R_HF_Lstray_dclink = 22000;
Z_HF_Lstray_dclink = 1/s/C_HF_Lstray_dclink + R_HF_Lstray_dclink;
Z_LF_Lstray_dclink = s*Lstray_module + RLstray_dclink;
Z_Lstray_dclink = Z_HF_Lstray_dclink*Z_LF_Lstray_dclink/(Z_HF_Lstray_dclink+Z_LF_Lstray_dclink);
ZCFi = 7/s/CFi_dc1;
sys_dclink = minreal(ZCFi/(ZCFi+Z_Lstray_dclink));
% figure; bode(sys_dclink,Z_Lstray_dclink,options); grid on
%[text] ### LCL switching filter
if (application690 == 1)
    LFu1_AFE = 0.5e-3;
    RLFu1_AFE = 157*0.05*LFu1_AFE;
    LFu1_AFE_0 = LFu1_AFE;
    RLFu1_AFE_0 = RLFu1_AFE/3;
    CFu_AFE = (100e-6*2);
    RCFu_AFE = (50e-3);
else
    LFu1_AFE = 0.33e-3;
    RLFu1_AFE = 157*0.05*LFu1_AFE;
    LFu1_AFE_0 = LFu1_AFE;
    RLFu1_AFE_0 = RLFu1_AFE/3;
    CFu_AFE = (185e-6*2);
    RCFu_AFE = (50e-3);
end
%%
%[text] ### Single phase inverter control
flt_dq = 2/(s/(2*pi*50)+1)^2;
flt_dq_d = c2d(flt_dq,ts_inv);
% figure; bode(flt_dq_d); grid on
% [num50 den50]=tfdata(flt_dq_d,'v');
iph_grid_pu_ref = 3.75;
%[text] ### DAB Control parameters
kp_i_dab = 0.25;
ki_i_dab = 18;
kp_v_dab = 0.25;
ki_v_dab = 45;

%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
Vac_FS = V_phase_normalization_factor %[output:231dc037]
Iac_FS = I_phase_normalization_factor %[output:1bc16b1f]

kp_rpi = 0.25;
ki_rpi = 18;
kp_afe = 0.25;
ki_afe = 18;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_grid_nom*s + (omega_grid_nom)^2);
res_min = s/(s^2 + 2*delta*omega_grid_min*s + (omega_grid_min)^2);
res_max = s/(s^2 + 2*delta*omega_grid_max*s + (omega_grid_max)^2);

Ares_nom = [0 1; -omega_grid_nom^2 -2*delta*omega_grid_nom];
Ares_min = [0 1; -omega_grid_min^2 -2*delta*omega_grid_min];
Ares_max = [0 1; -omega_grid_max^2 -2*delta*omega_grid_max];
Bres = [0; 1];
Cres = [0 1];
Aresd_nom = eye(2) + Ares_nom*ts_afe;
Aresd_min = eye(2) + Ares_min*ts_afe;
Aresd_max = eye(2) + Ares_max*ts_afe;
Bresd = Bres*ts_afe;
Cresd = Cres;
%%
%[text] ### Grid Normalization Factors
Vgrid_phase_normalization_factor = Vphase2*sqrt(2);
pll_i1 = 80;
pll_p = 1;
pll_p_frt = 0.2;
Vmax_ff = 1.1;
Igrid_phase_normalization_factor = 250e3/Vphase2/3/0.9*sqrt(2);
ixi_pos_ref_lim = 1.6;
ieta_pos_ref_lim = 1.0;
ieta_neg_ref_lim = 0.5;
%%
%[text] ### Double Integrator Observer for PLL
Arso = [0 1; 0 0];
Crso = [1 0];
omega_rso = 2*pi*50;
polesrso_pll = [-1 -4]*omega_rso;
Lrso_pll = acker(Arso',Crso',polesrso_pll)';
Adrso_pll = eye(2) + Arso*ts_afe;
polesdrso_pll = exp(ts_afe*polesrso_pll);
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:26989b55]

%[text] ### PLL DDSRF
use_advanced_pll = 0;
use_dq_pll_ccaller = 0;
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega0 = 2*pi*50;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:75c58cb8]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:6a06269e]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:0c76ff32]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:1ba8c347]
%[text] ### 
%%
%[text] ### Reactive current control gains
kp_rc_grid = 0.35;
ki_rc_grid = 35;

kp_rc_pos_grid = 0.35;
ki_rc_pos_grid = 35;
kp_rc_neg_grid = 0.35;
ki_rc_neg_grid = 35;
%%
%[text] ### Settings for First Order Low Pass Filters
%[text] #### LPF 50Hz in state space (for initialization)
fcut = 50;
fof = 1/(s/(2*pi*fcut)+1);
[nfof, dfof] = tfdata(fof,'v');
[nfofd, dfofd]=tfdata(c2d(fof,ts_afe),'v');
fof_z = tf(nfofd,dfofd,ts_afe,'Variable','z');
[A,B,C,D] = tf2ss(nfofd,dfofd);
LVRT_flt_ss = ss(A,B,C,D,ts_afe);
[A,B,C,D] = tf2ss(nfof,dfof);
LVRT_flt_ss_c = ss(A,B,C,D);
%[text] #### LPF 161Hz
fcut_161Hz_flt = 161;
g0_161Hz = fcut_161Hz_flt * ts_afe * 2*pi;
g1_161Hz = 1 - g0_161Hz;
%%
%[text] #### LPF 500Hz
fcut_500Hz_flt = 500;
g0_500Hz = fcut_500Hz_flt * ts_afe * 2*pi;
g1_500Hz = 1 - g0_500Hz;
%%
%[text] #### LPF 75Hz
fcut_75Hz_flt = 75;
g0_75Hz = fcut_75Hz_flt * ts_afe * 2*pi;
g1_75Hz = 1 - g0_75Hz;
%%
%[text] #### LPF 50Hz
fcut_50Hz_flt = 50;
g0_50Hz = fcut_50Hz_flt * ts_afe * 2*pi;
g1_50Hz = 1 - g0_50Hz;
%%
%[text] #### LPF 10Hz
fcut_10Hz_flt = 10;
g0_10Hz = fcut_10Hz_flt * ts_afe * 2*pi;
g1_10Hz = 1 - g0_10Hz;
%%
%[text] #### LPF 4Hz
fcut_4Hz_flt = 4;
g0_4Hz = fcut_4Hz_flt * ts_afe * 2*pi;
g1_4Hz = 1 - g0_4Hz;
%%
%[text] #### LPF 1Hz
fcut_1Hz_flt = 1;
g0_1Hz = fcut_1Hz_flt * ts_afe * 2*pi;
g1_1Hz = 1 - g0_1Hz;
%%
%[text] #### LPF 0.2Hz
fcut_0Hz2_flt = 0.2;
g0_0Hz2 = fcut_0Hz2_flt * ts_afe * 2*pi;
g1_0Hz2 = 1 - g0_0Hz2;
%%
%[text] ### Settings for RMS calculus
rms_perios = 1;
n1 = rms_perios/f_grid/ts_afe;
rms_perios = 10;
n10 = rms_perios/f_grid/ts_afe;
%%
%[text] ### Online time domain sequence calculator
w_grid = 2*pi*f_grid;
apf = (s/w_grid-1)/(s/w_grid+1);
[napfd, dapfd]=tfdata(c2d(apf,ts_afe),'v');
apf_z = tf(napfd,dapfd,ts_afe,'Variable','z');
[A,B,C,D] = tf2ss(napfd,dapfd);
ap_flt_ss = ss(A,B,C,D,ts_afe);
% figure;
% bode(ap_flt_ss,options);
% grid on
%%
%[text] ### Single phase pll
freq = 50;
kp_pll = 314;
ki_pll = 3140;

Arso = [0 1; 0 0];
Crso = [1 0];

polesrso = [-5 -1]*2*pi*10;
Lrso = acker(Arso',Crso',polesrso)';

Adrso = eye(2) + Arso*ts_inv;
polesdrso = exp(ts_inv*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:16217f48]

freq_filter = 50;
tau_f = 1/2/pi/freq_filter;
Hs = 1/(s*tau_f+1);
Hd = c2d(Hs,ts_inv);
%[text] ### Lithium Ion Battery
typical_cell_voltage = 3.6;
number_of_cells = floor(ubattery/typical_cell_voltage)-1; % nominal is 100

% stato of charge init
soc_init = 0.85; 

R = 8.3143;
F = 96487;
T = 273.15+40;
Q = 50; %Hr*A

Vbattery_nom = ubattery;
Pbattery_nom = Pnom;
Ibattery_nom = Pbattery_nom/Vbattery_nom;
Rmax = Vbattery_nom^2/(Pbattery_nom*0.1);
Rmin = Vbattery_nom^2/(Pbattery_nom);

E_1 = -1.031;
E0 = 3.485;
E1 = 0.2156;
E2 = 0;
E3 = 0;
Elog = -0.05;
alpha = 35;

R0 = 0.035;
R1 = 0.035;
C1 = 0.5;
M = 125;

q1Kalman = ts_inv^2;
q2Kalman = ts_inv^1;
q3Kalman = 0;
rKalman = 1;

Zmodel = (0:1e-3:1);
ocv_model = E_1*exp(-Zmodel*alpha) + E0 + E1*Zmodel + E2*Zmodel.^2 +...
    E3*Zmodel.^3 + Elog*log(1-Zmodel+ts_inv);
figure;  %[output:28c3084b]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:28c3084b]
xlabel('state of charge [p.u.]'); %[output:28c3084b]
ylabel('open circuit voltage [V]'); %[output:28c3084b]
title('open circuit voltage(state of charge)'); %[output:28c3084b]
grid on %[output:28c3084b]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink settings
heatsink_liquid_2kW; %[output:2330891c] %[output:55c0aace] %[output:6d4591c7]
%[text] #### DEVICES settings
danfoss_SKM1700MB20R4S2I4;

dab.Vth = Vth;                                  % [V]
dab.Rds_on = Rds_on;                            % [Ohm]
dab.Vdon_diode = Vdon_diode;                    % [V]
dab.Vgamma = Vgamma;                            % [V]
dab.Rdon_diode = Rdon_diode;                    % [Ohm]
dab.Eon = Eon;                                  % [J] @ Tj = 125°C
dab.Eoff = Eoff;                                % [J] @ Tj = 125°C
dab.Eerr = Eerr;                                % [J] @ Tj = 125°C
dab.Voff_sw_losses = Voff_sw_losses;            % [V]
dab.Ion_sw_losses = Ion_sw_losses;              % [A]
dab.JunctionTermalMass = JunctionTermalMass;    % [J/K]
dab.Rtim = Rtim;                                % [K/W]
dab.Rth_mosfet_JC = Rth_mosfet_JC;              % [K/W]
dab.Rth_mosfet_CH = Rth_mosfet_CH;              % [K/W]
dab.Rth_mosfet_JH = Rth_mosfet_JH;              % [K/W]
dab.Lstray_module = Lstray_module;              % [H]
dab.Irr = Irr;                                  % [A]
dab.Csnubber = Csnubber;                        % [F]
dab.Rsnubber = Rsnubber;                        % [Ohm]
% dab.Csnubber = Irr^2*Lstray_module/Vdab2_dc_nom^2
% dab.Rsnubber = 1/(Csnubber*fPWM_DAB)/5

% infineon_FF1000UXTR23T2M1;
% inv.Vth = Vth;                                  % [V]
% inv.Rds_on = Rds_on;                            % [Ohm]
% inv.Vdon_diode = Vdon_diode;                    % [V]
% inv.Vgamma = Vgamma;                            % [V]
% inv.Rdon_diode = Rdon_diode;                    % [Ohm]
% inv.Eon = Eon;                                  % [J] @ Tj = 125°C
% inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
% inv.Eerr = Eerr;                                % [J] @ Tj = 125°C
% inv.Voff_sw_losses = Voff_sw_losses;            % [V]
% inv.Ion_sw_losses = Ion_sw_losses;              % [A]
% inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
% inv.Rtim = Rtim;                                % [K/W]
% inv.Rth_mosfet_JC = Rth_mosfet_JC;              % [K/W]
% inv.Rth_mosfet_CH = Rth_mosfet_CH;              % [K/W]
% inv.Rth_mosfet_JH = Rth_mosfet_JH;              % [K/W]
% inv.Lstray_module = Lstray_module;              % [H]
% inv.Irr = Irr;                                  % [A]
% inv.Csnubber = Csnubber;                        % [F]
% inv.Rsnubber = Rsnubber;                        % [Ohm]

infineon_FF2400RB12IP7;
inv_t.Vth = Vth;                                  % [V]
inv_t.Vce_sat = Vce_sat;                          % [V]
inv_t.Rce_on = Rce_on;                            % [Ohm]
inv_t.Vdon_diode = Vdon_diode;                    % [V]
inv_t.Rdon_diode = Rdon_diode;                    % [Ohm]
inv_t.Eon = Eon;                                  % [J] @ Tj = 125°C
inv_t.Eoff = Eoff;                                % [J] @ Tj = 125°C
inv_t.Erec = Erec;                                % [J] @ Tj = 125°C
inv_t.Voff_sw_losses = Voff_sw_losses;            % [V]
inv_t.Ion_sw_losses = Ion_sw_losses;              % [A]
inv_t.JunctionTermalMass = JunctionTermalMass;    % [J/K]
inv_t.Rtim = Rtim;                                % [K/W]
inv_t.Rth_switch_JC = Rth_switch_JC;              % [K/W]
inv_t.Rth_switch_CH = Rth_switch_CH;              % [K/W]
inv_t.Rth_switch_JH = Rth_switch_JH;              % [K/W]
inv_t.Lstray_module = Lstray_module;              % [H]
inv_t.Irr = Irr;                                  % [A]
inv_t.Csnubber = Csnubber;                        % [F]
inv_t.Rsnubber = Rsnubber;                        % [Ohm]
% inv_t.Csnubber = Irr^2*Lstray_module/Vdc_bez^2
% inv_t.Rsnubber = 1/(Csnubber*fPWM_INV)/5

infineon_FF1800R12IE5;
inv.Vth = Vth;                                  % [V]
inv.Vce_sat = Vce_sat;                          % [V]
inv.Rce_on = Rce_on;                            % [Ohm]
inv.Vdon_diode = Vdon_diode;                    % [V]
inv.Rdon_diode = Rdon_diode;                    % [Ohm]
inv.Eon = Eon;                                  % [J] @ Tj = 125°C
inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
inv.Erec = Erec;                                % [J] @ Tj = 125°C
inv.Voff_sw_losses = Voff_sw_losses;            % [V]
inv.Ion_sw_losses = Ion_sw_losses;              % [A]
inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
inv.Rtim = Rtim;                                % [K/W]
inv.Rth_switch_JC = Rth_switch_JC;              % [K/W]
inv.Rth_switch_CH = Rth_switch_CH;              % [K/W]
inv.Rth_switch_JH = Rth_switch_JH;              % [K/W]
inv.Lstray_module = Lstray_module;              % [H]
inv.Irr = Irr;                                  % [A]
inv.Csnubber = Csnubber;                        % [F]
inv.Rsnubber = Rsnubber;                        % [Ohm]
% inv.Csnubber = Irr^2*Lstray_module/Vdc_bez^2
% inv.Rsnubber = 1/(Csnubber*fPWM_INV)/5
%[text] ## C-Caller Settings
open_system(model);
Simulink.importExternalCTypes(model,'Names',{'mavgflt_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_load_est_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqvector_pi_output_t'});
Simulink.importExternalCTypes(model,'Names',{'sv_pwm_output_t'});
Simulink.importExternalCTypes(model,'Names',{'global_state_machine_output_t'});
Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_thyr_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_grid_output_t'});
Simulink.importExternalCTypes(model,'Names',{'rpi_output_t'});

%[text] ## Remove Scopes Opening Automatically
open_scopes = find_system(model, 'BlockType', 'Scope');
for i = 1:length(open_scopes)
    set_param(open_scopes{i}, 'Open', 'off');
end

% shh = get(0,'ShowHiddenHandles');
% set(0,'ShowHiddenHandles','On');
% hscope = findobj(0,'Type','Figure','Tag','SIMULINK_SIMSCOPE_FIGURE');
% close(hscope);
% set(0,'ShowHiddenHandles',shh);
% 

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":35.8}
%---
%[output:1dd96712]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"   275"}}
%---
%[output:174c5e41]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"        1875"}}
%---
%[output:6e4cc00d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     5.918560606060606e-05"}}
%---
%[output:01f62bda]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     1.857555033442859e-05"}}
%---
%[output:231dc037]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     5.633826408401311e+02"}}
%---
%[output:1bc16b1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     3.818376618407357e+02"}}
%---
%[output:26989b55]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.345132058575099"],["81.455860923503209"]]}}
%---
%[output:75c58cb8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:6a06269e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:0c76ff32]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000250000000000"],["-24.674011002723397","0.996073009183013"]]}}
%---
%[output:1ba8c347]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.388772090881737"],["67.915215284996137"]]}}
%---
%[output:16217f48]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.091119986272030"],["4.708907792220440"]]}}
%---
%[output:28c3084b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAYEAAADoCAYAAAAJ1VgCAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQuQXkWVPiAlGSQYhodhGMIEmSAlOjzEijEsYgpBtxJdyt2ZSZVkQ0jFIqHcSmJeQLHZJckkZqyKECCGIcZdM5PatVgSd4uYjRqQgDwCs6UiBCZjGAY0T1EJCm52z4X+6em5j+57b997+va5VZZM\/r7d53yn7\/f1u487duzYMeCHEWAEGAFGwEsEjmMR8DLu7DQjwAgwAgECLAJcERgBRoAR8BgBFgGPg8+uMwKMACPAIuBZHTh06BDMmDEj8Lqrqwvq6+utILB582ZYvHgxrFixAlpbW4My8N\/wEX\/nWXCYX0ePHoVly5bBtGnToLm5Oc\/iIvPas2cPTJ8+HW6++WYtP9PY+Pjjj8POnTth4cKFVnwSWPb29gb5b9q0CcaPH69dVljstV+2lBBxXrRoETQ0NFjDzZLp1rNlEbAOMRcgBEAVhTyRUUWgrq4u+Oiffvpp2LBhQ2EisHLlSkCS1hFYQUwmNmLeU6dOhVmzZlkjM1GGLOAmsaIoAqIO3nnnnYXWBxPcykrLIpAD8vjhr1u3LsgJWxoy6YgPYu7cubBjxw7A1pWaRm15yR+4aFledtllMHLkyKBVhk\/SByrKVW1SyfLgwYNBy1W0lLGFKfLWzQN7EypxyESANqAAiGfy5MnQ0dEBSNT4CDLct29fjTzDCFJgMTg4GLwn4yT7ddddd8GqVatg69attTLRpylTpgTCoP673DMRscQYrV69GvDvMWPG1OxVbZDjIH5D\/0QrXY2tiH1jY2OkLTLu6IDAC+sOCoB4WlpaArzwwd6daLknCYTAVuAg8sE4qmXLv8mfSlydlWPf398ffBtqnRf1RfUFbRA4qnXy6quvrvmJmHzxi1+EG2+8cVhvU9Q1tcyw+OTw+TufBYtAxhDKAiBnJbrQ6keV9AGL3wW5qKQjflcruFx2WJki\/WmnnTZkOEiIgCBWkW737t1DiFsWEzWPrCKAeQubBW6y+KFgDAwMBGIl7FQFBYlNDHNFiYAgJBkrGccwAty\/fz+gAMfZoMZaxE4lWzn2UTaee+65Q4herg\/qb0jQ3\/jGN+DrX\/96TQDU+qNW7yibouIeJgKqAIgyhPhE1XkhZlGxFO+rdR5tu+eee+C+++4bIuBXXHEFPPLII6GNljBxKWooNCOlFP46i0AGyEVlPeOMM2otWNHCERV+y5YtAZmKv7E40RoVrXps3anEIVrFgqTxPexhyC3IsLFaUdGRvESPRG6ZidYU5oetSDV\/bH2Z5pEkAtjSThoiUFtpanohtmEEiziMGzduiLjpDAeJPOX3sTWtkroaS\/G7wEn0FL71rW8FrV7xe1gPR65uOsNB6vBP1N9R9Ued81HrJ+IksFZJPKq3qaZXyXX79u1D6rws0GHDZFGCL+o81klhtxAlEV\/szci9PLk3GTashTHHd4ocIsxAMYW8yiKQAeYwYlM\/fPFByIQtV04sXh4qUVv6+De2gEVrVP5ow0RA\/aDEkIvIN2o4SM7fNI88REDGTbSSxQeNtodNZss4quIWJwJqSxVxxN6NinMUyatVBolJ2KyO70cN7aB9cSIQNfSlikBUqzuqpygLn5jsVf0UDZcoEQjLI6wnmiRMao9C7SmE1XnZprD4iyEx2R55eCzJ9gx04OyrLAIZQhfW0ogSgajKGyUC+O9R5KQOncgumBK46AlkFQFVEJP+DoNdvHPbbbcFvRQxth7VojYVAZUA5L+ziIA8XBE1yRs2byR6dfI7ui3\/pKEXUX\/UVT1hdadoEVDrnBgeUofd8hIBeQ6KRWD4l8cikEEEbAwHqeaEkXqcCMitK9FTkIll5syZoXMC8genm4cYcpKHqNRJ5ai\/w2BXW79yTyfrcJA6jyEPJ6QdDjId2sH0sjiKiWpZBFSSUodekoaDkqpznsNB6hCn8EPMJ0X1BETvWPyu2qSKAsYqzXBQGBYsAiwCSd+I8e+2JoZ1usZR67fTTAzLIiCTlQxI3MoWkS5JBDBd1IoT\/E3gqaaJmiAXOKnjzjLJY77XX399MHkaNlwQNYmPNuhMDItWuUowUROoUThiPviIlWZhQxryqhrMZ82aNXDHHXcM80tdgSXySpoYxvH3pPkb3YnhJBFQP7a4Oh9mt87EsNoj4jkBFgFjktd5Ie8lojIBmvYEhL1h4944NKAzJ5CUB\/4ukzLaiz2MOXPmDFupIYhAJo44EYhbB6+7RFRMPsqEiQR75ZVX1lbeIOHccMMNMHv27NqwkypCuER0\/vz5sUtEZbINGx4MI8yw+SEsG20UPTWxlHjt2rVw\/\/33g5gfkcVNXfElBC4OXywnbomo2luJ2tgXNZ4vz1lFiYAq0IgHLk0WE7Zogzo\/g\/8mlynHU92QKM+xyb+pw17qfJnOt17FNKUOB+FHvWDBgmBNd9iOTnUtcdyySKrBSWpVUbXbN7tkYlSX56q9pChsBMmg2NrazetbXIS\/ogGAf4etetPZhc77BMJrT2kioLM8DgkU13a7\/EGxCLhDW1FDe0kb82QPTXYMu4NM+ZYmDa3pHAuC3yLvGCY0HIStfPxg8InqCeDvTU1NWmewlF9Nwy1gEaAameF2hY07J+2+VXMRrU0cSjI5b8cdlMqzNGxeSPdcIz47KDpupfQEUNWXLl0K7e3tgRCEiYAI2oQJE5wWgfI+GS6ZEWAEGIFkBEoRAWwd43PppZdGzgmEdf9MuuXJrnMKRoARYAQYgcJFALt0GzduhFtuuSU4DyZqYljtVnM3mysrI8AIMAL5I1C4CODwDy7Vw\/HSpNVBqrtiDiFsovi8887LHx3OkRFgBBiBnBDAU4THjh2bU275ZVOoCETN8KM7OhM8cRPFKAJ9fX35IUM4J598xTD45K9PvnJsaZBMoSKguhzXExCrh+SNP7hxJ+r0P58+Hp98ZaKgQRS2rPCpLlP1lZQIIPF3d3fXLvBQN4vF9RaoAmzj49m7dy\/JbqUNXzFPn\/z1yVffYkuVo0oVgTxJgyrAefoo8mKisIEqjTw5tjTiYMMKqhzFImAj2pbzZKKwDHCJ2XNsSwTfctEsAp4CbMNtJgobqNLIk2NLIw42rGARsIGqlCdVgG24zURhA1UaeXJsacTBhhVUOYqHg2xE23KeTBSWAS4xe45tieBbLppFwFOAbbjNRGEDVRp5cmxpxMGGFSwCNlDl4SDLqNLI3idi9MlXrF0++csiYJlPqAJsw22fPhzfiIJja+OLoZEnVY7iOQEa9cPICiYKI7icSsyxdSpcRsayCBjBZZ6YKsDmniS\/wUSRjJGrKTi2rkYu2W6qHMU9geTYkUvBREEuJLkZxLHNDUpyGbEIWA4JVYBtuM1EYQNVGnlybGnEwYYVVDmKewI2om05TyYKywCXmD3HtkTwLRfNIuApwDbcZqKwgSqNPDm2NOJgwwoWARuoSnlSBdiG20wUNlClkSfHlkYcbFhBlaO0h4PibgWLAqylpQUeeOABG3gOy5MqwDacZ6KwgSqNPDm2NOJgwwqqHGUkAvPmzYMlS5ZAc3NzIkZ4a9jy5cuDm8CKeKgCbMN3JgobqNLIk2NLIw42rKDKUdoiYAOUPPOkCnCePoq8mChsoEojT44tjTjYsIIqR2mLgBgOQnDEvb82gEqbJ1WA0\/oT9x4ThQ1UaeTJsaURBxtWUOUobRFAUI4ePQqLFi2CrVu3BhjF3flrA8S4PKkCbAMHJgobqNLIk2NLIw42rKDKUUYiIAMjXwI\/efLk2uXwNsDTyZMqwDq2m6ZhojBFzJ30HFt3YmVqKVWOSi0CAgC5d9DQ0BBMBOtMHJsCmJSeKsBJdqf5nYkiDWpuvMOxdSNOaaykylGZRUAGQ6wI6uzshPr6+jQ4pX6HKsCpHYp5kYnCBqo08uTY0oiDDSuoclRmEeCegI3qEp8nE0XxmBdVIse2KKSLL6dyIsBzAsVXIlEiE0V52NsumWNrG+Hy8q+ECKirg1asWAGtra3loSqVTBVgG+AwUdhAlUaeHFsacbBhBVWO0h4OEvsE9u\/fX9rkb1xgqAJsozIxUdhAlUaeHFsacbBhBVWO0hYBG6DkmSdVgPP0kYeDbKBJK08WAVrxyNMaqhylLQLYE+Czg\/KsEunzYqJIjx31Nzm21COU3r5KiMCMGTOgt7dXGwU+RVQbKqOETBRGcDmVmGPrVLi0jf3pi0fgun\/qgd9+96va7xSVULsnUJRBacuhqrJp\/Yl7j4nCBqo08uTY0ohD3lZ0P\/kazO5+Dg5986q8s86cH4tAZgiLz4CJonjMiyqRY1sU0sWWs3JbP6zctpdFwCbs3BOwiW65eftEjD75irXKF39ZBHLgkJUrVwa5LFy4MDQ3FoEcQCaahS9E4RMpiqrmS2xZBDKSi9idPGvWLBYBj1pPvhEFi0BGoiD8eiVFYPPmzbB48eIAdrxXAJ\/u7u7cj5TGXcrLli2DX\/ziFzB+\/HgWARYBwp96dtN8aRn7JvA4KYyTw5WZGMahmcHBQbj99tth6dKl0N7eHhB00pBNmk8ExQaf\/v5+Hg56F0AmijQ1yY13OLZuxMnUyi\/d8yw8vOdwNURA3jTW2NgY3DQmRCDvo6SxLBQZFJv169ezCLAImH57zqVnEXAuZFoGT1n7DPz0pSPVFwEcu8feQF53EGNeV155pVYvgyeGteqik4l8IkaffPVpDuTiOx6DfYferIYIYOBwiGbXrl1DhoPGjRsHuKO4ra0tl5NFsVexceNGuOWWW6Curi5xqAlFAJ8dO3Y4SXQmRg8MDAD2wnx5fPLXJ1+x\/vrg71WTW+H1z72zurEycwLojHyfgCCjPI+WlieeZbLD+4zXrFkzjP+4J1BdSfCpdeyTr770BPDIiCl3P1M9ESiacpImnVkEio5IceX5RIw++eqLCHzviVfh5p5fsQhkpQwWgfcQZKLIWpvovs+xpRubtJaJSeHj3zgAB+7927TZWHvP+OwgcblM0mmicRu7bHjDPQEbqNLI0ydi9MlXH3oCOBmMk8L4jHjhBzD4g04aH5VkhbEI4Ls4Xt\/T0zNkFZAQBzExnNRyzxsJFoG8EaWTn0\/E6JOvVRcBFACcC8D\/H1M\/Ao70fA36f\/4EnQ\/rXUuMRUCQPZ7hgxvE5EdeInrw4EFYvnx5cBVlEQ+LQBEol1OGT8Tok69VF4FvPzIAix7YE3w0q64bB8uvvwL6+vrK+YhiSmURIBeSZIOYKJIxcjUFx9bVyA21W14RhL2AZ2\/9FFBtqBqLgO5wkNhLELac00aYqQJsw1cmChuo0siTY0sjDmmtwKGfR148XFsNhAKw5aZLguEgqhyVSgQQoLB9AniQHA4R4W\/z588PhoKam5vT4mn0HlWAjZzQTMxEoQmUg8k4tg4G7V2T5TkA\/Cck\/rvaLoSJ548KUlDlqNQiQC1UVAG2gRMThQ1UaeTJsaURBxMrkPz\/6+cHYMl\/vDP+LwRA9ADEv1HlKBYBk2gTSctEQSQQFszg2FoA1VKWSP4bdr0Ca360b0gJN048O5gIVp9KiUDYUJBwuKWlJbcD5ExiRxVgEx900zJR6CLlXjqOLf2Y4b0A3U+8GpwKKj\/y+H+YF1Q5yrgngJe84PHREyZMgClTptSOks77ADnTqkAVYFM\/dNIzUeig5GYaji3NuGGrf073c8OIXwz94OqfpIcqRxmLgHyfAE764qawpqam4ORQ7CHYuF0sCVz8nSrAOrabpmGiMEXMnfQcWzqxSiL+T394FKxtv1DbYKoclVkEcCko3vqFm8fyvlRGG10WAROonEvrEzH65CtWREr+4tr+Vdv2hrb2xUejrvgx+ZgqIwLotHwkhNz637JlS3DPQEdHR3AHQJEPVYBtYEDpw7Hhn5qnT\/765GuZIoCt\/IHDb0LHQ8mkjy3+9svPCpZ84v\/SPlQ5yrgngACoR0egKKxbtw4aGhoK3RsgB4MqwGkrTNx7TBQ2UKWRJ8c2\/zgg4R9+4y247cEXY1v5WLIg+TV\/9xEYe3pdJtJXPaHKUalEIP8wZc+RKsDZPRueAxOFDVRp5MmxTR8HJHt8Orf3w94DRxMJX5D+mFNHwF3tF+ZK+GFeUOUoYxFQJ4ZlZ3lOIH0FNnmTicIELbfScmyT4yXI\/ju7XoGnfv26FtnLrfyv\/tU58IWLTrdO+pXtCcSJQN4XzSdXh\/dSUFVZEx900zJR6CLlXjqO7XsxQ7L\/+eAf4N6dLwf\/qK7Lj4tuMH5\/6gjouG4cnHzi+won\/Er2BOI2iMkOF32ZjCibRcA9wtO12Cdi9MlXjP+jvXsATj4DVj6015jo5eGc9k+eBTiBm2XiVrc+pk1HlaNyHQ5KC04e71EFOA\/f1Dx8Iwqf\/K2Sr2LYBuvv6u390H\/gKOw7\/GZwyYrJI4gdSf66Sz4EzWeeRJrso3yjylHGImASvCLTUgXYBgZVIgodfHzy1yVfBZkfA4C1P9kHv3r1j6lIXrTog\/8\/dQT8\/YSz4RPnnhJUDcote526K6ehylHaIqB7tzCfHWRaNczTu0QU5t4Nf8Mnfyn5Kkj+8b2\/g50vHIKXD71pNC6vRlJu0f\/DpHPhxBOOh7\/87lUYO3ZsHtWEfB7OiwB1hKkCbAM3SkRhwz+fh7+Kii3ujkVSRqLf\/NRr8OuD6YZq5FjJJD99wtlw5sj3136OatEX5W8R9TSpDKocpd0TSHKw7N+pAmwDF58+HMTPJ3\/z8FW04PGGq81PvhZUwTRj8WEteRyuOf\/Mk2DOVefACccfn3nIJg9\/bXxjNvKkylGpRQDPDFq8ePEQrFasWBEcJFfGQxVgG1j49OGwCLxXgwS59x14Ax545rfBhqg8yB1LEEsqz6kfAVMvPwvw\/8Vjc1zep7pMlaNSiQAKQE9Pz5B7A8ScQVtbWylCQBVgFoHsCFSdKJDc8X8jR7wP7t7+PLz6xvtyI3eV4L8yvgEaPnhiIQSvE\/mqx1bGgCpHGYuAem6Q7CRvFtOp9tnT+PThuNwTEOPu2355EHpf\/j3sO5Rfy12Qe\/D\/p46AlnNGwsyJjYGYiDtt8b9ttuKz12S\/hvpYBPKoMTF5UAXYhtssAjZQ1ctTDMn89vd\/hp4nX4MXfvPH4EWT3axJJQniRnK\/6OyT4caJjXDC8ccNIfikPFz53ae6TJWjjHsCWLl4OKjcT8ynD6eInoAg9j+9\/b\/BSpkn9v4uCHBe4+3q2DqS+2c\/Ug9fuvhMOP6444KfxUodn5ZMFhHbcr\/UoaVXSgSEEPDEcDlVjEUgHnd5R+rzv\/kj7HzhMPzPwO+tEztOpv71x86AU0acUDt73nRIhmNbzjdVRKmVEYGyJ4CjgkUVYBuVy0eieOUvp9bGt3e9dAQefelIsLY976EY0SoP\/v\/UEcEqmZbGkXDtR09\/R0SkMXeObXYEfKrLVDkq1XCQepjcpk2bYPz48dlrRIYcqAKcwaXIV6vy4YgW+5tv\/QU2Pfka7P7161Za62HE\/vHGkfD5d4ld\/t1GvEzyrEpsdX32yV+qHJVKBOQAy\/sF+GYx3aqfLR3VD0cehsGx9f6DR4OjBvIeWxfoyROo2GLHA8Ymnn9qIS32bBGMfptqbNnf7AhUVgRkaPCaSewldHV1QX19fXbUDHKgCrCBC9pJiyAK9aTHXX1H4JE9hwshdTEUc3nTB6H1E6Nh\/2uvwDnnnEN+uaN2AGMSFhHbPOzMKw+f\/KXKUZl7AuJ+YawUZR0eh2VTBTivj0XOJ82HI5M6\/verr\/8JHn3xCPTtfyNoqeNjesSvjm9qa31Ky5lw4egP1F7VWceexl8d2yim8clXxN8nf6lyVCoRyDIEdPToUVi0aBFs3bo1+AbjLqFR5x7ihpuoAmyDaMSHIzYjYRn434\/1vTdZWsQQzPkfOgk+3jAyWO4oPzrEboKLT0Thk68sAiZfgb20xiIQt2NYx0zsOeCzcOFCSFpphGLT398fpE16qiQCokX+8uF3TnhMexlHEmbBsMu7Z8TgSpgxp9XBRQ0nB\/eviqWNYv163sSuY5tI4xMx+uQri4DJV2AvrbEI5G2KLApq3vhbU1OT1llELokAEuzrb74N9z48APsOHrW22zRueaPp+vW8426Sn0\/E6JOvLAImX4G9tKWKQFyvQgwbTZgwwVkRwCGaQ2+8Bfc9MpCJ6OXW+rjRH4Brzj0GF5x3blArymyh26uWQ3P2iRh98pVFoKgvKL6c0kRATChPnjwZOjo6oK6uboilYTeZxR1VTaEngK3rOd3PGRG+OML3gtEfgBmfPhsO\/OGt2m7TqNAxUdD4eGxYwbG1gSqNPClwVBgSpYmAMAbFYHBwcJgQ7NmzB6ZPnw6rV68ONqKpf6vOIMD47Nixo9CID77+Nvzjfx+Ap1+Jvjy74ZQT4KyRJ8CEc+vgY6NPhMvOfu+s9jTGDgwMQGNjY5pXnXzHJ3998hUrow\/+Tpo0qfbd9fX1kfsGjUUAW+jz5s2DJUuWQHNz8xCHkKiXL18OnZ2d2vsE8J0FCxbAqlWrhuUXNkeA\/xY2UVy0yk5Z+0xoi18Mz9w99UJoHPUO2ec9ZMOtRXLfUW4GcWxzg5JcRkVzlC4AuYpAmvsETN6JmyguAuCo4R4k+c+Mq4cvX\/qh2lnuugFIk46JIg1qbrzDsXUjTmmsLIKj0tilLQLqmv2owuLW\/eM78mogMfmL6\/\/V1r0qDvj3\/PnzYcOGDaE9BpsAI\/n3PPUadDy0d4jbSP5bbrok95Z+UiCZKJIQcvd3jq27sUuy3CZHJZUd97u2CIhM4oaDdAxRN4vJE8NI9N3d3bX5AZOD6mwBjCt85vQ8N2Q3bfvlo2Ft+4U67lpJw0RhBVYSmXJsSYTBihG2OCqrscYikLVAW+\/bABgFYMrdz9RMxpb\/s7d+ypYL2vkyUWhD5VxCjq1zIdM22AZHaRcek1BbBMSSzZkzZ8L69euht7c3NNuyzg\/KG2Ac\/rlp03M1H78\/qwWuuqDYQ\/Gi4sZEkUfVp5kHx5ZmXPKwKm+OysMmzENbBPIq0FY+eQL8nccGYe6\/PR+Yiq3\/u9ouLGTCVxcbJgpdpNxLx7F1L2a6FufJUbpl6qRjEVBQ+pefvQpf2\/yrmgCUMfGbFDgmiiSE3P2dY+tu7JIsr4wIhO3klZ13eTio+8nXYHb3O0NAZa38SapI+DsThQ5Kbqbh2LoZNx2rKyMCUc7iqp9ly5bBtGnTEjd96QBmmiYrwLgM9OI7HiM7BCTjwURhWjvcSc+xdSdWppZm5SjT8nTT5zocpC7x1DUij3RZAZZ3AOMQ0MTzR+VhlpU8mCiswEoiU44tiTBYMSIrR1kxKu+J4TTHRuTlWBaANz3xKszpeWceoOw9ADp4MFHooORmGo6tm3HTsToLR+nknzZNrj2BqMPg0hpn8l5agNVhIIoTwSoOTBQmNcOttBxbt+JlYm1ajjIpI01aYxGImxiOu\/4xjXEm76QFWB4Gwl3A2BOg\/jBRUI9Qevs4tumxo\/5mWo6y7ZexCNg2KG3+aQBWewEUdgPr+M9EoYOSm2k4tm7GTcfqNBylk2\/WNKlEQL31K+4guKwG6r6fBmCXJoNlHJgodGuFe+k4tu7FTNfiNBylm3eWdKlEIOxe4LKFwBRg+VygiR8eBVtmX5IFx0LfZaIoFO5CC+PYFgp3oYWZclRRxhmLQN6XyuTlqCnAeDAcCgE+OAyU98UvefkVlg8ThU10y82bY1su\/jZLN+Uom7bIeacSgRkzZgTn\/+O1j\/JjckFM3g6aACzPBbjWC0DcmCjyrj108uPY0olF3paYcFTeZcflZywCmNnmzZuhp6cHurq6atdIilVDbW1t0NraWqQPQVkmAOPlMKt+2O9kL4BFoPCqVWiBLAKFwl1oYSYcVaRhqUQADQy7aWzFihWlCICJCLi6IkiuFEwURX4ixZbFsS0W7yJLq5wIFAmeTlm6AMsiQP14iCi\/mSh0aoSbaTi2bsZNx2pdjtLJK880xj0BsQqovb192JxAnoaZ5qULsLws1LUJYYEJE4Vp7XAnPcfWnViZWqrLUab5Zk1vLAJZ7xjOanDU+zoAuz4hzCJgq\/bQyZdFgE4s8rZEh6PyLlMnP2MRwExxYri\/vz9YIUTl0QFYvi\/AlSMiwvBloqBS6\/K3g2ObP6ZUctThqDJsNRYBly+VEUNBVC6MTxtwJoq0yNF\/j2NLP0ZpLayMCKQFwPZ7OgDXz\/1xYIaLewNk\/JgobNem8vLn2JaHve2SdTjKtg1h+Rv3BMowUqfMJICrsCpI4MBEoVMj3EzDsXUzbjpWJ3GUTh420miLgBgGmjlzJqxfvx56e3tD7aF6x3AVVgWxCNj4BGjlySJAKx55WuO8COQJho28kgAWQ0GuzwcgdkwUNmoQjTw5tjTiYMOKJI6yUaZOnto9ATkz146Slk8M\/dcbPgZfuOh0HWzIpmGiIBuazIZxbDNDSDaDSomAa0dJr9zWDyu37Q0qh6u7hOWazURB9jvPbBjHNjOEZDOojAi4eJR0VZaG8pwA2e87N8NYBHKDklxGlRIBl46SrsouYe4JjCX3UdswiEXABqo08qyMCCCcLh0lLYvAwmvGwsJrmmjUiAxWMFFkAI\/4qxxb4gHKYF6lRABxcOUoafmoiCrMByD2TBQZvkTir3JsiQcog3mVE4EMWFh5NQrgqs0HsAhYqT5kMmURIBOK3A1hEUgBqdzbaGhogA0bNkBzc3NoTlEAV+WoCJ4T4DmBFJ8Q+Vd8Ej0WAcPquGfPHliwYAGsWrUqIP6weQg5yzCAqzgfwD0Bw4rkWHKfSNG3uswikPFjVEVBzS4MYHmTWFXmA3z7cHzzl0UgI1EQfp1FIGNwsCewa9cu6OjogLq6umG5hQFctU1iwmkmioyVifDrHFvCwcloWqVEAFvl06dPh8HBwWGw5H2AnFzWpk2bIq+0DAO4ipPCvrWMffOXRSAj0xJ+vTIiIM4NwonaIm8WE2KwevXqUCFAgPHZsWNHrRqqWXiSAAAOa0lEQVRcdmd\/8N+XnT0Cvn3daMLVw8y0gYEBaGxsNHvJ4dQ++euTr1glffB30qRJta+vr6+P3JdofIBcmXcMh51ZJBBVVVaeFP73WS3w2QvqyYGf1iBuLaZFjv57HFv6MUprYeV6Au3t7ZFDM2lBintPPblUTasCXNVJYd+GR3zzl0XABnvQyLMyIoBwJk3S5gG5uhoI9wzMnz8\/cq+ACnBVJ4V9I0Xf\/GURyIM9aOZRGREo8qJ59WgKk4nhqk4K+0aKvvnLIkCTwPOwqjIikAcYNvJQAb74jscA5wVcv1Q+DCsmChs1iEaeHFsacbBhBYuADVSlPGWA5Unhv7n4TOi6\/qOWSy82eyaKYvEusjSObZFoF1tW5UQA5wUWL14coIjDNPh0d3dHbuayDbcMsDwpXJXjo2X8mChs16by8ufYloe97ZIrJQK4VBM3it1+++2wdOlSECuF4pZwFglwlSeFfRsj981fFgHbTFFe\/pURAXmfAG5YWrRoUU0EcEXP8uXLobOzE+rri12XLwN864Mvwt07Xw6i\/eytn4Ix9SPKi7yFkpkoLIBKJEuOLZFAWDDDCxHA1TzYG+jq6ipVBKq8Msi3lrFv\/rIIWGBfIllWRgQQT7FPQB4OGjduHODdw21tbdDa2lo47DLAVV4Z5Bsp+uYvi0Dh1FFYgZUSAUSN6vWS8sqg9stHw9r2CwsLclEFMVEUhXTx5XBsi8e8qBIrJwJFAadbjgC4ysdFCCyYKHRrhXvpOLbuxUzXYhYBXaRSphMAV\/FieRUSJoqUlcSB1zi2DgQppYmVEwF5n4DAJO5Yh5S4ab8mAJaXh1ZxZZBvY+S++csioP3JO5ewUiIQdt+vOFOo7IlhsTIIa8ihb17lXEXRMZiJQgclN9NwbN2Mm47VlREBQfZ4ocz48eOH+E5hiWjVl4f61jL2zV8WAR06dTONFyJAYbNY\/dwfBzWkigfHiarPROEmCehYzbHVQcnNNJURAYQ\/7Gx\/CsNBP3nql4B7BPCZ\/Zlz4J+nnO9mbUmwmomikmENnOLYVje2lRGBpPsE5BDipfMPPPBAIVFFgL\/7w90w5e5ngvKqeHAc9wQKqUqlFsIiUCr8VguvjAhYRSlD5qoIbLnpEph4\/qgMOdJ9lYmCbmyyWsaxzYog3fdZBCzHBgGedc+PYOW2vUFJLAKWAS8we5+I0SdffRv+qpwIhO0TWLFiRSnnBmFlQoA\/v+w\/ATeL4VPV5aG+fTi++csiUGDrouCiKiUCVPcJXDTv+\/DTl44ER0fjRrGqPkwUVY0sTwxXN7LvNFT7+vrIuXjcsWPHjplYRXmfwCk3fK+y9wrLMWIRMKmxbqXl2LoVLxNrWQRM0EqRtumiT8Lrn1sZvIkTwjgnUNWHiaKqkeWeQHUjW6GeAAaJ4nCQLAJVXh7q2xi5b\/6ywFdXBirTExAhojYxPOaTn4c\/TFwQmId3COBdAlV9mCiqGlnuCVQ3shXrCVAMVONnvgJvXHpDYFqVl4f61jL2zV8WeIrsko9NlesJ5ANLfrmwCOSHJbWcfCJGn3z1TeBZBCwzy+gv3wF\/HvPpoJSq3iMgIGSisFyZSsyeY1si+JaLZhGwDPCZ198Lb59+QeX3CPjWevLNXxYBy0RRYvYsApbBZxGwDHCJ2ftEjD756pvAswhYJhEf7hHg4SDLlYhA9iwCBIJgyQQWAUvAimyFCODSUFwiWuWHiaK60eXYVje2LAKWYytEoOrLQ33rQvvmL4uAZaIoMXsWAcvgCxGo+kYx30jRN39ZBCwTRYnZswi8C756M9nkyZOho6MD6urqhoUHr7GcOnVq7d8bGhpgw4YN0NzcPCwt9wRKrN2Wi\/aJGH3y1TeBZxH4\/ysfjx49CosWLYIJEyYE9w6Iv5HcFy5cOIxK8GiK\/v7+0N\/UxCwClpm4xOx9IkaffGURKPGjkoo2Pko6b7OR6Hft2hXaG1i5ciU0NTVpXVQjRKDqG8V8+3B885dFIG+GoZMf9wQiYhElAmqvISmUKAJVv0xGYMBEkVQb3P2dY+tu7JIsZxEIQUjMD7S1tQ1r7atzB\/h63PWVLAJJVdDd330iRp989a2XxyKgcJBo6eM\/h00M79mzB6ZPnw6rV6+G8ePHg\/p32JzAxA+Pgi2zq3uZDPcE3BUyXctZBHSRci8di4AUsyQBiAovzhHgEzaJjD2BEw48Dz9beo17tcPQ4oGBAWhsbDR8y93kPvnrk69YI33wd9KkSbWPrxJ3DGelkqQVQXH5x00Uowj4sFvYty60b\/5yTyArw9B9n3sC78YGiXxwcDByb4AIIe4RwLRdXV1QX18P+Pf8+fNj9wlU\/VpJgQ3VymTr8\/PJX598xfrik79UfS10iWjYZC9WhJaWloDsX3jhBeju7q4JhLpZbNOmTcH8QNiDPYGTdt8P79\/3qC0u4nwZAUaAEciEAA8HZYKPX2YEGAFGgBHIG4FCewJ5G8\/5MQKMACPACGRDgEUgG378NiPACDACTiPAIuB0+Nh4RoARYASyIcAikA0\/fpsRYAQYAacRYBFwOnxsPCPACDAC2RBwXgTwALrFixcHKMSdLZQNpnLexn0S69atCwqPWx4rp4u7c6EcL\/RK1fVV5GZ6wKCeFcWl0vVXXVYdVw+Ks96sJF1fxdEwuI\/I1XqchIzJychJeeX1u9MigJVmwYIFsGrVqgAP8d9hl87kBVhR+cib5XD\/hLxxTrZBPYUV\/+7p6altsivK3izl6Pqq+o3i76Lw6\/qr7q6X67srdVzXVyF2eCQM7gVysR4nfQNCDKnVWadFQCVAiiqbVDGifpfPSRJk0N7eHrlZTuTjIlGY+oqEMW\/ePDhy5AiEnUCbFvOi3tP1F2O5fPly6OzsDHbNu\/iY+Co34lysx1HxEQI3atSoIMm1116rdUdKUfF2WgTUA+XiDpgrCtA8yom6gU3cyBZXhmsfTxpfMc6XX345PPjgg7Vb6vLAvYg8TPyNu3CpCFuzlmHia1hPIOqyqax2Ff0++nb48OFgiEu+WbFoO6LKc14E5JvHTK6jpBKAMDvCWv66vRzds5mo+G\/qK4rcxo0bYe7cubB06VJnRUDu1UXFVtRnjJXO3BCVmAo7TGMr0m\/duhVmzZqlda0sNZ\/j7KE6j8UiQLAWmX48wgUkjTvvvDPykD2CrtbumdYhRcRl2bJlMG3atOAobYqtqiSMTWIrFj2IyeCkQxSTyi76dxNf1ftC1AMki7bdRnksAhZQ5eGg90B1UQDQepMhAySGnTt3Bi1Eqh9UUjU38VcdDnLNZ598TYp7WF3XeaeINE73BNThH90hkyKAzVqG7EvSxLDrKyl0fZWXGsr4ujZ0oOsvip58qm5SPcha52y8r+ur64Kngx1VEXdaBHiJKCTes6BTOctOo7uMULaT6gelg6Wuv+pkqYtDJLq+hg0Hxd0fooMztTRU66zTIoBB9nGzmNwDimodu7apKGpDUdRkP9UPSpd4dP2VN4u5uoFK11f5\/hBXfeWJYd0vgNMxAowAI8AIkEDA+Z4ACRTZCEaAEWAEHEWARcDRwLHZjAAjwAjkgQCLQB4och6MACPACDiKAIuAo4FjsxkBRoARyAMBFoE8UOQ8GAFGgBFwFAEWAUcDR8FsdTNTnE22NzrJSyknT54MHR0dUFdXlwiTuhY\/8YWCE4jllS0tLVaPB5fP7THBr2A4uDgLCLAIWADVlywpiYCJLXJ8XBABtBePyijicf3k0iIwqloZLAJVi2jO\/qg3W4kjGuSNPaKVii1vPNQNT4EUD16gMWXKlCH\/Li7VkFufmD6pBRq1mUjeMIj5hG2Ui\/JD\/DteZCJO6lRb3XK5mL\/8O5a9f\/9+2LFjB\/T29gZl4+8yDrfddhts2bIluPwIL4Mx8Vs9H8vkZNEwgUsi+aTfc65enB0BBFgECASBqglJB4DJrW\/0AYkPd3qKVqt8qJ049VOcFhq24zfuPgj1BM2wv+VzdmRM4\/y4+uqrYcaMGTBmzJhgCEn1Qy1HFg30M+zgPvk4b5Hf008\/HZzuGnb6aZzfYSIg36imHreQ1MtJIvmk36nWVbYrPQIsAumxq\/ybSUczJA3ByGc7qSIQ9q64MWzJkiVBi1k8UXbIBBlnS9yZO2lay3K5KmmG+SALycGDB4ccCoc+RvmNv4WJgHrZSpSIpPGNRaDyn\/UwB1kE\/Iu5kcfyUIg6XBNFvPJZMeIMGFUE1CEcYVTYmTFR4\/byuUJxIhBHbLpEKV+CjraKYTE177ArIWUx3L17N2BLXn2izsqJGg6S5wii\/NP1TbaFRcDo86hEYhaBSoTRvhMyCcrzAvIQjCB\/IRYDAwMg7o0NEwHd6wPLFAH0Yfr06TA4OFiba4jrCeiIgK7fUT2B\/v7+IRPFLAL263+VS2ARqHJ0LfimHg0sRACHbPDyd3koJ2k4CMm0q6sr8RL1MoeDcEJXJd2sw0G6fvNwkIUKzFnycBDXAX0EkiZv5SEYTIsTrDhMgSttROsdV87IE6LqxLA8kRw3dp\/nxLBMrjNnzhxiN\/4mt6xRBOSWuxjGihoOEnljz0GeaFYnhnX9jpoYFiugZKGV51HQDhE\/UZaIiZgED9tHwcNB+t9HVVJyT6AqkbTkhzoWLs8LqESPk55Tp04NLEHiWb16dTCx2dbWBq2trbW7HwSBqss2kzZExZ03nzRJrZYl\/FDFSxUB\/Fte7om2NzU1QU9PT9CL2b59+xCRkMlXLJXFJaIPP\/wwdHZ2Br0eE7\/DROChhx4KMMarNvGRl8SGzVGI4SzEF0Vv27ZtgUAl+a6z2c5SteNsC0SARaBAsLkoPxEImyfQRUJndZBuXjrpuCegg1K10rAIVCue7E3JCIRNYsftA0gyl0UgCSH+PSsCLAJZEeT3GQEFAXWHsRj+SgOUenZQ2PBTmnzVd\/jsoDxQdDOP\/wPQrFrOx382DQAAAABJRU5ErkJggg==","height":232,"width":385}}
%---
%[output:2330891c]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:55c0aace]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:6d4591c7]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
