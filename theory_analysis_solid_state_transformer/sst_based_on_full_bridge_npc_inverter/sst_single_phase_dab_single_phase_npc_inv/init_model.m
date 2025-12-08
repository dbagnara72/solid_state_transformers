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

model = 'sst_spdab_npc_inv';
rpi_enable = 0;
rpi_ccaller = 0;
%[text] ### Settings voltage application
application400 = 0;
application690 = 0;
application480 = 1;

n_modules = 2;
%[text] ## Settings and initialization
fPWM = 4e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM*3; % PWM frequency 
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
tc = ts_dab/100;

z_dab=tf('z',ts_dab);
z_afe=tf('z',ts_afe);

t_misura = simlength - 0.2;
Nc = ceil(t_misura/tc);
Ns_battery = ceil(t_misura/ts_battery);
Ns_dab = ceil(t_misura/ts_dab);
Ns_afe = ceil(t_misura/ts_afe);
Ns_inv = ceil(t_misura/ts_inv);

Pnom = 275e3;
ubattery = 750;
margin_factor = 1.25;
Vdab1_dc_nom = ubattery;
Idab1_dc_nom = Pnom/Vdab1_dc_nom;
Vdab2_dc_nom = 750;
Idab2_dc_nom = Pnom/Vdab2_dc_nom;

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:1dd96712]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:174c5e41]
%[text] ### AFE simulation sampling time
dead_time_DAB = 1e-6;
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
% single phase DAB
single_phaseDAB_TRsizing; %[output:6986c445]
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/8)
% Ls = L_d_eff;
f0 = fPWM_DAB/5;
Cs = 1/Ls/(2*pi*f0)^2 %[output:521048c6]

m1 = n1;
m2 = n2;
m12 = m1/m2;

Ls1 = Ls/2;
Ls2 = Ls1*m12^2;
Cs1 = Cs*2;
Cs2 = Cs1*m12^2;

mu0 = 1.256637e-6;
mur = 5000;
lm_trafo = mu0*mur*n1^2*S_Fe/L_core_length * 1e-2;
rfe_trafo = V1^2/P_Fe;
rd1_trafo = R1;
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
% iph_grid_pu_ref = 3.75;
iph_grid_pu_ref = 2.75;
%[text] ### DAB Control parameters
kp_i_dab = 0.25;
ki_i_dab = 18;
kp_v_dab = 0.25;
ki_v_dab = 45;

%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
Vac_FS = V_phase_normalization_factor %[output:60b575f9]
Iac_FS = I_phase_normalization_factor %[output:631fa7a0]

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
omega_rso = 2*pi*f_grid;
polesrso_pll = [-1 -4]*omega_rso;
Lrso_pll = acker(Arso',Crso',polesrso_pll)';
Adrso_pll = eye(2) + Arso*ts_afe;
polesdrso_pll = exp(ts_afe*polesrso_pll);
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:69046d7c]

%[text] ### PLL DDSRF
use_advanced_pll = 0;
use_dq_pll_ccaller = 0;
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*f_grid;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega0 = 2*pi*f_grid;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:116d3842]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:2081634f]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:6d47560a]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:35d1c6f7]
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
kp_pll = 314;
ki_pll = 3140;

Arso = [0 1; 0 0];
Crso = [1 0];

polesrso = [-5 -1]*2*pi*10;
Lrso = acker(Arso',Crso',polesrso)';

Adrso = eye(2) + Arso*ts_inv;
polesdrso = exp(ts_inv*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:81134edd]

freq_filter = f_grid;
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
figure;  %[output:6ac1ed0e]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:6ac1ed0e]
xlabel('state of charge [p.u.]'); %[output:6ac1ed0e]
ylabel('open circuit voltage [V]'); %[output:6ac1ed0e]
title('open circuit voltage(state of charge)'); %[output:6ac1ed0e]
grid on %[output:6ac1ed0e]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink settings
heatsink_liquid_2kW; %[output:1f5bcc1c] %[output:10d57663] %[output:74ffb24d]
%[text] #### DEVICES settings
% danfoss_SKM1700MB20R4S2I4; % SiC-Mosfet full leg
wolfspeed_CAB760M12HM3;

dab_mosfet.Vth = Vth;                                  % [V]
dab_mosfet.Rds_on = Rds_on;                            % [Ohm]
dab_mosfet.Vdon_diode = Vdon_diode;                    % [V]
dab_mosfet.Vgamma = Vgamma;                            % [V]
dab_mosfet.Rdon_diode = Rdon_diode;                    % [Ohm]
dab_mosfet.Eon = Eon;                                  % [J] @ Tj = 125°C
dab_mosfet.Eoff = Eoff;                                % [J] @ Tj = 125°C
dab_mosfet.Eerr = Eerr;                                % [J] @ Tj = 125°C
dab_mosfet.Voff_sw_losses = Voff_sw_losses;            % [V]
dab_mosfet.Ion_sw_losses = Ion_sw_losses;              % [A]
dab_mosfet.JunctionTermalMass = JunctionTermalMass;    % [J/K]
dab_mosfet.Rtim = Rtim;                                % [K/W]
dab_mosfet.Rth_mosfet_JC = Rth_mosfet_JC;              % [K/W]
dab_mosfet.Rth_mosfet_CH = Rth_mosfet_CH;              % [K/W]
dab_mosfet.Rth_mosfet_JH = Rth_mosfet_JH;              % [K/W]
dab_mosfet.Lstray_module = Lstray_module;              % [H]
dab_mosfet.Irr = Irr;                                  % [A]
dab_mosfet.Csnubber = Csnubber;                        % [F]
dab_mosfet.Rsnubber = Rsnubber;                        % [Ohm]
dab_mosfet.Csnubber_zvs = 4.5e-9;                      % [F]
dab_mosfet.Rsnubber_zvs = 5e-3;                        % [Ohm]

danfoss_SKM1400MLI12BM7; % 3L-NPC Si-IGBT
igbt.inv.Vth = Vth;                                  % [V]
igbt.inv.Vce_sat = Vce_sat;                          % [V]
igbt.inv.Rce_on = Rce_on;                            % [Ohm]
igbt.inv.Vdon_diode = Vdon_diode;                    % [V]
igbt.inv.Rdon_diode = Rdon_diode;                    % [Ohm]
igbt.inv.Eon = Eon;                                  % [J] @ Tj = 125°C
igbt.inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
igbt.inv.Erec = Erec;                                % [J] @ Tj = 125°C
igbt.inv.Voff_sw_losses = Voff_sw_losses;            % [V]
igbt.inv.Ion_sw_losses = Ion_sw_losses;              % [A]
igbt.inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
igbt.inv.Rtim = Rtim;                                % [K/W]
igbt.inv.Rth_switch_JC = Rth_switch_JC;              % [K/W]
igbt.inv.Rth_switch_CH = Rth_switch_CH;              % [K/W]
igbt.inv.Rth_switch_JH = Rth_switch_JH;              % [K/W]
igbt.inv.Lstray_module = Lstray_module;              % [H]
igbt.inv.Irr = Irr;                                  % [A]
igbt.inv.Csnubber = Csnubber;                        % [F]
igbt.inv.Rsnubber = Rsnubber;                        % [Ohm]
igbt.inv.Csnubber_zvs = 4.5e-9;                      % [F]
igbt.inv.Rsnubber_zvs = 5e-3;                        % [Ohm]

wolfspeed_CAB760M12HM3; % SiC Mosfet fpr 3L - NPC (two in parallel modules)
parallel_factor = 1.75;
inv_mosfet.Vth = Vth;                                           % [V]
inv_mosfet.Rds_on = Rds_on/parallel_factor;                     % [Ohm]
inv_mosfet.Vdon_diode = Vdon_diode/parallel_factor;             % [V]
inv_mosfet.Vgamma = Vgamma/parallel_factor;                     % [V]
inv_mosfet.Rdon_diode = Rdon_diode/parallel_factor;             % [Ohm]
inv_mosfet.Eon = Eon/parallel_factor;                           % [J] @ Tj = 125°C
inv_mosfet.Eoff = Eoff/parallel_factor;                         % [J] @ Tj = 125°C
inv_mosfet.Eerr = Eerr/parallel_factor;                         % [J] @ Tj = 125°C
inv_mosfet.Voff_sw_losses = Voff_sw_losses;                     % [V]
inv_mosfet.Ion_sw_losses = Ion_sw_losses;                       % [A]
inv_mosfet.JunctionTermalMass = JunctionTermalMass;             % [J/K]
inv_mosfet.Rtim = Rtim;                                         % [K/W]
inv_mosfet.Rth_mosfet_JC = Rth_mosfet_JC/parallel_factor;       % [K/W]
inv_mosfet.Rth_mosfet_CH = Rth_mosfet_CH/parallel_factor;       % [K/W]
inv_mosfet.Rth_mosfet_JH = Rth_mosfet_JH/parallel_factor;       % [K/W]
inv_mosfet.Lstray_module = Lstray_module/parallel_factor;       % [H]
inv_mosfet.Irr = Irr/parallel_factor;                           % [A]
inv_mosfet.Csnubber = Csnubber;                                 % [F]
inv_mosfet.Rsnubber = Rsnubber;                                 % [Ohm]
inv_mosfet.Csnubber_zvs = 4.5e-9;                               % [F]
inv_mosfet.Rsnubber_zvs = 5e-3;                                 % [Ohm]
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
%   data: {"layout":"onright","rightPanelPercent":51.7}
%---
%[output:1dd96712]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"     4.583333333333334e+02"}}
%---
%[output:174c5e41]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"     9.375000000000000e+02"}}
%---
%[output:6986c445]
%   data: {"dataType":"text","outputData":{"text":"--- INITIAL ELECTRICAL PARAMETERS ---\nNominal Power (Sn): 300.00 kVA\nNominal Primary Voltage (Vn): 800.00 V\nNominal Primary Current (I1n): 375.00 V\nNominal Secondary Current (I2n): 375.00 V\nNominal Frequency: 12.00 kHz\n----------------------------------------------------\nCore Section Area (S_Fe): 50.0501 cm^2\nPrimary Turns (n1): 6\nSecondary Turns (n2): 6\nPrimary Copper Area (A_Cu1): 125.00 mm^2\nPrimary Copper Band Length: 25.00 cm\nSecondary Copper Band Length (L_b2): 25.00 cm\nCore Height (AM-NC-412 AMMET): 29.00 cm\nCore Width (AM-NC-412 AMMET): 6.00 cm\nCore Length (AM-NC-412 AMMET): 95.00 cm\nCore Dept (AM-NC-412 AMMET): 8.34 cm\nCore Dept selected (AM-NC-412 AMMET): 10.00 cm\nSpecific Core Loss (AM-NC-412 AMMET): 16.00 W\/kg\n----------------------------------------------------\n--- LOSS ESTIMATION ---\nCore Mass (M_Fe): 37.09 kg\nCore Loss (P_Fe): 593.39 W\nCopper Loss (P_Cu): 66.60 W\nTotal Losses per Phase (P_tot): 660.00 W\n----------------------------------------------------\nEstimated Efficiency (Eta, cos(phi)=0.95): 99.77 %\n----------------------------------------------------\n--- LEAKAGE INDUCTANCE AND REACTANCE ESTIMATION ---\nCalculated Leakage Inductance (Ld_calc): 0.000024 H (24.13 uH)\nEffective Leakage Inductance (Ld_eff): 0.000031 H (31.37 uH)\nLeakage Reactance (Xd): 2.365 Ohm\nEstimated Short Circuit Voltage (Vcc): 110.86 %\n----------------------------------------------------\n","truncated":false}}
%---
%[output:521048c6]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     1.402049461134321e-04"}}
%---
%[output:60b575f9]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.919183588453085e+02"}}
%---
%[output:631fa7a0]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:69046d7c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"2","name":"Ldrso_pll","rows":2,"type":"double","value":[["0.004040205933898"],["1.129961081535149"]]}}
%---
%[output:116d3842]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"5","name":"Afht","rows":2,"type":"double","value":[["0","0.000010000000000"],["-1.421223033756867","-0.000188495559215"]]}}
%---
%[output:2081634f]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.018661060362323"],["3.911916400415777"]]}}
%---
%[output:6d47560a]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000250000000000"],["-35.530575843921682","0.995287611019615"]]}}
%---
%[output:35d1c6f7]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.466526509058084"],["97.797910010394418"]]}}
%---
%[output:81134edd]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.091119986272030"],["4.708907792220440"]]}}
%---
%[output:6ac1ed0e]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ1wHkeZ55\/dzaWsQIIthyQIxWvHkblAsooTQilaH4E7wMCdzO7ehyRX7XFCd\/iOZLm6xKUPAlxxQCypbKqobLgTWZUrV6xsFxAvNtSeoUwWLqt4CU4ibiEQxR9xhBLnQzEJiQKbJVc9ppXRaN73nY+ne55n5v9WpWL77Xm6+\/\/rHv31TE\/377z66quvEj5eFLjsssvo+PHjXupCJfkVOHHiBK1bty5\/IETwpgCYeZOapSLwYpHRaxBJzH4HBsYfexgYf1pz1CRponL0pwoxwEwXZfDSxcu0VhIzGBiP4wcGxqPYDFVJmqgM3alECDDThRm8dPGCgdHHi63FMDBsUnoJhJurF5lZKwEzVjmdBwMv5xKzVyCJGTIw7HhrB4SB8Sg2Q1WSJipDdyoRAsx0YQYvXbyQgdHHi63FMDBsUnoJhJurF5lZKwEzVjmdBwMv5xKzVyCJGTIw7HiRgfEoqdOqJE1Upx0tUXAw0wUTvHTxQgZGHy+2FiMDwyall0C4uXqRmbUSMGOV03kw8HIuMXsFkpghA8OOFxkYj5I6rUrSRHXa0RIFBzNdMMFLFy9kYPTxYmsxMjBsUnoJhJurF5lZKwEzVjmdBwMv5xKzVyCJGTIw7HiRgfEoqdOqJE1Upx0tUXAw0wUTvHTxQgZGHy+2FiMDwyall0C4uXqRmbUSMGOV03kw8HIuMXsFkpghA8OOFxkYj5I6rUrSRHXa0RIFBzNdMMFLFy9kYPTxYmsxMjBsUnoJhJurF5lZKwEzVjmdBwMv5xKzVyCJGTIw7HiRgfEoqdOqJE1Upx0tUXAw0wUTvHTxQgZGHy+2FiMDwyall0C4uXqRmbUSMGOV03kw8HIuMXsFkpghA8OOFxkYj5I6rUrSRHXa0RIFBzNdMMFLFy9kYPTxYmsxMjBsUnoJhJurF5lZKwEzVjmdBwMv5xKzVyCJGTIw7HiRgfEoqdOqJE1Upx0tUXAw0wUTvHTxQgZGOK+FhQUaGhqigwcPBi3dtm0bDQ4O1mz1vn37aHh4OPi+q6uLRkZGqKmpKbY8MjDC4Ueah5urLl7Sbq761PPfYswx\/5rnrVESM2RgIjRHR0eDfzGmZX5+nvr7+6mnp4e6u7uXcT9y5AiZ8hMTE4FpMcanpaWlpuGBgck7dfxeL2mi+u253trATBc78NLFS9ovCTAwDcZP2NBEi5rsy9TU1GLWJfr3aHkYGF2TFTdXXbyk3Vz1qee\/xZhj\/jXPW6MkZjAwdWjaDIzJxnR0dCTKwHR2dsZma8zFMDB5p47f6yVNVL8911sbmOliB166eEn7JQEGpsb4MZmX8fHxhutaZmZmqK+vj+bm5mhycjLW6NgqjIEJfw4fPqxv9FaoxbOzs9Ta2lqhHuvvKpjpYgheuniZ1kpiBgOT4BGSMSdxi3PNI6O9e\/cGa2Cam5uD9TDmU2vRLzIwuiYrfjvUxUvab4f61PPfYswx\/5rnrVESMxiYBjRNhmVgYIDGxsaora1tsbR9Wyn8yKhW2XAG5vjx43nHD673pICkieqpy+qrATNdCMFLFy9pvyTAwDQYP+E3jUyWxX5gYPRNvLQtxs01rWLFlwez4hmkaQF4pVFLRllJzGBgImMi\/BjImpRar0bHPUKq9bjJVINHSDImYNJWSJqoSdtc9XJgpmsEgJcuXsjACOcV3cguvDmd\/a63t3dxsa5d7Gu6hY3shMNN2TzcXFMKJqA4mAmAkKIJ4JVCLCFFi2J276Nn6MjxM\/T9mefo3mNnAjWQgfE4KJCB8Sg2Q1VFTVSGplc2BJjpQg9eunj5yMCcmn+ZzH+HfvIMTT\/+wqJZiVMKBsbj+IGB8Sg2Q1W4uTKI6DkEmHkWPGd14JVTwAIu52JmTIr5mP9\/9eiTdOKZhbpmxXZ1TfMKWrNqBX3sXWuQgfHJHwbGp9r56+KaqPlbgghJFQCzpErJKAdeMjikaUVaZsagGNNhHgGdWXiFvvz9xxMZFdMma1b+3dsvobWrm87+vXnFYnORgUlDLmdZGJicAnq+PO1E9dw8VBejAJjpGhbgpYuXaW0tZmGjYsrtuf8Jenz+5VRm5Q\/Xr6RNl68i8\/+wUamlEgyMx\/EDA+NRbIaqcHNlENFzCDDzLHjO6sArp4AFXG6ZmYyKMRmjh06kNiqm2e9762q66V1rgh4kMStxXYWB8TgAYGA8is1QFW6uDCJ6DgFmngXPWR145RTQw+XGqJx8doHuO34mlVGxxsRmVS5ddfbxT1azAgPjAXa9KmBgCgaQsnrcXFMKJqA4mAmAkKIJ4JVCLIdF7Zs\/9z76HE0dO0Onnjv7JlDSjzElxqi8Y90baP2F57EblVrtQAYmKSGGcjAwDCJ6DIGbq0exmaoCMyYhPYUBL09C\/7aarI99bCuNUWm\/+Bz6F1e9mS678LzgnzddvtJvJ0K1wcB4lB4GxqPYDFXh5sogoucQYOZZ8JzVgVdOAUOX20W0Npvyf378DP1o9oVM2RQT1mRUeq97U1BD+NGPJGYwMHzjp2EkGJiGEokqIGmiihJGcGPATDCcmKaBVzZe1qScfuFXdNfUXGqTsmhKVq2gq1rPpw+87cLFtSmN1qhIYgYDk238ZLoKBiaTbIVdJGmiFiaCsorBTBcw8FrOK7z2xPz5Fwuv0Pj3Hw8K2i30k1K2ZiS8kNZcm+exjyRmMDBJRwJDORgYBhE9hpA0UT12W3VVYKYLX1V5hXehNcQeffoluvuB05kzKSaGMSnvuWI1vfH15ybOpmQZLZKYwcBkIZjxGhiYjMIVdJmkiVqQBOqqBTNdyMrOK2xUvvX3T9OPf\/7LXCbFbKFvTMo1ay7w9qZPdERJYgYD43G+w8B4FJuhKkkTlaE7lQgBZrowl4GXXY9y9NTz9MjpF1PvlWKJhR\/3tLeeT+9\/24XBV43WpPgmLokZDIxH+jAwHsVmqErSRGXoTiVCgJkuzBp4WYNilLXb46fdJyVqUv7Z5auoc\/1KcrG5m+sRIIkZDIxr2qH4MDAexWaoStJEZehOJUKAmS7MEnhZg2IyHbffc4p+9uSLmR7zhE2KedTz3reupo2XXrCYRZGWSck6UiQws22HgclKMcN1MDAZRCvwEkkTtUAZVFUNZqpw1TwYkKsX0cWyx595ib529HQQPu0bPdEsilk02\/32S+jnZ34VLKA1n7KYlHr6S5pj6g3M\/Pw89ff30\/T0dKox397eTvv37091Td7CMDB5FfR7vaSJ6rfnemsDM13sOHiFH\/H8eO6X9K3\/9zSLQTFZlOvXryTzuMeakyoYlEYjiINZozqSfl8aAzM4OEgdHR2J+n3kyBEaHR2FgUmkVnULSZqo1aWQrudglk6voksn4RU2KPsfeopmTud\/xBMYklUrqP3S8+k\/bWoNzv3hPmiwaG1d1Z+Emau6o3FhYHwpTUTIwHgUm6EqSROVoTuVCAFmejAb0\/CPv3iCfu8Nb1o0EKOHTgRv8WRdJGszJdagdF6+kjatRwaFc1RImmPqDQwnGNexYGBcK8wbX9JE5e1ZeaOBmSy24TUop1\/4NX33p89mfs3Y9sw+xjEZlA9dfRG95eLXBV8t\/nvzClkilKw1kuaYegMTXgOzbds2Mo+SpH5gYKSSiW+XpImqS7niWgtm\/rU3Jxy\/Sq\/Svh+eplPPLuTKnoSNiFkYe+3vX0AbLnrNoGANin++0RolzTH1BsaKa9a0jI+PB39taWmh3bt3U1tbW\/G0Qy2AgRGFo2FjJE3Uho1FgUABMOMbCOG1J4d\/+iwdfez5IHjWt3fCGRSTPWm7+Dxa0\/Rr+uPrX7tPw6Dw8XMVSdIcK42BsbCibyVJysrAwLiaUm7iSpqobnpYvqhgloxp2Jw8\/\/IrwZs7edeehLMnxqD88caLqK1B9gS8kvGSVEoSs9IZmDDomZkZ6uvro7m5ORFZGRgYSdOwcVskTdTGrUUJZGBeGwPmsY75PPzkL+mhx19gNyeXNq+g6y9bSWtXNwX1ZH2DB3NM37yVxKzUBiY8NOyr0xMTE9Tc3FzIqIGBKUT2zJVKmqiZO1GxC8vMLLopm0H7jemncu8cG320Y8zJVW8+n65seX0uc5Jk6JWZV5L+aywjiVmpDUw4A2MGyuTkZOK9YlwMLBgYF6q6iylporrrZbkia2YWfqzz0ydfpJ+efpEeybmtfdicmD+bhbHvWPcGWn\/heYvmxGZQihgJmnkVoZeEOiUxK52BWVhYoKGhITp48GDAuquri0ZGRqip6Wyqs8gPDEyR6qevW9JETd\/6al4hkVk0c\/LCy6\/QN5nWnITNh1l3YjInH7hy6SnGkhfGSuRVzZmTvNeSmJXGwITfQsqTbYkaoEaLgM2jqa1btwb0zfEE9R5RwcAknyQSSkqaqBL00NCGIpiFMyfmz3977DmWNSdLzEnzCvrglRfSG5r+icoTjGuNnSJ4aRjHktsoiZl6AxN+64gj22KMkPmY\/WRs7J6eHuru7l42puwjqp07dwaPpvbt20dTU1M1Mz4wMJKn5fK2SZqoupQrrrXczMLm5OsPnqZjT70UdC7vq8TRzEn0zJ3w98Wp6b5mbl7uW4waJDFTb2AeffRR+vjHP06f\/vSnE69vSXMWUtjQRIeuMSwnT55MvHkeDIyuyS9poupSrrjWJmUWNiYv\/8M\/0oEfPU0nn8m\/CZvtefBWzqoVZBbEmsWw5tEOdorFLwnFzQy+mpPOMb4aa0dSb2BslsTFYY71YttHTZ2dnbHZmTjJYWB8DGm+OiRNVL5elTuSYWbP1jE9\/eaPnqafPPHLoNN5ztcJG5MgO\/JbcxLdKbYqmROuUYQ5xqWkvziSmJXGwExPT6ciaNar7N+\/v+Y1dk1NrcdS1sBs3ryZ7rzzTjL1J1kDE67w8OHDqdqMwn4VmJ2dpdbWVr+VoraaCsw9\/0rw3RMvvEIL\/\/AqfffYizT7i1eCv9vv8sjXcsE5weVvOv8cuuKic+ny1efStW9+7Vwd+32eOnDtUgUwx\/SNCEnM1BsY1\/iNkTEb4UXfZLIG5tSpU4sLd2uVtW1EBsY1Ld74kn7T4O2ZvGjhN3W+evRJOvHMQtBI7rUm5pHOe65YTW98\/blBfDzWKXYsYI4Vq3+W2iUxg4FpQNAs1B0YGKCxsbElZyvFPUKqVRYGJss0Kf4aSRO1eDWytSBsTI49\/RL94OQv2N7QWWJAVq2gt7359fSmc1+mP+pYegaa5NeIs6lanqswx\/SxlMQMBqbB+Km3g6\/JuKxdu3ZxDYwxMLfddhvt2rUrdrdfZGB0TVZJE1WKcsaQGENgF8GaP9\/94GmaeeolZ8bEZE3+5VVvpAtWnH3EUy9rAmZSRkqydoBXMp0klZLEDAYmMjLCbx3ZLIs53dosEo5+ouam3htL5loYGEnTsHFbJE3Uxq3lKRE2Jg8\/+SL99d8\/Q8effollAaxtYfgNnbNn6DQFO8SGv8\/amyoyy6qVhOvASwKFdG2QxAwGJsKu3k6+9rve3t7FV7bDG9k12ocGBibdRCm6tKSJmlUL+wjHXG\/\/\/MpvfkNfPXo6yJiYD8c6kyWZkVUraP1F59G\/3njxkoxJuEzW\/jS6rgzMGvWxTN+Dlz6akpjBwHgcPzAwHsVmqErSRK3VnfB+JiefXaCfnX6Rph9\/wZkxuar1fHrrm15Hv9+c7xRiBjyxITQwc9V3jXHBSx81ScxKaWDMBnPDw8PByDAHOD722GN1d8j1NYRgYHwpzVNPURM17tThrz7wJJ14mu\/NnGjGxKwzaQ\/Myevp8edeXnyko20BbFHMeEZc9aKAlz7mkpiVzsDYV5nNm0M33XRTsHbF7M9iDnistZbF1xCCgfGlNE89LiZq2Jy8Sq\/S1x94KlhjYj4cG61FjcllbzyPtvzBG+ncc3538XGONlOShqYLZmnqR9l0CoBXOr0klJbErFQGJrxz7oYNG6i\/vz8wMOaconpvE\/kaFDAwvpTmqSfNRI1mTczur2YXWFfGxGRMOi5bSetWv\/YoJ2xeeBTQFyUNM329K1+LwUsfU0nMYGA8jh8YGI9iM1RlJ2rYnPzwsV84fWV4w8Wvo3e\/ZVVw6vCSbErzazvCMnSttCEk3VxLKzJjx8CLUUxPoSQxK5WBMfzsidDhR0g2G1PrVGlP3PEatS+hE9YTzZqcfv5X9N2fzbPtZ7K4X8lvz825dNUK2nT5KhiThHyyFJN0c83S\/qpdA176iEtiVjoDY4ZD+NVmOzx27NiR+NBFV0MKGRhXytaOa9\/SeWx+gR489QI9cvpFlrUmYXPynree3ZreGBRsTe+fcbhGSTfXYpXQUTt46eAkdY6V0sBIHRIwMG7IWJPyVw89xWJQ7EZrq859hd5++cW08dILkDVxg449Kn4gskvqNCB4OZXXSXBJzGBgnCCODwoDk11s+7jnaw+cpr\/52XzmLIrNkJhHORsvPZ\/ecvHrFs1J9O0cSRM1u3LVuhLMdPEGL128TGslMSuVgbFvIU1PT9cdFdu2bYs9GsD1UIKBSaawMSt\/e+wM3fvoc8H\/w7vJNopgTYjZmn5L+0X0unN\/L\/NjHUkTtVG\/8f1ZBcBM10gAL128pM2xUhkYI65ZxLt3716amJhYPFDRGhuziHfLli2F7QkDA7N8strHP3vuf4L23P9kotkcNim9172pZgYlUbA6hXBzzaug\/+vBzL\/meWoErzzqFXOtJGalMjDhfWDM3i\/hT3gfmEceeYTMhnf79+\/3OgJgYM6ex2P+Gzt0ouEZPL6MSq1BIGmieh2oiisDM13wwEsXL2RgHPKCgXEobo7Q9z56hg786Cn6i3t\/XjeKMSz\/9tpL6Ia2VcFjn6J3jMXNNQf0gi4Fs4KEz1gteGUUrsDLJDErVQYmySOk7u7uxb1ivvjFL3odBlXKwARZlm+fpMkfPBGrsTUn29+7lt75W8PiFUaCyiRN1ATNRRGsgVE3BjDH1CETtc6sdAbGDIe4fWDMoY7msZJZI3P77bfT7t27qa2tzevoqYKBMetYRg+diF14a0zL0PvXUetKs6HbSq\/aZ6kMN9csqhV7DZgVq3\/a2sErrWLFl5fErJQGpnjE8S0oq4Ex2RZjWuIW4RrTMrh5XXC6cdGPhNKOC0kTNW3bq1oezHSRBy9dvExrJTGDgfE4fspmYIxxuWnPw8sW4xqj8icbL6b\/cH2LOtMSHg6SJqrHYaq6KjDThQ+8dPGCgXHMa2Zmhvr6+mhubm5ZTe3t7Uter3bclGXhy2Jg6hmXP++5QsXjoSTscXNNopKsMmAmi0ej1oBXI4XkfS+JWakyMAsLC8EeL52dnYv7vfT29pI9zHFwcDBYB1PUR7uBMcbluz+dp5u\/9rMlEpqMy4GPbVSdbYkbE5ImalFjVlu9YKaLGHjp4oUMjENe0deozV4va9euDQ5xNAt79+zZQyMjI9TU1OSwFbVDazYwcYtzy2pcLEHcXAuZJrkqBbNc8nm\/GLy8S567QknMSpWBiRoY88bRyZMng2MDwhvZNTc354aYJYBGA2OyLta82D4b41KmR0W1WEqaqFnGWxWvATNd1MFLFy9kYBzzMlkX84malu985zs0NTWFDEwK\/Y152fKlBxdfia6KcUEGJsUgEVYUPxCFAWnQHPDSxQsGxjGv8DoY8+jIGJrx8XFqaWkpZO+XcHc1ZWBM1uXGPQ8vNr9q5kXaRHU8bUoTHj8QdaEEL128pN0XS\/UISfpQ0GJgRg+dDPZ1sZ\/e6y6hO3qvkC4ve\/twc2WX1HlAMHMuMWsF4MUqp5dgkpiVysAkPQsJa2Dix3l0vYvdhM4YmCp+JE3UKuqfpc9glkW14q4Br+K0z1qzJGYwMFkpZrhOcgbGmJe7H3yK\/se3jgU9q7p5kZYqzTDcKnmJpJtrJQGk7DR4pRRMQHFJzEphYMzbRsPDww3Rbtu2LVjcW9RHsoH5yx88QX+296eL5qUKbxk1GgeSJmqjtuL7swqAma6RAF66eEmbY6UwMHYI1HuEJGGYSDUwX\/m7J+jj+2BeomMEN1cJsyZdG8AsnV5Flwavogmkr18Ss1IZmPQoll9h32I6ePBg8GXSrI05wmBgYIDGxsZqnnIt0cCEF+xW8U2jemNG0kTlGNtViAFmuiiDly5eyMAI5xXeR8ZmdHp6eoLdfGt9rOk5evRo3Ve1pRkYs+7l6s\/dt\/jYqIzHAeQZbri55lGvmGvBrBjds9YKXlmVK+46SczUZ2CsyZienm5INMthjmFDU6sCu8uv+V5LBiZsXky7zWvSVX3bqBZXSRO14eBGgUABMNM1EMBLFy9pc0y9gXGJP8maGlPmM5\/5DJlDI43Z0WBgoqdJw7zEjyLcXF3OLjexwcyNrq6igpcrZd3FlcQMBqYGZ7uDb1dXV93jB8wbUOZzzTXXJFoDE67u8OHD7kZZncjfO\/ES3fzNp4IS1755BX35T6q5z0sj8WdnZ6m1tbVRMXwvSAEwEwQjQVPAK4FIwopIYlZKAxP3WvWOHTvqrmOpNUaMkZmbm4s1MWbh7l133UW33norGagaFvHe++iZ4Hwj8yn7adJ5572k3zTy9qUq14OZLtLgpYuXaa0kZqUzMMa87N27lyYmJsjuuJt0MW7cUKr3dpExNzfccAN1dHSQhreQooczmkW7my5fqW8GeWqxpInqqcvqqwEzXQjBSxcvGBiHvFwcJWAX6IYNkelCvcXDk5OTgamJfop+C8kczmgOaTSfwc3raHDzWoc09IfGzVUfQzDTxQy8dPGCgXHIi8PAhN86sq9Hm5OsG+3gKz0DE31l+qFPXu+QRDlC4+aqjyOY6WIGXrp4wcA45pX3EVJ0I7vwIl77nXnjKJphkWxgom8d4dFRskGIm2synSSVAjNJNBq3BbwaaySthCRmpVsDY2BzLuLlHDxFPUIKZ182rV9JB27cyNmt0saSNFFLKzJzx8CMWVDH4cDLscAOwktiVkoD44AZS8giDEx44S7eOkqHUdJETdfy6pYGM13swUsXLzxCcsgrz9tGDpu1GLoIA3Py2QW65vNHgjZg4W46yri5ptNLQmkwk0AheRvAK7lWUkpKYla6DEz08VGtN4KKGAy+DQwW7uajLGmi5utJda4GM12swUsXL2RgPPKyu+maKs2bRLt37655UrSPZvk2MJ\/\/6xO06zsng67huID0hHFzTa9Z0VeAWdEE0tUPXun0klBaErPSZWBqATZmxuzpEt3PxeeA8G1gmm++J+ieWfuC16bTk5Y0UdO3vppXgJku7uClixcyMB55hTMwWU6i5m6qTwMzeugkjR46gexLDoi4ueYQr6BLwawg4TNWC14ZhSvwMknMSpeBkfbYKDzOfBkYrH3hmd2SJipPj8ofBcx0MQYvXbyQgXHIq95OvA6rTRzal4ExxwWYYwPMB28eJcazrCBurtm1K+pKMCtK+Wz1glc23Yq8ShKz0mVgigTbqG4fBia86y7WvjQiUv97SRM1X0+qczWY6WINXrp4IQOjjxdbi30ZmKs\/d1\/Q5v\/+r9bTf\/3na9jaX7VAuLnqIw5mupiBly5eMDD6eLG12IeB2XLHg3TvsTNBm3HmUT50uLnm06+Iq8GsCNWz1wle2bUr6kpJzPAIyeMocG1gcOYRL0xJE5W3Z+WNBma62IKXLl7IwDjkVW8Rr9kDxryhVOZ9YL79k2ep5y9+hOwL0xjDzZVJSI9hwMyj2AxVgReDiJ5DSGJWqgxM1Q2MWftisjBYvMszoyVNVJ4elT8KmOliDF66eCED44BX9PyjWlVs27aNBgcHHbQgWUiXj5DuffQMbfnSg0FDBt63lobevy5Zo1CqpgK4ueobHGCmixl46eIFA+OQV5X3gcHiXf6BhZsrv6auI4KZa4V544MXr54+okliVqpHSD7g5anDZQbGnnu0af1KOnDjxjzNxLW\/VUDSRAWUZAqAWTKdpJQCLykkkrdDEjMYmOTccpd0ZWDCO+\/i1encmBYDSJqofL0qdyQw08UXvHTxwiMkZl7hx0YbNmyg\/v5+mp6ejq2l6AMdXRkY+\/gIi3d5Bxdurrx6+ogGZj5U5qsDvPi09BVJEjNkYHxRJyIXBgZ7v7gDKGmiuutluSKDmS6e4KWLFzIw+nixtdiFgQk\/Prqj9wrqve4StvZWPRBurvpGAJjpYgZeunjBwDjkZR8nVekREh4fuRtQuLm609ZVZDBzpaybuODlRleXUSUxq8QjJGNsbrnlFvrEJz5BbW1tLtnWjc2dgcHjI7coJU1Utz0tT3Qw08USvHTxQgamIF7mKIE9e\/bQyMgINTU1FdIKbgMTfnw0uHkdDW5eW0i\/ylopbq76yIKZLmbgpYsXDExBvMp4FhI2r3M7mHBzdauvi+hg5kJVdzHBy522riJLYlaJR0gGpDnIcW5urjQZmPDjI7w+7WaqSpqobnpYvqhgpospeOnihQyMQ171FvG2tLTQ7t27S7MGJnz2ER4fuRlUuLm60dVlVDBzqS5\/bPDi19R1REnMSpeBWVhYoKGhIers7KTu7m6K\/r0RXFv+4MGDQdF6B0BGDVNXV1fdDA\/nGpjRQydp9NCJoI3YfbcR1WzfS5qo2XpQvavATBdz8NLFCxkYx7ziHhVZo9HT0xOYmnofc735mFOr611XyyiZTE+tE685DQxen3Y8kIgIN1f3GnPXAGbcirqNB15u9XURXRKzUmVg6p1GnfUtpLChaTQY9u3bR1NTUzWzMFwGJrz+5Z1tq+iv\/svVjZqG7zMoIGmiZmh+JS8BM13YwUsXL2RgHPJqZGCMGZmYmKDm5uZEragXLy6ALwODwxsT4ctdCDfX3BJ6DwBm3iXPVSF45ZKvkIslMStVBqbeepdG5iI6EozZGR8fp0brWux1SR5TmQxM+HP48OFMA\/Cjdz9JR3\/+cnDt0T\/D3i+ZRExw0ezsLLW2tiYoiSJSFAAzKSSStQO8kukkqZQkZqUyMAayeVS0ffuy\/P6fAAAgAElEQVT2JW8czczMUF9fH+3cuZM6OjpSjYUkr19b42QC19soj+sRUvPN9wR9wOvTqVCmLizpN43Uja\/oBWCmCzx46eJlWiuJWekMjDUxW7duXTIyJicnU5sXE8CYn4GBARobG4t9BTupeTGxOAxM+PXpvf\/xD+h9b12tbwYoabGkiapEssKbCWaFI0jVAPBKJZeIwpKYldLAcFKut4OvNS\/13jwKt4XDwOD1aU669WNJmqj+eq27JjDTxQ+8dPFCBsYhL2soent7M2VbTNPCbx01MihJHi9xGxi8Pu1wAEVC4+bqT2uumsCMS0k\/ccDLj86ctUhiVqoMTNq3huKgRjeyCy\/iDRukDRs2UH9\/P01PTy8J097eXvNNp7wZGJw+zTkNG8eSNFEbtxYlpP12CCKNFcAca6yRtBKSmJXKwBjQad828jk48hqY8PqXO3qvoN7rLvHZ\/MrVJWmiVk78jB0Gs4zCFXQZeBUkfI5qJTErlYGpdxaS4VUvO5KDZ+JL8xoYrH9JLDVLQUkTlaVDFQgCZrogg5cuXtKynKUyMNKHQl4Dg\/Uvfgnj5upXb47awIxDRX8xwMuf1lw1SWIGA8NFNUGcvAbG7v+yaf1KOnDjxgQ1okgeBSRN1Dz9qNK1YKaLNnjp4oUMDDOv8MLdWgtrbZXaHyFZAzO4eR0NbsYOvMxDaVk43FxdK8wfH8z4NXUZEbxcqusmtiRmyMC4YRwbNU8GJnz+0UOfvD7YhRcftwpImqhue1qe6GCmiyV46eKFDIxjXtHzkOqdj+S4KcvC5zEwdv2LCQoD44ccbq5+dOasBcw41XQfC7zca8xdgyRmpcvAxG0ul+SgRW7IcfE4DAzOP\/JB6mwdkiaqv17rrgnMdPEDL128pN0XS2Vg6m1kZ44E2LNnT93DFl0PpawGBhvYuSYTHx8312J0z1MrmOVRz\/+14OVf87w1SmJWKQNjsjMTExPU3Nycl2Gm6zkMDBbwZpI+00WSJmqmDlTwIjDTBR28dPFCBsYhr3rrXSTs0JvVwIQ3sMP6F4cDKBIaN1d\/WnPVBGZcSvqJA15+dOasRRKzUmVgDCTzqGj79u20e\/duamtrC7jNzMxQX18f7dy5M\/MhjxwDIKuBwQZ2HOqnjyFpoqZvfTWvADNd3MFLFy9kYDzwMiZm69atS2qanJws1LyYxmQ1MFd\/7j4y62CwgNfD4AlVgZurX705agMzDhX9xQAvf1pz1SSJWekyMFyQXMTJYmCwgNcFiWQxJU3UZC1GKTDTNQbASxcvZGD08WJrcRYDgxOo2eRPHQg319SSFX4BmBWOIFUDwCuVXCIKS2KGDIzHIZHFwGABr0dAkaokTdTiVNBVM5iBly4F9LVW0hyDgfE4frIYGCzg9QgIBqY4sZlqlnRzZepSqcOAlz68kpjBwHgcP1kMDBbwegQEA1Oc2Ew1S7q5MnWp1GHASx9eScxgYDyOn7QGJryA94NXXkhf+chVHluLqiRNVNBIpgCYJdNJSinwkkIieTskMSudgbF7vszNzS0j0t7ermon3vACXuzAm3yCcZWUNFG5+lT2OGCmizB46eJlWiuJWakMjN2Jt6WlhQYHB8WNjLQZmLCBOfCxjbTp8pXi+lTmBkmaqGXWmbNvYMappvtY4OVeY+4aJDErlYGpd5gjN8Qs8dIaGLuA19Q1\/4V3Z6kS1+RQQNJEzdGNSl0KZrpwg5cuXsjAOORlMzC9vb2F77ob182sBgY78DocNHVC4+ZajO55agWzPOr5vxa8\/Guet0ZJzEqVgTFgzDECRZ86XWuApDEw2IE37zTLf72kiZq\/N9WIAGa6OIOXLl7IwDjkZR8hTU9Px9aiaREvFvA6HCgJQ+PmmlAoQcXATBCMBE0BrwQiCSsiiVnpMjDCWC9pTpoMDBbwFk9S0kQtXg0dLQAzHZxsK8FLFy9kYPTxYmtxGgPztQdO00e\/8pOgbryBxIYgVSDcXFPJJaIwmInAkLgR4JVYKjEFJTErZQZm3759NDw8HACfnJykxx57jKampmhkZISampoKGwhpDAyOECgM02LFkiZq8WroaAGY6eCEDIwuTuHWSppjpTMwZgGv2cRuYGCAbrrppmA\/GLP2ZWhoiLj3h7FvPR08eDDgu23btrr7z6QxMPYIgU3rV9KBGzfqHe2KWy5poiqW0WvTwcyr3LkrA6\/cEnoPIIlZqQxMeB+YDRs2UH9\/f2AoOjo6nLydZMyS+Zg6bN09PT3U3d0dO6jSGJjmm+8JYsDAeJ+fyMAUJ3numiXdXHN3pgIBwEsfZEnMYGAYx0\/Y0MSFTWpg8AYSI5QcoSRN1BzdqNSlYKYLN3jp4mVaK4lZqQyMEdesfzHrXcKPkGw2pl52JO8wSrILcFIDM3roJI0eOhE0CQt485LJfr2kiZq9F9W6Esx08QYvXbxgYDzwMpvZbd26dUlNO3bsqPloJ2+TTOZlfHycurq66i4UNgYm\/Dl8+HBs1V\/+uzM0\/oMzwXcHP9xKLReck7eJuD6DArOzs9Ta2prhSlxSlAJgVpTy2eoFr2y6FXmVJGaly8AUCdYuIK71tlPSDAzeQCqS4mt147dDGRzStALM0qhVfFnwKp5B2hZIYgYDk5ZenfIzMzPBo6uxsTFqa2tbVjKpgcEbSIxQcoSSNFFzdKNSl4KZLtzgpYuXaa0kZqU0MOF9YOzwMPvBmLeRXH4ancOUxMDgDCSXhNLFljRR07W8uqXBTBd78NLFCwbGMS9jXvbu3UsTExPU3Nwc1JbkFecszQq\/dWT3hKm310xaAzO4eR0Nbl6bpWm4hkEB3FwZRPQcAsw8C56zOvDKKWABl0tiVqoMTL03gRplR7KMg+hGdkkW8R4\/frxuVTgDKQsJN9dImqhueli+qGCmiyl46eKFDIxDXr4NTNquJMnA4BXqtKq6K4+bqzttXUUGM1fKuokLXm50dRlVErNSZWAMNJNp2b59O+3evXtxIa2rR0hpB0kSA2PfQAoefX3h3WmrQHlGBSRNVMZulToUmOnCC166eCED45CXNSrT09MNazHnI+3fv79hOc4CaQzMmuYV9NAnr+esHrFSKoCba0rBBBQHMwEQUjQBvFKIJaSoJGaly8AIYRzbjCQGBmcgySEoaaLKUUV2S8BMNp9o68BLFy9kYPTxYmtxIwMTfoUabyCxyZ45EG6umaUr7EIwK0z6TBWDVybZCr1IErNSZmDi9oFxeZRA0tEEA5NUKRnlJE1UGYrIbwWYyWcUbiF46eKFDIxjXj73gUnblUYGBm8gpVXUbXncXN3q6yI6mLlQ1V1M8HKnravIkpiVKgOj\/TXqsIExC3jNQl58ilNA0kQtTgVdNYMZeOlSQF9rJc0xGBiP46dRBgaHOHqEkaAqSRM1QXNRRNg5LQDSWAHMscYaSSshiVmpDIwBrfkRkj3EEa9Qy5iykiaqDEXktwLM5DMKtxC8dPEyrZXErHQGxpqY4eHhJSNDwyJevEItazJLmqiylJHbGjCTyyauZeClixcMjD5ebC2u9wgJr1CzycwWCDdXNim9BQIzb1KzVAReLDJ6DSKJWSkzMF5ppqisnoEJH+KIPWBSiOqwqKSJ6rCbpQoNZrpwgpcuXsjA6OPF1uJ6BmbP\/U\/SjXseDuo68LGNtOnylWz1IlA2BXBzzaZbkVeBWZHqp68bvNJrVvQVkpghA+NxNNQzMHiF2iOIhFVJmqgJm1z5YmCmawiAly5eyMDo48XW4noGBqdQs8nMFgg3VzYpvQUCM29Ss1QEXiwyeg0iiRkyMB7RJzEweIXaI5AGVUmaqHJUkd0SMJPNJ9o68NLFCxkYfbzYWlzPwOAVajaZ2QLh5sompbdAYOZNapaKwItFRq9BJDFDBsYj+loGJvwKde91l9AdvVd4bBWqqqWApIkKSskUALNkOkkpBV5SSCRvhyRmMDDJueUuWcvA4BXq3NI6CSBpojrpYAmDgpkuqOClixceIenjxdbiJAYGr1CzyZ07EG6uuSX0HgDMvEueq0LwyiVfIRdLYoYMjMchAAPjUWyGqiRNVIbuVCIEmOnCDF66eCEDo48XW4trGRizgZ3ZyM58Hvrk9WTeRMKneAVwcy2eQdoWgFlaxYotD17F6p+ldknMkIHJQjDjNbUMjN0DBq9QZxTW0WWSJqqjLpYuLJjpQgpeunghA6OPF1uLYWDYpPQSCDdXLzKzVgJmrHI6DwZeziVmr0ASM2Rg2PHWDljLwGAPGI8QUlQlaaKmaHali4KZLvzgpYsXMjD6eLG1OM7AYA8YNnnZA+Hmyi6p84Bg5lxi1grAi1VOL8EkMUMGxgvys5XEGRjsAeMRQMqqJE3UlE2vbHEw04UevHTxQgZGOK\/5+Xnq7++n6enpoKVdXV00MjJCTU1NsS3ft28fDQ8PJyrbyMBgDxhZgwM3V1k8krQGzJKoJKcMeMlhkbQlkpghAxOitrCwQENDQ9TZ2Und3d1k\/97S0kKDg4PL+B45coRGR0dpYmIiMDjm2lpla2VgRg+dpNFDJ4LYMDBJp5CfcpImqp8e668FzHQxBC9dvJCBUcbLZFimpqZiszDR7+qVrWVgvvS9x+mT33g0UAV7wMgaHLi5yuKRpDVglkQlOWXASw6LpC2RxAwZmAbU6pmSuAyMzd7EhY17hGT3gDHl57\/w7qRjCOU8KCBponrobimqADNdGMFLFy9kYBTxsuthenp6gkdKcZ+ZmRnq6+ujubk5mpycpI6Ojpo9NAYm\/Dl8+DB99O4n6ejPX6aWC86hgx9uVaRO+Zs6OztLra1gook0mGmiRQReuniZ1kpihgxMjfFj17+Yr2st4jXZmb179wZrYJqbm4P1MOYTt17G\/HtcBubqz91H5lXqTetX0oEbN+obzSVuMX471AcXzHQxAy9dvJCBUcAriXmJLvg13TLZmIGBARobG6O2trZlPY0amPAeMDAw8gYGbq7ymDRqEZg1UkjW9+Ali0eS1khihgxMhFijN49scW4D8+e9V9DW6y5JMn5QxpMCkiaqpy6rrwbMdCEEL128kIERzss8BjLrWert\/WK7EPcIqd610QxMeBO7O3qvoF4YGFGjAzdXUTgSNQbMEskkphB4iUGRuCGSmCEDE8IW3cTOftXe3r5kr5fe3t7FxbrG8IyPjwdFG216FzUwe+5\/km7c83BwLfaASTx\/vBWUNFG9dVp5RWCmCyB46eKFDIw+XmwtjhoYbGLHJq2TQLi5OpHVaVAwcyove3DwYpfUeUBJzJCBcY77tQqiBmZ4\/wyN\/9\/ZoAD2gPEIImFVkiZqwiZXvhiY6RoC4KWLFzIw+nixtThqYOwmdmuaVwS78OIjSwHcXGXxSNIaMEuikpwy4CWHRdKWSGKGDExSagzlYGAYRPQYQtJE9dht1VWBmS584KWLFzIw+nixtThqYJpvvieIjT1g2CRmDYSbK6ucXoKBmReZ2SoBLzYpvQWSxAwZGG\/Yl+7EG97Ezrw+bV6jxkeWApImqixl5LYGzOSyiWsZeOnihQyMPl5sLa6VgRncvI4GN69lqweBeBTAzZVHR59RwMyn2vnrAq\/8GvqOIIkZMjAe6YcNDDax8yh8xqokTdSMXajcZWCmCzl46eKFDIw+XmwtDhsYbGLHJquzQLi5OpPWWWAwcyatk8Dg5URWp0ElMUMGxinqpcHDBgab2HkUPmNVkiZqxi5U7jIw04UcvHTxQgZGHy+2FocNjDlCwGRhzMfsAWP2gsFHlgK4ucrikaQ1YJZEJTllwEsOi6QtkcQMGZik1BjKhQ2M3cTOhMUuvAziOgghaaI66F4pQ4KZLqzgpYsXMjD6eLG1OM7AYBdeNnnZA+Hmyi6p84Bg5lxi1grAi1VOL8EkMUMGxgvys5WEDczVn7uPzF4w2MTOI4CUVUmaqCmbXtniYKYLPXjp4oUMjD5ebC0OGxjswssmq7NAuLk6k9ZZYDBzJq2TwODlRFanQSUxQwbGKeqlwa2BwS68HkXPUZWkiZqjG5W6FMx04QYvXbyQgdHHi63F1sCEN7HDLrxs8rIHws2VXVLnAcHMucSsFYAXq5xegklihgyMF+RnK4kzMHiF2iOAlFVJmqgpm17Z4mCmCz146eKFDIw+XmwttgYGu\/CySeo0EG6uTuV1EhzMnMjqLCh4OZPWWWBJzJCBcYZ5eWBrYLALr0fRc1QlaaLm6EalLgUzXbjBSxcvZGD08WJrsTUw2IWXTVKngXBzdSqvk+Bg5kRWZ0HBy5m0zgJLYoYMjDPMtTMw2IXXo+g5qpI0UXN0o1KXgpku3OClixcyMPp4sbXYZmCsgcEuvGzSOgmEm6sTWZ0GBTOn8rIHBy92SZ0HlMQMGRjnuF+rwBoY7MLrUfQcVUmaqDm6UalLwUwXbvDSxQsZGH282FpsDQx24WWT1Gkg3FydyuskOJg5kdVZUPByJq2zwJKYIQPjDPPywMbA\/M0Pf0ImA2M+vdddQnf0XuGxBagqjQKSJmqadle5LJjpog9eunghA6OPF1uLowYGu\/CySeskEG6uTmR1GhTMnMrLHhy82CV1HlASM2RgnON+rQJjYP73tx+gLV96MPhHk30xWRh8ZCogaaLKVEheq8BMHpN6LQIvXbyQgRHOa35+nvr7+2l6ejpoaVdXF42MjFBTU1Nsy48cOUJbt24Nvmtvb6eJiQlqbm6OLRvNwBz42EbadPlK4YpUt3m4uepjD2a6mIGXLl4wMIJ5LSws0NDQEHV2dlJ3dzfZv7e0tNDg4OCyls\/MzFBfXx\/t3LmTOjo6aN++fTQ1NVXT8BgDs+1\/fpdGD50IYsHACB4MRISbq2w+ca0DM13MwEsXLxgYZbzqmRLz3cmTJ2PNTVw3owYGBznKHgy4ucrmAwOjj0+0xZhj+hhKYoY1MA3GTy0DE83WJBmGxsB84PPfInOYo\/nMf+HdSS5DmYIUkDRRC5JAXbVgpgsZeOnihQyMIl52PUxPT0\/wSCn8sQZm8+bNdOeddwZrZpKsgfnlpgF65cK30O++9AzdP\/h2RWpUr6mzs7PU2tpavY4r7jGY6YIHXrp4mdZKYoYMTI3xYw2K+TpuEa\/9\/tSpU4sLd0dHR2lubq7uGpgrb\/k63XvsDOEYAfkTF78dymeERxL6GIVbjDmmj58kZjAwMeOnkXkxl8Q9QjKLegcGBmhsbIza2tqWRTaPkC74yF\/SqfmXadP6lXTgxo36Rm+FWixpolZI9lxdBbNc8nm\/GLy8S567QknMYGAiOBu9eRQubjIua9euXXy8ZAzMbbfdRrt27Yp9ldoYmDN\/NBGEgIHJPY+cB5A0UZ13tiQVgJkukOCli5dprSRmMDCR8dPoMVC4uNkDxpS3e7+YP5tP3CvX5t\/XXvkOev59Z8vgGAH5E1fSRJWvlowWgpkMDklbAV5JlZJTThIzGJjQuIhuYme\/sotzzWZ2Zp+Y3t7eYN8X8wlvZNdo07uwgcExAnImZK2WSJqo8tWS0UIwk8EhaSvAK6lScspJYgYD43FcrHnHB8i8hWQ+OEbAo\/AZq5I0UTN2oXKXgZku5OCli5dprSRmMDAex0\/ru\/6UXrrmI0GN2IXXo\/AZq5I0UTN2oXKXgZku5OClixcMjD5ebC0OZ2BgYNhkdRYIN1dn0joLDGbOpHUSGLycyOo0qCRmyMA4Rb00eMsH\/xu9\/E+3BP+IYwQ8Cp+xKkkTNWMXKncZmOlCDl66eCEDo48XW4sv+Tefo1+v+cMgHo4RYJPVWSDcXJ1J6ywwmDmT1klg8HIiq9OgkpghA+MU9dLgF\/37\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\/oKRtQ7nlCqTlZY\/6uPnmm4MDV\/GLg\/9RlYaZMTAnT54MGok1g\/5ZmRrT8DLl7c+4gwcP0rZt28g8afD5gYFxqDYMjENxGUKnnay2SnOjvf3227GIl4FBmhBpeJm59\/nPf54+\/OEPU2tra931TWnagLLpFEjDzK5Vsgt3zbXbt2\/HPEsnea7SaXjZBbw7d+4M1sCkyWDnamToYhgYLiVj4uARkkNxGUKnSZfCvDAInjNEGl6m7Pe+973gN0K8hZRT+ByXp2EWfYQEbjmEz3ipNl4wMBlBJ70s\/MgIi3iTquanXPSRUaNFvEWssvejhI5a0vAKv\/Ie7l0RaW4d6rppZRpm0fmH+6UbJvWipuElwXDCwDgeI3iN2rHAOcKneWUQ6ewcQjNdmoZXuEr8Js8EIEOYNMyii0KLeCSRoYuluiQNr7hHSL4f+cHAeBh+2MjOg8gZq6i3aZNdVGgeQ9T6jR4bbWUUPuNlSXnBwGQU2MFlaZiFN7IrYmM0B91XFzINL2Myt27dGvSxCF4wMOqGFxoMBaAAFIACUAAKwMBgDEABKAAFoAAUgALqFICBUYcMDYYCUAAKQAEoAAVgYDAGoAAUgAJQAApAAXUKwMCoQ4YGQwEoAAWgABSAAjAwGANQAApAASgABaCAOgVgYNQhQ4OhABQIK3Ds2DFatWpV4hOMzau6zz33HK1fv96ZkPa1+\/b2dpqYmEjcNi2HhNr++Xp11nd9zgYGArMqAAPDKieCQQEo4FOBtJudRTdLc9HWPCYkz7Uu+lIrpjEU5uPz8D4t2vjkUPW6YGCqPgLQfyigWAGJBiZtm8Lya\/khDQOjeNKUqOkwMCWCia5AgTIqEN6d1fTPPpZ55JFHFncBNf9ud0WO7ppsH3OsXr2a+vv7aXp6OpDJnotkt08\/ePBg8O9JHvuE6wg\/RrEnKmxwmIUAAASvSURBVFsOO3bsoO7u7mVYapWzBmbLli302c9+Nrgu+pgmvPtptB6j1S233ELvfOc7g+ttX5599lnq6+ujubm54JJPfepTdODAARobG6O2trbg32r1KW5MRQ2M+fsLL7wQ\/Gd1rHfuVPQcHVNH3L9pNHdlnINS+wQDI5UM2gUFoEDsSdLhH57RbEetA+aMlCMjI0E8Y2LMo4+Ojg6y5qinp2fRaNQ7tNOaHRuvqakp+MF7++230+7duwMz0CgDEy0fPlPGmCxjNK699tqgvTb+3r17g7U0xogMDAwsMR7heNakrVmzZvH6aB\/t359++umgza2trTQ0NBQYJftIqNHZX3EGZnx8nKxhi9M1PJxhYDC5ORSAgeFQETGgABRwokCjH4SNzEL0N\/uogYk7gbze4Y9xj3ii5eu1qdHBktED8kz7Gz1WCn9vDUzUkE1NTS0aGhMzbFDM32+77TbatWvXksXG9R4TxRkYk92xpsvWYcrFLWKGgXEyXSoXFAamcsjRYSigS4Hw45bo451aZiH6mKWrqys2AxN9lBNWJu7xT636wgd\/1jMwjRYRx5mVuH+LPlaLPiazGSb7aMj8P7zgNhzTZHXsgXzRkVHrMVCcgalXh31MZePDwOiag1JbCwMjlQzaBQWgwBIFwj+0w+tgwr\/lW0MSXZdiMxDRDIy5Npo5qCd7LXNS77FWOF5eA2Ni2bUs1mDFZWDSGJgHHniA7COq5ubmRKMOBiaRTCjkWAEYGMcCIzwUgAK8CoRNgM0wmMcUZr2IWcvR2dm5ZOFs2KREDUy99S5xrfbxCCm6xiVcpzEb9R4H2UdIYQMTl+0IP0IyGZjt27cvruFJQsvFI6RGZrLRo7Qk7UaZcikAA1MunugNFCiVAnFrYMJZkPCi1rjFqDYjYx8hGXHCJsfGNwt67SOWuHUoVlSuRbzhjEd4Xcw111yzbJFu1MCEr7VtNe0zC3LjDEzSRbwmhl3D0mjtUd5FvPYRn31zzPYjvHg5OpBhYEo1tVk6AwPDIiOCQAEo4EoB+8PNvgIcfjwUfgXaPFJ573vfu+RVaWNcPvShD9GnP\/3pxQxD1NTYrIx9vdr0w\/5grdWneq8cJ11YPDw8vBg+7nGQXTcS\/cEdrXvnzp3BK9Bm4a7tfzgDYyqJahh9jTr6Krm5ptYr4DbrZf5vTV\/ca9Th6+PMR3j9keF09dVX00MPPRSYqKjRtH2IZqdcjTnE1aEADIwOTmglFIACUIBNAWMo4t48SlpBkjUwSWMlLYcMTFKlqlMOBqY6rNFTKAAFKqhAdJ2PzbaE931JKwsMTFrFUN6FAjAwLlRFTCgABaCAIAWiuxPX2yU3SbOjhyvefffdwWWuzkbCYY5JqFSvDAxM9Zijx1AACkABKAAF1CsAA6MeIToABaAAFIACUKB6CsDAVI85egwFoAAUgAJQQL0CMDDqEaIDUAAKQAEoAAWqpwAMTPWYo8dQAApAASgABdQrAAOjHiE6AAWgABSAAlCgegrAwFSPOXoMBaAAFIACUEC9AjAw6hGiA1AACkABKAAFqqcADEz1mKPHUAAKQAEoAAXUKwADox4hOgAFoAAUgAJQoHoK\/H+b6NVx\/xXm3gAAAABJRU5ErkJggg==","height":300,"width":499}}
%---
%[output:1f5bcc1c]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:10d57663]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:74ffb24d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
