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
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/8) %[output:6e4cc00d]
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
omega_rso = 2*pi*f_grid;
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
omega_f = 2*pi*f_grid;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega0 = 2*pi*f_grid;
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
kp_pll = 314;
ki_pll = 3140;

Arso = [0 1; 0 0];
Crso = [1 0];

polesrso = [-5 -1]*2*pi*10;
Lrso = acker(Arso',Crso',polesrso)';

Adrso = eye(2) + Arso*ts_inv;
polesdrso = exp(ts_inv*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:16217f48]

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
%   data: {"layout":"onright","rightPanelPercent":34.7}
%---
%[output:1dd96712]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"     4.583333333333334e+02"}}
%---
%[output:174c5e41]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"     9.375000000000000e+02"}}
%---
%[output:6e4cc00d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     1.065340909090909e-05"}}
%---
%[output:01f62bda]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     4.127900074317465e-04"}}
%---
%[output:231dc037]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.919183588453085e+02"}}
%---
%[output:1bc16b1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:26989b55]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"2","name":"Ldrso_pll","rows":2,"type":"double","value":[["0.004040205933898"],["1.129961081535149"]]}}
%---
%[output:75c58cb8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"5","name":"Afht","rows":2,"type":"double","value":[["0","0.000010000000000"],["-1.421223033756867","-0.000188495559215"]]}}
%---
%[output:6a06269e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.018661060362323"],["3.911916400415777"]]}}
%---
%[output:0c76ff32]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000250000000000"],["-35.530575843921682","0.995287611019615"]]}}
%---
%[output:1ba8c347]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.466526509058084"],["97.797910010394418"]]}}
%---
%[output:16217f48]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.091119986272030"],["4.708907792220440"]]}}
%---
%[output:28c3084b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10XsV555+0tLESAraABAvFtQNy6jSNjCmpaghOqqU+2VOZbvohye02ddTWbXCyW\/CR5NJwSiBYUm1Slo9GpYrW3bOW3XYhsZttKXEIhCreEAPatgEsELYRwgFsqEkwmxC057lmxOjqfszMO8\/73jv633M4Rnqfee7M75l35q\/5fMv09PQ04QEBEAABEAABEACBEhF4CwRMiaKFrIIACIAACIAACEQEIGBQEUAABEAABEAABEpHAAKmdCFDhkEABEAABEAABCBgUAdAAARAAARAAARKRwACpnQhQ4ZBAARAAARAAAQgYFAHQAAEQAAEQAAESkcAAqZ0IUOGQQAEQAAEQAAEIGBQB0DAE4Hjx49TV1dX5G1oaIjq6+s9eZ7rZnx8nDZs2EAXXXQR9fX1UV1dndi7dMfVLKNJgRSHT33qU9Te3m6SpNA2\/f39NDg4GOVx48aN1NPTY5Xf3bt305YtW2jr1q2F5MHl279\/v\/j3wwoajEtLAAKmtKFDxotGoJqde5qA4Q5izZo11NLSIoInrYzS700rjEuHyB3offfdZyUOXNLYBoDfsX79+plkIQqY0ASnbYxh75cABIxfnvAGAjUhcPLkSert7aW9e\/fSzp07qyZgeOSnGu9Ngqo6w7a2NmMxokYobMSBSxqXSqAEjE3e4u8p+giMqqdHjhzBKIxLJUGaWQQgYFAhCklAH0rnDOpD4vrow8qVK+n666+PysAdmT6dokYLxsbG5nyu+7jiiivod3\/3dyObhoYGGh4epqampkQuulCI28dHJ\/hzNaXU2tpKN910EzU3N0cNt97x5\/nhqSj13gMHDkT540dNIV177bX02c9+NhIv6omz4N8rprrAUZ2mbq86QeVL71D1Mt566600MDCQ+N7Jyckof1NTUzN50mOoc2Tmt912G33xi18kVT7mH2et2KmpuaTOWsVVf68qb7xcKtaNjY0zIizOb8+ePdGUjHr0+hH3lycc4\/Uxy1dWPcwaqdGZcJ5V3uOiKP790tkqH1dddRXt27eP+PujYqeXmX+n3qHHNo9LUj0sZCOETBWeAARM4UM0vzIY77T00qtGOKmTinc87IfFgxIv8c+TOtiszp8\/S8ub6mzOOuusWWtglIDR88BCIUlw6CIm7seXgEn6Cz\/emcQ7tjSu\/Ps0AdPd3U2bNm2awz5LMPBn55xzDj3\/\/PORQEsSFfxOvaON5z2tXqj3PvTQQ4li5M4775xZd6LXN72DjguYuC\/1eZqIyaqznObw4cOpQknPU1y8xEWmEg8f+tCH6Bvf+MasxiNJhCR9v+IChG2S8si\/V+\/J861zKfoo0fxqcctdWgiYcscvuNyrBlr\/C1Q1\/lxYffSB\/8pWDaPeQeiNrf6Xp97hsUhQIwTKh3p3\/C99BTnpc70xvvzyy1MFjP4Xqq2fPAHDo0785E3lZI0Q8ajQsWPH5jDRRw2Y0\/Lly2eV0WQKKT69pdirePJoSzzunBdeD5I0MsQs161bF5VXH7ExWdhsMh0Ut4n\/rJgoscX5z3u3qntJ5VG\/Y6HLZU6bQtI5qvoUj+k999wTCaGkEZU0v\/FRODXqpPvIercaoVH1P4+Lj6my4Bo+FMiJAASMEzYkkiKQ9teZ6gC44V61alXiDhzd5tChQ4l\/VXO+dR\/8V7\/aMZS3CDfvL8c0gaA36Px+Wz++BIz+bhYj\/OgdZlrHonfgv\/d7v2csYJJGrJLey\/mIT5GljXCwLXfEKh8626T3xafSsgRM2tRZPE3WaEqS+I2XTU1PxoWQEm1pQiOvfurx1X2kxTU+mqNYKQGTNnWo77DT67L6XurTd6qd0LkkTVtKtSfwGzYBCJiw41u60lVbwOjbkPM6CFvhwfCTtlWb+tE753hnx771bdQmIzBsoy985Z95y258BCregdoKmDjH+ChNXDj5EjCqsqcJJ96ZlSRg4lNReSMwZRAwSSN+Kq7x+pc2AqP7SPtuQMCUrokNKsMQMEGFs\/yFcZ1Cik91qDUFaX\/NJg355wmYrKkffVSAo8B\/paYJGFM\/PDQfFxdqai0uYFgkmCyOjHfu+ghFfBqOO\/y8KSQeHcoTAHEfrlNIeu1OG9WIfwPiYiQ+GqHKrI\/EqfKouhNPkzSFlPfNk55CUmJXjVylCZikkSvFKD4Ck7boOj59lTWFlMQFU0h5tQWfmxKAgDElBbuqEJBexJslAPIETFbektaHpAmYPD883K7Ws8ShmwgYTpO0C0n5iu8k0Q+As1nEq6YS9DT83o997GPR6FDSw5ySyme6iJd9KlEXF05pC1z1NLoNv\/Pmm2+mG264Yc6CY04TFzD8u7QFwaqseYI5aXolbwRM55hWxizxoQuGT3\/606l1K8sH5yFpca\/pIl6dS94IZFUaGrwkCAIQMEGEMbxCmG6j1rdA522jTloYbDOFxJSzpifyFsnqJ\/Nm+eH3xLfcfuYzn6FHHnlkZtFq0giM3rmlLUTWfcfX5iQJHL0j19Py\/ysBk\/TeO+64Y9aJsny4nr7exmUbtS5E9A41aYt92vbtOFd9TQ77ZG7btm2jzZs3Rzj0kTS1myxtW3be+S1Z26j5XaYjE2lrV3gULkkcpI06MaOkLexJozhp4pd\/Hz\/5N20tkfJhMlIYXouGEkkQgICRoAqfogTydnyIvhzOKyaQNFUV32mWdg6P\/nKXg+wqzvw8daALTiXUknYm5eHBQXZ5hPC5DYEgBQw3bHwWBR+yldQQZh1wZgMPtrUhAAFTG+6+3po1hZY19ZX0fperBHyVY775SZpCYgZ5hz8mic5Q7q6ab3WgaOUNTsDkLe5Tn69evTq67Ez9zF9C24vTihbM+ZIfCJjyRzr+RwSXyFa8cBrcrVPduhCf2rURL5xTCM7qxiv0twUnYHi+l78k\/KSNwMSDyn9ZjI6OVvVW39ArFsoHAiAAAiAAApIEghIw\/FfdddddR52dnZGIgYCRrDrwDQIgAAIgAAK1IxCUgOGRFH74RMisNTA6bjWU3dHREU0p4QEBEAABEAABECg+gWAEDM+F79ixg6655hrii\/pMBIxa\/8Jh0m8xjoftPe95T\/EjiRyCAAiAAAiAQAIBvlV82bJlwbEJRsDwlBGfNcGnh+btQuIomooXtmUBMzExEVzwa10gcJWLANiCrRwBOc+otzJsQ+UahIBJ2tGgqkHS9fa2O49CDb7MV8XcK7ias7K1BFtbYub2YGvOytYSbG2JmdmHyjUIARMPYd4IDI\/W8CmUWdNGus9Qg29W9eWswBVs5QjIeUa9BVs5AjKeQ62z80LAqBEX3p20fPny6IZgdSy4qi5ZR6+HGnyZr4q5V3A1Z2VrCba2xMztwdacla0l2NoSM7MPlWuQAsYspOZWoQbfnICM5VNPPRXkwjIZWnZewdaOl4012NrQsrMFWzteptah9mEQMAY1INTgGxRd1ASNlRxesAVbOQJynlFvZdiG2odBwBjUl1CDb1B0URM0VnJ4wRZs5QjIeUa9lWEbah8GAWNQX0INvkHRRU3QWMnhBVuwlSMg5xn1VoZtqH0YBIxBfQk1+AZFFzVBYyWHF2zBVo6AnGfUWxm2ofZhEDAG9SXU4BsUXdQEjZUcXrAFWzkCcp5Rb2XYhtqHQcAY1JdQg29QdFETNFZyeMEWbOUIyHlGvZVhG2ofBgFjUF9CDb5B0UVN0FjJ4QVbsJUjIOcZ9VaG7dL3f5AO\/eu3ZJzX0CsEjAF8CBgDSA4maKwcoBkmAVtDUA5mYOsAzTAJ2BqCsjAbefAoXTnyKB2\/6SMWqcphCgFjECcIGANIDiZorBygGSYBW0NQDmZg6wDNMAnYGoKyMIOAsYAVoikEjExU0VjJcGWvYAu2cgTkPKPe+mcLAeOfaak8QsDIhAuNlQxXCBg5rmALtrIE\/Hvvv\/sQ9d\/9FKaQ\/KMth0cIGJk4QcDIcEUnK8cVbMFWloB\/7xAw\/pmWyiMEjEy4IGBkuKKTleMKtmArS8C\/dwgY\/0xL5RECRiZcEDAyXNHJynEFW7CVJeDfOwSMf6al8ggBIxMuCBgZruhk5biCLdjKEvDvnbdQ80JebKP2z7YUHiFgZMIEASPDFZ2sHFewBVtZAv69Q8D4Z1oqjxAwMuGCgJHhik5WjivYgq0sAf\/eIWD8My2VRwgYmXBBwMhwRScrxxVswVaWgH\/v6257mB548iVMIflHWw6PEDAycYKAkeGKTlaOK9iCrSwB\/94hYPwzLYzH8fFx6u7upoGBAWpqakrMFwSMTLggYGS4opOV4wq2YCtLwL93CBj\/TAvh8eTJk9Tb20sHDhyg4eFhCJgqRwUCRg442IKtHAE5z6i3\/tmuvOGbdOT4q5hC8o+2th73799P\/f39USYwAlP9WKCxkmMOtmArR0DOM+qtf7b1V90bOcU2av9sa+bx+PHjdN1111FnZ2ckYiBgqh8KNFZyzMEWbOUIyHlGvfXLlkdeeAQGAsYv15p72717d5SHVatWYQ1MjaKBxkoOPNiCrRwBOc+ot37ZQsD45VkIb7xwd8eOHXTNNdfQ5OSkkYDRM75v375ClKPsmWD2jY2NZS9GIfMPtnJhAVuwlSPgx3Nra2vk6LWz30vfu7QbIzB+sBbDC08ZrVmzhlpaWgi7kGoXE\/y1JccebMFWjoCcZ9Rbv2zVPUjsFWtg\/LKtiTde+9LV1UVjY2Nz3r9z585I1MQfbKOWCRUaKxmu7BVswVaOgJxn1Fu\/bNUWaggYv1wL4w0jMLULBRorOfZgC7ZyBOQ8o976ZasEzI+98gK98IVf9+u8AN7eMj09PV2AfNQsCxAwNUOPUQJB9OgI5OCCLdjKEfDnWV\/Ae9oLj9Nzf\/0H\/pwXxNO8FzAmccAUkgklext0BPbMTFOArSkpezuwtWdmmgJsTUnl2z3wxEu07vaHI8MFj+2hqf\/9+fxEJbOAgDEIGASMASQHEzRWDtAMk4CtISgHM7B1gGaYBGwNQRmY6etfTn9ggI586x8MUpXLBALGIF4QMAaQHEzQWDlAM0wCtoagHMzA1gGaYRKwNQSVY6ZPHy2pX0AnvvibNDEx4cd5gbxAwBgEAwLGAJKDCRorB2iGScDWEJSDGdg6QDNMAraGoHLMRh48SleOPBpZ9axdRoN\/+IsQMH7Qls8LBIxMzNBYyXBlr2ALtnIE5Dyj3vphq6aPePRlzycvpA\/\/3PsgYPygLZ8XCBiZmKGxkuEKASPHFWzBVpZA5d71xbsfumARffmTKynUPgxTSAb1JdTgGxRd1AQCRg4v2IKtHAE5z6i3lbHltS+bRh6lB558idToC\/8bah8GAWNQX0INvkHRRU3QWMnhBVuwlSMg5xn1tjK2+tqXzovPpds6V0QOQ+3DIGAM6kuowTcouqgJGis5vGALtnIE5Dyj3rqz1aeO9NEXCBh3pkGkhICRCSMaKxmu7BVswVaOgJxn1Fs3tvq2afbAIy88AqOeUPswjMAY1JdQg29QdFETNFZyeMEWbOUIyHlGvbVny+KFT9zlf\/k5dOOH6IwFp81yFGofBgFjUF9CDb5B0UVN0FjJ4QVbsJUjIOcZ9daObVy86OtedE+h9mEQMAb1JdTgGxRd1ASNlRxesAVbOQJynlFvzdnqa144VZp44c9C7cMgYAzqS6jBNyi6qAkaKzm8YAu2cgTkPKPe5rPlUZd7Hz9Of\/S3j88Y3965gjq0NS9xL6H2YRAw+fUlWPVqUHRREzRWcnjBFmzlCMh5Rr3NZhufMuLdRnxVgL5gN8kDBIxcnS2851CDX2vwaKzkIgC2YCtHQM4z6m062\/67D1H\/3U\/NGLB4ubVjBV16wcLcgITah2EEJjf04c4fGhRd1ASNlRxesAVbOQJynlFvZ7PlEZd\/nfoe\/dYX\/2XWB9t\/7b20YXWDcSAgYIxRhWcYavBrHSk0VnIRAFuwlSMg5xn19hTbaJ3LweP0R3\/z5joX\/n38gDrTSITah2EExqAGhBp8g6KLmqCxksMLtmArR0DOM+ot0ZYvjdPg\/ZOzINtMFyVFJ9Q+DALG4LsYavANii5qgsZKDi\/Ygq0cATnP87Xe8pbogbufii5h1J9KhYvyFWofBgFj8F0MNfgGRRc1ma+NlSjUN5yDrRxlsAVbHwR4mugL9z9NX4iNtqipItMFuiZ5CbUPC0bAnDx5knp7e2nv3r1RPDdu3Eg9PT2psd29ezdt2bIl+rytrY36+vqorq4u0T7U4JtUfEkbdARydMEWbOUIyHkOvd6yaPmHf3uBttw1Pgcij7Zccv7CaFs0\/7\/PJ9Q+LBgB09\/fH8WbRcvx48epq6uLOjo6qL29fU492L9\/P7H90NBQJFpY+DQ0NKQKnlCD7\/ML4uIr9MbKhYmvNGDri+RcP2ALtjYEeHro\/vEXads9hxKTqWki\/te3cFEvDLUPC0bAxGuGLmjin\/Hoy+jo6MyoS\/znuH2owbf5EkrYoiOQoHrKJ9iCrRwBOc+h1Nu0NS2KHAuVP\/+Nn6b3nF0nJlr0KIXahwUpYNQIDI\/GtLS0GI3ArF69OnG0hhOHGny5ZsjMcyiNlVlpq2sFtnK8wRZs4wR4auiuR56jfY8em7MQVxctff+pid63+PSqiBYIGLl6KuaZR14GBwdz17WMj4\/Thg0baGpqinbu3JkodPThNz3D+\/btE8v\/fHI8OTlJjY2N86nIVSsr2MqhBtv5zXbqxGsRgH9\/9Uf0+QdepAPPvJoIpOGM06Lf\/+l\/OJsWv+M0Uj\/L0XvTc2tr65zXTExMVOPVVX1HkCMwTJCFDIuTpMW5PGW0a9euaA1MfX19ZMtP2qJfjMDI1En8JSvDlb2CLdjKEZDzXNR6y1NC9x08Tv\/nqX9PHWFhKjw1dEXzO+nyFWcZHfEvR3K251D7sGAFDI+wdHd308DAADU1Nc1EU+1W0qeM0mz1EZgQ1Wu1vjxp7ylqY1VrLj7eD7Y+KCb7ANvw2bJg+dLYc3Tw6PdzBQvvHOq8eHEkXqQW4VZKHAKmUoJVTq\/vNOJRFvVAwFQ5EBmvQ0cgFwuwBVs5AnKea1FvWay88oMf0a33HskUK2qEpQyCJR4hCBjPdVYttB0bG7Py3NzcTHfdddecNPo0kBIpaVujk6aQ0qab+EWhBt8KvIBxLRorgWIU0iXYyoUFbMvLlsXKSyd\/SH95\/2SuWFGC5aM\/czb94Zp3R4Uu6ghLXkRC7cNqNoWUt1MoKSBqVCVJwMQPstMPp1OfdXZ2zizWVYt9+T04yC6v+st8jo5Ahit7BVuwlSMg59lnvWWxcvKHP6JbvpY\/sqLEyZJFC6jj4nPp0gsWlVasJEUHAsZznfUtYDxnb5a7UIMvyczEt8\/GyuR988kGbOWiDbbFYcvbl3lUhP\/9\/L7D9ORzrxiPrLBY+a2WBmo4862FXr\/ig3aofVjNRmB8BKVaPkINfrX4pb0HHYFcBMAWbOUIyHnOq7c8qjLxwiv0dwe+ayRU9JGVy5YvopZlC4MXKxiBkaufM571NTB59xZVITuZr4CAkYlAXmMl89b54RVs5eIMtrJsf\/zMxdGIysNPn6B7vnOMjrz4avSzyRPtBFq0gLrXLovMi7wzyKQ8vmxC7cNqPgKjr0XhRbfDw8Oztj37CmAlfkINfiVMfKRFR+CDYrIPsAVbOQJ+PCtR8o0nXqTdDx61Fiqci46fe3O9SlkX2Pqhme0l1D6s5gJGYY\/vSirSqEyowa\/GFyfrHehk5SIAtmArR8DOM0\/78HPPo8fo4SMnjKd+1AgK\/1vGrct2lGStQ+3DCiNg9PDpx\/wXYVQm1ODLfmXyvaOTzWfkagG2ruTy04HtXEY8msL\/Hf\/+D+mvHpiMDB548pRwMXnU6Enzu06j63\/1ZyNfmP4xIWdmE2ofVkgBo4ck7UA6s7D5sQo1+H7ouHtBR+DOLi8l2OYRcv98vrJVIuWpYydp\/8RL9PTxV61EihpR4TUqa95bTz+\/9MyZrcpKwMxXtu610SxlqH1YIQWMPgLD4cm7bNEshO5WoQbfnYiflGis\/HBM8gK2YOtKgKd8Fp\/5k\/Tn+47Q4WMnnUQKv5unff5r60\/Rd0\/8wHg0BfXWNWrZ6ULtwwojYLIOopMJqbnXUINvTkDGEo2VDFf2CrZgm0RATc2wSDnx6mv0lX953mkkRR9N+Y2fO5eWnlU3ZzTFJQKoty7U8tOE2ofVXMDou5CKMNqSVBVCDX5+tZe1QGMlxxds5zdbtcPnbw4cpfsPvmi1w0cnp7Ylf3DZmfTh5afulJNcm4J6K1NvQ+3DaiZg9F1HeUf5y4TU3GuowTcnIGOJxkqGK0Zg5LgWha0SKI9\/9\/v0pUeecx5F0UdSPtD4Dvr9DzXWdAEt2gSZuhtqH1YzAfPEE0\/Qpz\/9abr22mtn7ifKC13WXUh5aSv5PNTgV8LER1o0Vj4oJvsA2\/Ky1ad5uBQjDz4bCRSbA93ioyj886XnL6Ll73obrVpyhpfpHgnCqLcSVMO9kLhmAgZ3IclU1DJ5RWMlFy2wLTZbtaOHp2Nuv+9p+s7U95wFij6KsmLx26ntA++MCn\/pBQvlIAh5Rr2VARvqH+E1FzBjY2NWEWtubqak26itnFgahxp8SwzezdFYeUc64xBsa8+WF8qyQLn160fosWe\/X7FA4RLxzp7\/8os\/Rc+9fGpnjxIvcqWtrmfUWxneofZhNRMwMmGS8Rpq8GVomXtFY2XOytYSbG2JmdszW76vh58f\/uh1uvlrR+jQCye9CJRLL1hEy86ui85ImRlZeUOomOewvJaotzKxC7UPg4AxqC+hBt+g6KImaKzk8IJtZWzVFI86\/p692ZwsG3+7Gi3hEZQ1y+upZdmZ0WLZMk7zVEY2OzXqrQzdUPswCBiD+hJq8A2KLmqCxkoOL9ims1Xi5LyFb6Wbvno4OqzNdYGseovabvzu+gXU9oFz6H2LT5\/JAC4ZNK\/nqLfmrGwsQ+3DIGAMakGowTcouqgJGis5vPOVrdpezP\/+85Mv0j+\/cZGgj9ETPgL\/fQ2n00Vnv0aLFy8Ocg2KXI008zxf660ZHXerUPswCBiDOhFq8A2KLmqCxkoOb2hs41uLHzv6fXr46RMVnX+ij57w\/\/M244aFb6XLmhZFH6Utkg2NrVwttPcMtvbMTFKE2odBwBhEP9TgGxRd1ASNlRzesrBVwkRN6zCRV37wI\/ry2KnD2XxM7URi5I3Rk1\/+2XNmxInr1E5Z2MrVLjnPYCvDNtQ+rFACZvfu3bRly5YognyB4+HDh2l0dJT6+vqorq4uM7Lxu5Q2btxIPT09qWn4ULz169dHn\/PW7KGhIaqvP3VUdvwJNfgyXxVzr2iszFnZWhaJrX7myVcfO0YPHXmZjnhYd6JGSVicLD27jlicvO0nf1x8aqdIbG3rRdHtwVYmQqH2YYURMHwn0tTUFHV3d9OmTZsi8cHCore3lxoaGjLFCIec0\/PD6dQheR0dHdTe3j6nRqjbrrdt2xadAszCKUsohRp8ma+KuVc0VuasbC2rxVZfc3LgyAna9+ixKKuVrDlRZVULY5ve9XY6nwXKB86ZOeZeiRdbLj7sq8XWR17L5gNsZSIWah9WCAGjn8q7fPly6urqioQIiwt1fUDWCElSyHVBE\/+cBcuhQ4dyRZFKF2rwZb4q5l7RWJmzsrWslK2+5mSaiP7x316gf5l82bs44V07XZecR6\/84HXxkRNbhmn2lbL1lY8Q\/YCtTFRD7cOCFDBZ1xSoqabVq1cnjs4kVZ9Qgy\/zVTH3isbKnJWtZRrb+GJYPtF132PHvKw30UdFeFpH31Ic0pknqLe2tdHcHmzNWdlYhtqHFULAcCDUNI4+haRGY9KmgtJGXgYHBynthmslYNauXUt33HEH8VUGWANj81XwZ4vGyh9L5UmtNzl69Fk6fPJtdN\/B415HTSKRsmgB\/fTit9NHf+Zs+okf\/7HoMDY1leS6MNY\/CTmPqLdgK0dAxjMEjAzXWV71hbXqg61btxqPlOjO1Jqa+AJgJWCOHDkys3A3zVb54+Drz759+6pAI\/xXTE5OUmNjY\/gF9VTCA8+8Gnmanib6+8e+R1MnXqNnX34t+rfSp+GM0yIXi99xGr3\/XW+lS5bWRf+vHvV5pe8JIT3qrVwUwdYP29bW1jmOJiYm\/DgvkJfCjMD4ZsILdXk0Z2BggJqammbcJ00hpdnqAibE4PtmbusPf8meIsaX\/kXi4cy30i33HqGJ51859fsnT\/2+kmfmLJM3thGvaVpEP9NweiEWw1ZSrlqmRb2Vow+2MmwxAiPDVcxr1uJfHnFZunTpzMgOC5gbb7yRtm\/fnriVOtTgi8E3dBx6Y6WmVRjHf\/\/mFH370L9HZCo920Th1Q9ae3\/D6fQHl717Rpg8\/fTTdEnzm8LdMCQwMyAQer01QCBmArYyaEPtwwoxAqMW3fJ6lKwn62wXfdeRGmVJ234dFzdZO5Y4P6EGX+arYu61bI2VfuiaKiUfuManwvo4dG2OMHljIezmy5fSMy\/9P6tdOmVja15ram8JtnIxAFsZtqH2YYUQMBwyXsS7a9euWQfK6ee5rFu3LvNMmPhBdvoiXvVZZ2dntDWbH329TdqCX1WVQg2+zFfF3GtRGqskYfLtwye87s5hKvp0Du\/Q+c0PLqbXp9OPrDcnOdeyKGwrKUNR04KtXGTAVoZtqH1YIQRM1rZnfbTk4MGD0YF1d911l0yUU7yGGvyqQkx4mXRjFT+mngXEd579Hu39v897HTFR4kRtHV757nfQinNPtxox8R0Laba+81smf2ArFy2wlWEbah8GAWNQX0INvkHRRU0qbaz0+3Nen56m3d8+KiJMIoGyaAGtWV5PP7\/szIhJdEps\/QJRPpU4r5RtJe8OPS3YykUYbGXYhtqHFULAcMjyppD4SgB1VszNN98sE2WMwFSVa9Zha2oB7Nmn\/wTd9vWn6bCnu3NUAfXpnJb3LJx1A3GRhYlpgNANHP7RAAAgAElEQVQRmJKytwNbe2amKcDWlJSdHQSMHS8n66RzYPhSR3Vf0S233ELDw8OztkU7vcgyUajBt8TgxVy\/2G\/H1x+nB4++TjQtsDNn0QJa9VNn0CdWnzfrkLUQxIlJINARmFByswFbN24mqcDWhJK9Tah9WGFGYOxDUr0UoQZfgqASKK+9Pk1\/99B3vd46zPnlqZwVi99OV354ybwUJqYxQ0dgSsreDmztmZmmAFtTUnZ2ofZhEDAG9SDU4BsUfY6JfvswCxQ+dK3SA9fUrcO8M+e3Wxrohz+arukCWBcuRUuDjkAuImALtnIEZDyH2ocVRsDwYXIbNmygqampORHMu6tIJuRveg01+Gnc9Av\/Hvvu92nPI885ixRdnKxrfietOPft0cgJ35+DjkCu5oIt2MoRkPOMeivDNtQ+rBACRj\/eX533wme2qMsce3p6Zs5vkQlvttdQg6+Xmo+zv\/Ph79ITz9mNqCiBEm0dXnw6XXL+whm3eetN0FjJ1WawBVs5AnKeUW9l2IbahxVCwMTPgdGP+ueFvSMjIxS\/lFEmzMleQwz+yINHaeRbzxqNrCgh8kvvO4s2vbH2hEdQKn3QWFVKMD092IKtHAE5z6i3MmxD7MOYVCEFDG+XPnToEPHIS9adRjKhnuu17MFXC2tHHnyWWLhkPSxWLmtaRL9x0bniZ52gsZKrwWALtnIE5Dyj3sqwLXsflkalEAKGM6ffR6SLlnvuuYdGR0cxAuNQr1m4bP\/qYfof++euK2J3avrn1s4Vkfe8KR+HLGQmQWPlm+ib\/sAWbOUIyHlGvZVhCwEjw3XGq74Ohg+tY0EzODhIfCFjLc5+0YtbtuDzepZNux6d2Wasl4VFyu9\/qJE+uebdwhHNd4\/GKp+RqwXYupLLTwe2+YxcLcDWlVx2urL1YaYUCjMCY5rhWtiVIfg82vK1x4\/TVX\/7+BxELFpu7VghPiVkGxs0VrbEzO3B1pyVrSXY2hIztwdbc1Y2lmXow2zKo2wLIWBML3Osr693KWPFaYocfLW+Zd3tD88qJ4uWK5rfSV2XnFf1qSFT4GisTEnZ24GtPTPTFGBrSsreDmztmZmkKHIfZpL\/NBsIGAN6RQ1+0lSRGm3xsUvIAE1FJmisKsKXmRhswVaOgJxn1FsZtkXtwyotbU0FDO822rJlS24ZNm7cGO1IqtVTtODzqAvvJuq\/+6kZJGUSLirTaKzkajTYgq0cATnPqLcybIvWh\/kqZU0FjCpE1hSSr4JW4qdowV95wzdn3QPU\/7HltPZ9Z1VSxJqkRWMlhx1swVaOgJxn1FsZtkXrw3yVshACxldhpPwUJfg8ZaSvdSnjqIseIzRWUjWWcE2DHFqwBVtBAjKui9KH+S4dBIwB0VoHn6eM7v7OC9Rz5\/hMbv\/6d95Pv\/yBcwxyX1wTCBi52IAt2MoRkPOMeivDttZ9mEypangSr5o2Ghsbyy3bfL7MkcXL7m8fpa3\/eGq9S9lHXTACk1vdvRigI\/CCMdEJ2IKtHAEZzxAwMlxL4bVWwY8v1mXxsueTFxZ2W7RtMNER2BIztwdbc1a2lmBrS8zcHmzNWdlY1qoPs8mji20wU0jqJN+9e\/dGHEx3Lo2Pj1N3dzcNDAxQU1NTIsNaBf+vHniGuu88ODPyEpJ44UKhsXL5ypqlAVszTi5WYOtCzSwN2JpxsrWqVR9mm09b+0IJmKRt1Vu3biW+WiDv0e9SUtNTHR0dmWmV6Dlw4EDmdQW1CD5vk75y5NFgxQsETF6NruxzdASV8ctKDbZgK0dAxnMt+jCZksz2WhgBw+Jl165dNDQ0ROrEXVMhkgRKFzRpINWlkfx5kUZg5oN4gYCR\/Xqjk5XjC7ZgK0dAxjMEjAzXyKvvqwRMzpVhm+uuu446OzujiyOLImB43Quf86KeR\/7kF4JZ8xKvQugI5L5UYAu2cgTkPKPeyrCFgJHh6l3AqFus29raqK+vj+rq6hJzziM+\/KxatcpoDYzuZN++fSI0pk68Rn\/61RfowDOvRv7\/8mPn0kXnLRB5VxGcTk5OUmNjYxGyElwewFYupGALtnIE\/HhubW2d42hiYsKP8wJ5CXoKaWpqKlHE8MLdHTt20DXXXEPcGBVlEW\/\/3Ydmrge49PyFtOfKCwtUVfxnBX9t+WeqPIIt2MoRkPOMeivDFiMwMlxnea1kEW88e1m7i3iUZs2aNdTS0kJF2YWkTx2Ftl06reqgsZL7UoEt2MoRkPOMeivDFgJGhquYV7VAV18UzC\/LOkBv586dkaiJP9LBZ\/GyaeRReuDJl6JX83bpMtwmXWnw0FhVSjA9PdiCrRwBOc+otzJspfswmVzney3EFFIlu41UEfVdR2p7dENDQ+4t1kUYgdF3Hc2HqSNMc+R\/MSu1QEdQKUGIQzmCYFttthAwwsTj00dpoyFp2YgfZKcv4lWf8Y6j+AhLrQXMfJw6goAR\/jLhkEBRwBCHcnjBVoYtBIwM10SvaicRf8ijKMPDw6mn5FYjW5LB\/+xXJujP9x2OitGzdhn1rF1ajSIV4h1orOTCALZgK0dAzjPqrQxbyT5MJsdmXgsxhZSVVRYzvJ4lvpbFrHh+rKSCHx994TNf5tODxkou2mALtnIE5Dyj3sqwlerDZHJr7rWQAkYfgan1TdSMUir413zpCfqL+5+OonVb5wrqvPhc88gFYInGSi6IYAu2cgTkPKPeyrCV6sNkcmvutTACpmjTRjpCqeDXX3Vv9BreNj3fRl+43GiszL+otpZga0vM3B5szVnZWoKtLTEze6k+zOztclaFEDAmR\/\/LIcj3LBH8P\/37J+m\/fe3IvB19gYDJr3eVWKAjqIRedlqwBVs5AjKeJfowmZzaeS2EgLHLcvWtfQd\/vq99URFERyBXl8EWbOUIyHlGvZVh67sPk8mlvVcIGANmvoOvC5j5cmhdEmY0VgaVz9EEbB3BGSQDWwNIjiZg6wguJ5nvPkwml\/ZeIWAMmPkMPouXdbc\/TPzvfF37ghEYg0pXoQk6ggoBZiQHW7CVIyDj2WcfJpNDN68QMAbcfAZfH335s19dTl2XnGeQgzBN0BHIxRVswVaOgJxn1FsZtj77MJkcunkthIDJWsSbdqeRW3HdUvkM\/rrbHp53dx6lUUdj5VYfTVKBrQklNxuwdeNmkgpsTSjZ2\/jsw+zfLpcCAsaAra\/g66Mv8+nOIwgYg0rm2QQdgWegmjuwBVs5AjKeffVhMrlz91pTARO\/\/yitGBs3bsy9lNEdQX5KX8Hvv\/sQ9d\/9VPTC+bx4VxFHR5Bf91wtwNaVXH46sM1n5GoBtq7kstP56sNkcufutaYCRmV7vpwDo6aP5vviXQgY9y+saUp0BKak7O3A1p6ZaQqwNSVlZwcBY8crKGsfwX\/giZei3Uf88JUBfHXAfH\/QWMnVALAFWzkCcp5Rb2XY+ujDZHJWmddCjMBUVgT51D6Cr5+8i+mjUzFDYyVXd8EWbOUIyHlGvZVh66MPk8lZZV5rJmD0aaPly5dTV1cXjY2NJZam1hc6+gj+fL\/3KCmwaKwq+\/JmpQZbsJUjIOcZ9VaGrY8+TCZnlXmtmYCpLNvVTV1p8PXpo22\/tpw+sXr+nv2iRw6NlVw9BluwlSMg5xn1VoZtpX2YTK4q9woBY8Cw0uBf\/5UJ+vy+w9GbMH30JnA0VgaVz9EEbB3BGSQDWwNIjiZg6wguJ1mlfZhMrir3WggBo6aTQp1Cwu6j5IqKxqryL3CaB7AFWzkCcp5Rb2XYQsDIcM30ysLm6quvpj\/+4z+mpqamGuTg1CsrCb4+fbTpI0vos23n16wcRXsxGiu5iIAt2MoRkPOMeivDtpI+TCZHfrwWYgQmqyh8lcDIyAj19fVRXV1dqunJkyept7eX9u7dG9lkHX4XH\/Fpa2vL9F9J8HF4XXp00Vj5+RIneQFbsJUjIOcZ9VaGbSV9mEyO\/HgthYDp7++noaEhqq+vTy012\/DT09NDSqB0dHRQe3v7rDRK6KxevTr6TP3c0NCQetpvJcHH9BEEjJ+vqp0XdAR2vGyswdaGlp0t2NrxMrWupA8zfUct7AovYFiYTE1N5Y7AxOHpgiYPLF9pMDo6mvoO1+Drdx9d1rSIvvSHK\/OyMq8+R2MlF26wBVs5AnKeUW9l2Lr2YTK58ee1EAImaxEvj4wMDw9brYGxvZpASsDo61\/45F0+gRfPmwTQWMnVBrAFWzkCcp5Rb2XYQsDIcJ3xmja1o6Z6TF\/PIy+Dg4OUt65F+cuablI2rsFX00fsB9un50YQjZVprba3A1t7ZqYpwNaUlL0d2NozM0nh2oeZ+K6lTSFGYBhA0lSRibhIg2cy9aREE\/vIWiTMwdefffv2GcWsbcckTZ14jRrOOI32frzRKM18MpqcnKTGRnCRiDnYSlA95RNswVaOgB\/Pra2tcxxNTEz4cV4gL4UQMFlTPqa7kOJMx8fHqbu7mwYGBhKnn0zFC\/t1Ua\/6+hdc3phc4\/HXllxLALZgK0dAzjPqrQxblz5MJid+vZZCwJjsQopjYeGTls5k55HuzyX42D6dX1HRWOUzcrUAW1dy+enANp+RqwXYupLLTufSh8nkxK\/XQgiY+PoXvYh5C2yVrb7rKE+gmEwvVSpg9PUvx2\/6iN+oBeINjZVcIMEWbOUIyHlGvZVhCwEjw3XGK4+YbN68edaOI54G2rBhA23bto1aWloycxA\/yE5fxKs+6+zspLSbr7NuvHYJPm6fzq8waKzyGblagK0rufx0YJvPyNUCbF3JYQRGhpyFVxYx69evn5Vi586dueLF4hVOprYCRl\/\/0rN2GfWsXer03tATobGSizDYgq0cATnPqLcybG37MJlc+PdaiCkk\/8Xy69E2+Pr5L9g+nR4LNFZ+66nuDWzBVo6AnGfUWxm2tn2YTC78ey2EgNGnePKmivwjyPdoG3x9\/csjf\/ILtKR+Qf5L5qEFGiu5oIMt2MoRkPOMeivD1rYPk8mFf6+FEDC2J+f6x5Dt0Tb4K2\/4JvE0EgsXFjB4kgmgsZKrGWALtnIE5Dyj3sqwte3DZHLh32shBAwXy3S3kX8E+R5tgq+vf7n0\/IW058oL818wTy3QWMkFHmzBVo6AnGfUWxm2Nn2YTA5kvBZCwGTdhcTFztohJINltleb4OvrX7CANzs6aKzkai\/Ygq0cATnPqLcybG36MJkcyHgthICRKZo\/rzbBxwF25tzRWJmzsrUEW1ti5vZga87K1hJsbYmZ2dv0YWYei2EFAWMQB5vgqwW8WP+SDxaNVT4jVwuwdSWXnw5s8xm5WoCtK7nsdDZ9mEwOZLzWTMDoC3fTDpdTRS7LFBLWv9hVUjRWdrxsrMHWhpadLdja8bKxBlsbWua2EDDmrIKzNA0+DrCzCz0aKzteNtZga0PLzhZs7XjZWIOtDS1zW9M+zNxjMSxrNgITL378PqSs+5Gqjc40+DjAzi4yaKzseNlYg60NLTtbsLXjZWMNtja0zG1N+zBzj8WwLIyASbpgUU0zdXR0UHt7e82ImQZ\/57eO0qZdj0b5xAm8+eFCY5XPyNUCbF3J5acD23xGrhZg60ouO51pHybzdjmvhRAwWQfZ8f1IIyMj1NfXR3V1dXIkMjybBh8LeO3Cg8bKjpeNNdja0LKzBVs7XjbWYGtDy9zWtA8z91gMy1IIGB6dGRoaovr6+ppQMw2+uoEaB9iZhQmNlRknFyuwdaFmlgZszTi5WIGtC7X8NKZ9WL6nYlkUQsBkrXcpwgm9JsHHAl77io3Gyp6ZaQqwNSVlbwe29sxMU4CtKSk7O5M+zM5jMawLIWAYBU8Vbd68mYaHh6mpqSmiMz4+Ths2bKBt27ZRLS95NAk+FvDaV2g0VvbMTFOArSkpezuwtWdmmgJsTUnZ2Zn0YXYei2FdGAGjRMz69etnkdm5c2dNxQtnxiT4+gm8uIHarHKjsTLj5GIFti7UzNKArRknFyuwdaGWn8akD8v3UjyLQgmY4uE5lSOT4GMBr3300FjZMzNNAbampOztwNaemWkKsDUlZWdn0ofZeSyGNQSMQRxMgr\/yhm8Sr4PBAl4DoG+YoLEyZ2VrCba2xMztwdacla0l2NoSM7M36cPMPBXLCgLGIB55wccVAgYQE0zQWLlxM0kFtiaU3GzA1o2bSSqwNaFkb5PXh9l7LEYKCBiDOOQFHzuQDCBCwLhBckyFjsARnEEysDWA5GgCto7gcpLl9WEyb5X3Oi8FjNq2vXfv3ojwxo0bqaenJ5V2XvCxgNetoqKxcuNmkgpsTSi52YCtGzeTVGBrQsneJq8Ps\/dYjBTzUsDwwXj8sGgxua4gL\/hXjjxKIw8ejXxiB5J5xUZjZc7K1hJsbYmZ24OtOStbS7C1JWZmn9eHmXkpnlVhBIw682VqamoOpebmZtGTeHVBkxSivOBjB5JbxUZj5cbNJBXYmlByswFbN24mqcDWhJK9TV4fZu+xGCkKIWDUlE5DQ0PmVI4Esqx7mNT78oKPKwTcIoPGyo2bSSqwNaHkZgO2btxMUoGtCSV7m7w+zN5jMVIUQsCYiAgJXDzyMjg4SG1tbZmXRXLw9Wffvn0zP06deI3adkxGP2\/84EL6\/Z9fKJHVIH1OTk5SY2NjkGWrdaHAVi4CYAu2cgT8eG5tbZ3jaGJiwo\/zAnkphIBRIzCdnZ01OXWXhQxPXaXdeJ2lXnGFgHttxl9b7uzyUoJtHiH3z8HWnV1eSrDNI+T2OUZg3LgZp+K7kGp16zSvv+nu7qaBgYGZe5j0jGcFX9+BtOeTF9KlF2AExjToaKxMSdnbga09M9MUYGtKyt4ObO2ZmaSAgDGh5GijppDGxsYSPUgv4s0TT1nBv+ZLT9Bf3P90lG\/sQLKrAGis7HjZWIOtDS07W7C142VjDbY2tMxtIWDMWRXeUt91ZLKAOCv42IHkHm40Vu7s8lKCbR4h98\/B1p1dXkqwzSPk9jkEjBu3QqaKH2Rnsog3bQEU7kByDzEaK3d2eSnBNo+Q++dg684uLyXY5hFy+xwCxo2bVardu3fTli1bojQ7d+6kw4cP0+joaOYOIasXOBpnBR9bqB2hEhEaK3d2eSnBNo+Q++dg684uLyXY5hFy+xwCxo2bcSq1E4gX027atCk6D4bXvvT29lItzofRM54WfH0HUs\/aZdSzdqlxeWEIASNZB9ARyNEFW7CVIyDjGQJGhmvkVT8HZvny5dTV1RUJmJaWFspbYCuYrRnXJgIGO5DsI4GOwJ6ZaQqwNSVlbwe29sxMU4CtKSk7OwgYO15W1hAwVriCMUZjJRdKsAVbOQJynlFvZdhCwMhwnfHK6194vYs+haRGYzo6Oqi9vV04B+nu04KvdiBFo0g3faRm+Svri9FYyUUObMFWjoCcZ9RbGbYQMDJcZ3nl6aL169fP+t3WrVtrKl44M3kCZkn9gugMGDx2BNBY2fGysQZbG1p2tmBrx8vGGmxtaJnbQsCYswrOMi342IFUWajRWFXGLys12IKtHAE5z6i3MmwhYGS4lsJrUvCPHH+V+AwYfjovPpdu61xRirIUKZNorOSiAbZgK0dAzjPqrQxbCBgZrrO86ufAqA\/4PBjejVTLJyn42EJdeUTQWFXOMM0D2IKtHAE5z6i3MmwhYGS4znhl8bJr1y4aGhqi+vr66Pdqd1IRF\/HqIzDYQu1WOdBYuXEzSQW2JpTcbMDWjZtJKrA1oWRvAwFjz8w4hb6NOj7aUtRzYHALtXF4Uw3RWFXOECMwcgzBFmyrT0DmjRAwMlxnjbSow+v0VxVVwFw58iiNPHj0VP6xhdqpdkDAOGEzSgS2RpicjMDWCZtRIrA1wmRtBAFjjcwuAQuVzZs30\/DwMDU1Nc0SNkWcQsIt1HbxTbJGY1U5Q4wSyDEEW7CtPgGZN0LAyHCdJVTGxsZy38L3I9111125dj4NkoKPW6grJwwBUzlDdLJyDMEWbKtPQOaNEDAyXEvhNR58fQHvpecvpD1XXliKchQtkxAwchEBW7CVIyDnGfVWhi0EjAzXUnjNEjC4hdo9hGis3NnlpQTbPELun4OtO7u8lGCbR8jtcwgYN25WqZLOgSniVQL6GTB8hQBfJYDHngAaK3tmpinA1pSUvR3Y2jMzTQG2pqTs7CBg7HhZW5fpHBjefcS7kPjBGTDWoZ5JgMbKnV1eSrDNI+T+Odi6s8tLCbZ5hNw+h4Bx42aUqmznwOAMGKOw5hqhscpF5GwAts7ochOCbS4iZwOwdUaXmRACRoZr5LVsAmbd7Q8TTyNFeccZMM41A42VM7rchGCbi8jZAGyd0eUmBNtcRE4GEDBO2MwTVTqFpESQ2ord1tZGfX19VFdXl5gJfb1Nnm08+DgDxjyuWZZorPxwTPICtmArR0DOM+qtDFsIGBmus7y6LuI9efIk9fb20urVq6m9vZ3Uzw0NDcSn+8Yf\/XRfFjicNs2W08aDjzNg\/FQGNFZ+OELAyHEEW7CtLgGZt0HAyHAV88piaHR0NHEUJv5Zlm1cwOAMGH8hg4DxxzLuCWzBVo6AnGfUWxm2EDAyXMW8ZomSpBEYNXqTlCE9+LqAuenX30u\/8wsNYmUI3TEaK7kIgy3YyhGQ84x6K8MWAkaGq4hXtR4m6w6l8fFx2rBhA01NTdHOnTspfgu2njE9+PoZMDjErrLwobGqjF9WarAFWzkCcp5Rb2XYQsDIcPXuVa1\/Ycdpi3jjC4b7+\/ujfCStl+Hfc\/DV84Mll9Arqz4R\/fiXHzuXLjoPh9i5BnFycpIaGxtdkyNdBgGwlaseYAu2cgT8eG5tbZ3jaGJiwo\/zAnl5y\/T09HSB8lNRVkzES3zBL7+QR2O6u7tpYGBg5ibstBEYnAFTUYhmJcZfW\/5Yxj2BLdjKEZDzjHorwxYjMDJcvXnN23mkXlSpgNn6j0\/Rn\/3TocgdrhGoLHxorCrjl5UabMFWjoCcZ9RbGbYQMDJcvXnlaSBez5J19ot6WdIUUlZaPfg4A8ZbyAiNlT+WGIGRYwm2YFs9AjJvgoCR4erFa\/wQO+W0ubmZhoaGosPs+KyXzs7OmcW6LHgGBwcjU5uD7CBgvIQscgIB448lOlk5lmALttUjIPMmCBgZrqXwqge\/\/qp7ozxfev5C2nPlhaXIf1EzCQEjFxmwBVs5AnKeUW9l2ELAyHAthVcVfP0MmM6Lz6XbOleUIv9FzSQaK7nIgC3YyhGQ84x6K8MWAkaGaym8JgkYnAFTeejQWFXOMM0D2IKtHAE5z6i3MmwhYGS4lsKrCr5+iB2PvvAoDB53Amis3NnlpQTbPELun4OtO7u8lGCbR8jtcwgYN25BpEoSMHs+eSFdesHCIMpXq0KgsZIjD7ZgK0dAzjPqrQxbCBgZrqXwqoKPQ+z8hguNlV+eujewBVs5AnKeUW9l2ELAyHAthVcV\/CtHHqWRB49GecYhdpWHDo1V5QzTPIAt2MoRkPOMeivDFgJGhmspvKrg4wwYv+FCY+WXJ0Zg5HiCLdhWh4DMWyBgZLiWwisEjEyYIGBkuLJXsAVbOQJynlFvZdhCwMhwLYVXFfyVN3yT+CwYHGLnJ2xorPxwTPICtmArR0DOM+qtDFsIGBmupfCqgo9TeP2GC42VX56Y5pDjCbZgWx0CMm+BgJHhWgqvHPyvf\/s7xCMw\/OAQOz9hg4DxwxEjMHIcwRZsq0tA5m0QMDJcS+GVg\/\/X\/\/QQrbv9YQgYjxGDgPEIM+YKbMFWjoCcZ9RbGbYQMDJcS+E1LmCwhdpP2NBY+eGIUQI5jmALttUlIPM2CBgZrqXwysH\/3O5R4nNg+MEpvH7CBgHjhyM6WTmOYAu21SUg8zYIGBmupfDKwd\/4F1+j\/rufgoDxGDEIGI8wMYUkBxNswbZqBGReBAEjw7UUXjn4H\/3cV3AKr+doQcB4Bqq5A1uwlSMg5xn1VoYtBIwM11J45eC\/\/+r\/RQ88+RItqV8QXSOAp3ICaKwqZ5jmAWzBVo6AnGfUWxm2EDAyXEvhFQJGJkxorGS4slewBVs5AnKeUW9l2ELAyHAthVcO\/hmf+J84hddztNBYeQaKKSQ5oGALtlUhIPMSCBgZrqXwysF\/6VeGorziGgF\/IYOA8ccy7glswVaOgJxn1FsZthAwMly9eT1+\/Dh1dXXR2NhY5LOtrY36+vqorq4u8R379++n9evXR581NzfT0NAQ1dfXJ9ouff8H6cQv9UefdV58Lt3WucJbvuezIzRWctEHW7CVIyDnGfVWhi0EjAxXL15PnjxJvb29tHr1ampvbyf1c0NDA\/X09Mx5x\/j4OG3YsIG2bdtGLS0ttHv3bhodHU0VPLqAwRkwXkIWOUFj5Y8lRmDkWIIt2FaPgMybIGBkuIp5zRIl\/NmhQ4cSxU1ShpZ88KP0vUu7o4949IVHYfBUTgACpnKGaR7AFmzlCMh5Rr2VYQsBI8NVzGuagImP1phkoPHD\/5leWfWJyBQjMCbEzGzQWJlxcrECWxdqZmnA1oyTixXYulDLTwMBk8+oMBZqPUxHR0c0paQ\/SsCsXbuW7rjjjmjNTN4amIb\/+Ef06k+vi9yc\/sAA3bf79sKUtcwZmZycpMbGxjIXobB5B1u50IAt2MoR8OO5tbV1jqOJiQk\/zgvk5S3T09PTBcpPxVlRAoUdJS3iVZ8fOXJkZuFuf38\/TU1Npa6B0QUMLnKsOEQzDvDXlj+WcU9gC7ZyBOQ8o97KsMUIjAxXr17zxAu\/LGkKiRf1dnd308DAADU1Nc3J07m\/dgP9YMkl0e+P3\/QRr3mez87QWMlFH2zBVo6AnGfUWxm2EDAyXL15zdt5pL+IR1yWLl06M73EAubGG2+k7du3J26lfudvf4FeO\/u9uEbAW7ROOUJj5Rmo5g5swVaOgJxn1FsZthAwMly9ec2bBtJfxGfAsL06+4X\/n5+kLdf8ewgYb2Ga5QiNlQxXiEM5rmALtrIEZLxDwMhw9eI1foidchZh8uEAAAzQSURBVKoW5\/JhdnxOTGdnZ3TuCz\/6QXZ5h96d\/Qd\/S6+\/7WycwuslWm86gYDxDBQjMHJAwRZsq0JA5iUQMDJcS+G1\/qp7o3ziGgG\/4YKA8ctT9wa2YCtHQM4z6q0MWwgYGa6l8KoEDK4R8BsuNFZ+eULAyPEEW7CtDgGZt0DAyHAthVclYHrWLqOetUtLkecyZBICRi5KYAu2cgTkPKPeyrCFgJHhWgqvEDAyYUJjJcOVvYIt2MoRkPOMeivDFgJGhmspvCoBg3uQ\/IYLjZVfnpjmkOMJtmBbHQIyb4GAkeFaCq9KwOAeJL\/hgoDxyxOdrBxPsAXb6hCQeQsEjAzXUnhVAgbXCPgNFwSMX57oZOV4gi3YVoeAzFsgYGS4lsIrBIxMmCBgZLiyV7AFWzkCcp5Rb2XYQsDIcC2FVyVgcA+S33ChsfLLE6MEcjzBFmyrQ0DmLRAwMlxL4ZUFzJL6BcRTSHj8EYCA8ccy7glswVaOgJxn1FsZthAwMlxL4RUCRiZMaKxkuGIKSY4r2IKtLAEZ7xAwMlxL4ZUFDK4R8B8qCBj\/TJVHsAVbOQJynlFvZdhCwMhwLYVXCBiZMKGxkuGKUQI5rmALtrIEZLxDwMhwLYVXFjC4B8l\/qCBg\/DPFCIwcU7AFW3kCMm+AgJHhWgqvLGBwD5L\/UEHA+GeKTlaOKdiCrTwBmTdAwMhwLYVXCBiZMEHAyHDFNIccV7AFW1kCMt4hYGS4lsLrO3\/7C3Tzp34lmkbC448ABIw\/lnFPYAu2cgTkPKPeyrCFgJHhWgqvoQa\/1vDRWMlFAGzBVo6AnGfUWxm2ofZhb5menp6WQRaO11CDX+sIobGSiwDYgq0cATnPqLcybEPtwyBgDOpLqME3KLqoCRorObxgC7ZyBOQ8o97KsA21DwtGwBw\/fpy6urpobGwsqgFtbW3U19dHdXV1mTVifHycuru7aWBggJqamhJtQw2+zFfF3CsaK3NWtpZga0vM3B5szVnZWoKtLTEz+1D7sCAEzMmTJ6m3t5dWr15N7e3tpH5uaGignp6e1AgruwMHDtDw8DAEjNl3wZtVqF8qb4AqcAS2FcDLSQq2YCtHQMZzqHU2CAGTFPLdu3fT6Oho5ijM\/v37qb+\/P0qOERiZL06W11C\/VNUnOfeNYCsXBbAFWzkCMp5DrbPzVsDwlNN1111HnZ2dkYiBgJH54kDAVJ8rvzHUBqs2NGe\/FWzlogC2MmxD5RqkgFHrYTo6OqIppbQRGv79qlWrjNbAyFQreAUBEAABEAABeQITExPyL6nyG4ITMGpdC3NMW8TLC3d37NhB11xzDU1OTuYKmCrHBK8DARAAARAAARDIIRCUgDERL8yDp4zWrFlDLS0tZLILCbUIBEAABEAABECgWASCETCmO4\/i2631cOzcuTMSNXhAAARAAARAAASKTSAYAcOjKlNTU0Znv+ghwQhMsSsocgcCIAACIAACSQSCEDBpoyrNzc00NDQUHWbH58TwjqP4CAsEDL4YIAACIAACIFA+AkEImPJhR45BAARAAARAAAQqIQABUwk9pAUBEAABEAABEKgJAQiYDOy8rmZwcDCywAJft\/rJU3QbNmyI1ifl3U+l8+ZrILKud3DLTVipbNiqksev3QiLiJ\/S2HCNT1+jnciOgQ1b3RbtQWV1W33vk5ZRVOa5tqkhYFL4q2sGeA3NwYMHo63X\/P\/19fW1jViJ3q53luvWrZt1X1W8GPGrH\/jnXbt2gXlKvG3Y6i6Y65YtW2jr1q2phzyWqIp5z6oN1\/jOR6ynyw6HDVslDPkuO163iPbAvaor7nv37g3uD3EImJR6oe5I4i9QqOrV\/SthljLeoLMoHBkZMdophs4g\/y9Z\/RZ1E7bcKVx99dX00ksvUdYp1WbRDdPKps6y7Y033kjbt2\/HHzYG1cGWrV6\/0R4YAE4wUaNYF110ER05ciS63Diko0IgYBKCnna7tbrt2q0qzb9U+igWj1zFf84iggYru764sGVRfvHFF9OXv\/zlmZvb51+t9MfV5MJY8H2TgE2dTRqBybucF6znEnjmmWeiX\/JO3K6uLgiY+VBJkkZcuPFfunQpht0tKkB8VMDmL1bXc30ssldqU1u26vqMq666KrrEFGI8Ofw2XFnAHDp0KHKEtXL5XycbtuxNn\/rYuHFj1PnicSMQF4RuXoqXCiMwGSMw+oInCBj7ymvbYKk3cMdwyy23YBFvBnIbttwRfO5zn6OPf\/zj1NjYmLkWyT7KYaWw4arWE6mFu5x28+bNqLcpVcKGrZr62LZtWzTlYTN6G1aN9FMaCBg\/HEvhBVNIfsJkM2QM8WLH3IYt2953333RX7DYhZTN2YZrfAoJbMHW7ltcPWsImOqxLsSb9BEXLOJ1C0l8yihvoSl2GphztmGrb0\/X34Bh+bm8bbjG6zPaiez6a8MW4tC8LTCxhIAxoRSQDbZRVx5Mm22TGH63423DVveMUYJszjZc450Cpjn8sU2aQsL0nF0boVtDwLizK21KHGRXeeiyDq5SiyB5aiNtlAAHg6XHwJQtBIxdPbbhqh9kh8PW8jnbsGVBuH79+sgp2OazzbKAgKmMH1KDAAiAAAiAAAiAgDcC2IXkDSUcgQAIgAAIgAAIVIsABEy1SOM9IAACIAACIAAC3ghAwHhDCUcgAAIgAAIgAALVIgABUy3SeA8IgAAIgAAIgIA3AhAw3lDCEQiAAAiAAAiAQLUIQMBUizTeAwIgAAIgAAIg4I0ABIw3lHAEAv4JPPnkk7Ro0SLi27xNHj7v4cUXX6Tzzz\/fxNzJRp3Z09zcTENDQ8Z5K8sN46p81Tp7pNrvcwo6EoFAAQlAwBQwKMgSCDAB25Ndq3FYVSUipJK01awRLCj4qebtx2VhU8044F0gkEcAAiaPED4HgRoRKKKAsc2Tjq4snTQETI0qPF4LApYEIGAsgcEcBHwS0I+iZ79qWubgwYMzx6jz79WVCvErF9Q0x1lnnUVdXV00NjYWZU9d1Kju9tm7d2\/0e5NpH\/0d+jQKX\/2wZcuWmeJv3bqV2tvb5+BIs1MCZt26dXT99ddH6eLTNPrx8cqxeg+zuvrqq+myyy6L0quyHDt2jDZs2EBTU1NRks985jO0Z88eGhgYoKampuh3aWVKimVcwPDPL7\/8cvSf4ph1EWb8IkJ+R9LvyijufNZ9+AKBSglAwFRKEOlBwJFA0sWKeucZH+1Iu6GXX9\/X10fsj0UMT320tLSQEkcdHR0zQiPrxm+VH+Wvrq4u6nhvueUWGh4ejsRA3ghM3F6\/lI9FFguNiy66KMqv8r9r165oLQ0Lke7u7lnCQ\/enRNqSJUtm0sfLqH5+\/vnnozw3NjZSb29vJJTUlFDexaFJAmZwcJB0IcWcda56FYCAcfxCIBkIWBKAgLEEBnMQ8EUgSWDovvPEQvwv+7iA4fQjIyMznT3bZ91GnTTFE7fPylPeTdfxG4Y5P3nTSvrnSsDEBdno6OisMuoChd9x44030vbt22ctNs6aJkoSMDy6o0QX+8ziAAHj6xsCPyCQTQACBjUEBGpIQJ9uiU\/vpHWS8WmWtra2xBGY+FSOXsyk6Z+09+m3hmd13HmLiJPEStLv4tNq8WkyNcLE5UkSIrpPHtVRNxrHw5w2DZQkYDitvqg3S3hBwNTwC4VXzysCEDDzKtwobFEJ6J22vg6GO1O1VVkJkvi6FDUCER+B4bTxkYOs8qeJk6xpLd1fpQKGfam1LEpgJY3A2AiYhx56iNQUlelWdAiYon5LkC8QmE0AAgY1AgQKREAXAWqEgQUMrxfhtRyrV6+etXBWFylxAZO13iWpyNWYQoqvcdHfyWIjazpITSHpAiZptEOfQuIRmM2bN8+s4TEJtcQUUp6YzJtKM8k3bEBgvhGAgJlvEUd5C0MgaQ2MPgqiL2pNWoyqRmTUFBIXTBc5yj8v6FXTH0nrUBQQX4t49REPfV3MqlWr5izSjQsYPa3KK+ePF+QmCRjTRbzsQ61hyVt7VOkiXjXFp3aOqXLoi5fjlRACpjBfS2SkRAQgYEoULGQ1PAKqc1NbgPXpIX0LNE+pXH755bO2SrNwueKKK+jaa6+dGWGIixo1KqO2VzNB1bGm0czacmy6sDhpu7XJGpj4u7dt2xatc+GFu6r8+ggMlyHOML6NOr6VnNOkbQFXo178rxJ9Sduo9fRJ5dLXH3GcVq5cSY888kgkouJCU5UhPjoVXm1HiUDALwEIGL884Q0EQKDGBFhQJO08Ms2WyRoYU1+mdhiBMSUFOxB4kwAEDGoDCIBAaQnE1\/mo0Rb93BfbwkHA2BKDPQjUhgAETG24460gAAKeCMRPJ846JdfklfHLFe+8884omdTdSLjM0SQqsAGBuQT+Py6wLIOS8H\/IAAAAAElFTkSuQmCC","height":337,"width":560}}
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
