%[text] ## Settings for simulink model initialization and data analysis
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
simlength = 3.75;
% simlength = 1;
transmission_delay = 125e-6*2;
s=tf('s');

model = 'sst_llc_npc_inv';
rpi_enable = 0;
rpi_ccaller = 0;
%[text] ### Settings voltage application
application400 = 0;
application690 = 0;
application480 = 1;

n_modules = 2;
%[text] ## Settings and initialization
fPWM = 5e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM*4; % PWM frequency 
tPWM_DAB = 1/fPWM_DAB;
half_phase_pulses = 1/fPWM_DAB/2;

fPWM_LLC_pu = 0.54;
fPWM_LLC = fPWM_LLC_pu*fPWM_DAB %[output:174c5e41]

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

% t_misura = simlength - 0.2;
t_misura = 0.75;
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

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:6e4cc00d]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:01f62bda]
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
% LLC
fres = fPWM_DAB;
Ls = (Vdab1_dc_nom^2/(2*pi*fres)/Pnom*pi/8) %[output:1f071822]
Cs = 1/Ls/(2*pi*fres)^2 %[output:231dc037]

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
iph_grid_pu_ref = 2.75;
%[text] ### DAB Control parameters
kp_i_dab = 0.25;
ki_i_dab = 2;
kp_v_dab = 0.25;
ki_v_dab = 18;

%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
Vac_FS = V_phase_normalization_factor %[output:1bc16b1f]
Iac_FS = I_phase_normalization_factor %[output:693b28a9]

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
%[text] ### Notch filter
a = 0.98; 

omega_notch = 2*pi*100/(ts_dab/fPWM_LLC_pu);
C = cos(omega_notch);

B = [1, -2*C, 1];
A = [1, -2*a*C, a^2];
FLTnotchd = tf(B, A, ts_dab/fPWM_LLC_pu);
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
freq = f_grid;
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
%   data: {"layout":"onright","rightPanelPercent":47.6}
%---
%[output:174c5e41]
%   data: {"dataType":"textualVariable","outputData":{"name":"fPWM_LLC","value":"       10800"}}
%---
%[output:6e4cc00d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"     4.583333333333334e+02"}}
%---
%[output:01f62bda]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"     9.375000000000000e+02"}}
%---
%[output:1f071822]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     6.392045454545454e-06"}}
%---
%[output:231dc037]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     9.906960178361916e-06"}}
%---
%[output:1bc16b1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.919183588453085e+02"}}
%---
%[output:693b28a9]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:26989b55]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["2.831309534039184e-01"],["6.766822226281998e+01"]]}}
%---
%[output:75c58cb8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Afht","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-1.421223033756867e+05","-1.884955592153876e+01"]]}}
%---
%[output:6a06269e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Lfht","rows":2,"type":"double","value":[["1.866106036232337e+03"],["3.911916400415777e+05"]]}}
%---
%[output:0c76ff32]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000e+00","2.000000000000000e-04"],["-2.842446067513735e+01","9.962300888156922e-01"]]}}
%---
%[output:1ba8c347]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["3.732212072464675e-01"],["7.823832800831555e+01"]]}}
%---
%[output:16217f48]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["7.338637605205141e-02"],["3.802432508328568e+00"]]}}
%---
%[output:28c3084b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAArwAAAGlCAYAAAAYiyWNAAAAAXNSR0IArs4c6QAAIABJREFUeF7svQu4VlW1Pj5QxNCNCIJuAVPCSzsviBfMvFQHg7L0j55EJAUjFATt9EMQI8rskP0IJElFQI4BKuA+pqihgfrTg6KBmJAZ6kEhww2Iym0nghr\/Zyycn2sv1vetueaa173f+Tw+IHvOMcZ637Hm9+75jTlns127du0iNCAABIAAEAACQAAIAAEg0EgRaAbB20iZxWMBASAABIAAEAACQAAIRAhA8CIRgAAQAAJAAAgAASAABBo1AhC8jZpePBwQAAJAAAgAASAABIAABC9yAAgAASAABIAAEAACQKBRIwDB26jpxcMBASAABIAAEAACQAAIQPAiB4AAEAACQAAIAAEgAAQaNQIQvI2aXjwcEAACQAAIAAEgAASAAAQvcgAIAAEgAASAABAAAkCgUSMAwduo6cXDAQEgAASAABAAAkAACEDwIgeAABAAAkAACAABIAAEGjUCELyNml48HBAAAkAACAABIAAEgAAEL3IACACBiggsXLiQhgwZQq1ataJZs2ZR165drSNWX19PgwYNoqVLl1Lv3r1p4sSJpRg4vrFjx1JtbS1VV1dbjW3FihXUv39\/2rZtG02ZMoV69uwZ+Rfxnn322TR06FCrMWU5Gz58OM2bN69BvFljxM91PNfkyZNp0aJFNH36dKqqqpJ1ra1fnDM22qlTp0K5Uyk3tQVtwZDApaamxhk3Fh4TLpowAhC8TZh8PDoQkEHAZ8HL4mnChAmFRYsMDml90gTv+vXrqU+fPrR27VoaMWKEV4JXxNu6devcIk\/Hcwmx3b17dyeiKi5OBZ9FY2ksgjf+HPFf3lTfDYwDAr4hAMHrGyOIBwgAAWkEIHiloYo6CrySq+QyVhqb4FXBIA2nxiJ4i+aHTA6hDxBwiQAEr0v04Tt4BMTqp3iQtK\/946taF198MV177bWl5y63AijGiI7JfskP2W9+85tR2UG5\/uWAjouYcmPTVnjjz9StWzeaOnVqNDwepxBXwm7yq+NyYjWOqVhpSj7vL37xi1KJQ\/zZyq3WlVtZjD9\/clVLhtvkCi\/HEudBxBa3neSW+6StqCW\/euc+q1atSl3RlvmaPs+zckxxQZjEIu9zpeVZ0ofMM1SaMLL4StqXfVfSxqWVr4hyG5l3MfluJN8d\/n+ZdyyeS5z7N910E11++eWp3y5kzSnsUzwr\/91V+VLwHwp4AG8RgOD1lhoE5jsCacIl7UO0Ur\/kh37aV67CZlyAVOpX5IM8zVclwRvnKC72yz1zvI9NwVuuLEP8e1KMy3KbV\/BWshsXUeUEZtovD+X6Jn\/5ysIg7X0TOZcleLOe64QTTiiVecT9ZNmXrRuX4UtF8FbiQfxyJ\/MuxrlNE7uy84bAo0uXLqm\/8MWxlYkvucqtYxXf93kb8TVdBCB4my73ePICCMQ\/COOrmuKDt5z4i3\/ApPUVH4bx8WlCJfkhKz5Q4x\/qlWoT4+PjYi8tpizBm1x9TsMmHpfAoIjgFZvWZEsa0j7Iy30VnYfbPDW8aatn8bgELuW4icclOOMUFvXCaePj+VYOq7TV77Q8LCeGZJ8ruWopNq1lYZBVepCHrzzlB\/G4xLvEzyA2TwoOeOOd+Df+uXgX054rbZU9HlP8nY2LeJl3LDkniDGycwrHngefAtMnhgIBJwhA8DqBHU5DR0DmK3LxgSP6JlcRkwKCd\/unnUQQ\/xBKW7VJCluZjUHlThdIO\/GgkuBN2+Ge5j9td79NwZsmtt54443UExbycJtH8CZzPrnSJ4Rd3GZS6CRz6S9\/+UvqCRppK9flnisurCqJS9nVv3LPVU7wZq08Z52ikIevPIKuXFzJUybKCdZyzxvPg+QKcprgrfSOJX+WzJ08c4qIS2b+CH3+RvxNEwEI3qbJO566AAKVPjTTflbuAyTZ97rrrkv92jceatYqnsyHLPcpJ3jTYMmq4U0eLyX7gWlb8CZXIhcvXrxHPWxebvMK3kpfZ6cJ3mRtbxKzBx54IHqGci3tK\/CkqE37qj+tlKCS4JV5rnK5WWksj6lU1pCXLx2CN4l1JZtp70KlMok0wZv2TY2syL\/wwgul5xTxXLLfmhSYQjEUCDhBAILXCexwGjICeT9kIXjTz1q1LXjjvA0ePJheeumlPc71zcttHsEbFzpCxB1yyCF7lCRU+mXEhOAV72KaEIuvIJYTvLLPBcE7neLfKjAe\/AvNV77yldI3OxC8IX8yIHbfEYDg9Z0hxOclArIrLHzpQNGShjQA8q4qJW2U+9pc\/Pv48eNLlyiorvCmbQSrq6srnb9qW\/AyBsJnx44daevWrREsyd3oebjNI3iF77ioSavz1FHSkGcVMi2\/0mIoJ3hln6uc4C1XOiD74ufhS2WFVwhTcakIxzty5MhS3uR5F5977rmoBCX+bmTV8FZa4VUtaaiEbRqfslygHxDwGQEIXp\/ZQWzeIpBno0y5GknZTWtpoirPh2zabVblNkbFv14WX6fnFbxp2KRtABIf\/kyyqFVNHl9V7liyvJvWRCIlv75PExN5uFURvGknVXB8ujatlROWlWqr+UitLCGeJXiznqtcXGmiv1zftAkhD195BG9azvK7lHxv4ycmJMtFkpjHcz75fvGzya7wpj1znk1rlb5FkC1J8nZyRmBAoAwCELxIDSCgiEDWUUxiRahSv7jQ4b+XO680+WFYVPCyvXLHNCV95RW8cbGSBm3aiRLlKMgSvJU2\/aTZLCcKkn1luc36ZUTY5efg8gVxDXFabDLn3n7xi1+kV199tcEKYaUa2LTjsJKrgpVqSuMiNokdP0Pe5yq3oU32GcrliSxfeQQv+6qEjUo9PfsXp2qkPYus4E3jgu2Jby74qutyv0TG\/SZ\/4cuLj+LUiWFAwAkCELxOYIfTxoJA8gMx6+KJH\/7wh3TVVVcRfyBxk714IrlypEPwlhPYSV8qgpdtJ0VMGjZpK67xyzmyBG\/ygz9rR39cxGSd8SrDbaXTLtIuAknalL1MQsSattEu7ZeXSlhz\/2QZR9ovWmlYJuMX+Sv7XEk\/ccGVzIUsfpJziAxfKoIu7RfD+Hub911M2mNbRx555B6nbcistCa\/PYpvfC13wofALe1EjkqXkzSWORvP0XQRgOBtutzjyS0hIPPBZSkUuAkcgSI76CFmAidfInzZo+PKmUo7U1nCLboAgSAQgOD9lCb+IJk7dy7V1tZSdXV1WfLSvkoqt0oXRAYgSOMIQPAah7hROYiLlnKbm7IuYygHiBDMquMbFdABP0z825P450\/RDYDIj4CTAqFnIgDBG6tlbN26dUXBKz6IOnToUNppLn4j7tGjB4mNNJmoo0OTQgCCt0nRreVhs+q+k+fzyjoV81XWXCdrD\/3cIFCp\/p4jqnTLYrmI44s5qvnlBg14BQJyCDR5wVtut3oafMnjaEQf2dVhOUrQq7EhAMHb2Bi18zxpG7ny1rWmRSryEaLGDo+mvJTb4Kq6ei9EdE1NTWlBx1TssAsEXCDQ5AWvuCayW7duNH\/+\/MyShjSS2MbUqVP32AjiglD4BAJAAAgAASAABIAAEGiIQJMWvPEVW979LFPDW27FZNmyZUpiGQkJBIAAEAACQAAIAAEgYBaBJit4xddBffv2Jb4NS7UsQWwSkNm4tnbtWrNswjoQAAJAAAgAASAABD5FgDe+ou1GoMkKXq5jS15zmneFN0\/NE4tdvo5yyZIlyD0gAASAABAAAkAACBhH4LTTTiO+Kh7Ct4kK3rTNZ3lXePOIXc7oP\/3pT9SvX78o8fg2HDS\/EeBfTCZNmgS+\/KapQXTgLCCyiKJf\/vGOgbOwEAgrWvGOLVq0CIK3qa7wZh35k1WeIMoY8hz9IgQvEi+MCQN8hcFTPEpwFhZn4CssvuILN\/gcC4M7vGMNeWqyJQ3JdJVd4RViN+\/RL0i8MCYIESX4CosvfBiDr\/AQCC9izIthcQa+IHhTM1ZG8Ba5ZAKJh4kiLATCixbvWFicga+w+MIvleArPAQgeKUFb\/J83axSiEoHuWNyD+tVWbNmDc2YMYPGjBlDzZs3Dyv4JhotOAuLePAVFl8cLTgLizPoDgheJxmLxHMCu7LTDz\/8kNatW0eHHXYYBK8yinYHgjO7eBf1Br6KImh\/PDizj3kRj9AdELxF8kd5LBJPGTonAzGxO4G9kFNwVgg+64PBl3XICzsEZ4UhtGoAugOC12rCCWdIPCewKzvFxK4MnbOB4MwZ9EqOwZcSbE4HgTOn8Od2Dt0BwZs7aXQMQOLpQNGeDUzs9rDW5Qmc6ULSjh3wZQdnnV7AmU40zduC7oDgNZ9lKR6QeE5gV3aKiV0ZOmcDwZkz6JUcgy8l2JwOAmdO4c\/tHLoDgjd30ugYgMTTgaI9G5jY7WGtyxM404WkHTvgyw7OOr2AM51omrcF3QHBaz7LsMLrBGOdTjGx60TTji1wZgdnXV7Aly4k7dkBZ\/aw1uEJgheCV0ce5baBxMsNmdMBmNidwq\/kHJwpweZsEPhyBr2yY3CmDJ2Tgc8vf5UuubQ\/PfvHB6hTp05OYvDJKa4WtsQGBK8loDW5wcSuCUiLZsCZRbA1uAJfGkC0bAKcWQa8oLvzbn+JFr+xmf4y\/CgIXiKC4C2YULLDIXhlkfKjHyZ2P3jIEwU4y4OW+77gyz0HeSMAZ3kRc9sfgrch\/hC8lvIRgtcS0JrcYGLXBKRFM+DMItgaXIEvDSBaNgHOLANe0B0ELwRvwRRSGw7Bq4abq1GY2F0hr+4XnKlj52Ik+HKBejGf4KwYfrZHQ\/BC8NrOucgfBK8T2JWdYmJXhs7ZQHDmDHolx+BLCTang8CZU\/hzO4fgheDNnTQ6BkDw6kDRng1M7Paw1uUJnOlC0o4d8GUHZ51ewJlONM3bOnHs8\/TW+x9i09qnUKOG13zOYYXXEsY63WBi14mmHVvgzA7OuryAL11I2rMDzuxhrcMTBC9WeHXkUW4bWOHNDZnTAZjYncKv5BycKcHmbBD4cga9smNwpgydk4EQvBC8ThIPgtcJ7MpOMbErQ+dsIDhzBr2SY\/ClBJvTQeDMKfy5nbcd\/lQ0Bufw7oYOJQ25U0htAASvGm6uRmFid4W8ul9wpo6di5HgywXqxXyCs2L42R4NwYsVXts5F\/mD4HUCu7JTTOzK0DkbCM6cQa\/kGHwpweZ0EDhzCn9u5xC8ELy5k0bHAAheHSjas4GJ3R7WujyBM11I2rEDvuzgrNMLONOJpnlbELwQvOazLMUDBK8T2JWdYmJXhs7ZQHDmDHolx+BLCTang8CZU\/hzOefjyHjTGjfU8O6GDjW8uVJIvTMErzp2LkZiYneBejGf4KwYfrZHgy\/biBf3B86KY2jLAgTvnkhD8FrKPgheS0BrcoOJXROQFs2AM4tga3AFvjSAaNkEOLMMeAF3ELwQvAXSp9hQCN5i+NkejYndNuLF\/YGz4hjatAC+bKKtxxc404OjDSsQvBC8NvIs1QcErzPolRxjYleCzekgcOYU\/tzOwVduyJwPAGfOKZAOAIIXglc6WXR3hODVjahZe5jYzeJrwjo4M4GqOZvgyxy2piyDM1PI6rf77KrNdP7klyLD2LS2G1\/U8BbMs8mTJ9PcuXOptraWqqury1qD4C0ItOXhmNgtA67BHTjTAKJFE+DLItiaXIEzTUBaMDPnhfU0bM5KCN4Y1hC8BRJvxYoV1L9\/f2rdujUEbwEcfRyKid1HVirHBM7C4gx8hcUXRwvOwuFs3ILVNG7BGgheCN7iSVtfX0+DBg2ipUuXUqdOnSB4i0PqlQVM7F7RIRUMOJOCyZtO4MsbKqQDAWfSUDnvCMG7JwVY4VVMSy5lWLRoEXXr1o3mz58PwauIo6\/DMLH7ykz5uMBZWJyBr7D4wgpvWHxxOQOXNXBDDe9u7iB4FXJ44cKFNHLkSJo1axYtXrw4Vw3v7NmzoxVh0SrV\/SqEhiGaEMCHsSYgLZoBZxbB1uAKfGkA0bIJcGYZ8Jzu1q\/fLXC5XfXQRlr8xmYI3hiGELwKCdWnTx\/q27cvDR06lPJuWku64xrgAQMG5IwC3U0jsGPHDtq4cWO0EbF58+am3cG+BgTAmQYQLZoAXxbB1uQKnGkC0pCZmTNnRgtx3Lb2HEf\/2q8dBC8Er3q2DR8+nOrq6mj69OlUVVWVW\/COHz+eOnbsWAqABRVWedX5MDVy+\/bttGHDhmg1HoLXFMp67YIzvXiatga+TCOs3z4404+pTou8wsv\/\/WPThzT48V0l0yhp2A0FVnhzZFu8lKFr167RyLwrvFz3Gy9pyOEeXS0igK\/uLIKtyRU40wSkJTPgyxLQGt2AM41gGjQVP4OX3UDwQvDmTjde3Z03b17ZcSNGjIjKHNIazuHNDbfTAZjYncKv5BycKcHmbBD4cga9smNwpgyd1YFxwbvXB+\/S8jGnY6ENK7zFcxArvMUx9NECJnYfWakcEzgLizPwFRZfHC04C4Oz+JFkELyfcYaShoL5C8FbEEBPh2Ni95SYCmGBs7A4A19h8QXBGw5f593+UumEhs+9+jAtnXYtVnixwls8gSF4i2PoowV8GPvIClZ4w2OlfMR4x8JjE5z5z9lb739IJ459vhRo1bO\/puceuBOCF4LXXvKihtce1jo8YWLXgaJdG+DMLt5FvYGvogjaHw\/O7GOe12Nyw9oBC0fRs398AIIXgjdvKqn3h+BVx87FSEzsLlAv5hOcFcPP9mjwZRvx4v7AWXEMTVuIlzNw\/S4LXpwOtRt11PCazr5P7UPwWgJakxtM7JqAtGgGnFkEW4Mr8KUBRMsmwJllwHO6S5YzfLX1BloxczQE76c4QvDmTCjV7hC8qsi5GYeJ3Q3uRbyCsyLo2R8LvuxjXtQjOCuKoNnxyXKGn5y8g27\/6VAIXghes4mXtA7Baxfvot4wsRdF0P54cGYf8yIewVcR9NyMBWducJf1Gi9n+Hzbz9GUc5pRv379IHgheGVTSE8\/CF49ONqygondFtL6\/IAzfVjasAS+bKCs1wc404unTmvJcobbL6mhzp+sgeCNgYySBp0ZV8EWBK8loDW5wcSuCUiLZsCZRbA1uAJfGkC0bAKcWQZc0h2L3WFzVpbO3uXVXb5dDbqjIYAQvJIJVbQbEq8ognbHY2K3i7cOb+BMB4r2bIAve1jr8gTOdCGp106ydpdXdy85tRqCNwEzBK\/evCtrDYLXEtCa3GBi1wSkRTPgzCLYGlyBLw0gWjYBziwDLuEuWcogVnd5KHQHVnglUkh\/FySefkxNWsTEbhJdM7bBmRlcTVkFX6aQNWcXnJnDVsUyi91r5r5Kz6zaVBr+8NBudOaRB0b\/D90BwauSV4XHIPEKQ2jVACZ2q3BrcQbOtMBozQj4sga1NkfgTBuUWgzd\/ac6+o\/a10q2uIyByxlEg+6A4NWSaHmNIPHyIua2PyZ2t\/ireAdnKqi5GwO+3GGv6hmcqSKnf9y4Batp3II1JcPxUgYI3nS8UcOrPw9TLULwWgJakxtM7JqAtGgGnFkEW4Mr8KUBRMsmwJllwMu4S25SY7HLpQz8Z7xBd2CF10nGIvGcwK7sFBO7MnTOBoIzZ9ArOQZfSrA5HQTOnMIfOU8Tu7f1rSnV7ULwlucIK7yW8heC1xLQmtxgYtcEpEUz4Mwi2BpcgS8NIFo2Ac4sAx5zxxvU5rywbo8yhnJil4dCd2CF10nGIvGcwK7sFBO7MnTOBoIzZ9ArOQZfSrA5HQTO3MDPYvf8yS8R\/ykaly+M6tU5Om+3XIPugOB1krFIPCewKzvFxK4MnbOB4MwZ9EqOwZcSbE4HgTP78M95YX10i1q8lavZTUYH3QHBaz9j8dWCE8yLOMXEXgQ9N2PBmRvcVb2CL1Xk3I0DZ3axT57EwN5lxS73heCF4LWbsZ96Q+I5gV3ZKSZ2ZeicDQRnzqBXcgy+lGBzOgicmYefyxYWv7F5j1Vd9jz7B8fTN49tJx0EdAcEr3Sy6OyIxNOJpnlbmNjNY6zbAzjTjahZe+DLLL4mrIMzE6jutslCl\/+7eu7KBrW6YlU3q143LTLoDghecxlbwTISzwnsyk4xsStD52wgOHMGvZJj8KUEm9NB4Ew\/\/JWELnu7te8X6XvdD1VyDN0BwauUOEUHIfGKImh3PCZ2u3jr8AbOdKBozwb4soe1Lk\/gTBeSu+3whjSu042fviA8pN2cltc7dAcEb96c0dIfiacFRmtGMLFbg1qbI3CmDUorhsCXFZi1OgFneuC8d+k6umbuq6nGWOhWOls3TwTQHRC8efJFW18knjYorRjCxG4FZq1OwJlWOI0bA1\/GIdbuAJypQSrKFn69cHV0U1pakzlXN6936A4I3rw5o6U\/Ek8LjNaMYGK3BrU2R+BMG5RWDIEvKzBrdQLO8sHJQvfxle\/RyN+\/XnagCaErnEF3QPDmy1hNvZF4moC0ZAYTuyWgNboBZxrBtGAKfFkAWbMLcCYHKK\/icm0uHy9WbjX368e0pf\/T4\/DoXF1TDboDgreEwPDhw2nevHml\/58yZQr17NkzM\/cmT55MEyZMKPUbMWIEDR06tOI4JF4mrF51wMTuFR1SwYAzKZi86QS+vKFCOhBwlg6VODt39tJ1ZUUujxSruWd0OdCo0BVRQndA8EYIsNhdtmwZ1dbWUnV1NS1cuJCGDBlCWeKVxe7UqVNp1qxZ1LVrV1qxYgX179+fBg8eXFH0IvGk51QvOmJi94KGXEGAs1xwOe8MvpxTkDsAcLYbMnGqwhsbP6CJT\/w9U+ReeVYn+s7x7a2I3Dip0B0QvCWROn78+AYruiyC6+rqaPr06VRVVbXHZLB+\/Xrq06cPnXLKKTRx4sTSz5PiOW0WQeLlnludDsDE7hR+JefgTAk2Z4PAlzPolR03Zc7ExrO5L6yn2S+sq4ghr+R+9eg2dNFJ1XTmkQcq4110IHQHBG\/ZHILgLfp6NZ7xTXliD5VFcBYWc+ArLL442qbEmVjF\/eiTf9GPal+ruIrL2LDI5VKFS0491KnIxQpv+feq2a5du3aF99rpj1jU5WbV8ZYraejRo0eDVd9khOI3rdmzZ1OnTp1KP+ZyCjT\/EGhKE7t\/6KtFBM7UcHM1Cny5Ql7db2PnrG7rx5GwzarFFQh2OKA5ff8rHenkzx\/ghcjlb6Hjbe3atdSvXz9atGhRA92hngFhj2zyglfU7jKNvXv3rihaBdWibnfbtm3RP2WJZO4jBG8yXbj+d8CAAWFnUSOMfseOHbRx48aovrt58+aN8Akb3yOBs7A4BV9h8cXRNibOWNxyW7ftY5q6ZDO9+PaHUoSwyL2hRzviP\/k\/n9rMmTOj\/UXJBsG7G5EmL3hFYtTX19OgQYOiGl6xkS2ZNGl9RF1vhw4dytb+xgUv1w137NixZJoFFVZ5fZoydseyfft22rBhQ\/RbMQSvf\/ykRQTOwuBJRAm+wuKrMcyLz67aRK+u\/4AeWvGOtMDlUoVv1LSlbx97EH35iD339vjEIuuR+CrvkiVLaNKkSVjh\/ZQkCN5YtmaduCBWg5MrumJcchNc\/EVA8bhP00J2LI39q7tsBMLrAc7C4gx8hcUXRxsSZ2KT2f1\/3kB8mkK5M3GTLLDA5Y1mfU85NKrLNXlOrukMgO5oiLCXgnfLli10xx13EP+p2lq3bk3XX399ruEygnfkyJGlI8mEcbHK27dv37JHkyHxclHhvHNIE7tzsDwJAJx5QoRkGOBLEiiPuvnKmRC39Ts+ptuf\/oe0uGVo45vNQhe4yVSB7ghA8AoByQXXqo2\/iua6lbTGK7VpwrXcCq6wgRVeVTbCG+frxB4ekvYiBmf2sNbhCXzpQNGuDV8445vMdtEu+vWCNbnErRC43z3pEPra0W2DX8HNYh+CNyDBO2bMGKmbz5KkszAdO3ZsWcEranF5nDhzV6zu1tTUlK3F1VHDi+LxrFfUj5\/7MrH7gUYYUYCzMHgSUYKvsPjiaF1wxqu3\/++19+n3f96gJG4Pa\/M5YoHbpf1+XpykYJN1CN4ABC\/vjr\/66quj\/84666zc+fHMM8\/QbbfdRvfdd1\/FscmrhZO3rCWPIBPGkuNkTndA4uWm0ekAFxO70wduBM7BWVgkgq+w+DIteEVZAtfa8uYy2ZrbOIpcknBOzUH0w69\/PvrnkOtvdWQHdEcAgveTTz4h\/q9FixY6OPfCBhLPCxqkg8CHsTRU3nQEZ95QIRUI+JKCyatOOjgTwpYf7MHlG+j1DfIbypLiVlz00Nhqb3WRDt0RgODlGl4+LPmwww6LNoHxVb577723rhxwYgeJ5wR2Zac6JnZl5xiohAA4U4LN2SDw5Qx6Zcd5OBM3lfGfK9f\/kx5e8Y7Sqq1YqRXilv\/f5XW9yuA5GAjdEYDg5dMZ\/vM\/\/5P+8Ic\/0M6dO6l9+\/Z0+eWXE5+C0KZNGwdpU9wlEq84hjYt5JnYbcYFX+URAGdhZQf4CosvjjaNs7iw3Vi\/k+5a\/HYhYct++n+5Ax3aet\/oqt6mXpZQJEugOwIQvCJE3iT21FNP0V133UV\/+ctfon8+4YQT6KqrrqKvfe1rQZU8IPGKvLb2x+LD2D7mRT2Cs6II2h0PvuziXdQbC1vm7C9vrqNP9m1N9yzNv4ksHgMLWd5Q1vfUajq8bctGf2JCUfxVxkN3BCR446HyRrb777+fZsyYEV35WlVVRRdddBENHDiwwc1lKklhYwwSzwbK+nzgw1gflrYsgTNbSOvxA7704KjbSrzG9oW\/b6H\/9+r7yiu2HJtYoe136qH0lU9XbLFqq5u1dHvQHYEKXhH2rl276PXXX6c777yT\/vjHP9IHH3xAnTt3jlZ9zz\/\/fG9XfZF4dl5wXV7wYawLSXt2wJk9rHV4Al86UFSzES9DINpFtS9uoNXvbtcibC\/sdgj92zGN\/4xbNeTtjoLuCFzwxsPn+t7nn3+efvrTn0b\/XFtbS9XV1XYzStIbEk8SKE+64cPYEyJyhAHOcoDlQVcwd+kLAAAgAElEQVTwZZaEhqKW6M9vbaXHV75XSNRyxB0OaE7Nmzenbx3Xjr59XPvoIXBKglkuVa1DdzQCwSuELpc3sODlI8x69uxJN910E\/GVwj42JJ6PrJSPCR\/GYfHF0YKzsDgDX3r4igvbtzd\/SPcsWUf\/2PQhiX9X9SJqbPlEhDO6tIlE7cH7Ea1bty46QYlFL5rfCEB3BCp4WdQuX76c5syZUyplCOn0BiSe3xNDMjp8GIfFFwQv+AoPAfmI46L2nW076HfP1WkTtRwFn4ZwWufW9IV2+1VcrcW8KM+ZDz2hOwISvFyvu3bt2mij2sMPP0zvvfdeVKN77rnn0uDBg+noo4+mZs2a+ZBXmTEg8TIh8qoDJnav6JAKBpxJweRNJ\/C1JxXPrtocCc6nX3+f\/vvFDVpELXsRq7Xnd21PNdVVpY1keTePgTNvXh+pQKA7AhC827dvp7vvvptmzZpFdXV1kaitqamJzuL95je\/GZ3QEFpD4oXFGCb2sPjCCi\/4CgGB+AkID614h15d\/0\/tovbkzx9Ax1TvHx35pbu2FvNiCFn2WYzQHQEIXr5prU+fPrR58+agjh6r9Cog8cKaKDCxh8UXBC\/48gUBUX7wx1fepRVrt0W1tIvf2KwlPLEie+5x7ej4jq2MiNpKgWJe1EKjNSPQHQEIXl7hXbNmDXXp0sXbY8byZiwSLy9ibvtjYneLv4p3cKaCmrsxofIlVmlZfN729Fu0cp2+Vdp4+cFRB+9HfMSX+Le85QcmmA2VMxNYhGATuiMAwStWeMeMGROdvpC3LVy4kMaOHUuLFi3KO9RYfySeMWiNGMbEbgRWo0bBmVF4tRv3ka\/kUV5\/f387zX1hffTsulZp46L2lMMPoB5fPCgqawjhGl0fOdOemI3IIHQHBK+TdEbiOYFd2SkmdmXonA0EZ86gV3Lsiq\/4Ci3fJDbjuTojgpaNsog997j21Lplc+WNYkrgGhrkijNDj9PozUJ3BCR4+YQG1dapUyes8KqCh3E40zXAHMCHcVikmeRLiNolq7dEJx7oXqGNr9J279yavn5028iH7k1ivjFqkjPfnrUxxAPBG4Dg3bJlC91xxx3Ef6o2voDi+uuvVx2ufRwSTzukRg1iYjcKrxHj4MwIrMaMqvCVLDl4dtWmUqmBzpKDuKDtdtgBVHOomVMPjIFryLAKZ4ZCgVkJBKA7AhC8EjwG1wWJFxZlmNjD4oujBWdhcZbGV1LQPvfGJnpm1WZtR3fFERJn0x5+UEu66KRDqG7LjqgEQYjdsNC0Ey3eMTs46\/IC3QHBqyuXctlB4uWCy3lnTOzOKcgdADjLDZmTAaLcoO79enr8r+tp\/fa9jQta3hx2ZPvdt4hB0KrTjndMHTsXI6E7IHhd5B0h8ZzAruwUE7sydM4GgjNn0EdnzbKYjG8Im\/bM2ugcWm66yw2EaOXLFfiShW6HtaKzjmxTAsCHI7zcsWHOM94xc9iasAzdAcFrIq8ybSLxMiHyqgMmdq\/okAoGnEnBpNwpXm7w\/j930sKV79Pf39tuTMxyoCxoe37pIOI6WiFyIWaVKSw8EO9YYQitGoDugOC1mnDCGRLPCezKTjGxK0PnbCA4Kwb9s6t23wb2+oZ\/0kv\/2EZr3ttupNRACNePP\/6YTu74OTq5yyF00uc\/E7Ti58WeBqNNIIB3zASq5mxCd0DwmsuuCpaReE5gV3aKiV0ZOmcDwdme0ItVWf6J+PuTr75Hy\/6+1aiYZX+8OntCx1Z0XMeq0mawuJgFX85eFWXH4EwZOicDoTsCFry7du2iTZs20SeffEJt27alvfbaiz766KMgrh9G4jl535WdYmJXhs7ZwKbKmaibfev97bTmvQ\/p+TfNnGogiBWnG\/Cf\/1\/Xg2m\/FntHP8p7Bm1T5cvZC6LBMTjTAKJFE9AdAQpeFrqPP\/44\/fSnP6WNGzcSXypRW1tL++67L\/3gBz+gU045hUaMGOG18EXiWXzLNbjCxK4BRMsmGhNnYhOYKDPo1GZfmvT\/3qJV73xgbGU2vvrKq7N8RNeZiY1gOutnGxNfllPdmTtw5gx6JcfQHQEK3v\/5n\/+hwYMH00knnURHHXUUPfXUU5Hgraqqoh\/\/+Me0YMECGjNmDPXv3z9XUgwfPpzmzZtXGjNlyhTq2bNnpo2FCxfSkCFDSv26d+9O06dPj+Ip15B4mbB61QETu1d0SAUTEmfx0wz+950P6PGV79ErdfXWxOxZR7WJSg74P1dHdYXEl1QCNoFO4CwskqE7AhO8O3bsiMTlv\/71L2JB+swzz9DYsWMjwVtdXU07d+4kFq7vvvtupuiMPzqPWbZsWcmOELG8Ujx06NCyWT158mSaMGFCFAuL4\/Xr11OfPn2oQ4cOFf0j8cKaKDCxh8UXR+uas7SjueYtf4de2\/DPqH72H5s+LNXR6kZXiFZRN\/ut49pFLlyJWZnnc82XTIzo0xABcBZWRkB3BCZ4haC88sor6dJLLyUWpnHBy49zzz330LRp00riNSslV6xYEa0Gjx8\/vsGKLovgurq6ssJVxNK3b98GophjGjlyJM2aNYu6du2a6h6Jl8WKXz\/HxO4XHzLR2OAsfjTXX+vqaf7LG6PQTJwzK545Lma7H9Gajjx4v9RNYDIY+dTHBl8+PW9jiAWchcUidEeggveyyy6jK664IlXw8qrr\/fffT\/fddx+1b99eOSOzBG+a2JZ1hsSTRcqPfpjY\/eAhTxRFOIuXGDyx8j1avnYbrX7X3LFcaWL2uycdQl3a7+f9ymweTir1LcKXrhhgJx8C4CwfXq57Q3cEJnhFScMHH3xAd955Z3RjWXyFd+3atXT55ZfTYYcdFpUZ8EY2lZYsVUizwX0WLVpEN910U+STfXPr3bs3TZw4saJbJJ4KK+7GYGJ3h72q53KcxVdlX3\/nn\/TgS+9ELkyWGLB9cZrB4W1b0her96Pzux5cupFM\/Fz1WRvDOLxj4bEIzsLiDLojMMHL4b700kvEJQ28YY3\/e\/TRR+lHP\/oRrVq1in7\/+99HdbxTp06lr371q7mzMb4BLUu4ik1urVq1KpUv5K3hnT17dnTKhGhch4zmHwKY2P3jRERUt\/Vj6nBA8wbX2D7+2mZ6+e1t9Nrbm2nj9s\/OnDXxFOybG9fLdmnfki7sdgjt1axZ9G95j+YyEV8oNvGOhcLUZ3GCM785Yz0Sb7wo169fv2ihLq47\/H4Kc9E128VnfgXQnnvuuehYstWrVzeIlksYfvnLX9I555xT6Cnq6+tp0KBBUQ2v2BCXNCgEb\/I0ByGaK53yIH7TStrkWuIBAwYUih2D9SPA3yzwEXj8C0nz5rsFDpo9BF58+0M6tFVzYnH7Ut2H9MLaDyPn\/O8mmxCz7PvEDvvSaYe1jNzx\/6\/btvtmMDQ9COAd04OjTSvgzCba+X3NnDkzWoxLNgje3YgEI3g5WNbmfBrD3\/72N+IX79hjj40Eyd577z74vGgTm9n4CLS0kxpY8D755JN7bE4rt5ktHo8QvLxRrmPHjqUfcfxY5S3KnP7x27dvpw0bNkS\/FUPwFse34QkG2+nzbVvSrD\/V0ZLVWyLjLCZZ3Jpq8Y1fndu1pO8c15Za7L1X5I5\/JoSuKf+wuycCeMfCywpw5jdnrEXiq7xLliyhSZMmYYX3U9qCErymUy1L8HINL5dOJE9jyCN48ZuWaRb12MdXd\/lx5EsSWDwuXb2F\/ud\/N9Ga98xv+hKClf887fAqatviY+rV9bDolxSOhYX2mUcemP9hMMI4AnjHjEOs3QE40w6pUYOo4W0Ir\/eCd8uWLXTHHXcQ\/5nV2rVrF5U2HHfccRVXfcsdI5ZVmlBOEONYsixmwvt5U5\/YxUYvZk78ffP2j+ixv75HfIWt6Q1fcSHLtbI1h+5P55\/AJ7CUr5Vt6pyF9paBr9AYc3\/WdXiIuY0Ygjcwwct1lFdffTW98sorxCc1cGNhy43LG9LaWWedRbfeeisdcMABqT8X9br8Q3FDmhCzNTU1FS+QSJY1iNVdvt640kkNSDy3L35e7439w1gcw8W4rN+6g5au2UIr1\/3TupD90qFVdPQh+9Exh+wfUVTkooTGzlneHPa9P\/jynaE94wNnYXEG3RGY4OVwly5dSsOGDYsui\/j+979fusKXT2f47\/\/+b+JSg9tuu42+9KUv0cMPPxwdW8Z9r7322orZmbxaOHnLWrkSBnGEmTCedboD90PihTVRhDixx4\/fYrS3f\/QJLfzbe\/TqevtCloXr0YfsTyd\/fvcvnUWErGzmhMiZ7LM1xn7gKzxWwVlYnEF3BCZ4xWrs0UcfTTfeeCM1+\/T4H\/EYvJHthhtuoL\/\/\/e\/RObwtW7akcePGRQLzwQcf9CY7kXjeUCEViG8Te3xF9i9vb6O\/rfsn\/d1SjawQrFxacPhBLemsIw+kf+0i72778o0zqURrwp3AV3jkg7OwOIPuCEzwipIBXuG9+OKLU7ONb1i7\/fbbS8eJPfDAA3TLLbdEOxN9aUg8X5iQi8PWxB4Xsts+\/JiefO19es3SimxcyPIKLJcVnJRYkY2vzsoh566XLc7cPWHj8gy+wuMTnIXFGXRHYIJ306ZNNHDgQOrSpUt0w1mLFi0aPAGXNYwePZreeOMNuuuuu6hNmzZR\/S5fTvHYY495k51IPG+okApEdWJvePzW7jNjV66vpxVr662vyLLvrx7VJhKxTeHKWlXOpBICnbQjAL60Q2rcIDgzDrFWB9AdgQleDnfatGk0YcIEuuiii6INbOLcWl795drduXPnRvW6V111VVTv++Mf\/zg6o5eFry8NiecLE3JxVJrY+fgtblxS8Oa726MNX9wWv7H73002cV0t\/9nxwM\/R2Ue1idyJGtnk303G4pttfBj7xkjleMBXWHxxtOAsLM6gOwIUvLyKy4L3d7\/7HX3yyScNnoAvneCNbLzhjF\/GK664IrotjUUyn7jgS0Pi+cLEnnGkHcFVt+kDmr\/8bXp\/Z3NrJxdwjSwL19M6t6YvtNuvgZCNC1p\/kXQbGT6M3eKf1zv4youY+\/7gzD0HeSKA7ghQ8IqQ+YgyvumMjxDjxqu4X\/\/610s3l\/EtMLzq26FDB9p3333z5IXxvkg84xBnOhAXI7xSV0+PvfIurX7X\/Hmy8Ru+zjyyTSRoz+jS8CIEiNlM6qQ64MNYCiZvOoEvb6iQDgScSUPlRUfojoAFrxcZpBgEEk8RuBzDxAaw5Wu30cp19dGFCabKDOJC9riOVfTt4\/hShN3NxhFcOWBpMl3xYRwW1eArLL44WnAWFmfQHQEKXj567M0336R58+alXjaxY8cOevvtt6M7o0V9r29picTTw4gQtX9+ays9vvI9I+UGLFgPbdWcqvcnOrXLwXTCYa1LQharsXp4NGEFH8YmUDVnE3yZw9aUZXBmClkzdqE7AhS8fOICXxLBtbxpjU9u4JvOeJMan9LgY0Pi5WeFSxD+tHoz\/c\/rm7St1ArBymUF3zy2HdXv+IRE7Wx8dRYTe36+XI8AZ64ZyOcffOXDy4fe4MwHFuRjgO4ITPD+85\/\/pMGDB0cb0X7729\/S4YcfHm1M69atW3T72pw5c2jGjBnRpRPHH3+8fCZY7onEqww4i9v5f91If327vpC4jQvaUw4\/gI46eH+lEgNM7JZfEA3uwJkGEC2aAF8WwdbkCpxpAtKSGeiOwASvuHiCjyS75pprouh\/\/vOf0zvvvBOVMHDj1V\/epDZ+\/Pg9bmKzlFeZbpB4n0EkyhLGLVitJG6FqOUzZi86uToyzP+ms9wAE3tmSnvXAZx5R0nFgMBXWHxxtOAsLM6gOwIVvD\/60Y\/owgsvjKK\/55576JFHHqE777yTDjjggOj\/7733Xrr77rupXbt2XmZkU048IXAf\/ss7NP3Zt6X5EQK2Z81BdH7Xg5VWaqWdJTpiYldFzt04cOYOexXP4EsFNbdjwJlb\/PN6b8q6Iw2rZrt4R5jHbevWrVEJw0knnUSjRo2KIn3iiSeiVV4WukcccUQkfFns1tbWYtOaR1xymYLsKq4Qtz\/51hfo0Nb7al+xzQsLJva8iLnvD87cc5AnAvCVBy0\/+oIzP3iQjQKCN7AVXg735ptvji6duO6666hv375RPe+ll14a3bz2rW99i66\/\/npi3S6uFpZNBpv9mkLiiZXc2hfX0z1L1lWEV9wYNqpXZ+fiNi1QTOw23w49vsCZHhxtWQFftpDW5wec6cPShqWmoDvy4Oj9Ci8\/DK\/ycv0u\/8mi9sADD6SxY8dGm9VY6DZr1oxuuOEG6t+\/f55nt9q3sSfenBfWR6u58VvLkgCzyL2u5xEkLmCwSkBOZ5jYcwLmQXdw5gEJOUIAXznA8qQrOPOECMkwGrvukISh1C0IwcvRsrDdtm0btWrVKhK4fMXwc889R8888wz16tUrKnngf\/e1NcbEY3H7\/Jub6arZK8vCziL3tr41Xq7iVsoVTOy+vknl4wJnYXEGvsLii6MFZ2Fx1hh1RxEGghG8lR6Sxe+WLVuodevWtPfeexfBw9jYxpZ4U59ZSz9+8H9T8RIi98wjG16hawxcA4YxsRsA1bBJcGYYYM3mwZdmQC2YA2cWQNboorHpjqLQeC94xbFkY8aMoZ49e6Y+7\/z582ncuHHYtFY0GzLGi6t6h81JX9G95NRqEjW5hkMxbh4Tu3GItTsAZ9ohNWoQfBmF14hxcGYEVmNGIXgbQuul4OWrgrlc4YMPPqDNmzfTLbfcQhdccAF17dp1j8TgvlzXy33vu+8+at++vbHkKWI49MRjsXv+5Jf2qNHl1dxvH9+eBp\/VSes5uEWw1jEWE7sOFO3aAGd28S7qDXwVRdD+eHBmH\/MiHkPXHUWePW2sl4KXA501axbdeOONUe1uVuMyhpEjR0bHl\/laxxtq4rHQfWLlezTi96\/vQcOA0zvQ\/+lxeKMSuuIhMbFnvXX+\/Ryc+cdJpYjAV1h8cbTgLCzOQtUdplD2VvDu3LkzqsvdsGEDDRkyhP7jP\/6DzjrrrD1waNGiBbVp08ZboSsCDjHxKq3qPjy0W6MUuhC8pqYa83bxYWweY50ewJdONO3YAmd2cNblJUTdoevZg1rhFcHyhrT3338\/ulGNrw8OtYWWeHzMWLJWtzFsRpPNH0zsskj50w+c+cOFTCTgSwYlv\/qAM7\/4yIomNN2R9TxFf+7lCq8QufynbOOyhrZt2+KUBlnAKvSbvXQdXT331QY9WOw29lXd+ANjYteQSJZNgDPLgBd0B74KAuhgODhzAHoBlxC8DcHzUvCKkxnWrl0rTXWnTp1wSoM0WukduYTh4RXv0M8eeaNBh9v6fpH6dT+0oPWwhmNiD4svjhachcUZ+AqLL7xj4fEFwRuA4N2+fTstXryY+AQG2cblDmeccQa1bNlSdojVfr4nHovdOS+so3EL1pRw4VVdPmaMjxtrag0fxuExDs7C4gx8hcUXBG94fPmuO2wj6uUKr20QbPjzOfFY7F49dyU9u2pzA7HblEoYkjmAD2Mbb4VeH+BML56mrYEv0wjrtw\/O9GNq0qLPusPkc5ezHZTg3bRpEz377LP04osvEp\/O0K1bN\/ryl78cndKg0oYPH07z5s0rDZ0yZUrZyy3K2Z88eTLNnTs3s5zC18RjsTv3hfX0fxeshtiNkYyJXeWNcjsGnLnFP6938JUXMff9wZl7DvJE4KvuyPMMOvsGIXh589q0adNo4sSJlNzIxpvVhg0bRkOHDo1EsGxjsbts2bKSUF24cGF0\/NmIESMiWzJtxYoV1L9\/\/+hK49raWqquLv\/Vv6+Jd+eza2nUA59dEdzUNqeV4xkTu8wb4FcfcOYXH1nRgK8shPz7OTjzj5NKEfmqO1yhGITgvf\/++2nUqFF09tln07XXXktHHHFEhNeaNWvo5ptvjm5lGz9+PJ1\/\/vlSOAqhymPi1xWzCK6rq6Pp06dTVVVVRVv19fU0aNAgWrp0KclsmPMx8cYtWL1HzW5TLmOIE46JXepV8qoTOPOKjsxgwFcmRN51AGfeUVIxIB91h0sEvRe8QliyAL311lv32JTGG9yuueaaaOWXSxKKnNWbR\/ByKcOiRYuisor58+cHucLbdvhTpdzDym7D1xATu8tpSc03OFPDzdUo8OUKeXW\/4EwdOxcjIXgbou694BVHlF155ZV06aWXpubMPffcE5U8ZJUVVEo4FrATJkyIRHN81TdtDJc\/8FXGfP0xnyYRWg0v1+2eOPZ5iN0KCYGJ3cX0XMwnOCuGn+3R4Ms24sX9gbPiGNq0AMEbmODduHEjXXzxxXTBBRdEK7lpjVd+H3zwQbrvvvuoffv2ufJJ1O7yoN69e0d1wpWaEOB9+\/aNan3zblqbPXt2VAIhWqW631wPkqPzBVNfpsVvfHYiw6SLjqRLTm1a5+xmwYWJPQsh\/34OzvzjpFJE4CssvjhacOY3Z6xP4o3vMujXr1\/0bXRcd\/j9FOai836F96OPPiIuNVi5ciXdeeed1Llz5wZorF69mq644gqqqamJxOo+++yjhJYoneAa3korxcmyh7yCNxkcb3obMGCAUswqgx76Wz394sl3S0MvP7k1XfMVtVMuVPyHMobPgOZftvgXkubNm4cSdpOOE5yFRT\/4Cosvjhac+c3ZzJkzo2+ekw2Cdzci3gteDvLll1+mgQMHRi\/bN77xjeiCCW5cTvD4449Hdbtc0sD1tEWa2Mw2ePDg1JMa4qUMXbt2jVzlFby8Ua5jx46lMFlQ2Vrl5VKGL094qeSb63b\/NKIYZkXw9nks14Zv2LAh+q0Ygtdnpj6LDZyFwZOIEnyFxRdHC8785oxXeOOrvEuWLKFJkyZhhfdT2oIQvBzra6+9RqNHj6bly5fTrl27dqv1Zs3oxBNPpJtuuomOOeaYwpmYJXiT5\/YmHVY60sx1LQ2L3WFzVpZKGbBJrXK64Ku7wq+TdQPgzDrkhRyCr0LwORkMzpzAruzUte5QDtzQwGAEr3h+XuXlCyi48YUTKqcypK3Usj1RzyuzcU3Ek3eF19VXC8kjyG6\/pKZJXhks+x5hYpdFyp9+4MwfLmQiAV8yKPnVB5z5xUdWNBC8DRHyXvByHeVdd90VnbF79NFHE180UbSJel22I87cFau7XAsscw5vSII37VSG5WNOLwpjox6PiT08esFZWJyBr7D44mjBWVicQfAGKHj5lAa+ZOKggw6KhO\/ll18e1VZySUORlixRSJYk8Ort1KlToyJwUbOb9BfCCu95t7\/UoJThtr41dOaRBxaBrtGPxcQeHsXgLCzOwFdYfEHwhscXBG9ggpfD3blzJz3\/\/PM0Y8aM6E\/+fz6tQRxXlvcoMhdp6yrxnl21mc6f\/NlGtYkXHUOXn97BBQRB+cSHcVB0RcGCs7A4A19h8YV3LDy+XOkOX5HyvqQhCVxS\/PKxZVyGwKc4fOc736EWLVp4ibWrxEuu7qKUQS498GEsh5NPvcCZT2xkxwK+sjHyrQc4842RyvG40h2+ohSc4I0DuXXrVvrd734Xrfy2atWq0E1rpglykXhzXlgfncwg2sNDu6GUQZJoTOySQHnUDZx5RIZEKOBLAiTPuoAzzwjJCMeF7vAZoeAEL284e+qppyJxy2RyO+GEE6IVXj6jFyu8u9ONN6pxKQP\/yY2PIcPqrvyriIldHitfeoIzX5iQiwN8yeHkUy9w5hMb2bFA8DbEKAjBmxS5n3zySVTD+\/3vf5++\/e1vR8eT+d5sJ15ydZfFLoteNDkEMLHL4eRTL3DmExvZsYCvbIx86wHOfGOkcjy2dYfv6HgvePnWkD59+hDfCc2b0y677DK64IILGtxW5jvIHJ\/NxEu7ZAKru\/myBBN7Prx86A3OfGBBPgbwJY+VLz3BmS9MyMVhU3fIReS2l\/eCly+ZmDNnDvXq1Yu+8IUvFD6KzBXcNhMPq7vFWcbEXhxD2xbAmW3Ei\/kDX8XwczEanLlAXd2nTd2hHqW9kd4LXntQmPVkM\/FwMkNxLjGxF8fQtgVwZhvxYv7AVzH8XIwGZy5QV\/dpU3eoR2lvJASvJaxtJV7yVjVcIaxGMCZ2NdxcjgJnLtHP7xt85cfM9Qhw5pqBfP5t6Y58UbnrDcFrCXtbiYfVXT2EYmLXg6NNK+DMJtrFfYGv4hjatgDObCNezJ8t3VEsSnujIXgtYW0j8ZKru6N6HUGjenW29ISNyw0m9vD4BGdhcQa+wuKLowVnYXFmQ3eEhAgEryW2bCTerD\/V0Y9qX4ueCOfuFiMWE3sx\/FyMBmcuUFf3Cb7UsXM1Epy5Ql7Nrw3doRaZm1HeC94tW7bQHXfcEV0bfNxxx6Wi9OKLL9Itt9xCEydOjI4u87GZTrzk6u4PzuhI4\/\/9aB+hCCImTOxB0NQgSHAWFmfgKyy+sMIbHl+mdUdoiHgveMU5vGPGjKGePXvugS9fQjFhwgSaP39+k75a+NlVm6Ob1UTDNcLFXkV8GBfDz8VocOYCdXWf4EsdO1cjwZkr5NX8QvA2xM1Lwbtz504aPXo0PfDAA9Is\/9u\/\/Rvdeuut1LJlS+kxNjuaTjxsVtPLJiZ2vXjasAbObKCszwf40oelLUvgzBbSevyY1h16orRnxUvBy4+\/evVqmjlzJvHFE0899RSdeOKJqbertWrVirp160Znnnkm8d99bSYTD5vV9LOOiV0\/pqYtgjPTCOu1D7704mnDGjizgbI+HyZ1h74o7VnyVvAKCGRqeO3Bpe7JZOKNW7Caxi1YEwWHzWrqHMVHYmLXg6NNK+DMJtrFfYGv4hjatgDObCNezJ9J3VEsMjejvRe8bmDR79Vk4sXLGc7ociA9Mqyb\/gdoYhYxsYdHODgLizPwFRZfHC04C4szk7ojLCR2R+ul4BWruhzg9773Pbr33nuJ\/61Sa926NV111VXEf\/rYTCVespwBm9X0sI+JXQ+ONq2AM5toF\/cFvopjaNsCOLONeDF\/pnRHsajcjfZS8EuMX7MAACAASURBVIqTGRgW3oh2zTXX0Nq1ayui1KlTpyZ5SgPKGcy8PJjYzeBq0io4M4muftvgSz+mpi2CM9MI67UPwdsQTy8Fr17K\/bBmKvFQzmCGX0zsZnA1aRWcmURXv23wpR9T0xbBmWmE9do3pTv0RmnPGgSvJaxNJV7b4U+VngDlDPrIxMSuD0tblsCZLaT1+AFfenC0aQWc2US7uC9TuqN4ZG4seC94RT0vanj3TBCUM5h7aTCxm8PWlGVwZgpZM3bBlxlcTVoFZybR1W8bgrchpt4LXlHPixrePV8GlDPonyCERUzs5rA1ZRmcmULWjF3wZQZXk1bBmUl09duG4A1M8JZLgV27dtG7775Lc+fOpYceeohuueUWOu644\/RnjCaLuhMveTrD8jGnR2fwoulBABO7HhxtWgFnNtEu7gt8FcfQtgVwZhvxYv50645i0bgf7f0KbxZELHxvuOGG6Ea2iRMn0j777JM1xMnPdSfenBfW07A5K6NnwWUT+inFxK4fU9MWwZlphPXaB1968bRhDZzZQFmfD926Q19kbiwFL3gZtvvuu49uv\/323MeSDR8+nObNm1dCfsqUKdSzZ8+KTNTX19OgQYNo6dKlpX4jRoygoUOHVhynO\/FQzmD2hcHEbhZfE9bBmQlUzdkEX+awNWUZnJlC1oxd3brDTJT2rAYveHfu3EmjR4+mV155he6++25q166dFHosdpctW1YSyQsXLqQhQ4ZQJfEq6ok7dOhA06dPp6qqKlqxYgX179+fevToEa0wl2s6Ey9ZzjCq1xE0qldnqedGJzkEMLHL4eRTL3DmExvZsYCvbIx86wHOfGOkcjw6dUdYT54erfeCN+uUhjfffDMSriw6f\/azn1GzZs0yeREidfz48Q1WdFkE19XVlcRs0hCL4pEjR9KsWbOoa9eupR9Pnjw5qiWura2l6urqVP86E+\/ZVZvp\/MkvlfzgOLJMynN3wMSeGzLnA8CZcwpyBQC+csHlRWdw5gUN0kHo1B3STj3u6L3gzTqloUWLFnThhRfS9ddfTwcccEAhqLMEbznjLHinTp26hxCO99eZeP1n\/JX+8JeNkXnU7xaivOxgTOxmcDVpFZyZRFe\/bfClH1PTFsGZaYT12tepO\/RG5saa94LXFiwsWidMmEAydbzJmJLlEWkxi8SbPXs28TXIopVbEa703Kf83xeIyxq4XdujE8oZDCQJJnYDoBo2Cc4MA6zZPPjSDKgFc+DMAsgFXPACYbzxca79+vWjRYsWNdAdBVwEPTRYwcunM2zbto1atWolVcZQjiVRu8s\/7927d8U63DQbMrW\/PE4I3qQNLsUYMGCAdBLVbf2Yzpu5ttR\/2oXVdHJHHEcmDaBkxx07dtDGjRujEpXmzZtLjkI3lwiAM5fo5\/cNvvJj5noEOHPNQGX\/M2fOjL5pTjYI3t2IBCF4eWPab37zm2hjGq\/E8mYxPoZs4MCBUc3tT37yEzrvvPMKCV9x+gLbq1SLG08kUQtcU1NTtu5X9BeCl+uGO3bsWDLDgirPKu\/NT\/6Dbn5yt+DlcoY\/jejm9xsYaHTbt2+nDRs2RL8VQ\/CGQSI4C4MnESX4Cosvjhac+c0Zr\/DGV3mXLFlCkyZNwgrvp7QFIXinTZsWlRtwre6YMWMiwcsv3oIFCyKhuWrVqmhl9txzzy2UjULADh48OPOYsTxil4PSVUvDm9V40xq3M7ocSI8Mg+AtRHqZwfjqzgSqZm2CM7P46rYOvnQjat4eODOPsU4PunSHzphc2vJe8IqV3KOOOop++ctf7nGxBK\/+cg0t37omjgpTBVRW8Ioyhu7du0v71JF4OI5Mldn84zCx58fM9Qhw5pqBfP7BVz68fOgNznxgQT4GHbpD3pv\/Pb0XvOKUhmHDhtHFF1+ciugDDzwQXS0sW4pQ7ngxIWQrbVwTffLW++pIPBxHZu+FwsRuD2tdnsCZLiTt2AFfdnDW6QWc6UTTvC0dusN8lPY8eC94eeX2sssuo6997Ws0atSoVGTGjRtHTz\/9tPTFE6Jel40lL5CoVI8re8lEWpA6Em\/cgtU0bsGayDyOIzP7kmBiN4uvCevgzASq5myCL3PYmrIMzkwha8auDt1hJjI3Vr0XvHwawy9+8Qv6wx\/+QL\/+9a8j4Ssul+CfsdC97rrr6Dvf+Y70xRMC6uTVwslb1pLn6yb7JymrtDKsI\/FwnbC9lwQTuz2sdXkCZ7qQtGMHfNnBWacXcKYTTfO2dOgO81Ha8+C94GUo+Cy5K6+8kl599dXocgk+iowbH5HCK8Bf\/OIXiTe2xc+3tQehnKeiiYf6XTmcdfXCxK4LSXt2wJk9rHV4Al86ULRrA5zZxbuot6K6o6h\/38YHIXgZND6VgWt0+b+tW7dGOLL47dOnT\/Rfy5YtfcO2QTxFEy8peHGdsFm6MbGbxdeEdXBmAlVzNsGXOWxNWQZnppA1Y7eo7jATlTur3gteLlt46KGH6NhjjyU+qSHUVjTxUL9rl3lM7Hbx1uENnOlA0Z4N8GUPa12ewJkuJO3YKao77ERpz4v3gpdvu+LTGb773e9mno1rD7b8noomHup382NeZAQm9iLouRkLztzgruoVfKki524cOHOHvYrnorpDxafPY7wXvOKUBr5JbejQoT5jWTG2ool34tjnicsauN1+SQ1dcmp1sFiEEDgm9hBYahgjOAuLM\/AVFl8cLTgLi7OiuiOsp82O1nvBy4\/wxBNP0M9\/\/vNopffb3\/427bfffns82d57701t27Yl\/tPHViTxUL9rn1FM7PYxL+oRnBVF0O548GUXbx3ewJkOFO3ZKKI77EVpz5P3gldcPMEnNVRqfEKD7MUT9uD9zFORxEteOPH+xK+7eIQm5RMTe3h0g7OwOANfYfGFFd7w+CqiO8J72uyIvRe8fDrD4sWLoyPIKrV9992XzjjjDG9PayiSeNiwlp3Iunvgw1g3oubtgTPzGOv0AL50omnHFjizg7MuL0V0h64YfLLjveD1CawisRRJvPiGNa7d5RpeNLMIYGI3i68J6+DMBKrmbIIvc9iasgzOTCFrxm4R3WEmIrdWIXgt4a+aeMn63eVjTo+uFUYziwAmdrP4mrAOzkygas4m+DKHrSnL4MwUsmbsquoOM9G4t+ql4BV1uwzPrbfeStdcc01021ql1lhreJP1u7hwws5Lg4ndDs46vYAznWiatwW+zGOs2wM4042oWXsQvA3x9VLwbtmyhe64444o0u9973t07733Ev9bpda6dWu66qqriP\/0sakmXlzw8sour\/CimUcAE7t5jHV7AGe6ETVrD3yZxdeEdXBmAlVzNlV1h7mI3Fr2UvC6hcSMd9XEw4UTZvjIsoqJPQsh\/34OzvzjpFJE4CssvjhacBYWZ6q6I6ynlI82CMHL1ws\/9thjtGjRIvrpT39K+++\/P23dujW6iGKfffah66+\/no455hj5p3bQUzXx4oJ3VK8jaFSvzg6ib3ouMbGHxzk4C4sz8BUWXxC84fGlqjvCe1K5iIMQvI8++igNHz6cunbtSlOmTKE2bdpEJQ6\/\/vWv6ZFHHiE+kmzatGnUrVs3uad20Esl8XDhhAOiPnWJD2N32Kt6BmeqyLkZB77c4F7EKzgrgp79sSq6w36U9jx6L3jr6+tp0KBB0fm6vIGtqqqqATqbNm2iIUOGRLevsRhm8etjU0k8bFhzxyQmdnfYq3oGZ6rIuRkHvtzgXsQrOCuCnv2xKrrDfpT2PHoveMWJDVdeeSVdeumlqcjcc8890QpvY7tpbc4L62nYnJXRM2PDmr2Xgj1hYreLtw5v4EwHivZsgC97WOvyBM50IWnHDgRvQ5y9F7wbN26kiy++mC644ILoeLK0NnnyZLr\/\/vvpvvvuo\/bt29vJpJxeVBIPG9ZygqyxOyZ2jWBaMgXOLAGtyQ340gSkRTPgzCLYGlyp6A4Nbr014b3g\/eijj6L63ZUrV9Kdd95JnTs33LS1evVquuKKK6impoYmTpwYbWLzsakkHjasuWMSE7s77FU9gzNV5NyMA19ucC\/iFZwVQc\/+WBXdYT9Kex69F7wMxcsvv0wDBw6kbdu20UknnUSHH354hNC7775LzzzzDLVq1YruuusuOv744+0hl9OTSuK1Hf5UyQsunMgJeMHumNgLAuhgODhzAHoBl+CrAHiOhoIzR8ArulXRHYqughgWhOBlJN9++2361a9+RU888QTt3LkzArdFixZ0zjnn0I9\/\/GPq2LGj14DnTbzkhjVcKWyXXkzsdvHW4Q2c6UDRng3wZQ9rXZ7AmS4k7djJqzvsROXOSzCC1x1EejznTTxsWNODu6oVTOyqyLkbB87cYa\/iGXypoOZ2DDhzi39e73l1R177ofWH4LXEWN7Eq31xPQ25Fyc0WKJnDzeY2F0hr+4XnKlj52Ik+HKBejGf4KwYfrZH59UdtuOz7Q+C1xLieRMvvmHtW8e2o3t\/4G99siUIrbrBxG4Vbi3OwJkWGK0ZAV\/WoNbmCJxpg9KKoby6w0pQDp1A8FoCP2\/ixTes4UphSyTF3GBit495UY\/grCiCdseDL7t46\/AGznSgaM9GXt1hLzI3niB4LeGeN\/FwQoMlYsq4wcTuFn8V7+BMBTV3Y8CXO+xVPYMzVeTcjMurO9xEac8rBK8C1nwu8Lx580oj+Urjnj17VrSUJ\/FwQoMCKZqHYGLXDKgFc+DMAsgaXYAvjWBaMgXOLAGtyU0e3aHJpddmIHhz0sNid9myZaVrjBcuXEhDhgyhESNG0NChQ8tay5N44xaspnEL1kS2cKVwToI0dcfErglIi2bAmUWwNbgCXxpAtGwCnFkGvKC7PLqjoKsghgcjePlGtT\/+8Y\/01ltvpQLbunVruuqqq4j\/NNVWrFhB\/fv3p\/HjxzdY0WURXFdXR9OnT6eqqqpU93kSD1cKm2JQ3i4mdnmsfOkJznxhQi4O8CWHk0+9wJlPbGTHkkd3ZFsLv0cQgvfRRx+NrhcWF06kwd6pU6fSqqttWkwK3v\/qfyxdcOLBth+pyfvDxB5eCoCzsDgDX2HxxdGCs7A4g+BtyJf3gre+vp4GDRpE77zzDv32t7+lY489lpo1a+ZN1k2ePJkmTJhAWXW8IvFmz55NLM5Fq66u3uNZDr7umdK\/TbroSLrk1EO9ed6mEggm9vCYBmdhcQa+wuILgtd\/vtavX98gyLVr11K\/fv1o0aJFDXSH\/09iJkLvBS8T2KdPH7rsssvoiiuuMIOCglVRu8tDe\/fuTRMnTqxoRQjeZCcukRgwYEDpn+u2fkznzVxb+v9pF1bTyR0\/pxAhhhRBYMeOHbRx40biX0iaN29exBTGWkIAnFkCWpMb8KUJSItmwJlFsBVczZw5k2bNmrXHSAje3ZB4L3g3bdpEAwcOpHPPPdcrwSsySqxAcw1vbW1tJJDSmhC8XP\/bsWPHUhfuHx\/z7KpN1OeuVz8TwDedrpD2GFIUge3bt9OGDRui34oheIuiaWc8OLODsy4v4EsXkvbsgDN7WKt44gXC+CrvkiVLaNKkSVjh\/RRM7wUvxzlt2jR67LHHoj\/bt2+vkgdGx4jNbIMHDy57UoNsLQ1OaDBKlbRxfN0qDZU3HcGZN1RIBQK+pGDyqhM484qOzGBkdUemoUbSwXvBy79RPv300\/S73\/2OXn31VTr77LOpVatWe8Bv45SGcpzrFLzD5qykOS\/srsM5o8uB9Miwbo0k1cJ6DEzsYfHF0YKzsDgDX2HxhXcsPL4geBty5r3gFTW8XHxdqdk4pYHrdkeOHBnVyHTt2rUUjqjnrbRxTTbx4keS4UphdxMMPozdYa\/qGZypIudmHPhyg3sRr+CsCHr2x8rqDvuRufHoveB1A0u6V1Gvyz8VZ+6K1d2amhot5\/DGrxSG4HXHPiZ2d9iregZnqsi5GQe+3OBexCs4K4Ke\/bEQvIGt8NpPkWyPyauFs25ZY4syiffW+x\/SiWOfLwXw8NBudOaRB2YHhB7aEcDErh1S4wbBmXGItToAX1rhtGIMnFmBWZsTGd2hzVkAhoJZ4X3jjTdo7Nix9Pzzz9PBBx8cnYiw33770fXXX0\/f\/OY36bzzzvPqfN4k9zKJ9+yqzXT+5JdKQ5ePOT26WhjNPgKY2O1jXtQjOCuKoN3x4Msu3jq8gTMdKNqzIaM77EXj3lMQgvfll1+Ojibba6+96IgjjqB169ZFgnffffeN\/n3lypXRpRQ9e\/Z0j2iZCGQSjzer8aY1bix0WfCiuUEAE7sb3It4BWdF0LM\/FnzZx7yoR3BWFEG742V0h92I3HrzXvB+9NFH0bXCvGmNjyV76aWXopVecebt1q1bo\/N5ebWXN42xCPaxySQejiTzhzlM7P5wIRsJOJNFyo9+4MsPHvJEAc7yoOW+r4zucB+lvQi8F7x829XFF19M3\/3ud6MzbvlEhLjgZajuvPNOuvvuuyte\/GAP0nRPMokXP6EBR5K5ZQwTu1v8VbyDMxXU3I0BX+6wV\/UMzlSRczNORne4icyNV+8FrziW7Morr6RLL700VfDec8890epvpZvO3MD7mVeZxMORZK5Z+sw\/JnZ\/uJCNBJzJIuVHP\/DlBw95ogBnedBy31dGd7iP0l4E3gtecRRYu3btaOLEidElFGklDfvssw9NnTqV9t9\/f3vo5fCUlXjJExpuv6SGLjk1\/ZriHG7RVREBTOyKwDkcBs4cgq\/gGnwpgOZ4CDhzTEBO91m6I6e54Lt7L3gZ4UcffZT46C9e4T388MPpjjvuiOp1ua73tttuizatjRs3Lip78LVlJR6OJPOLOUzsfvEhEw04k0HJnz7gyx8uZCMBZ7JI+dEvS3f4EaW9KIIQvLt27YpuNxs\/fjx98MEHDdDZe++9o01tXPLAf\/e1ZSUejiTzizlM7H7xIRMNOJNByZ8+4MsfLmQjAWeySPnRL0t3+BGlvSiCELwCDj6RYenSpdF\/O3fupJNPPpnOPPNMatOmjT3EFD1lJR6OJFME1tAwTOyGgDVoFpwZBNeAafBlAFTDJsGZYYA1m8\/SHZrdeW8uKMHrPZoVAsxKvPiRZDihwT3TmNjdc5A3AnCWFzG3\/cGXW\/xVvIMzFdTcjcnSHe4ic+M5GMHLZQ2vv\/46\/f73v6dt27ZFaPFGtgsvvJA6d+7sBr0cXrMSD0eS5QDTQldM7BZA1uwCnGkG1LA58GUYYAPmwZkBUA2azNIdBl17aToIwculDD\/5yU+izWssfOOtWbNm1LdvX7rhhhuoRYsWXoLMQWUl3oljnyfeuMZtVK8jaFQv\/0W8t2BrCAwTuwYQLZsAZ5YBL+gOfBUE0MFwcOYA9AIus3RHAdNBDvVe8LLA5c1qd911Fw0bNowGDBhABxxwQAQ2C2G+dILP4L322mujjWu+tqzEazv8qVLoDw\/tRmceeaCvj9Ik4sLEHh7N4CwszsBXWHxxtOAsLM6ydEdYT1M8Wu8F77vvvkuXXXYZnXrqqXTjjTcSr+jGGwtiXt19+eWXI1Hs6wa2SomHI8mKJ7JuC5jYdSNq3h44M4+xTg\/gSyeadmyBMzs46\/ICwdsQSe8Fr7hpjVd3+YrhtPbII49Eq8Ch3rSGI8l0vd767GBi14elLUvgzBbSevyALz042rQCzmyiXdwXBG9ggnfHjh10zTXXRPW5fNNask6XjycbPXp0VN5w66230r777ls8SwxYqJR4OJLMAOAFTWJiLwigg+HgzAHoBVyCrwLgORoKzhwBr+gWgjcwwcvh8o1qXJ972GGH0XXXXUdHHHEE7bXXXsSrv3zT2sKFC+lXv\/oVHXfccaWn40so2rdvr5gm+odVSrx7l66ja+a+Gjn9fNvP0fIxp+sPABZzIYCJPRdcXnQGZ17QIB0E+JKGypuO4MwbKqQCgeANTPCKkgYWvXlap06daNGiRXmGGO1bKfFwJJlR6JWMY2JXgs3pIHDmFP7czsFXbsicDwBnzinIFQAEb2CCd\/v27bR48WLi0oY8jUsbzjnnnDxDjPaVFbw4kswoDdLGMbFLQ+VNR3DmDRVSgYAvKZi86gTOvKIjMxgI3sAEbyajgXSolHg4g9c\/EjGx+8dJVkTgLAshv34OvvziQyYacCaDkj99IHgDFrybNm2iZ599ll588cVo81q3bt3oy1\/+srdHkcWhLpd4OJLMn8khHgkmdj95qRQVOAuLM\/AVFl8cLTgLizMI3gAF7yeffBJdLsGnNPDf4403p\/GRZUOHDg3ypjUIXj8nEEzsfvICwRseL+UixjsWHpfgLCzOIHgDFLz3338\/jRo1is4+++zoRjU+pYHbmjVr6Oabb6bnnnsuOof3\/PPP9zYbyyVe8gze9yd+3dtnaEqBYWIPj21wFhZn4CssvrDCGx5fELyBCd76+noaNGgQVVVVRefstmzZssET8KY2PqeXV36nTJkS3Dm84xaspnEL1kTPhCPJ\/JlQ8GHsDxeykYAzWaT86Ae+\/OAhTxTgLA9a7vtC8AYmeMWxZHwO76WXXpqaQffcc09U8hDiTWuz\/lRHP6p9LXquM7ocSI8M6+b+LUEEqFULMAfwYRwWaeArLL6wwhseXxC8gQnejRs3RlcKX3DBBdFKblrjld8HH3yQ7rvvPunLJsTK8dKlS0smR4wYEdUCZ7XJkyfThAkTco0rl3jxM3i\/dnQbemDIiVnu8XMLCODD2ALIml2AM82AGjYHvgwDbMA8ODMAqkGTELyBCd6PPvqIhg8fTitXrqQ777yTOnfu3OAJVq9eTVdccQXV1NREm9r22WefzPQRq8YdOnSg6dOnR+USK1asoP79+1OPHj0iO+Uai92pU6fSrFmzqGvXrqVxgwcPriiWZQQvzuDNpM5aB0zs1qDW5gicaYPSiiHwZQVmrU7AmVY4jRuD4A1M8HK4L7\/8Mg0cODC6fOIb3\/gGnXHGGdFT8IUUjz\/+eFS3yyUNfEyZTOOriEeOHFkSrWIMi9m5c+eWLY0QQvmUU05pIIpZkC9btqxiSUW5xGs7\/KlSyBC8MuzZ6YOJ3Q7OOr2AM51omrcFvsxjrNsDONONqFl7ELwBCl4O+bXXXqPRo0fT8uXLadeuXdFTNGvWjE488US66aab6JhjjimcOcnV26RB3YIXR5IVpsyYAUzsxqA1ZhicGYPWiGHwZQRWo0bBmVF4tRuH4A1U8IqweZWXL6Dg1qZNG62nMsis1JYracgqhUhLPAhe7e+3NoOY2LVBac0QOLMGtRZH4EsLjFaNgDOrcBd2BsEbuOAtnAFlDHCZw5AhQ0hm45qo9922bVtkjY9D69mzZ8XQROLNnj2bOnXqFPVd82EVnT\/5pdK4d359lqnHg92cCGBizwmYB93BmQck5AgBfOUAy5Ou4MwTIsqEwd9Cx9vatWupX79+tGjRopLu8PsJzEbXbJeoDzDrx2vrQsDyxjexiS0tYHGyQ11dXaleN20DXNpYIXjjP9v5+TPog5MGRv\/U4YDm9MiA3UIYzT0C\/E0CnxBSXV1NzZs3dx8QIshEAJxlQuRVB\/DlFR1SwYAzKZicdZo5c2a0NynZIHh3I9LkBa+s2GWwxCpwckVX2ODb3sqt9ArBy306duwYgX\/vKx\/Tva98FP2dL5340wi5TXfO3qYm5JgvNNmwYUP0WzEEbxjEg7MweBJRgq+w+OJowZnfnPECXHyVd8mSJTRp0iSs8H5KW5MWvELAdu\/eveLKrkjxcqc7iFXevn37lj2aLK2WZsTvX6e7Fr8dmcelE35NJPjqzi8+ZKIBZzIo+dMHfPnDhWwk4EwWKT\/6oYa3IQ9NVvAKsdu7d++K5+7G4dKxwhv\/aiF+6cQlp1bT7ZfU+PGWIArctBZgDuDDOCzSwFdYfHG04CwsziB4IXilL5lIpraOGt644D1x7PPEJzVwwxm8fk0kmNj94kMmGnAmg5I\/fcCXP1zIRgLOZJHyox8ELwRvdHPbvHnzymakqNEtdy5vcrzMKnFa4sUvneDVXV7lRfMDAUzsfvCQJwpwlgct933Bl3sO8kYAzvIi5rY\/BC8Er5MMTCYezuB1QoO0U0zs0lB50xGceUOFVCDgSwomrzqBM6\/oyAwGgheCNzNJTHTIErzLx5wendSA5gcCmNj94CFPFOAsD1ru+4Iv9xzkjQCc5UXMbX8IXgheJxmYTLxnV21ucOnE+xO\/7iQuOE1HABN7eJkBzsLiDHyFxRdHC87C4gyCF4LXScYmE2\/cgtU0bsGaKBZe2eUVXjR\/EMDE7g8XspGAM1mk\/OgHvvzgIU8U4CwPWu77QvBC8DrJQgheJ7ArO8XErgyds4HgzBn0So7BlxJsTgeBM6fw53YOwQvBmztpdAxIJl78DF5cOqEDYb02MLHrxdOGNXBmA2V9PsCXPixtWQJntpDW4weCF4JXTybltFJJ8OLSiZxgWuiOid0CyJpdgDPNgBo2B74MA2zAPDgzAKpBkxC8ELwG06u86WTi4dIJJzRIO8XELg2VNx3BmTdUSAUCvqRg8qoTOPOKjsxgIHgheDOTxESHZOLh0gkTKOuziYldH5a2LIEzW0jr8QO+9OBo0wo4s4l2cV8QvBC8xbNIwUI88f61XzviFV7RHh7ajc488kAFqxhiCgFM7KaQNWcXnJnD1oRl8GUCVbM2wZlZfHVbh+CF4NWdU1L2IHilYPKmEyZ2b6iQDgScSUPlRUfw5QUNuYIAZ7ngct4ZgheC10kSVhK8uHTCCSUVnWJi94+TrIjAWRZCfv0cfPnFh0w04EwGJX\/6QPBC8DrJxnjiPbOuOQ2bszKKA5dOOKEj0ykm9kyIvOsAzryjBL9UhkVJZrR4xzIh8qoDBC8Er5OEjCfeva98hFvWnLAg7xQTuzxWvvQEZ74wIRcH+JLDyade4MwnNrJjgeCF4M3OEgM94on3q2e20ZwX1kdecOmEAbA1mMTErgFEyybAmWXAC7oDXwUBdDAcnDkAvYBLCF4I7n4TKAAAHRRJREFU3gLpoz40nnhXPbSRFr+xGYJXHU7jIzGxG4dYuwNwph1SowbBl1F4jRgHZ0ZgNWYUgheC11hyVTJcTvCO6nUEjerV2UlMcFoeAUzs4WUHOAuLM\/AVFl8cLTgLizMIXgheJxkbT7zvzPgHvfX+h1Ect19SQ3y1MJpfCGBi94sPmWjAmQxK\/vQBX\/5wIRsJOJNFyo9+ELwQvE4yMZ54J0z831IMuHTCCR2ZTjGxZ0LkXQdw5h0lFQMCX2HxhRXe8PiC4IXgdZK1IvHumbeQeIVXNAheJ3RkOsWHcSZE3nUAZ95RAsEbFiWZ0eIdy4TIqw4QvBC8ThJSJN6vfvcw8aY10ZaPOT06ixfNLwQwsfvFh0w04EwGJX\/6gC9\/uJCNBJzJIuVHPwheCF4nmVhO8OKWNSd0ZDrFxJ4JkXcdwJl3lGCFNyxKMqPFO5YJkVcdIHgheJ0kZFpJA25Zc0KFlFNM7FIwedUJnHlFR2Yw4CsTIu86gDPvKKkYEAQvBK+TjBWJ9+2fzCK+aY0bBK8TKqScYmKXgsmrTuDMKzoygwFfmRB51wGceUcJBG8OSprt2rVrV47+6KqIgBC8x181jZ5Zt3dkBbesKYJpYRgmdgsga3YBzjQDatgc+DIMsAHz4MwAqAZNYoUXK7wG06u8aZF4B\/adRGs+rILgdcKCvFNM7PJY+dITnPnChFwc4EsOJ596gTOf2MiOBYIXgjdCoL6+ngYNGkRLly4tITJixAgaOnRoZhYtXLiQhgwZUurXvXt3mj59OlVV7RayaS1N8OKWtUyonXXAxO4MemXH4EwZOicDwZcT2As5BWeF4LM+GIIXgpfWr19Pffr0oQ4dOpSE6ooVK6h\/\/\/7Uo0cPmjhxYtnEnDx5Mk2YMIGmTJlCPXv2TLVVSfBu7TmO\/rVfu6gLBK\/191\/aISZ2aai86QjOvKFCKhDwJQWTV53AmVd0ZAYDwQvBS7xCO3LkSJo1axZ17dq1hAiL2blz51JtbS1VV+953a8Qyn379m2wElzOXhxqkXibe\/9X6Z9x6UTm++qsAyZ2Z9ArOwZnytA5GQi+nMBeyCk4KwSf9cEQvBC8FVdvp06duocQFgNY2I4dO7asIK6UzZx4fQf9kHiFVzQIXuvvv7RDTOzSUHnTEZx5Q4VUIOBLCiavOoEzr+jIDAaCF4K3bJIMHz6cli1bVlbQ8grwokWL6KabbqLLL7+c1q5dG9nq3bt3xTII7gPBm\/luetUBE7tXdEgFA86kYPKmE\/jyhgrpQMCZNFRedITgheBNTUSxEa3SxjUWxPPmzaNWrVqVVoHT6oHTHHDi9fnhjVR\/5nWlH\/\/h8sOo+5eO8OLFQBANEcDEHl5GgLOwOANfYfHF0YIzvzljPRJvvCjXr1+\/aKGuU6dOfgdvITqcw0tEYsNaTU1NxdMWhOAVG9YEP0IsJ\/89zl+a4D1w3g+ijXIDBgywQDVc5EFgx44dtHHjxqiWu3nz5nmGoq8jBMCZI+AV3YIvReAcDgNnDsGXcD1z5sxoMS7ZIHh3I9LkBa+s2GWwWPA++eSTe9T4ltvMlhS8F\/7iPvrwi+dH\/8y3rE05p1kkqNI2yEnkNroYRGD79u20YcOG6LdiCF6DQGs0Dc40gmnBFPiyALJmF+BMM6CazbEWia\/yLlmyhCZNmoQV3k9xbtKCV6zMypyjy3hxDW\/apjZVwbt8zOma0x3mdCGAr+50IWnPDjizh7UOT+BLB4p2bYAzu3gX9YYa3oYINlnBK8SuzIYzAZlYDR48eLDSsWS9JzxBOz9\/RmQO1woXfZXNjsfEbhZfE9bBmQlUzdkEX+awNWUZnJlC1oxdCF4I3lLNbtYlE2kpmCxrEKu7p5xySsWTGjjxzp+8nD5ud0xk9pJTq+n2S2rMZDmsFkYAE3thCK0bAGfWIS\/kEHwVgs\/JYHDmBHZlpxC8ELxRLS6ftlCuic1n5UoYxG1rYrzMKnFS8OKWNeV32MpATOxWYNbqBJxphdO4MfBlHGLtDsCZdkiNGoTgheA1mmDljHPifWfGP3CtsBP08zvFxJ4fM9cjwJlrBvL5B1\/58PKhNzjzgQX5GCB4IXjls0VjT068c2u3lyxyOQOXNaD5iQAmdj95qRQVOAuLM\/AVFl8cLTgLizMIXgheJxmbFLx8QgMfTYbmJwKY2P3kBYI3PF7KRYx3LDwuwVlYnEHwQvA6ydik4H14aDc688gDncQCp9kIYGLPxsi3HuDMN0YqxwO+wuILK7zh8QXBC8HrJGuxwusEdmWn+DBWhs7ZQHDmDHolx+BLCTang8CZU\/hzO4fgheDNnTQ6BiQF7\/sTv67DLGwYQgATuyFgDZoFZwbBNWAafBkA1bBJcGYYYM3mIXgheDWnlJy5uODl2l3csiaHm6temNhdIa\/uF5ypY+diJPhygXoxn+CsGH62R0PwQvDazrnIHwSvE9iVnWJiV4bO2UBw5gx6JcfgSwk2p4PAmVP4czuH4IXgzZ00OgbEBS+uFdaBqFkbmNjN4mvCOjgzgao5m+DLHLamLIMzU8iasQvBC8FrJrMyrELwOoFd2SkmdmXonA0EZ86gV3IMvpRgczoInDmFP7dzCF4I3txJo2NAXPDyhRN88QSavwhgYveXm3KRgbOwOANfYfHF0YKzsDiD4IXgdZKxccE7qtcRNKpXZydxwKkcApjY5XDyqRc484mN7FjAVzZGvvUAZ74xUjkeCF4IXicZC8HrBHZlp5jYlaFzNhCcOYNeyTH4UoLN6SBw5hT+3M4heCF4cyeNjgFxwcvlDFzWgOYvApjY\/eWmXGTgLCzOwFdYfHG04CwsziB4IXidZGxc8OJaYScU5HKKiT0XXF50Bmde0CAdBPiShsqbjuDMGyqkAoHgheCVShTdneKCly+d4Msn0PxFABO7v9xghTc8btIixjsWHo\/gLCzOIHgheJ1kLASvE9iVnWJiV4bO2UBw5gx6JcfgSwk2p4PAmVP4czuH4IXgzZ00OgbEBe\/7E7+uwyRsGEQAE7tBcA2ZBmeGgDVkFnwZAtagWXBmEFwDpiF4IXgNpFW2SSF4uZSBSxrQ\/EYAE7vf\/KRFB87C4gx8hcUXRwvOwuIMgheC10nGQvA6gV3ZKSZ2ZeicDQRnzqBXcgy+lGBzOgicOYU\/t3MIXgje3EmjY4AQvGd0OZAeGdZNh0nYMIgAJnaD4BoyDc4MAWvILPgyBKxBs+DMILgGTEPwQvAaSKtsk5x435nxDzr9+CMheLPhct4DE7tzCnIHAM5yQ+Z0APhyCr+Sc3CmBJuzQRC8ELxOkg+J5wR2ZaeY2JWhczYQnDmDXskx+FKCzekgcOYU\/tzOoTsgeHMnjY4BSDwdKNqzgYndHta6PIEzXUjasQO+7OCs0ws404mmeVvQHRC85rMsxQMSzwnsyk4xsStD52wgOHMGvZJj8KUEm9NB4Mwp\/LmdQ3dA8OZOGh0DkHg6ULRnAxO7Pax1eQJnupC0Ywd82cFZpxdwphNN87agOyB4IwTq6+tp0KBBtHTp0hIiI0aMoKFDh+bKwsmTJ9PcuXOptraWqqury45F4uWC1XnnNWvW0IwZM6Ic6dSpk\/N4EEA2AuAsGyOfeoAvn9iQiwWcyeHkSy\/oDgheWr9+PfXp04c6dOhA06dPp6qqKlqxYgX179+fevToQRMnTpTKVzGmdevWELxSiIXTCRNFOFyJSMFZWJyBr7D44mjBWVicgS8IXlq4cCGNHDmSZs2aRV27di0hIrtam1wh5hVArPCGNRFkRYuJIgsh\/34OzvzjpFJE4CssviB4wVd4CEDwluWMBe\/UqVP3EMJpA7jvokWLqFu3bjR\/\/nwI3tDfhET8+DAOj1BwFhZn4CssviB4wVd4CEDwluVs+PDhtGzZskzxGl8hXrx4ca4a3tmzZ6MmNIC3Zu3atdSvXz8CXwGQ9WmI4CwcrjhS8BUWX+AsXL54cQ57UYia7dq1a1d4NOqPmEXskCFDKGvjmqj\/7du3b7TBTbYMgid3LqNYsmSJ\/uBhEQgAASAABIAAEAACCQROO+00mjNnDnAhCN4oCcTms5qamtImtnLZwavAdXV1pX6yglf8dszCFw0IAAEgAASAABAAAqYR4JVdrO7uRrnJr\/DmEbtpm93yCF7TiQ37QAAIAAEgAASAABAAAnsi0KQFryhj6N69e+bKLkPHq7vz5s0rm0dZ5RBIQCAABIAAEAACQAAIAAH7CDRZwSvEbu\/evaXP3U2jByu89pMWHoEAEAACQAAIAAEgkAeBJil4VS6ZKAcqBG+edENfIAAEgAAQAAJAAAjYR6BJCt6s0oQpU6ZQz549oxMYss7lheC1n7TwCASAABAAAkAACACBPAg0ScGbByD0BQJAAAgAASAABIAAEAgbAQjesPlD9EAACAABIAAEgAAQAAIZCEDwIkWAABAAAkAACAABIAAEGjUCELyNml48HBAAAkAACAABIAAEgAAEr+EcqK+vp0GDBtHSpUsjT3zjSW1tLVVXVxv2DPNZCMQ3L7Zq1YpmzZpFXbt2rThMHGcnOoHPLJT1\/VyFr7j35LXg+iKDpXIIqHCWnDPZtthIDKTNIqDCl3ivxC2imBPNcqRinXnlNnHiRJXhjWYMBK9BKsXE3aFDh1KiceItW7YMotcg7jKm066IljmRY8KECQ0+fNnOk08+KSWWZeJCn3QEVPhKWhIf5rggxk6WqXAmxBPPmdOnT6eqqqrotJzke2fnCZqWlyJ8nXLKKfiM8zRdxPtT9M4BTx8vV1gQvLngytc57VgzrDLlw9BEb7FKG181SvvlJO673M\/BpwmGGtpU4SsZVXxlHoLXX87SjnnMejfNP03j96D6jqV9xolz7gcPHkxDhw5t\/OB5+oTJb0ogeIkgeA0ma\/I3ZuGq3L8bDAWmYwiUOztZ5UxlIXjjKxwAWy8CRfkSHHFpEa8a9u3bFx\/Eeinaw5oKZ+ID+uyzzwY\/hvlJmlfhi21A8FomStKdeJfq6upoxowZNHr0aIp\/0yxpptF1g+A1RGmlVQmUNRgCXdJsuV84ZC4aSbrAaoYk6AW6FeEr\/h5ed9111KdPHwjeAlzIDlXhTPxiMmbMGFq1alVUxsBNtr5eNjb02xMBFb7YStov\/Cjz8ivD8A3JZ3xA8BrKzUpJprKSaCjMJmm23OTOX+uNHDlSuh43\/ls0NiKaS6UifMXfNY4QgtccT3HLKpyJXx63bdtG8a9fUcNrnjMVvpJ8z5s3L\/qn7t27l+qvzUcOD1kIQPBC8GblSOGfQ\/AWhtCYgaKTuwhMbILCDnJjVEWGVfkSAmr8+PHRVeGotzbLky7BW1NT00AwibmU7YuNbPaepGl4Un3HRO1vvC4ev6D4lTMQvBC8xjMSJQ3GIVZ2oPr1XdqKBsSuMg3SA1X4Snv\/IHilIS\/cUYUz8QtKjx499jg+Cd+KFaakogEVvir9IoJ9Kmb5ymMdgheCN0++KPfFpjVl6IwOVN2gwUHFd75C7BqlqWRcha\/41+NpUeKsULPcqXBWaQMoBK9\/fOFbTLOc6LIOwQvBqyuXKtpJm6SxymQF+opO0mp1ZSYF0WflypXSdb7unzb8CFT5Sj453j17uaDKWdqGXpl3096TNU5PKnxhhTeMXMD7A8FrJVPTDlHHCQ1WoK\/oJG2zmcwJDdh97IY7Vb4geN3wFf8mhI9FEhs6Zd6xtLIGmXHunrRxeFZ9x1DD6z\/\/ELwQvNayNHn4M75KtQZ9piOx6Yw7ph19FP\/lZMOGDdS\/f3\/iHeRpDTuTM+Eu3CEPX2lXd2OFtzAFuQ2ocJa8qhbHkuWGXXmACl\/J8iHwpQy\/kYEQvBC8RhILRoEAEAACQAAIAAEgAAT8QwDn8PrHCSICAkAACAABIAAEgAAQ0IgABK9GMGEKCAABIAAEgAAQAAJAwD8EIHj94wQRAQEgAASAABAAAkAACGhEAIJXI5gwBQSAABAAAkAACAABIOAfAhC8\/nGCiIAAEAACQAAIAAEgAAQ0IgDBqxFMmAICQAAIAAEgAASAABDwDwEIXv84QURAAAgAASAABIAAEAACGhGA4NUIJkwBASAABIAAEAACQAAI+IcABK9\/nCAiINCkEPjkk0\/ooYceou3bt9P3vve93M\/+9ttv0\/Tp02no0KHUvn373OOLDHjiiSfo5z\/\/OfEVuh07dqS5c+dGf+Zt4naxU045hSZOnJh3uHf9k7eljRgxIuLHRUveBDZlyhTq2bOni1DgEwgAAYcIQPA6BB+ugQAQICp65e\/kyZMjoVlbW0tpVwqbwnjr1q10xRVXRGJ3yJAhdMghh9AZZ5xBLVu2zO2ysQreQw89NLqS+6ijjqKjjz46Ny46BmzatImWLFlCf\/7zn6NfjCB4daAKG0AgPAQgeMPjDBEDgUaFQKiCV6dI1WnLh+Tw8XkWLlwY\/WICwetDhiAGIGAfAQhe+5jDIxBoMgjs2rWLHnnkEbr55ptp7dq1tM8++9CJJ55IN954Ix1zzDGU\/Lq5VatWNGvWLOratSvV19fTbbfdRvPmzaN33nmHmjVrRp06daJrr72WzjvvvOj\/hw8fHv1ctN69e5dKAl577TX62c9+Fq3scTvhhBPouuuuo9NOOy0Tfy6TuPXWW+kPf\/gDffDBB9ShQwf6wQ9+EJVctGjRgoR4ihuq9LU94\/DYY4\/Rb37zG3rzzTcjHE4\/\/XQaM2YMdenSpbTK3a1bN+revTv99re\/pY0bN0Z+OWbxvOxv586ddO+999KMGTMiTNn2wQcfTAMHDqTLL788io+bWPn+\/ve\/H\/n917\/+Rb\/4xS\/owgsvpDfeeCP6+3PPPUd77703XXLJJXT88cdHvAj82QaXmbCfu+66i9577z066KCD6N\/\/\/d\/p6quvpqqqqrI4pgle5nPQoEHRM33ta1+jX\/7yl\/Tuu+9GeTB69OhodZw5LdfKreTLrvBD8GamPToAgUaNAARvo6YXDwcE3CIwf\/78SKB+9atfpV69etHmzZvpv\/7rvyJhM3v2bGKBy0JkwoQJ9JWvfIW+9a1vRYJ0\/\/33j8Ts008\/Td\/97nfp1FNPpdWrV9OcOXMi4TV16tTI5osvvhgJtP+\/vXMHraKLovDBJmIhKra+AoKKlTYpVNBoEwPBQg1BsVNCCGoh0crCgGIaUwW0kKBNwAdYCCIxkCiKCopFAomFaB8IYiGx+Pk27Mtx\/pnJuTGPe5N1QAQzd845a67hmzVr7wHcLl26FHbt2hX2799v5+Tz27dvD2fOnDERHj58GL5+\/WpA3NLSUigMEAk4Apz8vW3bNoPqN2\/ehFOnThkUAmojIyPhzp07obGxsfSxPUCKq8ged+\/ebesBCFnPpk2bwv379w06OTf\/Tgzg3LlzoaGhwUCTdaAZGnCuvr6+cO\/ePYPgQ4cOhZmZGTsXIH39+nVbiwMvNxobN24M58+ftzmOHDlisM2+uBasZf369Qa1wC0wHd9wALYfP36sXIMPHz6ER48e2U0LoMln80YZ8E5MTNgaTp48adoyH2sH8suytQLe5f2\/rNmlQL0rIOCt9yuo9UuBGlYA6MQtBdAcjl6\/fm0w2tvba4CTF2nAnaXICTcxLnYCuHBaeTTt\/54FIbK1AB15XoDPM7XAHPOSuQXw8mANoAQaKaIDNoFnBoV1ACvOqsNn6mP779+\/h46ODoNEYNsd2Hfv3oXu7u5w7do1g32AFxf2wYMHYceOHTav75esMMcC2hcuXLA8LA4t4MjwOXCHvegNXVjzzZs3w+nTp+043x9u8927dwOOMiOGfAdeIB+HnPMcOHCg8i1j3QA08+OoVwu8nz9\/\/uumg4wt7jQ3QUXXxQE+L6sth7eGfwFoaVKghhQQ8NbQxdBSpMBKUwDgBHYBNpzHvC4K1WR4\/djjx4+Hnp4ekysLPJ8+fTKHtLW11R6TxwPnF9c4fmwf\/xygPHv2rAFnf39\/BSg5BmeSPeA4M3cq8NLJATgnnlHkYPq5AFBcY3+0nzqHxwVwjH3d6IITHu+1bH9EOCjq4njiHzjmOOi4vDjxPnCGWePBgwcLO0qUObych3niSARr5XsC7O\/duzf3v4Ec3pX220H7kQJLq4CAd2n11mxSYFUpgLvr4MTGyZoSWwAc3cUsA97fv3\/b425g8\/379wF3GIc2zupmQSgvXxuLDkwCgkePHv3ftcgDaj8oC3GpMArIAbtFkM35i85V9O+41Wg7Pj5uUDo2NmY6EXtwmMwDXj6D28sfHOPszcCVK1dsneSKyduiedHAlUZH4ifZMVeGN9t6LSVfK+BdVb86tFkpsOAKCHgXXFKdUApIgVgBHqMDY0NDQ+HFixeV4jXP0uYBL0DH43jyrcQJiB\/Q35ZitpcvX\/7lLhYB73yq8RcDePPAMwUQ80AYLQFScrwU061bt84yv\/TvBU5x0MuA99u3b1Z4197engS8eW5syrd7PsDb2dlZeCPCnALeFOV1jBSQAkUKCHj13ZACUmBJFZicnLQMLgVLQCkZTvKrQJjncl+9ehUAILKdXV1dlcffDmwUtsVZ1TjbCfiR4SXW4LGH1A2mRBpwp3FHUx3eokgDxWjEJ44dO2YQigbZF09k5\/BYBfu\/ceOGFaQxPAdLoVsZ8JbtLwbzPXv2hIsXL1oXDXLLFP9VM8qAl0w11521+mBubm6Yq6hfb9ExdLEgVzxXH+YUF7maPepYKSAF6ksBAW99XS+tVgrUjQI\/f\/4Mly9fNocWWPHisenpaYPRzZs3FwIvxwNFg4ODlcIq3E2ghhZWbW1thcDrRWtkTfn8li1bTDNcYz5LmzIKzzxSEQs6V9EaUEYxG4\/zU4G3qGjt2bNn1nKM7gRkZlOA16GNvC05Zh9EPbhZAFTLgLeoaM2BGSc+LlqjwwaRFHK8nismQkHhHK3M+HnemKtLQ1wQSMs5vg\/EXbjmRS\/uoBMFkD8wMGDdJhg\/fvywz\/7580fAWze\/GbRQKbA8Cgh4l0d3zSoFVoUCdAIgmkC3AyCV8fTpU3MOvQ2Vu478DFeXYqgvX74YVBFjwOUFguicgHtLJwNgzx1eByHAh9ZnzPX8+XPryEBhFOeh6IrPA2tAOA5zUc\/XlLZkdFpIBd64LRnOLEVvU1NTVqC1b98+uxmg6CwFeHF42ScasC8iDKOjo9aGbXZ21m4OyoAXjXHJcZZpQ0ZbMs4BgLLvtWvXVoCXGwcg+u3btwaYaE5m+MmTJ2HDhg3WUYG+yNUCL9eQ4rp4buaKu0Y42Me9jX3vdKagUweDGxrWArC7w+vXhX6\/cXGcHN5V8StHm5QChQoIePXlkAJSYNEUwN0lbgDMAFSM7IsGAELgDzD+9euX5Tibm5vthRW3b9+2IjWyqvTdBYjp\/EDvWW9hRSEWcEu\/WF7m4IVUvE6WzwPPACJwxmN6wJuetGUj78UTxBhOnDhRaSuWCrzMk33xBAAP4DqUV1O0Rs9hWrrRus1f5EGxGY7x8PCw9TfeunWrgXS2S4PvmRdPuNvtL55gTUQK4uI6f\/nH48ePrf8x1+Hw4cPh6tWrdjNSNMocXrTgWuKW48ID\/bQ443vhIw94+Vm8d6CZ7h9c11u3bgl4F+1\/sU4sBVaGAgLelXEdtQspIAWkwD8pQEyCwkL+lMFsyiTVtiVLOee\/HiOH918V1OelQH0rIOCt7+un1UsBKSAFkhWgzRt53DVr1phT7r1wiRTglhLzyPbITT55dKCAdz6q6TNSQAospgIC3sVUV+eWAlJACtSYAkQWiEQ0NTVZP2OK+VJfu5y6FQdeWqbR1WLnzp2BTC29fRkLAdWpayHfS7yFYkXmnU+7utS5dJwUkAK1q4CAt3avjVYmBaSAFFhwBchVU8DHG9nIVeP20iWCHDBFdUXFfNUsxIHXc9sUnwG+ywG8FEgyN11DGALeaq6kjpUCK0cBAe\/KuZbaiRSQAlJACkgBKSAFpECOAv8BdrFLKmO9EKEAAAAASUVORK5CYII=","height":337,"width":560}}
%---
%[output:2330891c]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"     1.320000000000000e+01"}}
%---
%[output:55c0aace]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"     7.500000000000000e-03"}}
%---
%[output:6d4591c7]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"     7.500000000000000e-03"}}
%---
