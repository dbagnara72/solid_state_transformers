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
application690 = 1;
application480 = 0;

n_modules = 2;
%[text] ## Settings and initialization
fPWM = 5e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM*2; % PWM frequency 
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
ubattery = 1250;
margin_factor = 1.25;
Vdab1_dc_nom = ubattery;
Idab1_dc_nom = Pnom/Vdab1_dc_nom;
Vdab2_dc_nom = 1500;
Idab2_dc_nom = Pnom/Vdab2_dc_nom;

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:1dd96712]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:174c5e41]
%[text] ### AFE simulation sampling time
dead_time_DAB = 3e-6;
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
danfoss_SKM1700MB20R4S2I4; % SiC-Mosfet full leg
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
inv.Csnubber_zvs = 4.5e-9;                      % [F]
inv.Rsnubber_zvs = 5e-3;                        % [Ohm]

wolfspeed_CAB760M12HM3; % SiC Mosfet fpr 3L - NPC
inv_mosfet.Vth = Vth;                                  % [V]
inv_mosfet.Rds_on = Rds_on;                            % [Ohm]
inv_mosfet.Vdon_diode = Vdon_diode;                    % [V]
inv_mosfet.Vgamma = Vgamma;                            % [V]
inv_mosfet.Rdon_diode = Rdon_diode;                    % [Ohm]
inv_mosfet.Eon = Eon;                                  % [J] @ Tj = 125°C
inv_mosfet.Eoff = Eoff;                                % [J] @ Tj = 125°C
inv_mosfet.Eerr = Eerr;                                % [J] @ Tj = 125°C
inv_mosfet.Voff_sw_losses = Voff_sw_losses;            % [V]
inv_mosfet.Ion_sw_losses = Ion_sw_losses;              % [A]
inv_mosfet.JunctionTermalMass = JunctionTermalMass;    % [J/K]
inv_mosfet.Rtim = Rtim;                                % [K/W]
inv_mosfet.Rth_mosfet_JC = Rth_mosfet_JC;              % [K/W]
inv_mosfet.Rth_mosfet_CH = Rth_mosfet_CH;              % [K/W]
inv_mosfet.Rth_mosfet_JH = Rth_mosfet_JH;              % [K/W]
inv_mosfet.Lstray_module = Lstray_module;              % [H]
inv_mosfet.Irr = Irr;                                  % [A]
inv_mosfet.Csnubber = Csnubber;                        % [F]
inv_mosfet.Rsnubber = Rsnubber;                        % [Ohm]
inv_mosfet.Csnubber_zvs = 4.5e-9;                      % [F]
inv_mosfet.Rsnubber_zvs = 5e-3;                        % [Ohm]
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
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     3.551136363636363e-05"}}
%---
%[output:01f62bda]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     1.783252832105145e-04"}}
%---
%[output:231dc037]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     5.633826408401311e+02"}}
%---
%[output:1bc16b1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     3.818376618407357e+02"}}
%---
%[output:26989b55]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.283130953403918"],["67.668222262819981"]]}}
%---
%[output:75c58cb8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:6a06269e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:0c76ff32]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000200000000000"],["-19.739208802178720","0.996858407346410"]]}}
%---
%[output:1ba8c347]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.311017672705390"],["54.332172227996914"]]}}
%---
%[output:16217f48]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.073386376052051"],["3.802432508328568"]]}}
%---
%[output:28c3084b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAY4AAADwCAYAAAAXW4N5AAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQtwV9d55+euk0GkdrAMsaNgLNkWjdN05UftIdhZ2lDHTreQTqctEp2GJYTQjZ3NDBAJ4Xi97BoQBDrj+FWWKCx9IGjcOIFME+pRHWyCSbzgKK3jxLKFFmSBg8E4jUPSNmbnu\/YnHx3dx7mP87jnfHfGg\/+6957zfb\/vnN\/vnvd5586dOwd8MQKMACPACDACigicx8KhiBQ\/xggwAowAIxAhwMLBBYERYAQYAUYgFwIsHLng8vvh06dPw5IlSyIne3t7obGxUYvDu3btgu7ubli\/fj0sWLAgygP\/hhf9rjLjOL\/Onj0La9euhUWLFkFra2uV2SWmNTg4CIsXL4ZPf\/rTSn4WsfHgwYOwb98+6Orq0uITYTkwMBClv2PHDpg1a5ZyXnGxV35Z04OI86pVq6CpqUkbbppMt5YsC4c16DljQkA3mcjC0dDQEBHFoUOHYNu2bcaEY8OGDYDEriLKRGZ5bMS0Fy5cCMuWLdNGgJSHKPp5SrLuWOexRXwW7brvvvuMloeitrrwHguH5iggWWzZsiXKBb9oRKKiSrR8+XLo7+8H\/IqTn5G\/8ERSoC\/Y66+\/Hi644ILo6w+vrEpN+co2yQR76tSp6AuZvsjxS5bSVk0DWy0y2YjkgTZg64OuefPmQU9PDyC540UEevTo0THCjSNVwmJ0dDR6T8RJ9Ov++++HjRs3wp49e8byRJ\/mz58fiYn8d7EFRLHEGG3atAnw94wZM8bslW0Q40D30D9qDcixpdhPnz490RYRd3SA8MKyg6JBV1tbW4QXXtiKpBZClqgQtoQDpYNxlPMW74nVKK3MirEfHh6O6oZc5qm8yL6gDYSjXCZvueWWMT8Rk49+9KPwiU98YkKrlsqanGdcfDRTQ62TZ+HQGD5RNMRsqHkvV8SsSk\/3iZBkoqL7cqWQv6xEohbF4+KLLx7XVUXCQWRM6R4+fHgc2aelUVY4MG3CiXATBRNFZmRkJBI4slMWISRD6oJLEg4iMRErEcc40jx58iSgaKfZIMeaYicTtBj7JBsvv\/zyceIglgf5HpL65z\/\/efjsZz87Jhpy+ZGLfpJNSXGPEw5ZNCgPEqykMk8CmBRLel8u82jbQw89BF\/84hfHif4HP\/hBeOKJJ2I\/dOIEyVQ3rUa6MZo0C4cmuKmAT5s2bexLmb6kqJLs3r07ImD6jabQVy+1HvArUiYb+vomYsf3sCUjfqnG9T1T5UDCo5aP+AVIX22YHn6tyunjV17eNLKEA7\/os7ov5K9B+XkS6DhSRhxmzpw5ThBVuqooTfF9\/GqXhUCOJd0nnKhF8oUvfCH6uqb7cS0psSiqdFXJXVNJv5PKjzyGJZdPxImwlok\/qVUrPy8T8qOPPjquzIuiHteFl\/SRQGUeyyTZTUJG8cVWk9iaFFutcV1uGHN8x2T3pSb60Z4sC4cmiOPIUCYLqkQiyYsFGk2TWwfi1z3+P35p01evWNHjhEOuhNQdRBAkdVWJ6edNowrhEHGjr3EiAbQ9bkBfxFEWxDThkL+IEUdsick4JwmDXJyQzMhmebwiqdsJ7UsTjqRuOVk4kr7uk1qkoljSgLfsJ33sJAlHXBpxLd4sMZNbLnKLJK7MizbFxZ+660R7xK67LNs1UUUtk2Xh0BS2uC+aJOFIKvBJwoF\/TyI0uVtHdC8v6VOLo6xwyCKa9TsuJPTOXXfdFbWGaKwg6cs9r3DIpCH+LiMcYldK0kB33DgYtR7Fd1RbGFndQlR+5NlQcWXHtHDIZY66ruQuwaqEQxxTY+FQJ0MWDnWscj2po6tKNiBOCNKEQ\/yKoxaJSEZLly6NHeMQK6lqGtQdJnafyQPrSb\/jgJa\/ssUWVdmuKnlsR+zqKNpVlbfbCZ8XBZUG60XhkIlN7hbK6qrKKsBVdlXJ3a\/kB42PJbU4qBVO92WbZCHBWBXpqorDgoUjq4S8dZ+FQx2r3E\/qGhxXabYnza+P676groukwXFROESCEwFJmxFEz2UJBz6XNFMH7xGe8jNJkwQIJ7kfXRQGTPdjH\/tYNIAc15WRNJEBbVAZHKevf5mUkgaRk3DEdPCiGXpx3S3ibCRM595774V77rlngl\/yzDVKK2twHMcTssajVAfHs4RDrmxpZT7ObpXBcbnlxWMc6hTHwqGOVaEnq56OK5Jm3hYHORDXj4\/dFipjHFlp4H2RyNFebMnccccdE2a4EHmIZJMmHGnrFFSn49IArEiySMpz5swZm7GEJPXxj38cbr\/99rEuMVm4cDruypUrU6fjigQd13UZR7Jx412YN9pILUKatv3AAw\/Al770JaDxHlEQ5Y8BEsU0fDGftOm4cqsoabFm0viEOAaXJByyqCMeOA2cBq3RBnm8Cf8m5inGU15kKo4ZivfkLjl5\/K9Q5ff4JSeEAyt9Z2dnNL8+bhWvPK87bbppnWKV9fVWJ198tlUkU3kqtNwaS8KBiAkFWteqbp9jkOYbfTTgM3GzBVV2I+B1HPlKj3XhUJl2iASL8+x9q3AsHPkKq82nk7odsxZbijbnWTlu09e65Z3V7aeypQzWRV45rh5568KBrQmsUHgltTjwfnNzs9L+Puqu23+ShcN+DFQtiOtHz1qFLadNX7XYzZVnfydVG0N+Lm6cS3UfLd6rKn\/JsSoc+KWwZs0a6OjoiMQjTjgoqLNnz\/ZOOPKHi99gBBgBRsA+AlaFA7+48bruuusSxzjimqF5ugfsQ8wWMAKMACPgFwLWhAObltu3b4c777wz2msoaXBcbt5zc9+vAsjeMAKMQP0QsCYc2DWFUyCxrzdrVpUMK42JxA2WX3HFFfWLAlvMCDACjICEAO6Y3dLS4iQuVoQjaRYEIqQyoJU2WI7CMTQ05CTYuowK0WfEMkS\/Q\/SZY62LOYqna0U4ZHPTWhw060pcuIULr5J2sAyxYoXoM5NJ8UpfxzdDLOMu++ykcKBY9PX1jR2QIy8ATGuVuAy2rgobos8sHLpKk5vphljGXfbZCeGosqi6DHaVfoppHTlyxNm+UF0+Y7oh+h2iz6HG2mUuY+HQyWyG0mYyMQS0A9lwrB0IgiETWDgMAR1q9wWTicECZjkrjrXlABjMnoWDwdaKAJOJVnidSpxj7VQ4tBrDwqEV3vGJuwy2LhiYTHQh6166HGv3YqLLIpe5jMc4dEXdYLpMJgbBtpwVx9pyAAxmz8LBYGtFgMlEK7xOJc6xdiocWo1h4dAKL3dVMZkYLGCWs+JYWw6AwexZOBhsrQgwmWiF16nEOdZOhUOrMSwcWuHlFgeTicECZjkrjrXlABjMnoWDwdaKAJOJVnidSpxj7VQ4tBrDwqEVXm5xMJkYLGCWs+JYWw6AwexZOBhsrQgwmWiF16nEOdZOhUOrMSwcWuHlFgeTicECZjkrjrXlABjMnoWDwdaKAJOJVnidSpxj7VQ4tBrDwqEVXm5xMJkYLGCWs+JYWw6AwexZOBhsrQgwmWiF16nEOdZOhUOrMSwcWuHlFgeTicECZjkrjrXlABjMnoWDwdaKAJOJVnidSpxj7VQ4tBrjlXCcPn0alixZAgMDA8qgtbW1wSOPPKL8fJkHXQa7jF9p7zKZ6ELWvXQ51u7FRJdFLnNZ7m3VUThWrFgBq1evhtbW1kzMBgcHYd26dbBt27bMZ6t4wGWwq\/AvLg0mE13Iupcux9q9mOiyyGUuyy0cukCqKl2Xwa7KRzkdJhNdyLqXLsfavZjosshlLsstHNRVhWD19vZCY2OjLtwKpesy2IUcUniJyUQBJE8e4Vh7EkgFN1zmstzCgf6ePXsWVq1aBXv27Inc37FjB8yaNUsBCv2PuAy2Lu+ZTHQh6166HGv3YqLLIpe5rJBwiEAdPHgQFi5cGP1p3rx50NPTAw0NDbqwzEzXZbAzjS\/4AJNJQeBq+BrHuoZBK2iyy1xWWjgIE7EV0tTUFA2GqwyeF8Q08TWXwa7aV0qPyUQXsu6ly7F2Lya6LHKZyyoTDhE8mkm1efNm42MgLoOtq4AxmehC1r10OdbuxUSXRS5zWWXCwS0OXcUnO10mk2yMfHmCY+1LJLP98Fo4eIwjuwDofoLJRDfC7qTPsXYnFrot8U445FlV69evhwULFujGUSl9l8FWcqDAQ0wmBUCr6Ssc65oGroDZM278CBz97jcKvKn\/ldxdVbSO4+TJk9YGwNNgYeHQX2hcySFEEg3RZyxvofnd99QJuL3vWTj957\/tSnUbZ0du4XDSC8EoFg7XI1SdfaGRSYgESqUltFh7Jxy8V1V1xFdVSqFVqlDJhIWjpaoq43w6G\/YOw4a9R\/xpcfDuuO6VORYO92KiyyKOtS5k3UrXO+FwC96J1nBXlesRqs6+EEk0RJ9DbGmxcFTHE1FKGzZsiP7t6uqKTZmFo2LAHU4uRBIN0WcWDvcqYa0Gx2nNyLJly1g4hLLEZOJexdJlEcdaF7JupYszqnCAnGdVlYwLrh1Zu3YtPPPMM9FOvNzieAtQJpOShatGr3OsaxSsEqZ2fWUQtu4fYeEogWH06q5du6J\/h4eHuatKApPJpGzpqs\/7HOv6xKqMpfMfeBr2v3DGX+FAQu\/u7o4wwnM58Orr66t0e3WcybVmzRq4++67YevWrSwcLBwRAiGSaIg+hxhrr4UDB6pHR0cjQkdi7+joiLqRsgaw8yoxpjdnzhyltHlwPC+69X0+RBIN0ecQhaNx+WNRxfRujENcCDh9+vToREASjiq3Vce0tm\/fDnfeeWd0QFSWKKFw0NXf319fVsxh+cjICGAMQrtC9DtEn7Fch+L33Llz4fXJU+EnH35j9mhQwoGzn5DgqziTXOwKE4kRTxu89957J3AltzjCkY8Qv75D9Dm0Fsf+58\/A\/Aef9lM40Csk9QMHDozrqpo5cyYsWbIE2tvbteyYq9LiGBoaCoc9A+3rD41MqECzcPhftT+3+3l48FvH\/BUO9Ew8j4NCqnObdRaOiRWHycR\/MmHhOAItLWHsVXXNPU\/C0dM\/h1\/52cvw8l\/8kZOFu1YLAFUQ5K4qFZT8eCZEwQzR55Bal2I31duPfhtOPPw5JysrC4eTYclnFJNJPrzq\/DTHus7RS7cdWxl39D0brd+Y0TgJTu\/+n\/4c5ESuq+6Sm7Y9iI4iwC0OHai6mWaIJBqiz6G0OP76O8fhv+36YVTZ\/ssHmuCrXR8BV8drS7U4cHB8586d42ZPkaDQ4HjWmETVlMTCUTWi7qYXIomG6HMIwiF2UWFrY\/enroXf+s33+SccJBC4ZxQu+hMvcTruqVOnYN26ddExsyYuFg4TKLuRR4gkGqLPvgtHnGigeLjMZYVbHCwcbpCn75UqDeUQSTREn30t4zimgTvg4kl\/dGFL4+arpkQ\/vRQOdEylq4rWesQt1tNBvS6DrcNfXyuVClYhkmiIPvtYxlE0cJEf\/osXtjDub796TDS8Fg50Lm4dB252iN1XeG\/lypVRN1Vra6sKF5R+hoWjNIS1SSBEEg3RZ5+EQ5w5RRUNReN7n\/vAhHrnMpcV7qpylV1cBlsXZkwmupB1L12OtXsxybIIxQL\/27j3SDTVVhSMNfOuhI+2vSs2CZe5jIUjK+o1uM9kUoMgVWQix7oiIDUnQ2Lx8OGX4C8Pjk7IreOGS6Hr1paoiyrp8lY44rqpCIS2trZKNjnMG1+Xwc7ri+rzTCaqSNX\/OY61uzGk8YoH9x2D\/\/3EyARDUSQW\/Oal8Cc3vjtVMOhFl7mscIsDj3LFrdRnz54N8+fPH9tWXfcmh1nFxmWws2wvep\/JpChy9XuPY+1WzEgsaMV3nHVxA98qXrjMZYWFQzyPAwe+caFfc3NztCMutkSqPgVQBWh8xmWwVX3I+xyTSV7E6vs8x9pu7FSEAi1Esfjap66F8978\/yJWu8xllQkHTrvF88BxQWCVBznlBdxlsPP6ovo8k4kqUvV\/jmNtNoYoFH\/z3ePw5Atnxg1sy1agUMy4aBLc33G1UjeUihcuc1lh4UDHxe1ExFbG7t27o3M6enp6olP7TF4ug60LByYTXci6ly7HWl9MUCT+\/p9fhr\/\/p5OpIkEtCvz3kT+7Blqm6uE4l7mslHDIq8dRSLZs2QJNTU1G126IRcllsHUVeSYTXci6ly7HunxMcIsPbCF8\/h+G4f+dOpspEiQUN105JZoJJQpHeWuSU3CZy0oJh07QiqbtMthFfcp6j8kkCyF\/7nOs1WNJ4xG4pcex0z9XEggSBex2Wvv7rfDOhvONCYXsmctcVlg45MFx0Wke41Av3FU8yWRSBYr1SINjPTFOKBBPPP8K7HrqBBx95Y3FdqoXtjzmzLwIVvxOszWBSLI1OOEQd8dtbGxUjWElz7kMdiUOxiTCZKILWffSDTHW2LV03msn4cevvxO2HXgxtzjEtSLSFt65EnWXuSx3iyNt0Z8IuOkDnChvl8HWVSBDJBPEMkS\/ffSZWgjnzgHg4rlnj\/+0sDhguRDHIkg0dNU9nem6zGW5hYOASuuq0glmVtoug51le9H7PpKJChYh+l1Hn8Wuo637R2Dg2L8UEgZRBFAc\/vNvTIMLJ50\/bkdZlXJTl2dc5rLCwuEq+C6DrQuzOpJJFViE6LeLPpMwnH7t3+D\/PDkKQyd\/VlgYxG6l1ksmw2c+dHlUVH756nFoaXljRlMol8tclls4VM8a572qzBVvF8nEhPch+m3aZxKFX\/z769D77RfhB6PFupHE8kDjCzdfdREsuek9cPE73hZNj8W8ksYeTPttovxm5eGVcGQ5a\/u+y2DrwibESoVYhuh3lT6LLYXtB0fhhR+XaylQ+aZV1FdMmwyfmnMZTHrbr2QKQ1bdqNLvrLxcue8yl+VucbgCapIdLoOtC7sQKxULR3JpEscUcEbSzqeORw\/nnaoalwO1CHCdw7y2aXDr+6ZGj2W1GMqW\/RDLuMtcVlo4cI+q7u7uceVi\/fr10WaHNi6XwdaFR4iVKlTh+PbAIJx7xzT46S9+CV\/\/p5Nw9NTZSgSByD\/696JJ8Hv\/cRq8792\/GgmCC1NXQyzjLnNZKeFQOXNcF1lyi+MtBEKsVD4Jh9hC+NZzp+E7R16NVjpX0UKQu48ua5wUnTg3+e3\/Ibp181VTTFfRQvmFWMa9FA55nyqxNPACwEJ1o\/BLIVaqOggHCQL+u2\/wNHxn6NXKuoxEQaBWgth1JN8vXLgceTHEMs7CYbDwuQy2LhhCrFQ2hYM2yfv+i\/8C3\/jnlytvHcjdRjdddRHgUaN4HTt2DG5qa9VVlJxNN8Qy7jKXcVeVs1VF3bAQK1WVwiF2FR06+hPY99wrpdcixEVPHFiecXEDXHfZBfA7V1889qjKWALHWr1e1P1Jb4UDA8OD4\/aLJ5PJ+BjIm9w9PvgKHBx6FY6erm4gWcxRFISr3jUZbvv1qdEYgo6BZY61\/fpmygIvhYPGONrb263NoIoLoMtg6ypwoZCJuKfRN555Gb7y1DGYNGlSpYPIcWMHuB5h7nsboW36BdFt3VNP08pJKLGWMQjRb5e5rFRXlbzh4Y4dO2DWrFm6+FEpXZfBVnKgwEN1rlQkBid+8gt4+PBL8MPjr0UIVDmjKE4M3vvud8CHr74YZl7yjrEVyypdRQXCU+krdY51GSBC9NtlLislHGJBELus+ATAMlUk\/7suVSoaOP7XX74eCcGB589oEwL6+o\/+vWgStExrgE\/c9B54Z8PbxloG+dF0+w2XYm0SqRD9DkI4xEKER8hia6S3txf4PA791UtnpSIheP3cOfir7xyHE6\/+QsssorhWAa45+I33XAC\/+\/43VifL3UQ6\/dYftWI5hOgzIhWi30EIB503jkFW2eDw7NmzsGrVKtizZ09Ug9LO75C7xNJaNC6DXYwqst\/KU6lICDDVR773Y3jupdeMCAG1Cq66ZDL87q9Pg5mXTJ4gBNmejn8ij99503b1+RB9ZuFwrzSW6qoq0z2FQoNXV1cXZA20Yz7Dw8PRs1lXaMKBYwQ4t\/+yyy6DI6fOwte\/fxJ+dELfOAF99ZMQXDltcnT05nUzLhwXGhPjBSGSaIg+s3BksZ75+4WFI23leBE3RCGR38d7zc3NSrO3fBIOah089qPT8HeHX4pg0T1ofPnFDTD7yinRKWpRfm9udW1CCPKWmxBJNESfWTjy1gz9zxcWjipNSxMh6tKaPXu2d8JBM4oe2ncMnqngnAOKibiuAMcJZl0xBea0XuS8EOQtUyGSaIg+s3DkrRn6n7cuHDQ2Mm\/ePOjp6YGGhoZxXscdHJW2+66LLQ4UCPwPZxnh6Wj7X3hjplGeSxSD5qkNcO1lF0ZrCzDd8147ydtQ5AGzxs+ycNQ4eDlNd5HLyAXrwkGGoICMjo5OEI\/BwUFYvHgxbNq0KVojIv+WY+EK2Ejod\/Q9qywSdPgNTin9g2sugZapDWPbWaedjBbq11iofrNw5GTfGj\/uCpfFQeiMcKAgdHZ2wsaNG6G1NX0Tt7TxEASbrv7+fqPF5tir\/wb\/q\/8UHHrx54n5Nl14Plz\/nknwyRvfGEPA32WvkZERmD59etlkavd+iH6H6DMWzFD8njt37rh6ODQ05GS9LCwc2IW0YsUKWL169QSiRxFYt24dbN68WXkdR56t2NMGy02rdFrLgloRnbe2aNm3iEoUf4U6Wbe0GMWx1gKrk4ma5rI8IGgRDhUREFsNNACO6zPkKbdyWvh75cqVsG3bttiWiSmwkwQDxeKPr78UVn+kJU8cSj3LZFIKvlq9zLGuVbhKGWuKy4oYmVs45MV4SZmmLejDd+QFgOLgOObR19c3Nt6RZ08s3WCjYHz50Euw9hvjm5AoGLs\/da2VYzaZTIoU\/Xq+w7GuZ9yKWK2by4rYRO\/kFg56Ma2rqoxBZd\/VCTauq7hj57PRTCa68ICdrje7osraXvR9JpOiyNXvPY51\/WJW1GKdXFbUptLCUTZjXe\/rAhtFY\/6DT4+ZbbOFIWPHZKKrNLmXLsfavZjoskgXl1Vhb+4WB62rWLp0KWzduhUGBgZi7VDZr6oKB+Q0qgYbWxd7vn8S7tr9\/FhWX\/2v18B\/enNBnQ4f8qbJZJIXsfo+z7Gub+zyWl41l+XNP+353MJRZeY60qoSbBSNvqdOwIa9RyJTsZVxf\/vVcPNVb0yldeViMnElEvrt4Fjrx9iVHKrksqp9YuFIQfQL\/3gU\/sfXXxgTDVuD31lBZzLJQsif+xxrf2KZ5YmXwhG3FYgIRN27qrClcXvfs86LBhrIZJJVBf25z7H2J5ZZnngpHElO4zTbtWvXwqJFizJXgGcBV+R+FWBjF9U19zxZC9Fg4TC3XqZIeaz6HRaOqhF1N70quEyXd1q6quR1GLqMj0u3LNgoGjh7iqbcYveUa2Mast9MJiZLmN28ONZ28TeZe1ku02mrFuEosuVIVU6WBRvXaez47onInA1\/0ApLb3Z\/Dygmk6pKj\/vpcKzdj1FVFpblsqrsiEtHi3Ak7XSr0xFKuwzY4loNnEH1vc99wITJpfNgMikNYW0S4FjXJlSlDS3DZaUzz0igsHCkDY6nnQmu26GiYMt7T9Whi4qwZDLRXarcSZ9j7U4sdFtSlMt024XpFxYOE8YVyaMo2OKA+N2\/dwV85kOXF8neyjtMJlZgt5Ipx9oK7FYyLcplJowtJRzysa5pu9yacAbzKAo2zqKiA5Pq0kXFLY4j0NLCs6pM1S2b+YQomEW5zEScSglH3IFKtsWjCNji2Ebnh5th1W31IqMQKxVWjhD9DtHnUGNdhMtMiEaprqqqD3KqyuEiYNe5tRFqpQrVbxaOqpjC\/XSKcJkprwq3OGhwHA9ewrPAxUvlICddDuYFW2xt4PboXbc26zJNW7pMJtqgdS5hjrVzIdFmUF4u02ZITMKFhQPT2rVrF+zcuRN6e3vHjoglQWlvb4cFCxaY9CXKKy\/Y8x94Gva\/cCbawNDVvaiyQGQyyULIn\/sca39imeVJXi7LSq\/K+6WEAw2JOxFw\/fr1VkQjr3CIM6k+9GuN8PCytiqxNZYWk4kxqK1nxLG2HgJjBngtHMZQVMwoD9idX3kOvrj\/xSjlOq3bkKFgMlEsHB48xrH2IIiKLuThMsUkK3uscIuDZk91dHRMGOOozLoCCamCLW9kWLcpuCI0TCYFCkpNX+FY1zRwBcxW5bICSZd+pbBw1P3McR8GxSn6TCal60FtEuBY1yZUpQ31UjgQFRwcHx4eBpxZ5cqlCjYNite9mwrtZzJxpfTpt4NjrR9jV3JQ5TIb9pZqcSxZsqSWZ46L3VQ3XzkFdt9+rQ3sK8uTyaQyKJ1PiGPtfIgqM9BL4agMnYoTUgFb7KZ6oONq6Ljh0oqtMJsck4lZvG3mxrG2ib7ZvFW4zKxFb+VWuMVhy+CsfFXA9qmbiruq6rU9TFb5zbrPwpGFkD\/3VbjMlre5hYMW+C1duhS2bt1au64qn2ZTUaFhMrFVfczny7E2j7mtHL0SDlsgquabBbZv3VTc4uAWh2rdqPNzIQpmFpfZjGfuFodobB23Ve976gTc3vds5EadF\/2JcQixUoUqmBxrm3RpNm9vhaOO26qLe1PVedEfC0eY05BZOMySt83cvBSOOm6r7ts0XB7j4IOcbBKbybxDFExvhQPXcdRtW\/XG5Y9F5b2uW6jHVdYQKxV3VZmkbft5hVjGvRQOLEp121Z9y+Mj0P3VQa\/GN0Il0FD9DpFAQ421t8KBAa3Ttuo+jm+EWqlC9ZuFw37rx5QFXguHKRBV80kC29fxjVAJNFS\/WThUmaD+z7FwGIxhEtg+7YYrw8lkYrCAWc6KY205AAazZ+FwAOy\/+e5x+PTOH3o3vhHql3eofrNwGCQTy1mxcJQMgDiO0tTUBNu2bYPW1tbYVJPA9nV8I1QCDdVvFo6SZFKj11k4SgRrcHAQOjs7YePGjZFYxM3kEpOPA9vn8Y1QCTRUv1k4SpBJzV71VjiQ1BcvXgyjo6MTQtLW1ga9vb3Q2NghE\/l5AAAPSUlEQVRYabhkIZETTwLbx\/Ub5DuTSaVFzOnEONZOh6dS47wUDtqnCruOTJ4AiC2OAwcOQE9PDzQ0NEwIVBzY4sC4L\/tTiY4zmVRaX51OjGPtdHgqNc5L4TB95rjYutmxYwfMmjVLeYzjv+95Ae5\/7Gj0PO5PNaNxUqUBtp0Yk4ntCJjLn2NtDmvbOXkpHNTi6OjoSCRxHcCTgGzatCk2XwSbrv7+\/uh\/P\/mVE3DoxZ9D04Xnw55F03WYZTXNkZERmD7dP7+yQA3R7xB9xnIQit9z584dV+yHhoayqoGV+6W2Vc\/qNtLlUdyuvJSXrNK+D4yj3\/wVqqukuZcux9q9mOiyyMsWB50EODAwEIubrsFx+QwQOfM04fDhfPE4sJlMdFVd99LlWLsXE10WeSkcusCS05VnUeGajpUrVyau5ZDB9n1gnFscfAKgqbpoM58QBZOFo2SJkzdSzDM4vmHvMGzYeySywMeBcRYOFo6S1asWr7NwuBWmUmMc6AqOc3R3d0deIaHj1dfXlzhdVrf7skr7vGKcsAyxUoUqmBxr3QziTvretjhwkBoX\/919992wZs0aoBlWaYPXusMig33NPU8CDpDjFFxfjoqVMWQy0V2q3EmfY+1OLHRb4qVwiOs4cCroqlWrxoQDxyXWrVsHmzdvrnzleFawRLBDmFEV6pd3qH6zcGQxgD\/3gxMOHJPAVoeOLUeyioUIts9bqYs4MJlklQp\/7nOs\/YlllideCgeNb+D2H2JX1cyZMwHPIm9vb4cFCxZkYVP5\/aQWh49bjfAYxxFoaeHB8corkYMJhiiY3goHli+Xj44VZ1SxcDjIBiVNCpFMQvQ51G5Jr4WjZN2v\/HUR7BBmVIVaqUL1m4WjcspwNkEWDoOhEcEOYUZVqAQaqt8sHAbJxHJWXguHuI6DcE5boKc7FgR2KDOqQiXQUP1m4dDNIO6k761wxJ3GR3tY2R4cF4Wj69YW6Lq12Z0SUbElTCYVA+pwchxrh4NTsWleCgcJBB7iJJ+N4cJ03BD2qOJZVTyrqmKucja5EAUzOOFwYQFgKDOqQu2yCdXvEAk01Fh7KRwYzLidal3pqvrM3\/4Q\/urg8egLytfNDbnFwS0OZ5sIFRsWomB6KRxZ53GI5QbP5njkkUcqLkrxyRHYoUzFDfVrLFS\/QyTQUGPtpXAYUYECmRDYNBX35iunwO7bry2QUn1eYTKpT6zKWsqxLotgfd5n4TAYKwT7W\/\/3B4DCgdfc9zbClz\/ZZtAC81kxmZjH3FaOHGtbyJvP12vhiFvHsX79eiv7VGFoZeHwfSpuqM34UP1m4TBP4LZy9FY4XF3H8Zf\/cBjmP\/h0FG+f96iiAs1kYqtqm8+XY20ec1s5eikcLq\/jWPbQP44dF8vCYavY6883RBIN0edQW5csHPo5ZCwHBHvjw0\/CJ\/\/6B9HffJ+KG2qlCtVvFg6DZGI5Ky+FAzF1tavq\/Sv+Dva\/cCYK++k\/\/23L4defPZOJfoxdyYFj7Uok9NvhrXCQeHR3d49D0fbgOAmHz+eMi4AzmeivxK7kwLF2JRL67fBaOPTDly8HBPvM7\/dGL4WwhiPULptQ\/WbhyMcHdX6ahcNg9JrffyP85MMbohw7brgUHui42mDudrJiMrGDu41cOdY2ULeTJwuHQdxF4UDRQPHw\/WIy8T3Cb\/nHsQ4n1iwcBmM948aPwE9v7oxyZOEwCLyFrEIk0RB9DrVbkoXDIKlM\/60\/hZ9d9\/EoxxDWcIRaqUL1m4XDIJlYzoqFw2AAROEIYQ1HqAQaqt8sHAbJxHJWLBwGA3DpH94D\/zrjpijHENZwhEqgofrNwmGQTCxnxcJhMADv+thfwL9P\/TUIZQ1HqAQaqt8sHAbJxHJWLBwGAzD1z74Mr0+eysJhEHNbWYVIoiH6HOpHAguHQWZpXP5YlFsoi\/9CrVSh+s3CYZBMLGfFwmEwACQcoSz+C5VAQ\/WbhcMgmVjOioXDYABIOEI4wIlgZTIxWMAsZ8WxthwAg9mzcBgEm4QjlMV\/oX55h+o3C4dBMrGcFQtHTADoIKiBgYHo7rx586CnpwcaGhomPH3w4EFYuHDh2N+bmppg27Zt0NraOuFZEo5QFv+FSqCh+s3CYZnNDWbPwiGBffbsWVi1ahXMnj07OpucfqMgdHV1TQgNnvsxPDwce09+mIXDYMm2nFWIJBqiz6F+JLBwKBAMisOBAwdiWx0bNmyA5ubmSGSyLhKOUFaNh1qpQvWbhSOLAfy5z8KhEMsk4ZBbJ1lJoXCEtPgvVAIN1W8WjiwG8Oc+C0dGLGm8o729fUKrQh4LwaTSThhk4fCn4mR5EiKJhuhzqB8JLBwpDEAtCnwkbnB8cHAQFi9eDJs2bYJZs2aB\/DtujOP8l38Ev7p\/I\/T392dxjxf3R0ZGYPr06V74kseJEP0O0WcsE6H4PXfu3HFVYGhoKE+VMPbseefOnTtnLDcpoyzRSLILxzzwihtIxxZHSKvGQ\/0aC9VvbnHYYivz+XKLIwbzrJlUaWFKGyxH4Qhp8R\/i5HIB01ndQvQ7RJ9DLeMux9paiwPJf3R0NHHtBhEOruHAZ3t7e6GxsRHw98qVK1PXcbBw6KRrd9J2uWLpQilEn1k4dJWm4ulaEY64AW90oa2tLRKI5557Dvr6+sZERV4AuGPHjmi8I+7CFsfkw1+Ctx\/9dnFU+E1GgBFgBBxAgMc4HAgCm8AIMAKMACNQHgErLY7yZnMKjAAjwAgwArYQYOGwhTznywgwAoxATRFg4ahp4NhsRoARYARsIcDCYQt5zpcRYAQYgZoiwMJR08Cx2YwAI8AI2ELAG+HATRK7u7sjHNP2srIFdBX54nqWLVu2REmlTUkWn0s7u6QKm3Snoeoz2ZF3U0zd9hdNX9VveWp7Wrkoaoup91R9pm2HcB1Y3ct3FrZ5dgbPSqvK+14IBxakzs5O2LhxY4QN\/X\/cQU9VgmcyLXEhJK5zERdFinbIuwzj7507d44toDRpc9m8VH2W\/ccPiDp\/PKj6Le++INaDupV9VZ9JKHG7IVzLVefynVU\/SEhdLMteCIdMlq6qdFZBSbsv7s9FhNHR0ZG4EJLSqjOZ5PUZSWXFihVw5swZiNtpuQz+Jt9V9Rtju27dOti8eXO0q0Kdrzw+ix+GdS7fSfEicZwyZUr0yG233aZ0FpHJ+HshHPKmh2mbIJoEt6q8kk5MpBMU0\/Kpa8Uq4jPG\/YYbboCvfe1rY6dLVhUDU+nk8Tvt8DNT9laRTx6f41ocSQfAVWGbjTTQx1deeSXqhhNPSrVhS1Ke3giHeEJgnqNmXQpGki1xLQzVVpXqnmCu4ZDXZxTI7du3w\/Lly2HNmjW1Fw6xNZkUayrnGDuVsS\/XYkz25I01Pb9nzx5YtmyZ0pHSrvqeZpfL43UsHDUoUXkrFrmExHLfffclbgjpsut5fMZn165dC4sWLYrOJXH1K00F7zx+04QQGhDP2gBUJX8bz+TxWT6PR94E1Yb9uvJk4dCF7JvpclfVRIDrLBroTZ7uCySPffv2RV+eLlc2lWqQx2+5q6quvofoc5GyoPKOqWe8aHHIXVOq3TimQK4iH9GnrMFxX2aaqPosTuMUsa5rN4aq3yiY4i7SWeWiinKoKw1Vn30RSxUcXf4Q8EI4eDruW8Wwrt0VcRVJdYqm+K7LlU2FLPAZVb\/lgeI6d9uo+hzXVZV2Po8q5i4+53JZ9kI4MOghLwAUW1xJX991XRiWtCgsaQKEy5UtDzmp+i0uAKz7YjhVn8XzeeruMw+O56kV\/CwjwAgwAoxAbRHwpsVR2wiw4YwAI8AI1AwBFo6aBYzNZQQYAUbANgIsHLYjwPkzAowAI1AzBFg4ahYwNpcRYAQYAdsIsHDYjgDnzwgwAoxAzRBg4ahZwNhcRoARYARsI8DCYTsCnuYvr2pOc1P3imdxrcO8efOgp6cHGhoaMpGXF9hlvmD4AVr30NbWpvW8FXFTwTz4GYaDszOIAAuHQbBDysol4chjixijOggH2ot7dJm4fNnG3QRWvufBwuF7hDX6Jx9bSntDiSt76WsYv\/Bx11rcCpsuPNls\/vz54\/5Op52JX7n4fNaXbtJqYnFHAUwnbgV9kh\/0dzxpjrYtl7\/uxXwxffE+5n3y5Eno7++HgYGBKG+8L+Jw1113we7du6PTK\/HUvjx+y5t75tlmPU4Us4Qh677GosZJO4YAC4djAamLOVk7mopf+egTkiVuD0Ffx+LuvbQVOp1BEbdtSNrhXPL+XHG\/xc0ARYzT\/LjllltgyZIlMGPGjKh7S\/ZDzkcUGvQzbodi8XwUSu\/QoUPR1vdxW8Kn+R0nHOKxufK+TlmtqSxhyLpfl7LLdpZHgIWjPIZBppC1J1RW95C4MaUsHHHv0rGwq1evjr7M6UqyQyTVNFvSNgYs8lUu5isTbZwPovicOnVq3G636GOS33gvTjjk0\/CShKeIbywcQVb1WKdZOLgsFEZA7KaRu5KSyFrcyI42qJOFQ+5eIgPjNrRLGocQN0FME440MlQlV\/qyHx0djUylLjs57bgzwkUBPXz4MGCLQb6SNvJL6qoSxzyS\/FP1TbSFhaNwVfHuRRYO70Jq3iGROMVxDrF7iASDBGZkZAQ6Ozujvv044VA9R9qmcKAPixcvBhQMGjtJa3GoCIeq30ktjuHh4XGD5Swc5utDCDmycIQQZUM+ymcqkHBgd9KKFStA7GbK6qpCAu7t7YXGxsZU6212VeGgtkzUZbuqVP3mripDhZqz4a4qLgPVIZA1gC12D+GzOMiMXSg4Q4laCTjjSBwUlgfHxcH0tLGIKgfHRUJeunTpOLvxnvgFj8IhthCoiy2pq4rSxhaKONguD46r+p00OE6tH1GcxXEhtIPiR3lRTGgiQNw6F+6qqq7+1D0lbnHUPYIW7Zf79sVxDlkccOB34cKFkbVIVps2bYoGd9vb22HBggVjB3ER6cpTZLMWuaUd7pM1UC\/nRX7IgicLB\/4Wp9ai7c3NzbBz586otfToo4+OExaRsGlaMk7Hffzxx2Hz5s1R6yqP33HC8c1vfjPCGM9gx0ucfhw35kJdbYgvCuXevXsjUcvyXWUBpcWiyVlrRoCFQzPAnDwjkIZA3LiHKmIqs6pU01J5jlscKiiF8QwLRxhxZi8dQCBuID9tnUaWySwcWQjxfV0IsHDoQpbTZQRiEJBXmlPXXBGw5L2q4rrGiqQrv8N7VVWBol9p\/H\/5X2fFCvYXRgAAAABJRU5ErkJggg==","height":240,"width":398}}
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
