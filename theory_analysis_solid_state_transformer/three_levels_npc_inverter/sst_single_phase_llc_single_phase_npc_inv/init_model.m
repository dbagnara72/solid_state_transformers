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

model = 'sst_llc_npc_inv';
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
fPWM_DAB = fPWM*4; % PWM frequency 
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
% LLC
fres = fPWM_DAB;
Ls = (Vdab1_dc_nom^2/(2*pi*fres)/Pnom*pi/8) %[output:6e4cc00d]
Cs = 1/Ls/(2*pi*fres)^2 %[output:01f62bda]

m1 = 12;
m2 = 13;
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
ki_i_dab = 5;
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
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"275"}}
%---
%[output:174c5e41]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"1875"}}
%---
%[output:6e4cc00d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"1.7756e-05"}}
%---
%[output:01f62bda]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"3.5665e-06"}}
%---
%[output:231dc037]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"563.3826"}}
%---
%[output:1bc16b1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"381.8377"}}
%---
%[output:26989b55]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.2831"],["67.6682"]]}}
%---
%[output:75c58cb8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.0001"],["-9.8696","-0.0016"]]}}
%---
%[output:6a06269e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.0156"],["2.7166"]]}}
%---
%[output:0c76ff32]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.0000","0.0002"],["-19.7392","0.9969"]]}}
%---
%[output:1ba8c347]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.3110"],["54.3322"]]}}
%---
%[output:16217f48]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.0734"],["3.8024"]]}}
%---
%[output:28c3084b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjMAAAFTCAYAAADMTo0FAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQm4FMW1xw+EfecCsoogm8YoECSiEVwwmuiDBPTJYp4E2VREVMCLcYu4AII+DaigIEIii1FREBU1xihiCCgg+hAR2RFB2RFRhPedInXt2\/RMV1f3menp+ff38aFM1anq36mp+s+prcSRI0eOEB4QAAEQAAEQAAEQyFECJSBmctRzqDYIgAAIgAAIgIAiADGDhgACIAACIAACIJDTBCBmctp9qDwIgAAIgAAIgADEDNoACIAACIAACIBAThOAmMlp96HyIAACIAACIAACEDNoA3lP4F\/\/+hf17NmT6tWrR1OmTKFmzZplnMmBAwdo+PDhNHfuXBowYAAVFhYW1YHrN3r0aJo8eTIVFBRktG6rV6+m3r1705YtW2j69OnUrl07Vb6u71lnnUXdunXLaJ38CmNWEydOLFZfvzz68yjea9asWbRw4UIaNWoUlS9f3rToyNI5fcZGW7ZsGartpGubkVU6A4Y0lzZt2mTNNxl4zbwtAmImb12PF9cE4ixmeGC85ZZbQg9Itt72EjM7duygPn360PLly2nkyJGxEjO6vrVq1Qo8gEfxXlpIderUKSsDplN4aJ+HrUtSxIzzPZzC3Pa7gXzxIgAxEy9\/oDYgUIwAxEywBqF5uaNbJlaSJmZsGHhxSoqY4XcL0z5M2hDSZI8AxEz22OdEyTpqoSvrNRXj\/DX629\/+lvr27Vv0bql+ues8OqE7nbsDPeecc9RUUKr0qWA6B6hUeb0iM853atWqFd19990qu7OeumPUdt3h\/FRCxMlU\/0J0v+\/1119fNO3kfLdUv7JTRQSc7+\/+NWriW3dkhuvi9IOum9O227ecxuuXsHs6hNOsX7\/eMxJlMnUS5F25Ts7B3s0i6Ht5tTN3GSbvkK5T8POX277pd8Urn9eUop4CNfkuur8b7u8O\/7\/Jd8zZlrjt33zzzXTdddd5RgX9+hQuU78r\/3e2ppRzouPPwUpCzOSg0zJVZa9ByauDTJfO3aF7hcG1Tefgki5dmE7aq6x0YsbJ2inkUr2zM00mxUyqqTL9726hZerboGImnV3nAJlKPHgJw1Rp3cLaj4HX90a3OT8x4\/dezZs3L5p6c5bjZ990nZaJv2zETDo\/aOFu8l10+tZLyJj2G5pHgwYNPMW8k61J\/dzRqSiib5nqf1FOMAIQM8F45U1qZyfnjEboTjXVwO7sPLzS6o7Omd9rEHJ3oLqzdHbY6dYCOPM7B3KvOvmJGXfUyIuNs16aQRgxoxcAm04zeXXSqaYHgvg2yJoZr1+9znppLql846yX9hl\/4fT6HK\/8zvaWipVX1MqrHaYa6Ezfyx1t0AuA\/Rj4TQcF8VeQKSFnvfR3id9BL0TXPuBFzPrf+HP9XfR6L6\/omLNOzu+sU6CZfMfcfYLOY9qncN2D8Mmbzj4hLwoxkxBHRv0aJtMWujPRad2\/\/t2DQ+fOnT137Dg7GK9fW27RYrLIMtUuHK+dQenEjNdOEK\/yvXbBZFLMeA2kmzZt8tyJFMS3QcSMuw26f6HrQdtp0z2IudvSp59+6rnTzCvilOq9nINmOuFg+qs91XulEjN+ESO\/3UZB\/BVksE5VL\/durFRiJNX7OtuBO\/LjJWbSfcfcn7nbTpA+RdfLpP+Iuj+FPXkCEDPyjHOuhHQdotdnqToHd9p+\/fp5huK9wvLp6mDSGaUSM17O8Fsz495ia1I+l5NpMeOOIHzwwQfHrD8J6tugYibdFIOXmHGvpXEze\/3119U7pHq8piXcgsVr+sVreiedmDF5r1SDe7q8nCfdVFNQf0UhZtysg34X001deYkZrwirqYD71a9+Zdyn6PcyjXbmXMed5xWGmMnzBuD1+kE7UIgZ77NEMi1mnH67\/fbbadmyZcecWxPUt0HEjHMQ0wN0jRo1jpkmSic0JcSMbuNeg6zzl38qMWP6XhAzo8gZDdRTUsxYT1NBzGDAkSIAMSNFNsftmv4y4gPTwk4zBRVUJpGRVFMZ+t\/Hjh1bdACcbWTGa1EtHy6nIzmZFjPuaND27dsVWveujSC+DSJm9Ps6ByyvdRVRTDMFiR54tS+vOqQSM6bvlUrMpJrOMe0igvjLJjKjRYc+EJHrO3To0KJ2EyQyw2cP8W4353fDb81MusiM7TRTOrZe\/jT1BdLFlwDETHx9k9WaBVl0mGpNgukCYK8BM0gH6nXKaqpFps6Qv57iCCpmvNh4LabUHTs7Uq8NcW\/hTbU1O+gCYN1Y3FMqXgNFEN\/aiBmvHV1cv6gWAKcSDenWMvG2Yj+R5Sdm\/N4rVb28BF2qtF5f+iD+CiJmvNosf5fc31vnziL3FJ6bubPNu79f\/G6mkRmvdw6yADhd9M\/kx1BWO18UbkUAYsYKW35kSrcd1O9cESchZweY6jwOd0cXVsywvVRbVd1lBRUzzoHIqyV47bxK1WL8xEy6BZReNlN1+O60pr71E5raLr8HTynpqw+86mZyrkv79u3pnXfeKfbLPt2aE68twe5f8+nWcDgFipudjlgEea9Ui4NN3yFVOzH1VxAxw2WlY+O3e8xLkHH5eveZ17uYihkvX7A99i1HHDkCmuoHgrNct5gPyic\/evpkvCXETDL8KPYW7s7O79C8q666igYOHKg6G35MD81z\/+KLQsxw+V7iyevuI\/fdTCa\/3twDlBcbr0iJ82BBPzHj7tT9dr44Byi\/M0xMfJtuV5jXIYZum6YH4em6ei1a9hKm6VhzevfUmlc78GLprr9uv6bv5S7HOZi624Kff9xfahN\/2QzWXqLf+b0N+l1022NbJ5xwwjG70ky+Y+6or3MTQaqdcJqb1861dAcrinWiMJwRAhAzGcGc7EJMOqVkE8DbRUUgzE4TDFRReSG+dky3z6d6A68zg+L7tqhZEAI5L2a485s5c6bvpXKpphzCXsIWBHZS00LMJNWzMu\/lHJBSLRT1O0guVc20GLLNL\/PGsBqUgDPq6YzAhF1MjfYR1BO5kz6nxYwWKCY35LpX6OeOi+JfU4iZ+PsobjVMtwaE62p7q3GQPiFuTFCfHwmkW+\/GqWx+hDqny2zbF3wUXwI5K2ZS7VZJ94tt4cKFRdtm4+uS3KsZxEzu+SwONfZaFBt0HYnXe+j2iAErDl62r4PJ5Z1BrGuB1KZNG4wDQcDlSNqcFTP6yG2+0XjOnDm+00zcwfGjt7zmiH9QTRAAARAAARAAAR8COSlmnFNGvPvBb82MVvhVqlRR2z71g3l1fD9AAARAAARAIPcJ5JyY0cKke\/fuxKfPmiwA1uFFnmfVkRltp2HDhmlDjnw8N\/\/BAwIgAAIgAAK5ToAPQeQ\/SXtyTszwdJH7yHi\/yEwqp+mV8anm1lnEDBs2jBYtWpQ0v+N9QAAEQAAE8pDAGWecQWPGjEmcoMkpMeO1I8kkMpOqveqIzaBBg1SUx\/1oscOOr1+\/fh42e9lXZpH48MMPqy8W+EbLGmyj5em0BrZgK0dA1rJuu0lcHJ9TYsZvO2eq02bDipkkOl72K2Nm3S8yZmYFqbwIgK1cuwBbsJUjIGs5yW03p8SMl5tNIjOpHOh39kySHS\/7lTGzDr5mnGxSga0NNbM8YGvGySYV2NpQM8+TZL55IWb0mTQbNmwo2sLttSg41TQTIjPmX5YgKdetW0dTp06lXr16UaNGjYJkRVofAmAr10TAFmzlCMhahpiR5RvKuldkhv9t3Lhxx1w2556m8puWSrLjQ0GPKPO3335LX3zxBdWtW5fKlSsXkVWYYQJgK9cOwBZs5QjIWk7ymJbzkRlJ1yfZ8ZLcTG1jUDAlFTwd2AZnZpoDbE1JBU8HtsGZBcmR5DENYiZNS0iy44N8AaTSouOSIovIjBxZsAVbSQKytpM8pkHMQMzIfnvSWIeYkUMPtmArR0DOMtqtHFu2DDEjyze21pPs+DhAR8cl5wWwBVs5AnKW0W7l2ELMyLKNtXWIGVn3oOOS4wu2YCtHQM4y2q0cW4gZWbaxtg4xI+sedFxyfMEWbOUIyFlGu5VjCzEjyzbW1iFmZN2DjkuOL9iCrRwBOctot3JsIWZk2cbaOsSMrHvQccnxBVuwlSMgZxntVo4txIws21hbh5iRdQ86Ljm+YAu2cgTkLKPdyrGFmJFlG2vrEDOy7kHHJccXbMFWjoCcZbRbObYQM7JsY20dYkbWPei45PiCLdjKEZCzjHYrxxZiRpZtrK1DzMi6Bx2XHF+wBVs5AnKW0W7l2ELMyLKNtXWIGVn3oOOS4wu2YCtHQM4y2q0cW4gZWbaxtg4xI+sedFxyfMEWbOUIyFlGu5VjCzEjyzbW1iFmZN2DjkuOL9iCrRwBOctot3JsIWZk2cbaOsSMrHvQccnxBVuwlSMgZxntVo4txIws21hbh5iRdQ86Ljm+YAu2cgTkLKPdyrGFmJFlG2vrEDOy7kHHJccXbMFWjoCcZbRbObYQM7JsY20dYkbWPei45PiCLdjKEZCzjHYrxxZiRpZtrK1DzMi6Bx2XHF+wBVs5AnKW0W7l2ELMyLKNtXWIGVn3oOOS4wu2YCtHQM4y2q0cW4gZWbaxtg4xI+sedFxyfMEWbOUIyFlGu5VjCzEjyzbW1iFmZN2DjkuOL9iCrRwBOctot3JsIWZk2cbaOsSMrHvQccnxBVuwlSMgZxntVo4txIws21hbh5iRdQ86Ljm+YAu2cgTkLKPdyrGFmJFlG2vrEDOy7kHHJccXbMFWjoCcZbRbObYQM7JsY20dYkbWPei45PiCLdjKEZCzjHYrxxZiRpZtrK1DzMi6Bx2XHF+wBVs5AnKW0W7l2ELMyLKNtXWIGVn3oOOS4wu2YCtHQM4y2q0cW4gZWbaxtg4xI+sedFxyfMEWbOUIyFlGu5VjCzEjyzbW1iFmZN2DjkuOL9iCrRwBOctot3JsIWZk2cbaOsSMrHvQccnxBVuwlSMgZxntVo4txIws21hbh5iRdQ86Ljm+YAu2cgTkLKPdyrGFmJFlG2vrEDOy7kHHJccXbMFWjoCcZbRbObYQMwHY7tu3j5555hnavXt3gFzFk1atWpWuuuoq6\/xRZoSYiZLmsbbQccnxBVuwlSMgZxntVo4txEwAtjt27KA+ffrQ8uXLA+QqnrRly5Y0e\/Zs6\/xRZoSYiZImxIwszeLWMSjI0QZbsJUjIGs5yWNaiSNHjhyJCp8WM4WFhdSuXbvAZhn06NGjIWYCk8vNDBgU5PwGtmArR0DOMtqtHFtEZgKw3blzJ91xxx1qmqh169YBch5NunTpUnryySdp3LhxgfNKZEiyipXgFdQmOq6gxMzTg605q6ApwTYoMfP0YGvOyiZlkse0SCMzhw8fJv5TqlQpG86xy5Nkx8cBNjouOS+ALdjKEZCzjHYrxxaRmQBseZppwIAB1KRJE+rVqxe1aNGCSpYsGcBCvJJCzMj6Ax2XHF+wBVs5AnKW0W7l2ELMBGDLu5kmTJhAf\/nLX2jv3r1KzPCU069\/\/WuqXLlyAEvxSAoxI+sHdFxyfMEWbOUIyFlGu5VjCzFjwfbAgQO0ZMkSJWreeustZeHcc89Vwub000\/PmWkoiBkL5wfIgo4rAKyAScE2ILAAycE2AKyAScE2ILCAyZM8pkW6ZsaLKy8KfuONN9TC3lWrVlHdunXVFFSXLl2oVq1aAV2R2eRJdnxmSXqXho5LzgtgC7ZyBOQso93KsR09fy3NWLyV9jx5BU2fPt1qx7Fc7cJbFhczuoq8A3zjxo00c+ZM+tvf\/kZff\/01tW3blvr166eiNnFcNAwxE76BpbOAjkuOL9iCrRwBOctot3JsB85YqcRMtRf6QMxEhfnQoUO0YsUKGjFihDI5efJkKigoiMp8ZHYgZiJD6WkIHZccX7AFWzkCcpbRbuXYQsxEyFaLmFmzZtErr7xC3HB79uxJQ4YMoUqVKkVYUjSmIGai4ZjKCjouOb5gC7ZyBOQso93Kse30yFJ6d80uRGZsEfO5M6tXr6YXX3yxaHopV3Y5QczYet0sHzouM042qcDWhppZHrA142STCmxtqJnlgZgx41QsFa+P+fLLL2nOnDnEUZi1a9eqrdkchenRowcdf\/zxVKJECQvLmc0CMSPLGx2XHF+wBVs5AnKW0W7l2ELMBGB78OBBmjdvHk2ZMoU+\/vhjtai3ffv2dMUVV9CZZ55J5cuXD2At+0khZmR9gI5Lji\/Ygq0cATnLaLdybCFmArDVF01u27YtZ7Zfp3s9iJkAzrdIio7LApphFrA1BGWRDGwtoBlmAVtDUBbJWt3zHm3Y8S3WzJiw48jMF198QQ0aNIjlVmuTd3CmgZgJSixYenRcwXgFSQ22QWgFSwu2wXgFSQ22QWgFSwsxE4CXjswUFhZaHcjD4mH06NE0e\/bsAKXKJYWYkWPLltFxyfEFW7CVIyBnGe1Wji3ETAC2EDMBYCEpxIxgG8CgIAcXbMFWjoCc5YKb\/qGM49A8A8ZazCxfvtwgtXeSli1bZiQyw7us+DTidAf2ITJj7UajjBgUjDBZJQJbK2xGmcDWCJNVIrC1wmaUCWLGCNPRRHxr9jPPPEO7d+8OkKt40qpVq6oLKSUfPvemd+\/e6m4oiBlJ0ulto+OSYw+2YCtHQM4y2q0MW174y9NMiMzI8M2KVb7Re\/jw4TR37lziKBDETFbcoApFxyXHHmzBVo6AnGW0Wxm2Cz7bRZ0fXQoxI4M3O1Z5emnhwoXUqlUrdagfxEx2\/AAxI8sdg4IcX7AFWzkCMpYhZmS4Zs0qr4EZOnSoOtTvgw8+MF4zM3jwYDrjjDOK6l2nTh3iP3jCEeBBYevWrVS7du2cO1Ax3JvL5wZbOcZgC7ZyBKK1zP0r\/3n640P09MffIzITLd7sWNOLk7t3707dunVT1yyYLgB217hXr1505ZVXZudFElQqn0u0fft2tXapbNmyCXqz7L8K2Mr5AGzBVo5AtJanTZtGU6dOpW9+fhV91\/CXEDPR4s2ONT6\/ZsuWLTRq1CgVBQgiZsaMGUP169dHZCZi1\/H6Jb7Di6Nc5cqVi9h6fpsDWzn\/gy3YyhGI1rKOzFw7Zzut+7YSxEy0eDNvzTm91KxZM1WBIGJm+vTpVocAZv5Nc6tErD2Q8xfYgq0cATnLaLfRs3XuZGLrOGcmBGO+RXvv3r30ww8\/EG+95huzDx06RKVLlw5h1TwrR2UmTpyYMsPIkSPV1JP7wTkz5oxtUqLjsqFmlgdszTjZpAJbG2pmecDWjFOQVBAzQWilSMsiZvHixXTnnXfSqlWrirZClylTRi3Ebdu2rbqQkm\/XzvSDyEymiR9bHjouOR+ALdjKEZCzjHYbPdvR89fS6PnrlOGS33xFVV4rpCTONpQ4wopD6Fm6dCn94Q9\/oI4dO1LTpk3p9ddfV1uhec3Kgw8+SH\/961\/pgQceoIsvvlioBqnNQsxkHPkxBaLjkvMB2IKtHAE5y2i30bPVdzKx5RMPr6cdc0ZAzATB\/N1339Edd9xBhw8fprvvvptY2PB0jz7XhaeZ7rvvPvr666+LFuUGsR82LcRMWILh86PjCs8wlQWwBVs5AnKW0W6jZes8X4Yt39rmID1y+7UQM0Ew663Qffv2pUsuuYT0jdjOQ+rmzZtHkyZNSntwXZAyo06LNTNREy1uDx2XHF+wBVs5AnKW0W6jZdvpkaX07ppdymjDgnI04YIS1LNnT4iZIJi1mOHzWLp06eIpZjg68uyzz9Ljjz9O1atXD2I+I2khZmQxo+OS4wu2YCtHQM4y2m10bGcs3koDZ6wsMvhIj5Op8Q\/rIGaCItbTTPv37yfeLfTRRx8Vm2batm0bXX\/99dSoUSMaMWIE8aLguD0QM7IeQcclxxdswVaOgJxltNto2Lp3MHFUZtltZ6qgAiIzFow\/\/fRTuuaaa9QuJl4A\/Nxzz6ldTOvWrVOLf\/mW7aeeeopat25tYV0+S5IdL0\/PvwR0XP6MbFOArS05\/3xg68\/INgXY2pL7MR8LGY7I6Okl\/mTOta3p7KbVIGbC4F2xYgXdc889aou282nRooWKyPD27Lg+EDOynkHHJccXbMFWjoCcZbTbcGy9hMyADg1o5O+OHhab5DFNdGu2dgvv\/t61a5eKyPC9Jk2aNKEaNWpQyZIlw3lOOHeSHS+Mzsg8Oi4jTFaJwNYKm1EmsDXCZJUIbK2wqUxeQkZPL2mrSR7TMiJm7N2T3ZxJdnx2yR4tHR2XnBfAFmzlCMhZRru1Y8tCpvOjS5Wg0Y9byCAyY8dWrYd55plnaPfu3b4WOEpz5plnqohNnKI1EDO+rguVAB1XKHxpM4Mt2MoRkLOMdhucrfssGbbAQobXyfDfzifJY5pYZGbnzp3q0DyGxwfj8dO4cWP199q1az09dvnll9Ntt91GlSodvd0z20+SHZ9ttlw+Oi45L4At2MoRkLOMdmvO1mtaSQsZ3rnk9SR5TBMTMwySt2PfcMMN1K9fP+rcubO6xoAfPv2XrzYYN24cPfTQQ2p79ttvv01\/+tOfqH\/\/\/sRn08ThSbLj48AXHZecF8AWbOUIyFlGu\/VnyyLm\/tfW0fR\/f1EsMUdh\/tztJOrQLPWZbUke08TEzIEDB2j48OFqS\/Z1112nbsp2PrwoePz48bRx40Z13UHZsmXpySefVJEcPkQvDk+SHR8Hvui45LwAtmArR0DOMtptarYsYsa+vo7+uqi4iNHRGK9pJbe1JI9pYmJGnwDM58xceOGFnh567bXX6LHHHiu6zuDNN99U0ZrZs2fLfVsCWE6y4wNgEEuKjksMLabw5NCCLdgKEjjWdKrpJC1ixnc\/WZ0hY\/IkeUwTEzO8APimm25SkRn+u1SpUsVY81QT35z92Wefqb95nczMmTPp5ZdfpmnTppn4RTxNkh0vDs+gAIgZA0iWScDWEpxBNrA1gGSZBGx\/BPfwm+vprpc+9yTJU0p8dswp9Sods8g3Hfokj2liYoaBPv\/883TrrbfStddeSz169FBny\/DDC4JnzJihojA8xcQLfz\/55BO1YLhVq1YqTxyeJDs+DnzRccl5AWzBVo6AnOV8b7dzP9xOt8\/5rNgWaydtFjHju59EDQvKBxIx2kaSxzRRMcPRl6lTp9L9999P33\/\/fbFvQOnSpenmm2+mXr16qYP0eH3Npk2bVJRG73qS+8qYWU6y480IyKbK945Lki7YytEFW7CNigBvq37v81008lXvHb66nKMixnw6KVX9kjymiYoZDZS3aS9atIhWrjx6g2fz5s3pF7\/4BdWqVUv9P4sZjtbUrFkzVhdOJtnxUX0Zw9jBoBCGXvq8YAu2cgTkLCe53fLaFxYl6dbAuKMwIzo3pVYNKltFYby8lOQxLSNiRq7py1pOsuNlyZlZT3LHZUZALhXYgq0cATnLSWu3+kTe0fPX0ozFW33BsdjpfVZ96tLquMgEjLPQJI9pomKGt19v2bKF\/v73vxcdnOcE+91336mppbvuuosKCgp8HZ3pBEl2fKZZepWXtI4rDkx1HcBWzhtgC7apCLB44duq+QwY563V6YixgBnQvgFdcmotEQEDMRNBe12wYAENHDiQ9u7d62mtcuXKdMEFF6jD8vi\/4\/ZAzMh6BIOCHF+wBVs5AnKWc6Xd6ikjXvOy4LOdSrgEES+\/bFKNerStq8SL+8oBObq4NduKLa+D+eMf\/0gbNmyg0aNHU+3atdX\/t2zZUu1sevXVV+mJJ55QO5r4TqY4PhAzsl7JlY5LloKMdbCV4cpWwTY\/2bJw+WL3QZr2ry3GwoVJsVg5vno5KryoccbFi9tTSR7TxKaZ9KF53bp1o+7duyumfOLv9u3b6fbbb1f\/zyKHdzUNGzbsmBOC5b4u5paT7HhzCnIpMSiArRwBOctot8ll64y4zFuxnT7asi+QcNHipX3T6tTt9DpZFy8QMxG0VS1mBg0aROeff76yOG\/ePBWRGTVqFFWsWFH9\/6xZs5TIqVKlSgSlRmsCYiZanm5rGBTk+IIt2MoRkLOc6XbL0ZZ\/r9tN\/1i1I7Bo0cKF\/77t4hOpTpWysRMvEDMRtNX9+\/ers2Nat25NV111lbK4ePFitT5m0qRJVLduXXVtAZ\/2O3nyZCwAjoB5rpnIdMeVa3zC1Bdsw9BLnxdsc4ut3lG0aO1uNUW0cee3KQ+lS\/dmem1L11bH0fkn1Yi9cPF6lyT\/QBebZmKQLFR4TQwLmIsuukhNMfXt25euuOIKat++Pd177710+PDhousM5L4idpaT7Hg7ItHmwqAQLU+nNbAFWzkCcpbDtFsWLfxn8frd9OYndpEW\/WZ6nQvvMqpWoXROCheImQjbKd\/PdM899xBPOfHJvjy1xIt+x44dS3w6MN\/X9NBDD9HFF18cYanRmYKYiY6ll6UwHZdszXLfOtjK+RBss8dWC5bDR47QmNfWWUdZnKKF\/\/v8FgXUtXVt4wsb5QjIWk7ymCYamWG38Fkz33zzDVWoUEEt8uVIzPLly2nJkiXUoUMHdRow\/3scnyQ7Pg68MSjIeQFswVaOgJxlbrfvr9pIB35SiUqXKUdjX1+nCjPd9pyqZnqK6Den1FTnufCT6W3RctTMLSd5TBMXM+kws7Dh6A3fmF2yZElzj2QoZZIdnyGEaYvBgCvnBbAFWzkC4S3zwlsWE0\/\/+wtauGZX6AiLM9LC26A7n1aLTq77443SmTzLJTwdOQtJHtPExIzezVRYWEjt2rXz9A4fqvfAAw9gAbBc2421ZQy4cu4BW7CVI5Desl5wy39v3vUt\/XXRF5FEV5yChf\/7VyfXoN+2PC5voyw2\/oWYMaTG1xOsWLGCDhw4oE795bUxPXv2pBYtWhxjgdM+\/fTTtGfPHnr88cepevXqhqVkLlmSHZ85iqlLwoAr5wWwBVs5AlS0G2j91wdozOvr6MiR8FNBzvoWnxaqSUQl8nJaKGofJnlMizwy8\/LLL9MNN9ygFvj6PXxg3siRI6lLly6xXDeTZMf7+SYTn2PAlaMMtmAQ9lUXAAAgAElEQVRrQ8B5aBznX75pL7368VeRRlacERaeEurS+jhqflxFJVa43ZY+uFMd3VGuXDmbV0CeNASSPKZFLmZYxHBUZteuXXT99dfT4MGD6ec\/\/\/kxeHknE9\/HFNfFv1zhJDs+Dt94DLhyXgBbsE1FQE8DrfpyP81b8RWt2f5NZGtW3NEVFiuNapSnIb86gTbtPFh0D1GqNSxot3LtNuljWuRiRruCF\/fu3r1bbccuU6aMrIeErEPMCIH9j1l0XHJ8wTb\/2DqjKocOH6G\/f\/I1Ldu4V0SoMF19FkuL2hXplHoVqdl\/oiv6MxsPoN3aUDPPk+QxLVIxowUMb8c2fTgyU7VqVexmMgWWoHTouOScCbbJY6t3AH22\/Rv6+yc76MNNckLFKVbOaFyVzmlenUr8Z91KGLHi5xW0Wz9C4T6HmDHkp3cw8Tkypg\/foo3rDExpJSsdOi45f4Jt7rDVB8FxpINPr3370530+VcHxCIqTjHCIoWngS5tXVst6tXTP9nayox2K9du2TLEjCHfgwcP0ocffkj8t+lTtmxZOu2004j\/jtuTZMfHgTU6LjkvgG322eq1KVyTDTsO0Huf76a3V+9UFQt7CFy6t9NChNerXHRKTWrVoLJKnguHxKHdyrVbiBlZtrG2DjEj6x50XHJ8wVae7fdlq9O2b4jWfX2AZi3Zqgq0vcTQtLZOocJRlXaNqxVFU5wRF1N7cUuHdivrkSSPaZGumUnlBt7dtGzZMnWNAUdgTjnlFDr11FPVbqY4P0l2fBy4o+OS8wLY2rPltSn8vLlqBy1etzsjIkULEY6msGDhu4JqVzkarc721I89yeA50W6DMwuSI8ljmqiY4QXBzz33HN122230\/fffF2POZ8zccccd1K1bN3XhZByfJDs+DrzRccl5AWx\/ZOs8kZboCP173R76x6odGRUpXBgLlY4n1aA2J1SmEwrKqzUqZzetJtcIctAy2q2s05I8pomKmTfffJOuvvpq6tq1Kw0YMIDq1KmjPLV161aaOHEizZs3j8aPH0\/nnHOOrActrSfZ8ZZIIs2GjitSnMWM5QtbvXi2QpmSNGvJl\/R\/X+xTHCTXpGjQzimfn9WrROe1KKAKZX5SFE3J1iJauVYlbzlf2q08Se8SkjymiYkZvtJg+PDh6hJJjsC4F\/jyIuERI0bQDz\/8oP6O41k0SXZ8tr5MznLRccl5IVfZFl80+y2t+eobeu6DL9Vx+dLrUbxESvPaFVQ0pUq5o9FjnFIr12bZcq62W1kq0VlP8pgmJmb0Nu2+ffvSJZdc4ukNjsxMmjQJW7Oja6s5ZQkdl5y74sTWLVA27fqW3lm9U02zZEqgaCHCf\/N0z2kNKqspnsplSwVeQBsntnItKDuWwVaWO8SMBd+dO3dS\/\/796dJLL6Xu3bt7Wpg5c6ZaU4OLJi0AJyALOi45J2aCrT6XhP9eunEPvbN6F\/ER+Zma5vGKpPzqpzXo58dXKYqiuNNEQTwTbKOoZy7aAFtZr0HMWPDlO5pGjx6tbtF+4IEHqH79+sWsbN68mYYMGaJ2NRUWFsZyEXCSHW\/h0sizoOOKHGmRQVu2eg0KGypVsgTNXLJV3d2TTYFySr1K1L5pNapavrSoSDH1hi1bU\/v5nA5sZb2f5DFNbJqJXbJmzRoVneEG2rlzZ2rbtq3y1OLFi2nOnDnqVtTHHnuMmjdvLutBS+tJdrwlkkizoeOKFGcxY5otn4XC3zM91cNnovx73W5aK3zCrNebORfMnlBQjk5vVJWa1qoQC4ESxBNot0FoBUsLtsF4BU2d5DFNVMww6A0bNtCoUaPojTfeII7WqF98pUrRBRdcoBYIN2zYMKg\/MpY+yY7PGMQ0BaHjsvOC8\/h7Pll2yfo96kyUTC6S1TV3ChT+b46inFa\/slqHktStx2i3du3WJBfYmlCyT5PkMU1czGjs3333He3bd3TbJO9wiuPuJXcTSbLj7b8O0eVEx\/UjS6dA2bb3O3Wi7Cdbj64\/yeQiWS7PKVCa1KpANSqWpl5n1ovF3T3RtT57S2i39uz8coKtH6Fwnyd5TBMTM7wAePbs2XTeeefRCSecEMtbsf2aRZId7\/fumfg8yR2X86A2vkV+yYY99OYnmTuoze0\/p0BpXLM8nVSnYrEISj6dMhu2bSe53YZlEzY\/2IYlmD5\/ksc0UTHD62Xef\/99aty4sTrpl9fN1K5dm0qUKCHrsYisJ9nxESEKZSaXOi7nwlge+P+5eict3bCHVm\/7JuOREw292KFt9SvRWSdWo+oVji6SPa4C0fbt26lWrVrUvB5OmQ3VUF2Zc6ndRvnembAFtrKUkzymiYkZdgmvkeHdTLNmzaJXXnmF+I4mXgTMwoYjNtWrV5f1XEjrSXZ8SDSRZM9mx6W3Fet7eHbs\/57+tXYXrdh8dCo001M7XKZTnPD\/n9uigC5rXVvVxfm5ycmy2WQbSeOIsRGwlXMO2MqxZctJHtNExYzTLW5hwycEt2\/fnn7\/+9+rv+N4P1OSHS\/7lTGzHnXH5Vx3whGTJet3kxYr2RAnboHy07oV6fiC8tSqwdELViWndqJma+bR\/EgFtnJ+Bls5thAzAmz3799PL774orqfqaCgACcACzDOBZPpOi535OT4gnK0cM0ueveznbQ+wyfHOlkW215co7y63fi4ymWKkjijJiYRFCk\/YVCQIosj9+XIgq0kW4iZiOhyJGbJkiVKxMydO1dZ7dixI\/Xs2ZPatWuHyExEnHPBjI6gVC97mKYuWE8rth1S\/s9G9MQpTngpV5uGVdSFgSX\/s65LMnoi6SuIGTm6YAu2cgRkLSd5tkF0msktYL7\/\/nu1ZubKK6+kDh06UOXKR8PtQR8+WZijOvqZPn26EkTpntWrV1Pv3r1py5YtxZJ16tRJnYNTvnz5Y7In2fFBmadL746irNi8l9Z8dYBWbd2fcYHijIbwHTztTqxKp59QhXbsP6Tu5IlL5CRK\/l62MODKEQZbsJUjIGs5yWOamJjRF00uX76cWrRoQb169VKRGN5dEeZhIcMOmTx5spqi0s4ZOXKkWlic6uF0Q4cOpSlTplCzZs2MqpBkxxsBcCTSW40PHzlC49\/amFGh4p7a6dCsOrVrXLXoVFv+PJtTOkFZZiI9Blw5ymALtnIEZC0neUwTEzO8c+nll1+ms88+m+rVqxfJdmwdXRk7dmyxSAwLHI64pIqwcPPgHVULFy5Mm8bdjJLseK+vjBYs63ccoGeWfEl89P27a3aJfLtYfPCi8LqVS9GZTWtSo5oVqHntiqqsXJ3aEQFlaRQDriU4g2xgawDJMgnYWoIzzJbkMU1MzBiyjSSZiZjhNPzwpZamT5Idzwx4p8+cD7fRyi\/2RyZatBD5ZZNq1PaEqlS2dMmU0zvouExbYvB0YBucmWkOsDUlFTwd2AZnFiRHkse0nBczHHG55ZZbKN26GT3lVaVKFXrnnXeKfD9gwIC04iZpjp+xeCtN\/\/cX1sLFGTE5u0l1OqvJ0cPYbCMp6LiCdEPB0oJtMF5BUoNtEFrB0oJtMF5BUydtTHO+f86KGe0Ufhk\/UaKnp3ixr47MaIHDF136LQAePHgwnXHGGUXc6tSpQ\/wnzs+WPYfUmpIXPvyKpv3ri0BVrVelFLU7oRJ1bV2bypQqQWc3lTnckDuurVu3qlOhvRZgB6o0EhcjALZyDQJswVaOQLSWuX\/lP\/rZvHkzDRs2LO2P\/2hrkDlrOStmNCLeMcW3b\/Pt3HpRsCk+P5XqFExOm7yYmXdkxfUZ\/95OmrJkt2\/1WLTw0\/Vnlem0OmXV+hX9b76ZI0hw8ODBoiP3y5YtG4FFmNAEwFauLYAt2MoRiNbytGnTaOrUqccYNdkBHG1N5K3lvJhhRDryMmjQoLQ7mtw4\/fJpMTNmzBiqX79+Ufa4RWb01ujTRy0mjsikeng6iNey\/HfrWmpqKJPCxatOLES\/\/PJLFeUqV66cfGvPoxLAVs7ZYAu2cgSiteyOzCxatIgefvhhRGaCYN63bx8988wzdO6559KJJ57omXXVqlX0xBNP0K233hrqniY\/UZKq3n75\/CI3QXhIpeVFvJ0fXZpWwMy+uhX9pGSJ2G1fxvy4VKvASapyZMEWbCUJyNrOhTHNloBYZEavSeE1Kl4H2h0+fJieeuopdRqw6fRQqrNi\/ByU6nO\/s2f87NpCD5uPIzG8ZXrgjJWepjjq8tyAltSkVoWwRYnmh5iRwwu2YCtHQM4y2q0cW7Yc1zEtireOVMzwuSEPPvggTZgwwbhuPXr0oDvuuINM1kzo9TFsXC\/a1dGVNm3apFzI67WuxmtRsLvScXQ8CxmOxOgzYZx1ZhEzvvvJdHbTo7uM4v6g45LzENiCrRwBOctot3JsIWYCsuXD655\/\/nnas2ePuoeJb8R2rjfR5ipVqkSnnHIKtWzZkipWPHpYmunjvs7Affovb9ceN27cMaf9+uWLs5hh8fL5Vweo64Rlx2BiETPn2taxm0by8yc6Lj9C9p+DrT07v5xg60fI\/nOwtWdnkjOOP9BN6m2SJtLIjLNAkzUzJhXMZpq4OD5VNCZXRYz2KTouudYNtmArR0DOMtqtHFtEZmTZxtp6HMQMC5lW97xXjFOuTSelcjI6LrnmD7ZgK0dAzjLarRxbiJkAbHU0hrNcfPHF6m6m3bvTn3dStWpVuvzyy4mnneL2ZFvM8Im97kW+uR6NcfoYHZdciwdbsJUjIGcZ7VaOLcRMALZ6BxNn4csg+ZZqvjU73cNrZkx3MwWoSiRJsylmvITMqC7NqH\/7BpG8WxyMoOOS8wLYgq0cATnLaLdybCFmZNnG2nq2xIxbyCRlWsntbHRccs0fbMFWjoCcZbRbObYQM7JsY209G2LGS8j0aFuHCi9qHGtWNpVDx2VDzSwP2JpxskkFtjbUzPKArRkn21TZGNNs6xo0n\/huJqyZMXeJe7FvUiMymgg6LvO2ETQl2AYlZp4ebM1ZBU0JtkGJBUsPMROMl0qt189gzYwZPK\/t14\/0OJk4KpPUBx2XnGfBFmzlCMhZRruVY8uWIWYi5HvkyBHatWsXvfbaazR79my67777Ut7dFGGxVqYy5XgWMtfNXEl8z5J+Ci9qlMipJacj0HFZNUujTGBrhMkqEdhaYTPKBLZGmKwTZWpMs65giIxi00x+dWJRM378eHVSMN\/fVKpUKb8sGf88U44fPX8tjZ6\/ruj9+p3dgEZ3bZbx9810gei45IiDLdjKEZCzjHYrxxaRGUG2HJ157LHH8nprttc6mWW3nSlIPT6m0XHJ+QJswVaOgJxltFs5thAzQmz1pZQfffSRitBUqVJFqCR7s5mIzHR6ZKm6AZufpC\/4dXsCHZd92\/TLCbZ+hOw\/B1t7dn45wdaPULjPMzGmhauhfW6xaSZ9GnCq3Uyff\/45zZ8\/n2688Ua6+uqrqUSJEvZvIZRT2vHuqEzSF\/xCzAg1VA+zGBTkWIMt2MoRkLUsPabJ1j69dTEx47ebqXLlytS7d2\/q27dvLK8ykA7J5fP0km6SGBTkvvpgC7ZyBOQso93KsZUe02Rr7m9dTMz4Fx3\/FJIqduizn9KTCzcrCEm6bymIV9FxBaEVLC3YBuMVJDXYBqEVLC3YBuMVNLXkmBa0LlGnz7iY4V1M33zzDVWoUCGWU0tOwJKOL7jpH0VFXX9eQ\/pTpyZR+zb29tBxybkIbMFWjoCcZbRbObaIzIRgy4t8p02bRitXrqQRI0ZQ+fLlidfS3HTTTbRx40YaPnw4dejQIbaiRkrMuBf9zrm2tYrO5NuDjkvO42ALtnIE5Cyj3cqxhZgJwfb555+nW2+9Va2Lufbaa5WYOXjwoDqFkEXOkiVL6JFHHqGzzz47RClyWSXEjHutTD4cjpfKQ+i45Nou2IKtHAE5y2i3cmwhZizZ6ghMixYtaPDgwcccisdRGz799+uvv6ZRo0YpoRO3R0LMuKMy+XKmjJdv0XHJtXiwBVs5AnKW0W7l2ELMWLLVu5muueYauvDCCz2tvPnmmzRu3Li8OTSPryvo\/OjSIhZ87xJvx87XBx2XnOfBFmzlCMhZRruVYwsxY8mWrym47rrr6Nxzz6WrrrrK08qTTz5Jb731Vt4cmvfbR5fSO\/+5f4nXyORzVIYbBDouyy+XQTawNYBkmQRsLcEZZANbA0ghkkjMNoSoTqRZxXYz8a6lCRMm0LPPPkv3338\/\/fznPy9a6MufffDBB3TzzTfTZZddlheH5rnXyozrfhJd8Yu6kToz14yh45LzGNiCrRwBOctot3JsEZkJwXbbtm00dOhQWrBgAdWvX59q1qyprHHUZu3atWrh79ixY+m4444LUYpc1ihV7D9W7aBLJy5Xlc3Xc2XcnkLHJdd2wRZs5QjIWUa7lWMLMROSLe9e4gsleWfTrl1H7yCqVq0ade3aVa2lKVu2bMgS5LJHJWbcUZmOJxXQ3\/q3lKt4jlhGxyXnKLAFWzkCcpbRbuXYQsxYsuWppLfffpuaNGlCDRo0sLSS3WxSYobPlTm7abXsvlwMSkfHJecEsAVbOQJyltFu5dhCzFiy3blzJ\/Xv31+tienWrZullexmi0rMYDu2tx\/Rccm1b7AFWzkCcpbRbuXYQsxYstW7mS655JK8FzPOqwvy+ZA8d1NCx2X55TLIBrYGkCyTgK0lOINsYGsAKUSSqH6gh6iCWFax3Uxc40WLFqmD8Xr06EHt27f3PBivRIkSVLVqVSpZsqTYS9oajsLxo+evpdHz16kqYDt2cU+g47Jtmf75wNafkW0KsLUl558PbP0ZhUkRxZgWpnzJvGJiRh+at3z50R08qZ6WLVsm+tA85xTTL5tUo7kDW0v6M6dso+OScxfYgq0cATnLaLdybNkyxIwFX97F9OGHH6q7mNI9vJvptNNOi+WuprCOd+9iwsJfRGYsvkpWWTAoWGEzygS2RpisEoGtFTbjTGHHNOOCspBQLDKThXeJvMiwjn\/r0x3UdcKPZ8vk+4m\/bgeh44q8yRYZBFuwlSMgZxntVo4tIjOybGNtPayYcU4x\/erkGjSr32mxft9MVw4dlxxxsAVbOQJyltFu5dhCzARgq9fJcBY+2ZdP\/83XNTOYYvJvOOi4\/BnZpgBbW3L++cDWn5FtCrC1JWeWL+wPdLNSspMq0mmmffv20TPPPKPe5OKLL6aXX36Zdu\/enfbNeCfT5ZdfTpUqVcoOgTSlhnH8jMVbaeCMlco6djF5Q0bHJdfkwRZs5QjIWUa7lWOLyIws21hbDyNmnFNMfNovL\/7FU5wAOi65FgG2YCtHQM4y2q0cW4iZEGz5SoN3332XlixZQtdcc43asbR\/\/3664447qHTp0nTttddSw4YNQ5Qgm9VWzGCKycwv6LjMONmkAlsbamZ5wNaMk00qsLWhZp7HdkwzLyF7KSOdZnK\/Bt+WPXDgQOrQoQPde++9VKVKFeKpqClTptDMmTOpXLly9Nhjj1Hz5s2zRyBNybaOd04xsfkdD54Xy\/fLdqXQccl5AGzBVo6AnGW0Wzm2iMxYsj1w4AANHz5cnfp75513HnP6L193cOutt1LFihVpxIgRVKZMGcuS5LLZihkclGfmE3RcZpxsUoGtDTWzPGBrxskmFdjaUDPPYzummZeQvZRikRm9s6lv377E9zN5PfPmzaNJkyYl6gRg9xTTIz1Oph5t62TPwzEuGR2XnHPAFmzlCMhZRruVY4vIjCVbfWv2pZdeSt27d\/e0MmvWLHr22Wfp8ccfp+rVq1uWJJfNRsUu+GwXdX50aVGlcOpvav+g45Jru2ALtnIE5Cyj3cqxhZixZHvo0CEaPXo0rVixgh544AGqX79+MUubN2+mIUOG0KmnnkqFhYVUqlQpy5LkstmIGVwsae4PdFzmrIKmBNugxMzTg605q6ApwTYosWDpbca0YCVkL7XYNBO\/0po1a6h\/\/\/701VdfUceOHen4449Xb8r\/\/9JLL1HNmjVVVKZJkybZI5CmZBvHY72MuSvRcZmzCpoSbIMSM08PtuasgqYE26DEgqW3GdOClZC91KJihl9r+\/btasfSc889R3v37lVvWrlyZeLpJ96uXatWrey9vU\/JNo4vuOkfRVYLL2pEhRc1ju37Zbti6LjkPAC2YCtHQM4y2q0cW7ZsM6bJ1ig66+JiJrqqZt5SUMdjvUwwH6HjCsYrSGqwDUIrWFqwDcYrSGqwDUIreNqgY1rwErKXA2ImDfugjv\/npzupy4RlyiKuMPBv1Oi4\/BnZpgBbW3L++cDWn5FtCrC1JWeWL+iYZmY1HqkgZiIUM871Muc0q06zr2kVDy\/HtBbouOQcA7ZgK0dAzjLarRxbtgwxI8s3ttaDON59vgzWy\/i7FR2XPyPbFGBrS84\/H9j6M7JNAba25MzyBRnTzCzGJxUiMxFFZrBeJnijRscVnJlpDrA1JRU8HdgGZ2aaA2xNSdmlg5ix45bzuYI43nkfE9bLmLkeHZcZJ5tUYGtDzSwP2JpxskkFtjbUzPMEGdPMrcYjJSIzEUVmcL5M8AaNjis4M9McYGtKKng6sA3OzDQH2JqSsksHMWPHTeXik34XLlxImzZt8rRStWpVuvzyy6lSpUohSpHJGsTxre55j3jdDD9YL2PmD3RcZpxsUoGtDTWzPGBrxskmFdjaUDPPE2RMM7caj5SikZkFCxbQwIEDiw7L83rlli1b5vxFk+7Fv7iPyaxxo+My42STCmxtqJnlAVszTjapwNaGmnkeiBlzVkUpDxw4QMOHD6ctW7aoO5oaN25MJUqUsLCUvSymjncv\/t3x4HnZq3QOlYyOS85ZYAu2cgTkLKPdyrFly6ZjmmwtZKyLRWZ27NhBffr0oSuvvJK6dOkiU\/uAVllUTZw4sSjX9OnTqV27dimtmDoel0sGdMR\/kqPjsuNmkgtsTSjZpQFbO24mucDWhJJ9GtMxzb6E7OUUEzP79u2jm266iX7zm9\/EQsywkGFHTp48mQoKCooU6siRI6lbt26eHjB1PBb\/2jVgdFx23Exyga0JJbs0YGvHzSQX2JpQsk9jOqbZl5C9nGJihl\/p+eefp1dffZXGjBlDvNA3W8\/q1aupd+\/eNHbs2GKRGBY4PA02atQoKl++\/DHVM3U8Lpe08yw6LjtuJrnA1oSSXRqwteNmkgtsTSjZpzEd0+xLyF5OMTFz8OBBev\/994mncpYuXUoXXXSRui3b\/WRzN1MUYgaLf+0bLzoue3Z+OcHWj5D952Brz84vJ9j6EQr3OcSMBT+9Zmb58uVpc2drN9OsWbPolltuUWIr1boZ7fjBgwfTGWecUfQederUIf7Dz7\/W7aOuj68o+mzLfWda0MrPLNxxbd26lWrXru0ZGctPKtG8NdhGw9HLCtiCrRyBaC1z\/8p\/9MNHpQwbNiztuBdtDTJnTSwyk7lXCFaSFiica8CAAVRYWJjSgDOtM1GvXr3UwmZ+Zn+8l+5582v13\/WqlKK5vRoEq1Aep+bo3fbt26lWrVpUtmzZPCYR\/auDbfRMtUWwBVs5AtFanjZtGk2dOvUYo36bX6KtRWas5Z2Y0Vj11vENGzakPOdGixle81O\/fv0ijzgjM10mrqBF6\/epzzq2qE5\/6XVSZjyXgFLYB19++aWKcpUrVy4BbxSfVwBbOV+ALdjKEYjWsjsys2jRInr44YcRmbHBzGGtcePG0SuvvEInnniiEg78K\/z++++n888\/nzp06JC182f0wuBBgwZ57mgymV\/E4l+bVnE0D+bH7dn55QRbP0L2n4OtPTu\/nGDrRyjc5yZjWrgSspdbNDKzZs0a6t+\/P5UpU4aaNm2qrjRgMVO6dGkaMmQIsUrkc1\/SnfUiiSasmMHi33DeQccVjl+63GALtnIE5Cyj3cqxZcsQMxZ8Dx06pE7+Xb9+vdqavXLlSvX\/+pwXPoeGF+BWrFiRRowYoQSP1MMOHDp0KE2ZMoWaNWtWVIyfY\/0+d5\/8u+y2M4lvzMZjRgAdlxknm1Rga0PNLA\/YmnGySQW2NtTM8\/iNaeaW4pdSLDKzc+dOFZW57LLL1BQOQ3SKGUYxe\/Zs4gVKWuBI4dHrY9i+PlNGR2XatGljfc6MU8ywiGExg8ecADouc1ZBU4JtUGLm6cHWnFXQlGAblFiw9BAzwXip1Hprdt++femSSy7xFDPz5s2jSZMmiYsZXX33dQbpTv\/lPH6Ox8m\/Fg3DkQUdVzh+6XKDLdjKEZCzjHYrx9ZkTJMtXda6WGRGR0Nq1KhBf\/zjH2nJkiWe00w8vXTffffFcmtuEDFTeFEjKryosay3EmYdHZecQ8EWbOUIyFlGu5VjCzETgu2CBQvoxhtvpH79+tHxxx9Pjz76KP35z39W23GfeOIJeuedd2jChAlqV1Mcn3Rixr34F+tlgnsQHVdwZqY5wNaUVPB0YBucmWkOsDUlZZfO7we6ndV45BKLzPDrHTlyhF566SW1wPfrr48eLKcf3tF0zz330KWXXkolS5aMBw1XLYKImTnXtqazm1aL5XvEtVLouOQ8A7ZgK0dAzjLarRxbRGYiYMs7l3g307Jly4hPz+QrDFq1auV5V1MExUVmIoiY2fHgeZGVmy+G0HHJeRpswVaOgJxltFs5thAzsmxjbT2dmBk9fy2Nnr9O1R87mezciI7LjptJLrA1oWSXBmztuJnkAlsTSvZpMM1kz05NNW3cuJHmz59PHKHhhxcFd+zYsdgVASGKEMuazvHYyRQeOzqu8AxTWQBbsJUjIGcZ7VaOLSIzIdiyePnf\/\/1f+stf\/kJ8iJ7zKVWqFA0cOFD94f+O42MqZsZ3P4l6\/qJuHF8h1nVCxyXnHrAFWzkCcpbRbuXYQsxYsuWIDB+I9+CDD6qt2XzWTKVKlZQ1FjkvvPCC2qp91113UdeuXS1Lkc2WSsy4dzI90uNk6tG2jmxlEmgdHZecU8EWbOUIyFlGu5VjCzFjyXbPnj103XXXUefhX\/8AABuLSURBVNu2bdXfJUqUKGaJxc748eNpxYoVSvBooWNZnEi2dJEZ5wWT2Mlkhx8dlx03k1xga0LJLg3Y2nEzyQW2JpTs02DNjAU7fQLwNddcQxdeeKGnBT5nhoWM9HUGFtVXWVI53n0nE3Yy2RFGx2XHzSQX2JpQsksDtnbcTHKBrQkl+zQQMxbsvvvuO3WODK+H4Wkm97oYXkPDQoannG677TbRiyYtqp9WzGAnky3R4vnQcUXD0csK2IKtHAE5y2i3cmzT\/UCXLTUz1kUPzdu2bRsNGzaMGjVqpC6drFu3rppu4gP0ZsyYQS+++CLde++9xW6y5s+rV6+embf3KSWVih3y7CqasnCLyv3LJtVo7sDWsahvrlUCHZecx8AWbOUIyFlGu5VjCzFjyVZPMy1fvjyQBT5Qj2\/TjsOTSsw4t2XjTiZ7T6HjsmfnlxNs\/QjZfw629uz8coKtH6Fwn2OayYIfn\/T74YcfqhN\/gzxly5ZVi4bj8KRyvHPxL8SMvafQcdmz88sJtn6E7D8HW3t2fjnB1o9QuM8hZsLxy9ncJmIGO5ns3YuOy56dX06w9SNk\/znY2rPzywm2foTCfQ4xE44f7d27V93LxFNOHHk55ZRT6NRTT83Ju5ncO5kgZuwbBzoue3Z+OcHWj5D952Brz84vJ9j6EQr3OcSMJb\/Dhw\/Tc889p3Yrff\/998Ws8K3Zd9xxB3Xr1i2nTgB2ihncyWTZMP6TDR1XOH7pcoMt2MoRkLOMdivHli1DzFjyffPNN+nqq69WJ\/wOGDCA6tQ5ekru1q1baeLEiTRv3jx1cN4555xjWYJsNi\/Hz1i8lQbOWKkKhpgJxx8dVzh+EDNy\/MAWbLNDQLZUiBkLvgcOHKDhw4erk305AsPTS86HFwaPGDGCfvjhB\/V3mTJlLEqRzeLleFwwGR1ziJnoWLotgS3YyhGQs4x2K8cWkRlLtnprdt++fdW9TF4PR2YmTZqUUycAY1u2ZYPwyIaOKzqWEDNyLMEWbDNHQLYkRGYs+O7cuVMdlHfppZdS9+7dPS3MnDlTral5\/PHHY3NQnrOiXo5vdc97xBdN8oNt2RYNw5EFYiYcv3S5wRZs5QjIWUa7lWOLyIwlW76ugG\/F5oskH3jgAapfv34xS5s3b6YhQ4aoXU2FhYWxXATsFjPu27KX3XamWjeDx44AOi47bia5wNaEkl0asLXjZpILbE0o2adBZMaS3Zo1a1R0hhto586diw7DW7x4Mc2ZM4fKlStHjz32GDVv3tyyBNlsbsdjW3a0vNFxRcvTaQ1swVaOgJxltFs5tojMhGS7YcMGGjVqFL3xxhvE0Rp++NLJCy64QC0QbtiwYcgS5LL7iRnclh2OPTqucPzS5QZbsJUjIGcZ7VaOLcRMRGz5Fm2+IZsf3uEUx91L7ld1ixlsy46oMfzHDDquaHkiMiPHE2zBNjMEZEvBNJMs39hadzuez5dhQcMPbssO7zaImfAMU1kAW7CVIyBnGe1Wji0iM7JsY23dLWZwxky07kLHFS1PRA\/keIIt2GaGgGwpiMzI8o2t9XRiBtuyw7sNYiY8Q0Rm5BiCLdhmnoBsiRAzsnxja93t+IKb\/lFU10d6nEw92h69ngGPHQGIGTtuJrnA1oSSXRqwteNmkgtsTSjZp4GYsWeX0zmdjq\/XvBXxgXn6wW3Z4V2Ljis8Q0QP5BiCLdhmnoBsiRAzsnxja93p+EM1T6LOjy4tqisOzAvvNoiZ8Awx4MoxBFuwzTwB2RIhZmT5xtZ6OjGDM2bCuw1iJjxDDLhyDMEWbDNPQLZEiBlZvrG17nT8P3fXptHz16m68hUGHJnBE44AxEw4fulygy3YyhGQs4x2K8eWLUPMyPKNrXWn4\/+ytirOmInYU+i4IgbqMAe2YCtHQM4y2q0cW4gZWbaxtu4UM\/e+X5beXbNL1RcH5kXjNnRc0XD0sgK2YCtHQM4y2q0cW4gZWbaxtp5KzOCMmWjcho4rGo4QM3IcwRZsM0tAtjRMM8nyja11p+MvfuZAUT1xxkw0LoOYiYYjBlw5jmALtpklIFsaxIws39ha144f\/ciTNOD1I0X1xBkz0bgMYiYajhhw5TiCLdhmloBsaRAzsnxjax1iRtY1EDNyfMEWbOUIyFlGu5Vjy5YhZmT5xta6dvzAux8lXgCsH5wxE43L0HFFwxHRAzmOYAu2mSUgWxrEjCzf2FrXju9044PEW7P5wRkz0bkLYiY6lm5LYAu2cgTkLKPdyrFFZEaWbaytazHTstd9xIfmQcxE6y50XNHydFoDW7CVIyBnGe1Wji3EjCzbWFvXYqZu1z\/RysPHq7rijJnoXIaOKzqWiMzIsQRbsM0cAdmSMM0kyze21rXj9519Mx2q2ULVs0fbOsRbs\/GEJwAxE55hKgtgC7ZyBOQso93KsUVkRpZtrK17iRkcmBedy9BxRccS0QM5lmALtpkjIFsSIjOyfGNrXTt+1+8mF9URB+ZF5y6ImehYYsCVYwm2YJs5ArIlQczI8o2tdXZ8977X054LRxfVEQfmRecuiJnoWGLAlWMJtmCbOQKyJUHMyPKNrXWIGVnXQMzI8QVbsJUjIGcZ7VaOLVuGmJHlG1vrXtNMy247U501gyc8AXRc4RmmsgC2YCtHQM4y2q0cW4gZWbaxtu4WMzgwL1p3oeOKlqfTGtiCrRwBOctot3JsIWZk2cbaunuaCWImWneh44qWJ8SMHE+wBdvMEJAtBdNMsnxja90dmcGBedG6CmImWp4YcOV4gi3YZoaAbCkQM7J8A1k\/cOAADR8+nObOnVuUb+TIkdStW7e0dlavXk29e\/emLVu2FEvXqVMnGjVqFJUvX\/6Y\/BAzgVwTODHETGBkxhnA1hhV4IRgGxiZcQawNUZllRBixgpb9Jl27NhBffr0oYYNGxYJEC1SWJQUFhamLJSdOHToUJoyZQo1a9bMqHJuMYMD84ywGSdCx2WMKnBCsA2MzDgD2BqjCpwQbAMjC5QBYiYQLrnEqQTJrFmzaObMmTR58mQqKCjwrACnWbhwYcoojFcmiBk5X7JldFxyfMEWbOUIyFlGu5Vjy5YhZmT5hrbOQmXcuHFpoy6jRx89+C5d9MZdEbeYwem\/oV1VzAA6rmh5Oq2BLdjKEZCzjHYrxxZiRpZtJNZZqLDwSBWZ0dNTVapUoXfeeaeozAEDBvhOTfXs2ZP0dQY4\/TcSdxUZQccVLU+IGTmeYAu2mSEgWwoiM7J8Q1nXzkm3CNhrXY3X+hu\/yMytbQ4S72iqU6eO+oMnHAEWM1u3bqXatWt7LsAOZz2\/c4OtnP\/BFmzlCERrmftX\/qOfzZs307Bhw2j69OnUrl27aAvLsrUSR44cOZLlOlgXr0VKmzZtAq2F0QX6qVT3NFO1F\/qorL169aIrr7zSut7IeJTAwYMHafv27VSrVi0qW7YssERIAGwjhOkyBbZgK0cgWsvTpk2jqVOnHmMUYiZazqGshRUyXLi2MWjQIM+t3U4xwwfmTbighKozIjOhXFeUed26deqLxuKwUaNG0RiFFUUAbOUaAtiCrRyBaC27IzOLFi2ihx9+GJGZaDHbW9MiI90ZMSbWg4oZvpcJT3QE\/CJj0ZWUf5bAVs7nYAu2cgRkLSe57ebcNJN2ht\/iXWeTSOVAv7NnnJEZnP4b\/ZcsyV+s6GkFswi2wXgFSQ22QWgFSwu2wXgFTZ1kvjklZkwPyHM7WJ8avGHDhqIdTya2nGKmUbl99GjnWkHbDtKnIaAXow0ePJjOOOMMsIqQANhGCNNlCmzBVo6ArGUsAJbla2ydt2BPnDgxZXq9qCnVuTPu\/H7XIGzatEmt\/J5ftz+V2fAuVfjgSeO6IiEIgAAIgAAIxI0A\/3AcM2YMNWjQIG5VC1WfnIrMhHpTy8wsaPgPHhAAARAAARDIdQIsYpImZNgnEDO53jJRfxAAARAAARDIcwIQM3neAPD6IAACIAACIJDrBCBmct2DqD8IgAAIgAAI5DkBiJk8bwB4fRAAARAAARDIdQIQM7nuQdQfBEAABEAABPKcAMRMnjcAvD4IgAAIgAAI5DoBiJlc9yDqDwIgAAIgAAJ5TgBiJs8bAF4fBEAABEAABHKdAMRMrnsQ9QcBEAABEACBPCcAMZOiAej7nObOnatStGzZsuhepzxvM8av77w+ol69ejRlyhRq1qxZ2vz6PiydCNy9cdmwdVrasWMH9enTh7p3707dunUz9mm+JLTh6+4zmJW+YiVfuJm8pw1b3V6XL1+O\/tgEcpo0zJ+fwsLCkJbilR1ixsMfulPiAVg7nBsAD7STJ0+mgoKCeHkxhrVhXlu2bKFRo0ZR+fLlKdV9Wc6qc5pbbrml2ADAdlhQmgihGGIQqZINW3dF9IDidz+ZyAvE3KgNXz3YNmzYsFibd7fnmL+6ePXCsG3Xrh3645Ae0n3sgAEDIGZCssyJ7F4DL37JmrvO65p5L4HotJjqc3Avzt2GrdtzzugXxEw0fLnPmDlzZrEfO35t3vwblYyUtm3Xqz9evXo19e7dmwYNGoTIokHzcEcNIWYMoCUhifvXg36nVP+ehHeO8h28Ona2n+rf05WtxYzzV1mUdc01W2HZap79+vWjJ554AtNMrgZgw1cPFGeddRYG1jRfKBu2ut8YN25csegsxIx5z6Xb54YNG2j8+PF0\/\/33k3PWwdxSvFNimsnln3S\/pjDVZNaYU4k+k6kmdwnotIoTCcPW2bZZzGDNzLHt2YavFog8Jb1+\/Xo1VcqP6Toxs29V7qeyYctv7fWDBtPPdu0hydFCiJkAYsYmsmDX5HI7V6pOi8PMQ4cONV7\/4vxFgbVKR9tEGLbO9su2IGbMxUy6tqsFN68Rc4bvvdaA5fY3O1ztw7Rd3fYnTpyoKtGpU6eitUnhapVfuSFm8sjf6ZwNMWPWEMJ2WroUvUgVO0J+5G7LVg+4Y8eOJZ6yw1ok77Zsw1ezbdOmTbEBVvclXJJeCG\/2DUpmKhu2TEKvtXGu74JQtGsjEDN23HIyF6aZwrvNNpzsLBlCJthgm24Kz6tNQ8xEx1eLGY4WuLe74geQvxA3abteghBrGIP31RAzwZnldA4sAA7nPtuFflyqc9U9IjLH+sGGrXMaxMuzOMvnRyo2fNMtUoeYCccWkfJwfbE7N8RMtDxjb82rA8IvWXO3ea0vMPkS6TTvv\/++8boa81olI6UtW\/fboz17twdbvl6bA0zafDJapdlb2LBNN1WHyIwZd2eqJLdJLAD2aA9eB2BhJ5P5F8dr4a7JTibsUPBnbMsWYsafrTMyyNtY9aJzk7brNdVkks+sVslIZdt2sWYmOv9DzETHMmcs4TqD8K7S617Yktc2VadA\/Prrr9UhWLwjxOvB7oXiVIKw9TqxGpGZ9O3bhq\/7yH1szfZmbMPWPVUKtnb9M8SMHTfkAgEQAAEQAAEQAAFxAphmEkeMAkAABEAABEAABCQJQMxI0oVtEAABEAABEAABcQIQM+KIUQAIgAAIgAAIgIAkAYgZSbqwDQIgAAIgAAIgIE4AYkYcMQoAARAAARAAARCQJAAxI0kXtkEABEAABEAABMQJQMyII0YBIAACIAACIAACkgQgZiTpwjYIgAAIgAAIgIA4AYgZccQoAARAAARAAARAQJIAxIwkXdgGASI6fPgw\/fOf\/6Rvv\/2WfvOb3wRmsn37dnr22Wepe\/fuVL169cD5w2RYvHgxjRgxgj7++GPi27Uff\/xxqlWrVmCT6W6WDmwsBhncVxeMHDmSunXrlpWauY\/6x23zWXEDCs0yAYiZLDsAxSefQNh7kLxucc8Etf3799Pw4cNp06ZN1L9\/f6pZsyaddtppVLZs2cDFJ1XMNGjQQImYE044gY4\/\/vjAXKLIsGfPHvq\/\/\/s\/JTjvvfdegpiJgips5BoBiJlc8xjqm3MEclXMRClAorQVhwYQx\/fRt0tDzMShhaAOmSYAMZNp4igvUQSOHDlCb7\/9Nj388MP00UcfUfny5emcc86hoUOHUsOGDSndbb98g+2MGTPUFNInn3xCpUqVop\/97Gc0ePBg6tChA5UoUYKcNwwzuAEDBlBhYaFiuGHDBhozZgy99tpr6v87duxI1113Hf30pz\/1ZcxTV9OmTaOZM2cS31h+yimnqOgLT4NxPfTA6DSUbiqFObz77rv0yCOP0Pvvv684sK1BgwZR\/fr1SQ\/+bdu2pdatW9Of\/\/xnWrVqlSqXWen35fIOHTpEr7zyCk2ZMkUx5f8\/6aSTqG\/fvtS5c2dVP350xOqqq66iUaNG0XfffUf3338\/nXfeebR582YaN24czZ49m8qVK0dXX321snH77bcru82aNVM2Dh48SHPnzqXHHnuM1q5dS40bN6YrrriCevTood4h1eMlZvSNxHXr1qUzzzxT1WnNmjXUrl07GjJkiIpqsU9TPakicKaROYgZ32aPBAkmADGTYOfi1eQJLFiwgAYOHEi\/+93vlIjhkD+vKyldujRNnDiRKlSooIQBixIWG+eff74SG\/zv9913nxpIe\/XqRa1atVLTOSwweCB+6qmn1KDPA\/5zzz1Hf\/\/739Wgf+KJJ1KLFi2UTRYKPED+93\/\/t3rRv\/3tb7RkyRIlKM4+++yUL79t2za6\/vrrVXk8yNerV49efvllmjNnDl1zzTXK7q5du4jXy4wdO1bVN91UCgsZFmS33nortW\/fXtWHBdKkSZOIB3YWej\/5yU+oT58+SrSdfPLJdOWVV6rpKn7PdevWqb\/539kWM2A2vXv3VqKAmXK0gUXSQw89RBdffHGRmGFxwmKJ61yyZEn6xS9+ocQOv9+XX36pBFDFihVp6tSptHfvXrVuSYsZFh+8HuiNN94o8sGyZctUWhZXd911F1WqVMmTYzoxw\/Vk\/\/\/P\/\/yPYst1X758uWoPLGwgZuS\/lygh\/whAzOSfz\/HGERJgkcKigH+F86DJDw9cN910k1q\/wIOX1zQTR1WGDRtGXbt2LbZwdOXKlfSHP\/xB5dcLSt2\/zPft26eETe3atemPf\/xj0RoWjmCwCNiyZYsSIV4DMYuF8ePHq6jG5MmTlTDihxcps6DgAVcLC9OplK1bt6qIUZs2bVR9dOSEoyo33HCDEjm8eJjFDEdPWOyxAOFHv++NN96oFjizcOF1Ohw5YYGibekyzjrrrKLIFL\/DLbfcQhMmTKALL7xQ2dPv98ILL6hoS\/PmzdW\/OwWcFjO8KJvr++ijj6r66Yfrze\/DLFmgej3pxMxbb71VTFCyiOLIDIutVH7hMhCZifCLCVN5RwBiJu9cjheOkgBHER544AElTC655BLP3UZB1szotJ06dSKePvEa5D799FMVteAIyOmnn17sdXjKi6MszqkUZwIWCzwVxdMpHNXQYoHTcISEBQdPs3DZpmKGIzgcaeEyU0UenNNMLED0dItpGXoKp1q1akX15sGfp5Kc75ru\/XhKjYUcp2\/atCnxtNl7772nRBNHyvTD4oNFBwskPaXnbjPpxAynZXHrnKbiurJoYqHI7L0eiJkov5mwlW8EIGbyzeN430gJ8NoTno5gAcEPr8tgIcJrO3T0IZ2Y4UgFR1I+\/\/xz+vDDD9V0Eu9Kca6NcQ9yXutZnC\/FAuXpp58mXp+SahB2iiWdxj1AmwoNXpfCgi6VgGL7qWyl+neOMvE0Ea9j4UgJR1F4+obX4Wih4CVm2B88tcRrXjjS43yYG0e0uJ68C4kjQDzNl+rp0qWLis547d5KJ2Z4asktgkzWs0DMRPrVhLE8IwAxk2cOx+tGT4CnNliQvPrqqzRv3ryihcB67YqXmOHBmtdm8ILV77\/\/XgkfjhbwdAcPsM6oQCoxY7NrxSvyE1bMeIkKk0iGl8hhli+99JJay8LrbmrUqKG48FoYFgR16tRJK2a++OILFV3iSJGJmPGKopi0EBsxw3VKJTK5TIgZE\/JIAwLeBCBm0DJAIGICGzduVGtemjRpogZlXuPCAywPrnodDEcZeG0MrxVx7pzRgzGv1dC\/7t2DHEcqeJqJF+vqqSjTVzCZZurXr5+qq2lkJtU0086dO9UUzq9\/\/Wu1aJcZ8DSUM2rhLkNPdfH7M5vKlSurV2OGzJSnbtJFZtK9n1N08VTP3XffrRY589ohXqgc5EknZriO7PcyZcoUmeSyn3jiCRUVSnUeTao0HPXiheZcz4KCgpTVNIn+BHlHpAWBXCIAMZNL3kJdY0WAD5XjaYgffvhBTTXp6QgeUHng5UPmUokZHrh4ezIPbnqRKkclXn\/9dbWmhQf+VGJGLwDmyAWvGeFoBT8c7XnwwQfVAM27fvQ0lxOa3wJg3nnE64A4QmQqZlItAOapId5VxIuK+R1NxIwekLkOzh1ZvKiat46zGEonZlItANaLcPlwOecCYBZuLGouv\/zyonU8vIOMd3mxYORoitfjt5vJubiaRR1vt2c\/cVmpDh3kqB5PffG6Gl5MzQ+zZUHIvoWYidXXH5WJGQGImZg5BNXJLQLPP\/+82q3D0YeLLrpIVZ4HJV6Iq7fi6mgBf8YDOm+55vNHODLDa2z4bx7g+LyY+fPnq4Hr97\/\/fZGY0YMcD2ocseAdSHpLOP9SZ8HA0QDOy1NdLKAuu+yylGeamGzN5nU3pmLGuTX7v\/7rv4j\/cISFdxNxfXkA5wW8JmKG87Fo4ak3fi9+PxY4vPWb1xede+65acUMM+bo1rXXXqu2l\/P6GbbBAoGFTNWqVYvEDIvCO++8U03r8WJqntr77LPP6K9\/\/avaKcZi87jjjgssZtgenwjsLPurr74qtrtKizbn2T363Zk9M+CHRR1PtXF+LWa0X\/gcI+dCY0RmcqvvQG2jJQAxEy1PWMszArylmUUETyHw9A8\/7kPSeLDnBcIcxeEFqrxugnchseDhXTO84JcHLBYBHAlgEcQDsd7Gy3lYoHA5vHBXL0rlwZl35\/CiYS6DD9zTgoe3Aad7vA7N47x8Do7e4WQqZrgc96F5LBp4CoyFGgutIAuAV6xYoQ4D5MFZH0LIZ\/lw1Ir\/MB+OcqRbq8Nn9fBAzwJRH5rHO5b0VI8+NE8fXMg+4cXG7Ac+M4jFRLo7qNJFZrhNcFSJt6Dztn0WSbzbjcWHfrzEDH\/mfHeejuL3ZkHFU00QM3nWueB1AxGAmAmEC4lBAARylQBvzeYTl\/kwP5vLMp3vHXRrdiaYITKTCcooI64EIGbi6hnUCwRAIDABnori9UscXeL1J\/qsF55S4vNtOGLlPgMmcCEptprrs3DYXhRlBK0XxExQYkifJAIQM0nyJt4FBEBATenxibt8iCHvomKBY3rVgyk+HZlx3prNC75ZQGVazODWbFOvIV2SCUDMJNm7eDcQyEMCvGaFd1LxTi9ex8SnDfO9WLwomC+2THfZoykuLWZ4lxU\/vJCXD0rMhphxX2Zqc\/6Q6XsjHQjElcD\/A1EjfgOtSseuAAAAAElFTkSuQmCC","height":271,"width":451}}
%---
%[output:2330891c]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"13.2000"}}
%---
%[output:55c0aace]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"0.0075"}}
%---
%[output:6d4591c7]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"0.0075"}}
%---
