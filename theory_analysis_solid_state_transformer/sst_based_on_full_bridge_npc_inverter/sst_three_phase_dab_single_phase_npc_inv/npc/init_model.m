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
rpi_ccaller = 0;
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
Ls = (Vdab1_dc_nom^2/fPWM_DAB/Pnom/3) %[output:6e4cc00d]
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
iph_grid_pu_ref = 2.75;
%[text] ### DAB Control parameters
kp_i_dab = 0.25;
ki_i_dab = 18;
kp_v_dab = 0.25;
ki_v_dab = 45;

%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
Vac_FS = V_phase_normalization_factor %[output:1bc16b1f]
Iac_FS = I_phase_normalization_factor %[output:9a535e97]

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
%   data: {"layout":"onright","rightPanelPercent":36.1}
%---
%[output:1dd96712]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"   275"}}
%---
%[output:174c5e41]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"        1875"}}
%---
%[output:6e4cc00d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     7.891414141414142e-05"}}
%---
%[output:01f62bda]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     1.393166275082144e-05"}}
%---
%[output:1bc16b1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     5.633826408401311e+02"}}
%---
%[output:9a535e97]
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUEAAADCCAYAAADXXXDzAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ1wV9WVP360JbRSDeBWJoSggrV1S9XdTmRLo6aVutOAszvdQLotRaeLC+J2G5qQ2B2HERAYbBWpI9U0i7tDcFuhkJ3dQTZaWxuxbgXWWgUlZGMaRQhFqMVureych\/fvzc37uPe9+\/VezpvpVPK\/99x7fuec3zv3851x6tSpU0APIUAIEAIjFIEziARHqOVJbUKAEAgQIBIkRyAECIERjQCRoGPzv\/TSSzB\/\/nyoq6uD5uZm7b05evQo3HTTTVBZWQmrVq2CsrIy7W3wAlevXg2dnZ3Q3t4OU6ZMMdqWKPzkyZOwdOlSmDBhghEsTSqza9cuaGhoCJpYsGCBUv9dYi5iwvxtzpw5UF9fbxIybbKJBLVBmU6QCxLENnfs2AG33HJLuk7H1BID0mRbYjcYkWzatAmqq6sTdVPt28MPPwyTJk2Skp3YOFeAkXdfXx+0tbVBeXm5SnXwiQSx49gftEUaXZQU11SYSFATkHkRY5p0+YBETExmuSLmKsGnigMSYEtLC8gSrIo\/FI0EVV9GKliZKFtoEsSg2LBhQ4AbDpH4IRoL1quvvjpwbHzuvPPOISk8Xx+Hq2w4yQII6544cSIY\/onyRWOxIGJ\/Z8EkBiMrh0Mi7DsbGrFyAwMDQ4ZM4nAXf8QhIcsq8N9sONzU1BRkf3v37g1kTJs2bcjbmgUj\/ibqyg\/XZXC95557YPny5cPaYv0J6wNrH\/HEh2HA24UfNvKYMxwwA2TTCuxvYltRfYj6+\/79+0tDVdYvbCOqL2GBKvaF+ROzF9MZ\/x1GtFH1cXqD+fL48eNLePOYibjyuDG\/uvLKKwOfwQczOF7nmpqa4O\/Hjh0r+Yvoj3yfVV8wJohNRWZhSVAcIohvchbIzFnCfmdzW2PHjg2IhAUYMzI6HTrM4OBgbMYTJ1vMlngSZMEsOhULPuz75z73uSFzfnEkiMTW39+v1Ffsz7333lt6gcjgynATdRNJVuwLjxMSNJI5ymI24vXG+SY+82M2WLx4celFFvY7I3MRU5W+oR\/E9UUczia9qJDI+BdXUn0RN9GXRRvxOPAvRd4fmC9j22J\/8SWC85XspSn6u+gjtuehVQgvrGwhSTAuK2BEFjZ3xYZuX\/va14YtJsQFVJzRk4Y6UZkg\/2aNG4olBViU00ctxPD9ufXWW4PgZJkh6sK\/DPDvItYyw2Exq8GMj7XFz4uFEQ2\/6MIPu7AvGKh8BsRnrGJ2FZWthPUNX0ZxLzJcAIobAob9xv+NEX7UnKCIgxjISS8mLC9mgywTDXspiu2JPrxz584hUwMMS\/YCSvL5rKSlu34hSTDMwUWyWLdu3ZBVTP53cdjIQGfDCDHDiSPBpLeiDAnGTXzrJkHUjRE+Bv+SJUuAObcqrlGZIMvurrjiilJWGvbiCSNBNr3BBwISHy5YiCQoDtmwDiPJqEwwrG9RJBjVF3FVNOwlxus2a9as2EwwaT4yiQTZywBfNiLOYSQothdFgiIZsakbIkHdNJ1CnolMkO+GGEBFygRRTxagM2bMgAMHDpSGwqq4iiQo4haWdapkgrxNkrIlFthRL7K4vslkgnFu6jITnDp16pBRDcvm2ZYpHZmgqDuRYArSMlFF5e2Y5ORsTjDKcZKyPfHNys+hRM0Jxk0088MPMYtg8zVsjkccDocNaUX8+SGhuGdNBtekuVSchMf5KMzG+cWfNHOC4vxj3JAsbG5MnOeN6ptIZElDdR7TpGxddU5QtGGcTRgJYn9w\/poNZeOGw2nmBPmV86R4MBHvWWQWcjjMAJFZxcQ5rjvuuCOoErc6zK+kqmSCrC+qq8NRc1ji6jCfueF\/45AQV6zDVofZii\/DJW5Fm5UJW6mUwZWtxIttPfvss8F8Ej5s1XHMmDEBKeLDFkOwb8w2UavDWJ71LyxLjVsVxboqfWPEg4sEjEDYggGzcdz2mbjVXZnMSWZ1mGEuvnT5VWz0Y3yZM\/+IWtTj64g+hYsn4lQDbyNaHQ6hZHQ2fMJORIhgits2sjB8XN24eTZTbZJcNQRU95vxmZ7qhmO1no2s0mFbp+IQULWbazSNZ4IMkKijQPh7R0eHlSNdPNhEgq5db3j7YduW+O05ST1GX8KFHBdH9pL6lqffxS1g2HdxV0BSgkEnRt5FCFP4ZcuWBf+KOs+JqXtvb6\/SWUkdDkUkqANFvTLEIR8\/3JVpKc9nh2X0s1lGnL7hDwvE9YPZkM4Ov4sSEk1VVVVAclHDYX5+SdXpbToFtUUIEALFRMDYcBhT6o0bN8Jtt90WrAKGkSB7c0+fPj3Y5U\/DmWI6GWlFCPiMgBESRHJbsWIFzJs3L7hOKW5hhAdHJEX+twsvvNBnHKlvhAAhkAGBrq4umDx5cgYJ6asaIcGwnfrYxaR70hgJzp07d9h1RUiCPT096TX1oGYRdEAYSQ8PnInrQhHs4VIHIyQoukhUJoiTqI2NjdDa2hpkjDgcxrJh95C5BEmXyxdBByJBXd6gT04R\/MqlDtZJUCQ+PmuM27zrEiRd7nrw4EFnKb8uHVAO6aETzeyyimAPl\/FthQSzm7kYQ7AiOCuRoA5v1iujCH7lFQmG7dVKMhme8ti6dWtSsUy\/uwQpU8e5ykVwViJBXd6gT04R\/MplfA\/LBMXhapKpcDi7cuXKYJe+ycclSLr0KoKzEgnq8gZ9corgVy7jm4bD+nwxUVIRnJVIMNHM1gsUwa+8IkFfr8FxCZIury6CsxIJ6vIGfXKK4Fcu4zs0ExTPDYpXTOkzn7wklyDJ9zK+ZBGclUhQlzfok1MEv3IZ34nDYf5sr61rrsLcwyVIuty1CM5KJKjLG\/TJKYJfuYzvRBJkpor6AI0+U8ZLcgmSLh2L4KxEgrq8QZ+cIviVy\/iWJkHeZGxF+K677gJbl1e6BEmXuxbBWYkEdXmDPjlF8CuX8S1NgnHXe+szZ7QklyDp0q8IzkokqMsb9Mkpgl+5jO9YEhQvQog71qbPpOGSXIKkS7ciOCuRoC5v0CenCH7lMr5DN0vjVdr4ERn2xH1ARp8paU7QFpZZ2ylC0BGZZ\/UCvfW9JEH2KUK9qqaX5hKk9L0eWpPIQxeSeuSQPfTgqEOKy\/imY3M6LCgpg4JOEihLxcgeloCWaMY7EhSHw0k60AUKSQid\/p2CTg4nW6XIHraQTm7HKxJM7q6bEi5B0qUxBZ0uJPXIIXvowVGHFJfxLb1FRoeiWWS4BClLv\/m6FHS6kNQjh+yhB0cdUlzGN5GgDgtKyqCgkwTKUjGyhyWgJZohEvQcJInuSRWhoJOCyVohsoc1qBMbIhJMhIiu15eAyFoRIg9rUEs1VAR7EAlKmNolSBLdkypSBGdFRUkPKXNbK1QEe7iMb5oTtOaqRB4WoZZqqgjkUZSXktckyD6I3tnZCXV1dXDjjTfC3XffDTZvkEFDuwRJKqIkClHQSYBksQjZwyLYCU1VXP1l6P\/xvzjpUGwmyAhw+vTpMGnSJOjo6IBVq1bB9u3bobu7O\/jvsrIyKx0nErQCs1QjRB5SMFkrlHd7dDzzGizqeAGOfvsaa5jxDcWSIP\/lucHBwRIJ9vf3B1+Ys5kNEgk68Y\/QRvMedEwp0sMPn\/KaBBEivF5\/YGAAZs+eDdu2bQuGw4sWLQqGxs3NzdZQJBK0BnViQ0QeiRBZLZB3e6ze0Qurdxz0MxNklty1axc0NDSUDGvqw0tIuPiEkSuRoNW4im0s70FHmaA\/vhQkWnkgQRuQMaJdsGABkaANwDO0QSSYATwDVfNuDyJBAMC5x2XLlgXugbdXUyZoIFI0isx70FEmqNEZNIhq\/dHLcP9PXvFzOCx+VyRMXx1X7uMwuKqqCnp7e2k4rMGpTIsgEjSNsJr8vNtj1nd3w5MHjvlJgsF4\/V2Cqq+vL1kGP86OhIUZG\/43bpe555571Cz3bmn8jsnGjRvhtttug3Xr1sWSIP7Y1dWVqh0fKuGqekVFhQ9dydQH0iMTfNor59ketbW1cPy61fDO6HF+kiC\/RWbKlCkl4\/Gf3MStM7hdpr29Xdm4uA9xxYoVMG\/ePED5tDCiDKGTCnnPPGg47MRtQhvtO\/oWfHL5U8FvXu4TZJnghg0bgH1sSVzAyJIJil+zYyiFLY7Q6rA\/jksk6I8tsCd5tseTLx+DWfft9psEsXc8WfFzgLo\/wk6ZoF\/BFdWbPAcdrxPp4d7f2Hyg15mgTZiIBG2inb4tIo\/02JmomVd78EPhs4\/sg9cfutkEPIky6RaZRIj0Fcirs4oIkB76fEKHpDzaAwkQh8H4\/\/iMebQZen\/5cx1wKMtIJEHxtAhrAb8w19bWBuXl5cqNpqlAc4JpUDNTJ49BF4YE6WHGP5KkigTYdF0VfG9hLfT09CRVNfJ74gUK+PnNxYsXw2OPPRas4uIWj6VLlwLeLMNvmzHSO04okaBphOXlE3nIY2WjZJ7sgQsht2x+oZQBVpaPgj3fusrpVXmJJNjY2Aitra2wZcuWYEMzEp\/uBREZRyESlEHJTpk8BV0cIqSHHX9hreB1WXhjDHsYAeK\/Xca31H2CuCJcU1MTXKLw4IMPBrfJ9PX10XBY0Yco6BQBM1yc7GEYYIAg48MbYkTy++HfTYOLzx9d6oC3JMh6uH79epg5cybgxmgkQrxGy+aFqq7fFLpchYJOF5J65JA99OAYJuX5gd\/Cl77\/XGnYy2d\/2xdeDpgF8o\/3JGgOKnnJLkGS72V8SQo6XUjqkUP20IMjSsGM779eHIR1j\/UNIz78HUkvjPxYD1zGd+LqsD6YsklyCVK2nr9Xm4JOF5J65JA90uOIpPf7t9+Bxh\/sCy4\/CHuQ+L7zN5fANVOTd5C4jG\/phZGos8O0RUbekSjo5LGyUZLsIY8ykt76x\/vgxdfejCQ9lvF987oqmHHxecOGvHGteUeCUWd6eSVszwu6BEneVWg4rAsrG3KIBIejzDYvr3m0F\/oGT8YSHiM9zPjWz7k0ECbO9cna0WV8p8oEZRXTWc4lSLr0oKDThaQeOSPZHozs1u7shd4jyWTHEEeS+\/zHx8HCmompCS\/Mei7jm+YE9cSTlJSRHHRSAFkuNBLsgWTX3XMMNj39KvT95q3QRYso2JHwPnvpWLj1mkqthOc9CcrcJo1K0LE59YgdCUGnjoq7Gnm3B8vm\/vjGq7DpxVPwdM8bykRXGtKeNwqaZk4OyC7tkDaLJSkTlEDPJUgS3ZMqkvegY0qSHlLm1lKIEd2OXx2Bzr2HU5EcP1d37SXlcOu1lXDmGWc4IbsoUFzGNw2HtbiqnBAiDzmcbJVybQ9GcEff\/ANsfGoADhz+XWqS44numkvK4e8\/MxFGve9Mr4guzq7ekyDeHt3S0lLSwdR3h30FSVdQug460mMoAibtwQhu36E34Ud7XodXjr6VieB4kqs8bxT8bfUEqJ784UAhHA5PnjxZl3mdyPGaBPGiU7xOi12bxeYMq6urQz+NaQpBlyDp0slk0Onqo4yckawHI7e33zkF9z\/xSrBvDp+oDcMyeLIywXzceaNg0tgy+HL1BfCRMR8IfkqaoyuCPVzGd6otMnSLjIprv1e2CM6K2hRRD0ZuJ\/\/wR\/jeT\/vhpUO\/00ZuPJH9xUXnwvUfHwefqDhHiuBkPK0I9vCWBNEAlAnKuKFcmSI4a95IkJHbz3vfgMf3HdUyLOWtzbI0zOAuOn803PyZiVDGzcVh+0mZnJz3RJcqgl95TYIIPc0JZnXT0\/WL4Kyu9WCkhv3Aoeh\/\/PII9GRcUAizLk9uVePKYP70CTD2g+\/Xlr3p8aji+JX3JKjTYGlluQQpbZ\/FekSC4UjyxPbI7kPw+ItHg4Kqm3tl7MST24UfPgVfv\/7SYLsIP2SVkeNTmSL4lcv4pi0yFr25CM4qkwnypLbvtTdh+\/8chv8dPGmF2CaPHw1fqb4Axn8oOWsbKfaw6OKpmyISlIDOJUgS3ZMqkteg40lt4Njv4YHHX4LDb51ljNT4rAzn2v604hz44hV\/AuUffJ\/WjC2v9ijiCMNlfCeuDuOHliorK63fJC0a2iVIUgwnUcjHoPvJS7+BR549BAePmMvURFKrHFsG0yo+BDM\/Nq6EmunFgzDz+GgPCTcaVqQIeriM78ThcNi1WrRZOo2rmlsY4TM1JLWnD74RDD9NzKkxzfm5Nfzbn1V9GBr+\/CPw\/rPP1JqtpUNarlYRyENmekIODbelvCbBMGhwtXjz5s2xH1riyTPusgWRZKPKugRJl3vIBB1PaIdP\/B9seubV0p41m6R21UXnQv2VH4Gzzzq9aMA\/RTihUBTyKIoeLuM7VSaIX59rb28H\/rZpPkhOnjwJK1asCL5TjGWQNLu7u0OH1HgapaOjI3G47RKkrCTIiO3He3vgmUNnGl0kCMvUJpaPgisrxwTXIom\/p9FNhszTyLVdh\/SwjXh0ey7jW2pOELvOjs2lgS3uhAkSZG9vb+IRPJcgRenMyG3nC4Owbc\/rQTET2Zo49Jw28Ry4\/rJxUHHue1\/ssjmnRuSRJgrM1SmCPVzGd2ImqMN0cZkgnkjZsGFDqZlNmzYBnksWH5cgIdlt2X0IHnvxqLYzoqgfrnziIsHU80dD3bTxcFZO9qsVIeiKMowsih4u49soCfIXtIaRGw6bly5dCtOnT4f6+vrgooYlS5aEDrVtgoSkd0vHC0qEx2drSGx\/dfn5cPH40x+XZr8Reeh4peqTQfbQh2VWSTbjW+yrURJkjTEybG5uDs3yWDmRFPnOIkj4dHV1ZcU7tP4rx\/4AC7cdgoHjb0fKnzDmbLjgnLPh8gkfgNkfO30AHv8m+\/T390NFRYVscW\/LkR5+mSbP9qitrS2B2dPT4wRYKyQYR2681qzc3Llzh5GliTcFZnz4oZl\/ffrVYeCza43Wz71U2wF4yjyc+Hhko2QPf+xhIr5ltUtcGGlsbITW1tYhK8FJV2lh5sfXw\/JNTU2wZs2aIXLEcjgcxjnCsEUYnSAh+W3d8zos+\/cDQ3BC4vvGZ6uCY1cmHgo6E6iml0n2SI+d7po641u1b6EkqOO7w6IMNicYRpDz58+HgYEBiNt6owskJMBZ9+0e8tUtJL\/tCy\/XlvFFGYGCTtU9zZYne5jFV0W6rvhWaZOVTZUJpmkoax0dIP305WMw+77dpa7YIj\/WIAVdVi\/QW5\/soRfPLNJ0xHfa9q3MCabtHF8vC0iY\/T36q0Fo2rK\/JBIzv09ffK6OrknLoKCThspKQbKHFZilGskS31INxBQaRoJsJffw4cOwdu3aYI5u7969w0Tk6bvDm595DRZ2vBDogNnf+jmXWidAbJuCLqu76q1P9tCLZxZpXpFgFkVM1k0LUsczr8EijgBtzP1F4UBBZ9JD1GWTPdQxM1UjbXzr6E+hh8M4DP7k8qdKGaBLAqRMUIe76pVBJKgXzyzSvCVB\/sSHqGAehsNIgOx8r4s5QBEzCrosYaK\/LtlDP6ZpJXpLglEK4TxhTU1N7OmPtGBE1VMF6ZuP7Ie2n\/06EPfQVy+DL3xivO4uKcujoFOGzGgFsodReJWEq8a3kvCEwqmGw0mbpXV2kMlSAenJl48FewHxwYWQPd+6ykSXlGVS0ClDZrQC2cMovErCVeJbSbBE4VQkKHOpqkTbSkVUQJr13d2lyw+QAG1eMxWnFAWdksmNFyZ7GIdYugGV+JYWKlkwcbM0fmMkbItM1JVXku0qF5MFiV8NrvvEeNj41cuU2zJVgYLOFLLp5JI90uFmopZsfJtoO1UmaKIjSTJlQWJZoO3TIEn9x98p6GRQsleG7GEP66SWZOM7SU6a3xNJUDzrG3dBapoOyNaRAYnfEtPwqQtg\/ZyPyoq3Uo6CzgrM0o2QPaShMl5QJr5NdSKWBKOuwJK9El9np2VAwsUQXBTBx6e5QIYDBZ1Oj8gui+yRHUNdEmTiW1dbopzEOcE0V2mZ6GwSSHwW+OmLzoXtiy430Y1MMinoMsGnvTLZQzukqQUmxXdqwRIVpTJB8ZLTuHv\/JNpMVSQJpMf3H4W\/vv\/0GWcfNkaHKUlBl8r0xiqRPYxBqyw4Kb6VBSpUSJwTxKFvS0sLsNVgJMCGhgaw\/QH2JJD4BRFf9gWKdqCgU\/BMC0XJHhZAlmwiKb4lxaQqlkiCKDXqgtRULaasFAcSPxT+zhcvgXlXTUjZitlqFHRm8VWVTvZQRcxcee9J0Jzq8pLjQPrn7gH4xg\/3BcJ8XBBhWlLQydvbRkmyhw2U5drwlgTF7TFy6pgpFQdSHobCiAoFnRnfSCuV7JEWOf31vCVBVBUvS6iqqgq+C+zyiQKJHwr\/w7WVcPsXLnLZzdi2Kej8Mg3Zwx97eEuCebhKiz8m5\/NQmDJBfwKOpif8s4W3JOgTVFEg5WUoTCTokzed7gtlgv7YhEhQwhZhIOVhgzSvGgWdhKEtFiF7WAQ7oSmvSDBPH1riSbB55mRonlnlj1VDekJB55d5yB7+2MMrEvQHlqE9CQPpwSd\/XfqMpq+nRCgT9NWjaDjsk2W8JkGfb5HJ03wgzUH5FHI0J+ibNbwlQRu3yPCnUeI+3iSClLf5QCJB38KOMkGfLOItCUZtltb1jREk2RUrVsC8efNgypQpEHdXYRwJ5mE+kEjQp5CjTNA3a3hLgiwTtHWLTBy5iiDx+wPzMB9IJOhb2FEm6JNFvCVBBMnmLTIqmWDe5gOJBH0KOcoEfbOG1ySIYJm+RYY\/mRL1AScRpPJvPB7Y0dcLVMOcjLZk+BV6ZA9\/7OE9CdqCipFhc3PzsA+7I0j4dHV1wcDxt6FuY3\/w75WfHw8zp3zQVhcztdPf3w8VFRWZZPhQmfTwwQrv9SHP9qitrS0p0tPT4wRYqfsEbfUsajUa2+ffFPzH1fMyH0jDYVteJN8OZYLyWJkuOWIzQXH1GYfdTU1NsGbNmmC1mH94kP7zl0fgS99\/LviZSNC0ew6XT+RhH\/O4FotgjxFLgmhY2flGHqQ8LopQJugXcZA9\/LLHiCZBWVPwIH1y+VOAm6XztChCQSdraXvlipBBFcWviAQl\/J6BxJ8UWVgzEZbPvliith9FKOj8sAPrBdnDH3t4TYLs63IiXHFH3ExAG0aCeTkpQkFnwiOyyyQSzI6hLgnekiDbsjJnzhxvrtdfvaMXVu84GGDv+03SooNQ0OkKGT1yyB56cNQhxWsSbGxshNbW1mGrtToUV5HBQPr6v+2Dh3YNEAmqgKe5LJGHZkAziiuCPbwlQbQNHmXr7e0F3MDs8mEg5XVluCgT2KSHyygIb5tIMJtNYjdL+\/ihpTwel6M5wWxOaqp2EcijKC8lrzNBUw6oKhdB+vF\/\/wpweww+N39mIqy8IT8rw0VxVtJD1XPNly8CmRMJSvgJgvTQo8\/CrPt2B6XztjJM5CFhZMtFikAeRfErr0mQneft7OyEuro6uPHGG+Huu++Gu+66C8rLy625LYK04uFuWNTxQtBm3laGi+KspIc1l5duqAhk7i0J8hcaTJo0CTo6OmDVqlWwfft26O7uDv67rKxM2lhZCiJId2z+GSze\/GIgJk9nhmlOMIvlzdUtAnkU5aXkLQnyFxwMDg6WSBCv7lm5cqXVbBBBuqzxEXjywLEgKo5++xpz0WFIMgWdIWBTiiV7pATOQDVvSRB1Xb16NQwMDMDs2bNh27ZtwXB40aJFwdDY5rYZngQry0cFw+G8PRR0flmM7OGPPbwmQYRJPDp35513Wj9BgiAdu6EtsFreLk6g4bA\/wcb3hEjQH7t4T4I+QFV12afg+HWrg67cMetiWHT1RB+6pdQHCjoluIwXJnsYh1i6ASJBCagqP3U9\/PbTTUHJPG6PwX5T0EkY2mIRsodFsBOaIhKUsAVPgnncHkMkKGFky0WIBC0DHtOc1yTI7xNkOixYsMDqogi2O+Ev\/xHe+uisoAt53B5DJOhPwLGeEAn6YxNvSZAR4IQJE0qkx\/6G8NncJ3jBDf8Ev7\/w2sBqedweQyToT8ARCfpnC29JUPwQEoMOvwtie5\/g+V+5H94edwnkdXsMkaB\/gUeZoD828ZYEESLcHsNOirDTIbh3sKqqyuo2mXE3\/wDeGT2OSNADvyXy8MAIXBeKYA9vSTDuKi3eDfCq\/a1btxr1jDxfoUXDL6OukVp4EcijKCMMb0kwtXcZqMhI8DtfvATmXTXBQAvmRVLQmcdYpQWyhwpaZssSCUrgy0gwr3sEi\/LGJj0knNVykSKQufckiFfst7S0lEzr4tgcI8G8bo8h8rDMDBLNFYE8iuJXXpMgLoLg4khbW1twfyCbJ6yurra6V5BIUCKqLRUh8rAEtGQzRbCHtySYZYuMeOnCpk2bAIlTfHC7zfz584ObavCJ+p4xI8G8nhYpyhub9JBkJovFiASzgR37oSUUnSYTRPJctmwZ3H777UH2iISIclg2yXc5bAtOmEqMBPO6UZrII5ujmqhdBPIoil95mwkyx8s6J8iG0Hj\/oJgNyn7SE0kwzxuli+KspIcJOs4mswhk7j0JZjMRAA55m5qaYM2aNcM+4o4Z4oYNG0pNRA2bkQTzeo8gU64IzkokmDUa9Ncvgl8VmgT575TU19cP8QDxNxwaL1myBNrb24eRJZLg2Uf2wdPLZur3IksS8bMEFRUVlloz1wzpYQ7bNJLzbI\/a2tqSyj09PWnUz1wncU4wSwthFzDEyYsjTCTBxddUwrK6i7J0yWndIryxKRN06kKhjRfBrwqZCcYRWpQbsTpz584dNneIJJjnjdKos0tD6wxd0kMnmtllFcEeLnUwkgnKEqC4BSduFRlJcPSz34f39\/0su9eQBEKAEPAOgUINh8W9fwxtXPSYOnUqNDY2QmtrazDvx5fFewvD5gO9sxZ1iBAgBAqDgJFMsDDokCKEACFQeASIBAtvYlKQECAE4hAgEiT\/IAQIgRGNAJHgiDY\/KU8IEAJEguQDhAAhMKIR8J4E+dVjF5\/6TOMdMn0WP2VaV1dn9et9Mnr+c7VIAAAF5UlEQVTJ6MHLiTseKdOeiTKyOvDlom4yMtE\/WZmyevC3N+UlXhgGLr5dhG17TYL8xQvomEuXLoXp06db\/cCTrJOycrJ9xosj8MGjhLL7KlX7kqW8rB6sDabDL37xC2+2OcnqIF7wIXupRxZ8VerK6sG\/hPB4Zh7ihSdAvEPAxYXNXpOg+GlPdM7u7m7vMiYxG+I\/RyrbZ9lyKsGTpawq9tj\/5557Dp5\/\/vnQizKy9CVtXVkdMHt64oknrF4SrKKTih78lXX43\/jg7U2+PvzRWuyj7a9Yep8JincNxp0o8cXIafvsm8Oq6MGCFIdfqEfYbUEu7COrAxL44cOHoaurC\/bu3Rt5sa8LHbBNWT1kM0ZXeiS1S8PhEITE7CgPJJimzz7qpaIHOm9NTQ2MHTs28sq0pAAw8busDlju3nvvLQ3jfXshyeqBGPKfyY26ls4E1jpkEgmGoCj7BtRhAF0yVPscd32Yrj6lkSOrBz+U9G1hRFYHcQ7Qt5eSrB5iv30j8yQ\/JBIMQUh2LiQJXJu\/q\/TZt2DjcZLVQ7wUF2X4cgZcVgdZkrHpR2ltwc8B+uxfYVgSCYagksc5Dtk+++6gsnqIwRp1g7gLApHVgb\/NiK2qIpH7sqAgq0dYJogfMFu1ahWUlZW5MIFSm0SCEXDJ7o9SQttw4ag+80YOy6B82ysoo4fPJIh9k9WBL+ebHVT04L8H5ON+x7jQIxI0TEwknhAgBAiBMAS83idIJiMECAFCwDQCRIKmESb5hAAh4DUCRIJem4c6RwgQAqYRIBI0jTDJJwQIAa8RIBL02jzUOUKAEDCNAJGgaYRHkHz+ejCVa5x8PNnArqQyufGb35aTtyNuRXJrIsGCWFPlFhqVsirwpN0A7isJdnR0GN9oHPetbRXsqWx6BIgE02PnVU0VYlMpq6KkePxMti6R4FKYO3cuVFdXy0JG5TQiQCSoEUzTosTbqNmQk79NmJ126O\/vh\/nz5wMem8InrizKvemmm4JrpPCJG5rxt5TwZfk+RA0ho74xjSR44sSJ4H+dnZ3Dzh7zsrFNdvEmOxs8ZsyYoB7rN38aB\/XG+m1tbVBeXi79nWuR0MU+xp3GEEk97qVDmaDpqEmWTySYjJE3JfjbTsTg4QMNO4y3CrPsQrzdJawsu7E77iYY8QZs8QacuOGweHszX\/aBBx4ISKy9vR2mTJkS3EnIzrxim42NjdDa2hr8xtcbHBwMiH7x4sWl28b53\/G8LOLQ19cXkCA+SPZ4Jhizrrj+hpEg3nzME23UuVwiQW9CRqojRIJSMPlRSLz3ju9VXLYhkhVfFjNG\/iZslBl1hlMkyDBS5G825vsXRzgqpIF937x5c0BqSILihQ2iLL7d\/fv3Az\/PF5eFhZEgT3pxLwsVfSgTdB9bRILubaDUA\/6APD\/sFEmQHxLi0A0fduMzXxaHwA0NDcP6ELa6KxKZCgnGkXQcabCsFjNFfGbMmAHHjx8PJcGwb7Xwfd65cye0tLQM0zXsuxZhJIgV2c0y\/M0zmKHyD5Ggkks7L0wk6NwE6TvADxu3b99e+v4KZnd8hhQ3HA7LBKN65CITRJLms0txOJwlE4xDnjLB9H6Zt5pEgjmyWFiG0dvbG2Qn4hAX58rWrl0bzH1hPX7OLW5OkM3dzZkzZ9hX\/XTOCfKEumXLlsAKLMsSM9UlS5YE84Xsrj82xxc2HFaZE2QLNQwncfjOD51FDPkXkHhXHz9kZ\/OSKDvsXj8aDrsPQCJB9zaQ7oG4OsyvULKAHj9+fDBUxMUGnMjHZ\/369cG\/2YKAWBbL8KvDcRudo1aHUUbSPkF+dRjL84sMUSTID4dx+I8LJKgLDu3xCbvElU0FYHnUC7PksNVhrB\/1iceoTBAJWPwYkzg05jFifdizZ09AguJCD5GgtPsbK0gkaAxaEuwDAmn3LibNCerSjUhQF5Lp5RAJpseOanqIQNg2ojRX5RMJemhcQ10iEjQELIl1g4A4XE97Vb54dlict9ShHZ0d1oFidhn\/D1kwlOYt2xY2AAAAAElFTkSuQmCC","height":194,"width":321}}
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
