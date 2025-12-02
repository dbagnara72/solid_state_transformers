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
ubattery = 1250;
margin_factor = 1.25;
Vdab1_dc_nom = ubattery;
Idab1_dc_nom = Pnom/Vdab1_dc_nom;
Vdab2_dc_nom = 1500;
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
%   data: {"layout":"onright","rightPanelPercent":47.6}
%---
%[output:174c5e41]
%   data: {"dataType":"textualVariable","outputData":{"name":"fPWM_LLC","value":"       10800"}}
%---
%[output:6e4cc00d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"   275"}}
%---
%[output:01f62bda]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"        1875"}}
%---
%[output:1f071822]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     1.775568181818182e-05"}}
%---
%[output:231dc037]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     3.566505664210290e-06"}}
%---
%[output:1bc16b1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     5.633826408401311e+02"}}
%---
%[output:693b28a9]
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAX8AAADnCAYAAAD2Blb9AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnX2UV8V5xx9TY1xRg6tGXDe4vixIkxZfitkSrDFUY5PuJrVJdpe2cBAJaTCnDSBvlmM5R2CXQE6MEkNwpdiWhZ4kVPijEs\/W4AtqDBjSGqoorLiuiQiiScWjUdrn4qyzs\/dl5t47986d57nn5MTld+\/MPN+Z+cxzn5k7c9zRo0ePAl+sACvACrACpBQ4juFPqr7ZWFaAFWAFAgUY\/twQWAFWgBUgqADDn2Cls8msACvACjD8ibWBQ4cOwbRp0wKru7q6oLa21ooCGzduhAULFsCyZcugtbU1yAP\/DS\/xd54Zh9l15MgRWLJkCUyZMgUaGxvzzC4yrT179sDUqVPh61\/\/upadacr42GOPwbZt22DevHlWbBJa7tq1K0h\/\/fr10NTUpJ1XWN1rP2zpRtR5\/vz5UFdXZ003S0W3lizD35q0nLCsgG0gqPCvqakJOvuOHTtg7dq1hcG\/s7MTEM46A6sAkkkZMe1JkybBjBkzrEFM5CEP3Cat2XZdm5RFbYO33357oe0hbVmLeI7hn4PK2OFXr14dpISehQwb0RFmzZoFPT09gN6Ueo\/qackdW3iSl112GZxyyimBF4ZXUscU+aplUiF58ODBwFMVnjF6lCJt3TTw7UEFhgwALAO+BYirubkZOjo6AAGNl4Dg\/v37B6AZBkahRX9\/f\/CcrJNs1x133AHLly+HLVu2DOSJNrW0tAQDgvrv8puIqEusoxUrVgD+PXLkyIHyqmWQ60H8hvYJr1ytW1H39fX1kWWRdUcDhF7YdhD84ho7dmygF174Nic89aSBQWgrdBDpYD2qecu\/yV0lrs3Kdd\/b2xv0DbXNi\/ai2oJlEDqqbfLqq68esBM1+fznPw833HDDkLdL0dbUPMPqJ4fuX9kkGP4Zq04Gv5yUeFVWO1NSxxW\/C6iosBG\/qw1b9XBk2MoDwOmnnz4o7CPgL4Aq0t25c+cgYMelkRX+mLbQSegmD3o4UPT19QWDlCinOpAg0EQ4Kwr+AkSyVrKOYeA7cOAA4MAbVwa1rkXdqZCV6z6qjOeee+4gwMvtQf0NwfzNb34TbrrppgHwq+1Hbd5RZYqq9zD4q+AXeYhBJ6rNi0Esqi7F82qbx7LdeeedcNdddw0auK+44gp46KGHQp2VsEGlqJBnRqQU9jjDP4PUopGeeeaZAx6r8GhEQ9+8eXMAUfE3Zie8T+HFozenAkN4wQLO+By+UcgeY1gsVjRwhJZ4A5E9MeE9YXroNarpo7dlmkYS\/NGzTgoFqF6Zer8YZMPAijqMGjVq0KCmE\/YRacrPo\/eswlytS\/G70Em8GXznO98JvFzxe9gbjdzcdMI+apgn6u+o9qPO6ajtE3USWqvwjnq7VO9XoXr\/\/fcPavPywBwWDosa6EWbxzYpyi0GI1G\/+PYiv9XJb49h4Susc3ymyFBgBsRYfZThn0HeMKCpHV50BBnUcqPE7FUvXfay8b\/R4xXep9xZw+CvdiQRWhFmRoV95PRN08gD\/rJuwisWHRnLHjZJLeuoDmpx8Fc9U9QR34hUnaPgrjYZBJIosxq\/jwrhYPni4B8V4lLhH+VlR70ZygOemMRV7RQOSxT8w9IIe\/NMGpDUNwj1zSCszctlCqt\/EfqSyyOHwZLKngEHlXuU4Z+hysI8iyj4RzXaKPjjv0dBSQ2RyCaYglt4\/lnhrw6ESX+HyS6eWbRoUfBWImLnUR60KfzVji\/\/nQX+clgiavI2bF5IvMXJz+h6+kkhFtF+1FU6YW2naPirbU6EgdTwWl7wl+eYGP7v9zyGfwb42wj7qMUJg3kc\/GVvSrwZyECZPn16aMxf7mi6aYjQkhyKUieLo\/4Ok131duU3m6xhH3WuQw4bpA37mIZw8H55UBQT0DL8VTipIZaksE9Sc84z7KOGMoUdYr4oyvMXb8Pid7VM6mCAdZUm7BOmBcOf4Z\/UR7R\/tzXhq\/MKHLX+OiwUIMIAURO+MvxlSMlCxK1UEfclwR\/vi1pBgr8JPdV7oia+hU5qXFmGO6Y7efLkYFI0LCwQNTmPZdCZ8BVeuAqWqInRKB0xHbzEyrGw0IW8SgbTue222+DWW28dYpe6okqklTThi\/H1pPkZ3QnfJPirnSyuzYeVW2fCV30D4pg\/w18b7jo35r3UUwafqecvyhsW18YQgE7MPykN\/F2GMZYX3yhuvPHGISsvBABkYMTBP24du+5STzGpKIMSwXrllVcOrKRB0Fx\/\/fUwc+bMgfCSOvjgUs85c+bELvWUIRsWBgwDZdj8D+aNZRRvZmJJ8KpVq+Duu+8GMf8hD2rqgC4Gtjh9MZ+4pZ7q20nUB3lR8Xp5TioK\/urAjHrgEmMxEYtlUOdf8N\/kPOX6VD8klOfQ5N\/U8JY6H6bT1326p9SwD3bmuXPnBmuyw77AVNcCxy1vdLVSkrwoV8tNrVwyENVltupbUZQ2Ai44yNr6+pZavQh7xcCPf4etYtP5apzX+Q9uPaXBX2eZG4IT12ZXuSMx\/KuDq6gQXtIHdbKFJl\/4VkeZ8kuaFELT2b4D+yJ\/4etA2Ae9euwoeEV5\/vh7Q0OD1h4p5TfP8BIw\/F2tmaHlCosrJ30tq6YivEsMGZnsh1Mdlcoradi8j+6+Q7y3z9B6K8Xzx1F88eLF0N7eHgwAYfAXlTV+\/PhKw7+8rsI5swKsACsQrUAp8EdvGK9LL700MuYf9ppn8vrNlc4KsAKsACvgEPzx1W3dunVw8803B\/u1RE34qq\/P\/DrNzZgVYAVYgfwUKNzzxzAPLrnDeGjSah\/VTDFHEDYBfP755+enCqfECrACrECOCuCOvuedd16OKWZPqlD4R83Yoxk6EzdxE8AI\/71792ZXpAIpsK0VqKSUReS6TSmc44+5WK+Fwl+tnzjPX6wGkj\/YwQ9uonbjc1FcW+2RbbWlbPnpct2WXwc2SuBivToFfwR+d3f3wMEZ6kdecW8HLoproxFhmvv27XPuFZJtzUcBrtt8dHQtFRf5VCr886wgF8XN0z45LQaELWXLT5frtvw6sFECF\/nE8LdR05bTZEBYFrjE5LluSxTfYtYMf2Li2jKXAWFL2fLT5botvw5slIDhb0PV99J0UVxb5jIgbClbfrpct+XXgY0SuMgnDvvYqGnLaTIgLAtcYvJctyWKbzFrhj8xcW2Zy4CwpWz56XLdll8HNkrA8LehKod9LKpaftKUYIhqU7KXkq0Mf4sscVFcW+ZS6jSUbGX42+ox5afrIp845l9+uzAuASUgUrKV4W\/cFSrzAMPfYlW5KK4tcykBkZKtDH9bPab8dF3kE3v+5bcL4xJQAiIlWxn+xl2hMg8w\/C1WlYvi2jKXEhAp2crwt9Vjyk\/XRT6x519+uzAuASUgUrKV4W\/cFSrzAMPfYlW5KK4tcykBkZKtDH9bPab8dF3kk7bnH3cQS5S0Y8eOhU2bNhWivIvi2jKcEhAp2crwt9Vjyk\/XRT4ZwX\/27NmwcOFCaGxsTFQTD2pZunRpcPhKEZeL4tqymxIQKdnK8LfVY8pP10U+acO\/fPniS+CiuLY0owRESrYy\/G31mPLTdZFP2vAXYR+UURytWL6k75fARXFt6UMJiJRsZfjb6jHlp+sin7Thj\/IdOXIE5s+fD1u2bAnU1Dl0vSjZXRTXlu2UgEjJVoa\/rR5Tfrou8skI\/rKE8vm6zc3NA+fuliWzi+La0oISECnZyvC31WPKT9dFPqWGv5BTfhuoq6sLJnh1JoTzrg4Xxc3bRpEeJSBSspXhb6vHlJ+ui3zKDH9ZVrHCZ+XKlVBbW1uo4i6Ka0sASkCkZCvD31aPKT9dF\/mUGf7s+RffsCgBkZKtDP\/i+1JROXoFf475F9VshuZDCYiUbGX4l9enbOdcefirq32WLVsGra2ttnXTSt9FcbUKnuImSkCkZCvDP0VnqMgjLvJJO+wj1vkfOHCgtEnduHp2UVxb7ZISECnZyvC31WPKT9dFPmnDv3z54kvgori2NKMEREq2Mvxt9Zjy03WRT9rwR8+f9\/YpvxExINyoA1uloDTYUbK18vCfNm0a7Nq1S7vd866e2lIZ3Uip01CylQd2o25QmZu7n\/gVzOzeDYe+dZVTZdb2\/J0qdUhhXBxZbWlGCYiUbGX42+ox5abL8LesP8PfssAlJc\/wL0n4ArKlUredW3uhc+s+9vxttSmGvy1ly02XCiCEypTspWIrw98yQxj+lgUuKXkqgGD4l9TACsiW4Z9R5M7OziCFefPmhabE8M8osKOPM\/wdrZgcikWlbr2D\/8aNG2HBggVBE8B9\/fHq7u62srWz2EpixowZDH8AoNJpqE2AUrOXSjvGlT446evFah\/0wvv7++GWW26BxYsXQ3t7OzQ1NUGSd57GWcAtJZYsWQJPPfVUkAd7\/gz\/NO2oKs9QASKlgc4b+Msfe9XX1wcnewn429jSGd8w8Ort7eWwz3sEY0BUBeXm5eS6NdfM9SdaVj0JDz93uPqefxz8MTyD3n9eZ\/xiXvhmgW8Ya9asYfgz\/F3v55nLx\/DPLKFzCVx866Ow\/9Cb1Yc\/Kove+Pbt2weFfUaNGgX4BXBbW1tuO33iQHLllVdqhZRwwhevnp4e5yo\/7wL19fUBvnVRuCjZivVJyV4Ktl7V3AqvX3NssYoXMX80RN7PX0Aozy2eMYS0bt06uPnmm6GmpiZxPoFX+\/g5FFDyhCnFwanYih4\/ev5ewd82auTVRHJeeFj8bbfdNiR7hr\/tGiknfYZ\/OboXkSuFuhVbOzD8M7SopJVEDP8M4jr8KAVAyPJTspeCrWKy9wNvvAKvfO9LTvU0443dxKEuSbt7xq3JT6MAw\/991Sh0GmEtJVuphEIo1W3trAcCc49\/5Wl4+Z6vpkGftWeM4Y8lwZDMhg0bBq3qEYOCmPBNgnXeFrHnn7eibqTH8HejHmyUwve6\/ft\/exrueaw\/kO7kh5fD\/p\/+hw0ZU6dpDH8BefzYCj+6ki95qefBgwdh6dKlwZGPRVwM\/yJULj4P3wGhKkrJXp9tlSd6R9aeCK\/f\/Vewd+\/e4jtQTI4Mf6eqQ68wPncayjDksI9e+3f9Lhn8WNbNX7sEJl9zafXhrxv2Ed8ChK3MsVF57PnbULX8NCkNdAz\/8ttb1hIg+Fu++2TwURdeP\/rqWPjUqFpwkU\/Gnr8QJ2ydP27whqEg\/G3OnDlByKexsTGrnlrPuyiuVsFT3EQJiJRsZfin6AwOPfLws4cD8Itr+oR66LzuGP9c5FNq+DukubPi2tKIEhAp2crwt9Vj7KcrlnSKnLomfwz+4uKPDGTM8LdYBy6Ka8tcSkCkZCvD31aPsZcuevro8YsLJ3fvaBsDEy4cPihTF\/mUyvMPC\/kIS8eOHZvbxm4mVeaiuCblN7mXEhAp2crwN+kF5d2L8fwbu3cHO3XKV\/u4EbCqfUxowVzkkzH8cX993MZ5\/Pjx0NLSMrCls42N3Uyq10VxTcpvci8lIFKyleFv0guKvRe3aXjk2cOw\/omXhmSM3j6u6MH\/j7pc5JMx\/OUtnXEyFz\/mamhoCHbyxDcCW6d5JVW1i+ImlTnt75SASMlWhn\/aHmHnuSgPH3ND0E+48DSYe01DLPRFyVzkU2b445JOPGgFP\/qycZiLbrW6KK5u2U3vowRESrYy\/E17Qr73I+zxi9yf7nttSEhH5BQV008qiYt8MoY\/Gilv3SB7+5s3bw72+e\/o6Ai2YS7yclFcW\/ZTAiIlWxn+tnpMeLoI+86t+4LzdaMuEcpJCuskldxFPqWCv7rFAw4Gq1evhrq6ukLX9suCuyhuUoNI+zslIFKyleGftkckP4eg\/\/5DffCLvt9EevWyd\/+F\/1+m+Y9\/fkFywpp3uMinVPDXtLfQ21wU15YAlIBIyVaGf\/YeI76sRY\/+hUNvJoIec0Tv\/kuXjYC\/+cTZQQHiJm7TltBFPhnDX53wlcXgmH\/apmH2HCUgUrKV4a\/fD9JAXoB95Gknwh3tY6xAPsoC7+Gf9wHu+k3Bzc+nTcpvci8lIFKyleE\/tBcg5F99421YdO+zwY\/q2vq4fmO6IsekD5reW2n4x33YJQuR9yEuuiK7KK5u2U3vowRESrZShT8C\/tevvwX\/9OiL2qEauc8g5NGbX\/S58+GsUz9UqEev23dd5FOuYR9dIWzc56K4NuykCghbWrqWrm+DnQjPoM6rfvIC7H7pt7D\/1TcHdr3U1V\/E4T95wfAgPn\/+GTVOQt7rsI9uZRV9H8O\/aMWLyc83GCapVjV7Zbj\/82P98Pi+11LBXegivPi\/\/sTZUDccP6QavEdOkn6u\/u4in7Q9f92ze3lvH\/vNr2qAyKIIJVtdfKvDTcsQyAf\/921Yu\/1F6H3lSGa4o53owbdc8AEYc+G5lfLg07blSsM\/rdFFPeeiuLZspwRESrYWDX\/02vF\/I049Ae7e\/iL894vpQjJq\/B3\/xhj8zE+NhDFnDxv4WV1CSaluXeSTtudvC2R5peuiuHnZpqZDqdNQsjUv+MuhmCd6X4MHnnkV9h\/M5rGrYZmP1p4IX7miHk476YPBT2nWxlOqWxf5lBr+uKfPggULBnFp2bJlwQZvZVwuimtLB0qdhpKtOvAX3vrZH\/4QrHu0H37+wutBMzNZAhnVLgXA0Wu\/7tKz4NOjayO99jzaNqW6dZFPqeCP4N+wYcOgffvFnEBbW1spA4CL4ubRQcLSoNRpqNgqYuvP9u6H+58\/Dp7q\/21uUJc98ysuPA0ubzgVrhx1DOwIfBxQ0njuWds3lbpFnVzkkzH81X195AbAH3ll7Q56z1PqNFW21Wb4RQ7DiBj7RWcPg69MqIcTjv9A6lCMXgvM564q162pAgx\/U8UM7ndRXIPiG91KqdO4aKuA+rtHj8I9j70EP+t9Lai\/NOvXdUIwv193cnAeLIZ6yvbWjRpqws0u1m2e9slpucgnY88fDeKwj60mopcupU5TlK0C6O+8exR+9OTLsO2ZQ7kDXQ6\/YFz9gjNPghl\/Ug8nnfB7AxX\/zmsvwXnnnafXECp+V1F164JM3sBfDAA84VtOs6LUabLYKoB++I23YdPPX4Ydzx+bHM3TQ1eBjqtg8IQnXMcuLpN4ehZ7y2mN6XOlZKsX8C97YjeqqbkobvpuEf8kpU4j2yrH0Lc\/dxgeee4wPH\/wSCBWHqtdZNXllS\/4758aXQtfvPSsVEA3aQdU69ZEoyre6yKfUoV91E3e1q9fD01NTaXWiYvi2hLEJ0CIpYtnnPxB+N6DL8DeA8dgnrd3HuahjzprWBBLT+Ohc91mV8Cndpykhot8SgV\/2VB5vT+f5JXUBPL53dVOI3vm+N8\/e\/51eODpQ3D06FErMA8D+p+OOR0uG3mqU0A3qXVX69bEBt17KdnqJfzlisbjHPGtoKurC2pr3\/9ARLcxZLnPRXGz2BP3bJGdRp4I3fyLA9Cz+6A1z1yFOf5dN+xdaBt\/frCLo0seug91a8sG3XSLbMe6ZbJ1n4t8yuz5i\/N7UbSyNnXDvF0U11ZDSttpZM\/87XfehR\/sfBkeefZVqzBXgT56xDC49mNnQONHTgryTfrIKK2ttrS3nS4leynZ6iKfUsE\/S6jnyJEjMH\/+fNiyZUvQj+IOf1HnFuLCSi6KawsUYZOgb73zLmx84tfw+L7DhcJ85Ok1cNXo02DcuR8+lu+hN3PdhpcSIFA\/SvZSstVFPhnDP+4LXx3Y4ZsCXvPmzYOklUM4yPT29gb3Jl0uiptU5qjfhYf+xlvvBPu3iE\/9bUyCqp75x845OdjTZfRZ7+\/GKN+T1qa0z1ECBMM\/bStx\/zkX+WQM\/7xllgcDNW38raGhQWuvIBfFDdNKgP3oUYDlP94XHFuXJ9TlNeX4IVHjWcOg9Y\/OghGnHvs6VCfUkncdZ0mP4Z9FPbefpVS3LvKpVPjHvUWI8ND48eMrC38E\/a6+38Cah\/oyrUOX15zjR0QNw96G1gmjrIRZXMMFJUCw5+9a68uvPAx\/SUsxUdzc3AwdHR1QU\/P+ag68LezksLgto10Rt2XVk0agl88mvXrM6XDpe8sU474KpQRESrYy\/PODrWspucInWZdSPX8sCA4C\/f39QwaAPXv2wNSpU2HFihXBB2Tq32rlorh49fT0FFrvj79wBLqeeA12vPhmZL51px4PZ59yPFz+0RPhs6NPHrgP\/z3N1dfXB\/X19WkerdwzlGzFyqFkLwVbJ06cONDn9u7d61T\/M4Y\/euSzZ8+GhQsXQmNj4yBjENBLly6FlStXaq\/zx2fmzp0Ly5cvH5KeqlTc\/EBRI6uI2d\/YvTvUwxce+\/LrRsFFI4ZZ2SedkjdMyVb2\/J1iY66FKYpPJoXOFf5p9vM3eSZuArgIcfHAjRs37A6WM8oXAv+GCfXQ8odnWoG9WqGUgEjJVoa\/CbqqdW8RfDJVRBv+6pr7qIzi1u2LMA\/+Py7fFJO6uH5fXc6pDgr495w5c2Dt2rWhbwg2xUXYh3n67eNGQPu4s3Nd165TgZSASMlWhr9O66\/mPTb5lFYRbfiLDOLCPjqFUD\/ykid8EfDd3d0D8X+TDeRsiYvefst3nxwwDb38iRedDn\/36ZGFePlhmlICIiVbGf46BKnmPbb4lEUNY\/hnyczms3mLi95+z\/8cgtk\/eHoQ+Dd\/7ZLSoC8KQgmIlGxl+NskRLlp582nPKzRhr9Yejl9+nRYs2YN7Nq1KzT\/svb3yVvczq290Ll1X2Ajevt3tI0pPLwTVcGUgEjJVoZ\/HkhzM428+ZSHldrwzyMzm2nkJa4a30fwu+Dty9pRAiIlWxn+NglRbtp58SlPKxj+ipqqx+8a+BkQeTZ\/99KiNNhRstUL+Id9eSt3oSqHfbqf+BXM7N49EOpxEfwMf\/eAnWeJKAGRkq1ewD+qoeMqniVLlsCUKVMSP9bKs7OItLKKi+Gei2991HnwM\/xttB530qQEREq2ZuWTjRaaa9hHXappo8BRaWYVF8EvPt5a1T4GcA2\/qxelTkPJVh7YXe1x2cuVlU\/ZSzA0hVzhn2Z7h7yMyiLuTT98BroeeTEoyjcmnguLPndsnyBXL0pApGQrw9\/VHpe9XFn4lD338BRyhX\/UJm22Ci+nm1ZcNdzz83\/44yKKmykPSkCkZCvDP1O3cPrhtHyyaZQx\/OMmfOOOWbRpBKadVlx5C2YEf9xWyrZt0E2fEhAp2crw1+0B1bsvLZ9sWmoMf5uFyZJ2GnHlrRsmXX423NF2UZYiFPYsJSBSspXhX1gXKjyjNHyyXchU8FdP2YrboM22ASJ9U3Ex3IN79uD\/u\/ghV5xulIBIyVaGf1G0KD4fUz4VUcJU8A\/bV7\/sAcBUXHlN\/79c\/wfw2Y+fUYTeueRBCYiUbGX459I9nEzElE9FGGEM\/7wPc8nLSFNxRay\/al4\/AyKvFuNmOpQGO0q2mvKpiNaZCv7Tpk0L9t\/H4xXly+RglryNMxFXjvVPbqqDb395dN7FsZoepU5DyVYe2K12m1ITN+FTUQU1hj8WbOPGjbBhwwbo6uoaOK5RrAJqa2uD1tbWoso\/kI+JuFVc4SMLSgmIlGxl+BeOjcIyNOFTUYVKBX8sXNjJXsuWLSsF\/FgeXXHldf0TLhgOm2deUpTWueVDCYiUbGX459ZFnEtIl09FFjw1\/IsspE5euuLKE71VWdev2k8JiJRsZfjr9PRq3qPLpyKtM4a\/WNXT3t4+JOZfZMHVvHTFlSd6q\/A1b5imlIBIyVaGf5kEsZu3Lp\/slmJw6sbwz3qGry3jdMWtnfVAUIQbPnkOLP\/LUbaKYzVdSkCkZCvD32q3KTVxXT4VWUhj+GPhcMK3t7c3WPHjyqUj7tZfHoT2u34RFLmqIR8GhCstzk45KA12lGzV4ZOdFhWdqjH8q3yYiw8hH4Z\/0V2k2PwoAZGSrV7Av9iuoJ9bkrjyKp\/rx58DK75YzZAPw1+\/TVTxTkpApGRrEp\/KaKvGnn8ZhdTJM0lc+cOuKod8GP46raG691ACIiVbk\/hURovVhr8I90yfPh3WrFkDu3btCi2vq2f4+hLyYfiX0U2Ky5MSECnZWmn4F9f80+UUJ64PH3bJqlDqNJRs5YE9Xd+vwlPewL9qWzrL8J\/3mfNg3mcaqtBeIstICYiUbGX4V7pbxhbeG\/hXbUvn1Q\/1wYJNe4LKqXq8nwHhLyC4bv2tWy\/gX8UtnX2K9zMg\/AUE162\/desN\/Ku0pfOgeP+Fw2Hz16q3kZvaJSiFQijZyvBn+BepgPZqH7lQVdrSWV7iieCfcOHwIvW1khclIFKyleFvpbs4kagXnr9QsipbOt+57QW4+d5ng2Iz\/J3oB0aFYPgbyVWpmynVrVfwL6KVyQNMXV0drF27FhobG0OzjhLXt3g\/e4dFtLzy8qAEREq2MvwN+tSePXtg7ty5sHz58gD4YaEmObkwcX1b3y\/spdRpKNnKA7sBICp2K8M\/Q4Wpg4GaVJi4Psb7GRAZGlEFHqU02FGyleGfofOh5799+3bo6OiAmpqaISkx\/DOI6\/CjlADBA7vDDTFj0Rj+KQREj3\/q1KnQ398P69evjzw9LExc+aD2Q9+6KkXubj5CCYiUbGX4u9nf8iiVN\/CXgawKY2tjN5HnihUrQgeAMHEvvvVRwLj\/yNoTgy97fbkoAZGSrQx\/X3roUDu8gL\/Y1wdX3xR9klfYthJCZhQXr56enuD\/+1\/\/HTSv6wv++7JzToTvXzfCm5bV19cH9fX13tgTZwglW1EHSvZSsHXixIkDzXvv3r1O9Vnjj7zKOsNX3UxOVVEdWeXJXh82c5PtpeQNU7KVPX+n2JhrYbzy\/Nvb2yPj73mopq7uwTX\/c+bMiVzrr4rbubUXOrfuC4riw2ZuDP88WpX7aVAa7CjZ6gX8sfskrbzJq4upXxGbTPj6OtnL3mFercvNdCgBkZKtXsC\/Kge4+\/hlr8AVpU5DyVYe2N0ckPMolRfwz0MIG2nI4vr6ZS\/D30bLcStNSoMdJVsZ\/hZzMQ0mAAANQUlEQVT7WRT8fZvsZe\/QYiNyIGlKQKRkq1fwx7j\/ggULgu6CsXi8uru7I7\/Atd2vZHHllT6+TfYy\/G23pHLTpwRESrZ6A39cb49f3N5yyy2wePFiECt\/4tbh2+5Ssrj3PfUKTOr6ryBLX7Zx5tU+tluQG+lTAiIlW72Av7zOHz80mj9\/\/gD8cXnm0qVLYeXKlVBbW1tob5LF9Xmylz3\/QptV4ZlRAiIlW72HPy7NRO+\/q6urVPj7uq0DT\/gWzuLCM6QEREq2egF\/7A1inb8c9hk1ahTg2b5tbW3Q2tpaeKcR4vq+0oc9\/8KbVqEZUgIiJVu9gT\/2BlePcZThv6p9DLSP82dPH\/b8C+VwKZlRAiIlW72Cfyk9IyZTIa6vB7jwhK9rLc5OeSgBkZKtDH87\/SVIVYjb\/cSvYGb3bm9X+nDYx2IjciBpSkCkZKtX8JfX+Ys+E7f3ju1+JcRF8OMAgJdPB7iw52+7BbmRPiUgUrLVG\/iHHaYu9vwpe8LX92We7Pm7AWlbpaAEREq2egF\/AXk8yKWpqWlQH3BhqadY5jnhguGweeYltvpoqelS6jSUbOWBvdRuZTVz7+Ff9kdeP\/nZLwHhj9fkpjr49pdHW63QshKnBERKtjL8y+pR9vP1Av4oU9jBKi6Efe758U5o+e6TQU36uKGbaKKUgEjJVoa\/fQiXlYMX8E\/az18WFw9z37RpUyF6o7iy5+\/jnj4M\/0KaUqmZUBrsKNnqBfxL7RkxmaO4M+78z4GjGxn+rtaUWbkoAYI9f7O2UaW7Gf4WawvFbV25FVY\/1Bfk4uNWzuz5W2xAjiRNabCjZKtX8A9b579s2bJS9vXBfovifnz2D+Hh5w7DyNoTA\/j7elHqNJRsZc\/f1x77\/keoLll43NGjR4+aFsjVdf6nXv+vgHv7MPxNa9Td+xn+7tZN1pJRqlsvPH+X1\/kf\/kJX0B59XuPP3mFW5Lj9PCUgUrKV4W+x3zV8\/HJ4\/ZrOIAefl3ky\/C02IgeSpgRESrZ6AX\/sHy6GfRj+DpDLQhEoAYIHdgsNyJEkvYG\/GADEAe5C3zInfEde\/mfw2wlzg6L4vMyTAeFIb7ZUDEqDHSVbvYK\/pbafOtm6z34D3ryoheGfWkE3H6QECB7Y3WyDeZSK4Z+HihFpjPjirfDWyE8Gv\/q8xp8BYbEROZA0pcGOkq0Mf4ud6yOTvwe\/O2O098s8Gf4WG5EDSVMCIiVbGf4WOxfD36K4JSZNCRA8sJfY0CxnzfC3KHDtrAeC1H1f48+AsNiIHEia0mBHyVaGv8XOJeDfPm4ErGofYzGn8pOm1Gko2coDe\/l9y1YJGP62lAUAAf9\/\/9uL4U8aT7OYU\/lJUwIiJVsZ\/uX3LVslYPjbUlaCP3r96P37fFECIiVbGf7+9lqGPwCoh8E0NzdDR0cH1NTUDKl5PDFs0qRJA\/9eV1cHa9euhcbGxiH3Cs\/f9w+8GBD+AoLr1t+6JQ\/\/I0eOwPz582H8+PHB1s\/ib4Q6HgivXriNRG9vb+hv6r0C\/r6v8WdA+AsIrlt\/65Y8\/MOqFgG\/ffv2UO+\/s7MTGhoatM4IYPj72XE47ONnvVIb6Bj+Ie04Cv7qW0JSFxDwP\/Stq5JurfzvlIBIyVZqQKRUtwx\/Bbsi\/t\/W1jbEuw87KD5u4ziEv++HuAj5KHUaSrYy\/Cvvl0UawPCXpBGePf5T2ITvnj17YOrUqbBixQpoamoC9e+wmD+FD7wYEP4CguvW37pl+L9Xt0ngj2oCOAeAV9jkMHr+x7\/yNDy++DP+tqD3LOvr64P6+nrv7UQDKdlKzV4KdTtx4sSBfrp3716n+myqM3yzWJC0wicu7bgJYIQ\/ha972TvM0vrcf5ZSmIuSrez5AwACvL+\/P3Jtv+ieuMYf7+3q6oLa2lrAv+fMmRO7zt\/34xuFNi42JFtYpWQrakjJXrbVVq\/RS7dQzz9sEheLOXbs2ADyzzzzDHR3dw8MDOpHXuvXrw\/i\/2EXev4n7bwbTtj\/iJ7lfBcrwAqwAgUqQD7sU6DWnBUrwAqwAqxAhAKFev5cC6wAK8AKsAJuKMDwd6MeuBSsACvAChSqAMO\/ULk5M1aAFWAF3FCA4e9GPXApWAFWgBUoVAGGf6Fyc2asACvACrihQOXhjxvDLViwIFAzbu8fN+TWLwV+47B69erggbglrvJ9cecd6Odczp269orSmW78V45V4bnq2qoujY5rBy7Zp5ZF116xhQt+B1TlthxVFya7FBdRn5WGPzaWuXPnwvLlywOtxH+HHfZShJh55SF\/4IbfPsgfu8l5qDui4t8bNmwY+DAur\/LYTkfXXtV2HPSrNuDr2qp+CS+39Sq1b117xUCHW7fgtzxVbctx4EdnzqX2Wmn4q\/BzbWRNC015DyMBgfb29sgP3EQ+VQWEqb0IitmzZ8Phw4chbEfYtLoX8ZyurViXS5cuhZUrVwZfuFf1MrFXdt6q2pbVehKD2vDhw4Ofrr32Wq3zSYqo70rDX93oLW7jtyLEzCOPqNPOxOlncXlUscOksRfredy4cXDvvfcOnAqXh\/a20zCxNe6QI9vlzCt9E3vDPP+oQ57yKl8R6aBdr776ahDGkk8xLCLvpDwqD3\/5pC+TYx+ThCnr9zBPX\/eNRnffpLJsC8vX1F4c4NatWwezZs2CxYsXVxL+8ltcVN2Ktoya6cz9uFSnoiymdSvu37JlC8yYMUPr+FYX7Y5r5zpOXFE2MfyLUlozH9MOI5JFWNx+++2RG99pZl\/4bSb24r1LliyBKVOmBFtau+ZJJYlnYqtYyCAmeZM2NkzKu4zfTexVz+tQN3Yso\/x55uniAoXKwx8rSOzvTzXsU1XwY92ZhAYQCNu2bQvq28XOlAQLE1vVsA\/bm6Su27+7WH+Vhr8a5tENj7jdTI5tey3CWUkTvj6sitC1V14yKNdhlUIEurbiQCfvcJvUDlxt07r2+jDYxdUBwz\/nFkp9qWcVQwFhTUB3OaD8rIudSad569qqToBWNQyia29Y2Cfu\/A4drV26x8X2WmnPHyuX2kde8ttOlCdcxY+Boj4EiprEd7Ez6cJG11b5I68qf\/Ska698fkeV7eUJX92ewPexAqwAK8AKFK5A5T3\/whXjDFkBVoAV8EABhr8HlcgmsAKsACtgqgDD31Qxvp8VYAVYAQ8UYPh7UIlsAivACrACpgow\/E0V4\/tZAVaAFfBAAYa\/B5XIJrACrAArYKoAw99UMb5\/QAH1K9Q4aWx\/oSqviW9uboaOjg6oqalJrC31Y6rEBwq+QayRHzt2rNVzGuRN1Uz0K1gOzi5HBRj+OYpJLSmX4G9SFrmeqgB\/LK\/Yv8p2G\/NhK2nbGvmSPsPfl5q0ZId6lKDYR0f+GlN4pehp406buCWvuPDkopaWlkH\/Lk4zkr1NvD\/J44z6AlT+yhvTCfvCOcoO8e94epTYOln1suV8MX35d8z7wIED0NPTA7t27Qryxt9lHRYtWgSbN28OTpzDU7hM7FY3KzTZ6jlsYEuCe9LvlpoZJ1uCAgz\/EkSvSpZJu1DK3jbahMDDz\/KFlyrvNiq2YBZ72YdtzxC3K6u6j1HY3\/JGaLLGcXZcffXVMG3aNBg5cmQQKlLtUPORBwu0M2xHVflcBZHejh07gu22w7aijrM7DP7y8ZXqnjhJbzVJcE\/6vSptl8uZrADDP1kjsnck7Z+TFGqRN95T4R\/2rDieceHChYGHLK6ocshgjCtL3KZoabxjOV8VlmE2yAPIwYMHB+3WiTZG2Y2\/hcFfPeEqavBIYxvDn053Z\/jTqetUlsohDzUsEwVceSMvsUGXCn81VCMKF7ahV1RcXt70LQ7+cUDTBaTwsPv7+4OiivCXmnbY2bvyILhz505Az129ojYyiwr7yHMAUfbp2iaXheGfqptU8iGGfyWrrfhCy\/CT4\/5yqEVAXwwSfX19IA7lDoO\/7hmtZcIfbZg6dSog9MVcQpznrwN\/XbujPP\/e3t5BE8AM\/+L7gw85Mvx9qMUCbVD3Zxfwx9DM7NmzQQ7ZJIV9EKJdXV1QW1sba0GZYR+cqFVhmzXso2s3h30KbNgEs2L4E6x0XZOTJmXlUAveixOnGI7AlTPCW8eVMPJEpzrhK08Qx8Xm85zwlaE6ffr0QeXG32RPGuEve+oiXBUV9hFp45uCPIGsTvjq2h014SveQuQBVp4nwXKI+hN5iToRk9th30Fw2Ee3d1T\/PoZ\/9evQqgVqrFuO+6uAx8nMSZMmBeVB4KxYsSKYsGxra4PW1taBg3cEONXll0kfMsUd9pE0+azmJexQBy0V\/vi3vGwTy45HbG7YsCF4a7n\/\/vsHDQ4ydMWSV1zq+eCDD8LKlSuDtxwTu8Pgf9999wUa43nGeMlLW8PmIETYCvXFwW7r1q3BwJRku85HclYbHyduVQGGv1V5OXFWACBsHkBXF53VPrpp6dzHnr+OSn7cw\/D3ox7ZCkcUCJucjlvHn1Rshn+SQvx7WgUY\/mmV4+dYgQgF1C+CRZgrjWDq3j5hYaY06arP8N4+eahYrTT+D9GUJdEtYyM4AAAAAElFTkSuQmCC","height":231,"width":383}}
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
