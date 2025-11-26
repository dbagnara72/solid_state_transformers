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

n_modules = 1;
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAh4AAAFHCAYAAADuqkXoAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ1wXtV95g9tJrFSSmwBAYTiGIxM3HbWxi6sUE2cVCFuuyPTkmQlebbbdZWudwNldsEryTVhQkNsSWMzZfloFVbV0O5IdtOFxN5OlxKXkLiKG2Kw2iYkFsgfCMXhQziERLSl9c7\/Okc+urof59z73Pfe977PO5MJ1nvuc+\/9\/f\/3nOc9X\/e8M2fOnFH8kAAJkAAJkAAJkEAFCJxH41EByjwFCZAACZAACZCAR4DGg4lAAiRAAiRAAiRQMQI0HhVDzRORAAmQAAmQAAnQeDAHSIAESIAESIAEKkaAxqNiqHkiEiABEiABEiABGg\/mAAmQAAmQAAmQQMUI0HhUDDVPRAIkQAIkQAIkQOPBHCABCwIzMzOqq6vLKzk0NKTq6+stjkpWZGJiQm3evFmtXbtW9fX1qbq6umRCjkdV8h5tLk1z+N3f\/V3V3t5uc0ihy\/T396vBwUHvGrds2aJ6enqcrnfv3r1q27ZtaufOnYXkIfd36NChzJ8PJ2gsXEgCNB6FDAsvqmgEKtkohxkPqdjXr1+vmpubM8ETdo9ZnzfsZpI0ZNLwPfXUU06NepJjXAMg59i0adPcYWU0HmUziq4xZnl7AjQe9qxYkgRyITA7O6t6e3vV\/v371cjISMWMh\/S0VOK8QVB1I9bW1mZtInSPgEujnuSYJEmgjYfLtfnPU\/QeD52nJ0+eZK9HkiSpoWNoPGoo2EW5VbPLWa7J7Do2f+2vXr1affazn\/UuWxogc9hB\/zofHx9f8L2pcdNNN6lPfvKTXpmGhgY1PDysmpqaAlGYDby\/vL83QL7XQy+tra3q3nvvVatWrfIqXLPBjtORIRt93sOHD3vXJx891HLXXXep3\/\/93\/dMh\/74WcjfNVPTmOjGziyvGy+tZTaE5j0+8MADamBgIPC8U1NT3vVNT0\/PXZMZQ5OjMH\/wwQfVH\/\/xHyt9f8Lfz1qz00NYQY2sjqt5Xn2\/\/vvSsW5sbJwzT35++\/bt84Yu9MfMD79enOHz52OUVlQeRvWMmEzkmvW1+82M\/\/ky2WqN22+\/XR04cEDJ86NjZ96z\/E2fw4xtHJegPCxKvcPrKA4BGo\/ixKL0V+JvbMwb1pVnUOPibzBERxp9bTr83wc1jFGNtnwXdm26kbjwwgvnzfHQxsO8Bmngg4yCaT78OijjEfSL2t8I+BukMK7y9zDj0d3drW699dYF7KMaevnu4osvVq+88opnrILMgJzTbCD91x6WF\/q8zzzzTKCJePTRR+fmVZj5ZjasfuPh19Lfh5mPqJyVY06cOBFqcMxr8psOvznUjf4NN9ygvva1r82rL4LMQ9Dz5TcOUiboGuXv+jxx2iaXovfKlL6SrZIbpPGokkCV4TJ1xWr+4tOVttyf+WtfftXqCs2s2M1K0vylZzZU0rjrX+RaQ5\/b\/8tacw363qxEb7zxxlDjYf4idNWJMx7SyyOfuCGPqB4Z6YV57bXXFjAxf6ULpxUrVsy7R5uhFv8wkGav4ym9G\/64y7XIfIegnhhhuXHjRu9+zR4Smwm3NsMm\/jL+f2sm2iTJ9cedW+de0P3ov4lBlXsOG2oxOep88sf0iSee8AxMUA9GmK6\/10v38pgaUefWPSI6\/+O4IIaUylDX8R6iCdB4MEMqRiDs15CuuKXCXbNmTeCKDrPM8ePHA3\/Fyo2YGvIrW69AiZscGvdLLaxhNytiOb+rDsp4mOcWEyEfs6ELaxDMhvd3fud3rI1HUA9R0HnlOvxDSWE9ClJWGlB9HSbboPP5h5yijEfYEJP\/mKjeiyDT6r83PYznNzDabIUZhLj8NONraoTF1d97ollp4xE2xGau2DJzWT+X5jCXrjhMLkHDexWrYHiiqiFA41E1oar+C6208TCXo8ZV7K6GQaIRtLzWVsdsVP2NlGiby2ltejykjDkhU\/4tSzf9PT7+hs\/VePg5+ntF\/IYHZTx09ocZHlnpE2Q8\/EM2cT0e1WA8gnrYdFz9+RfW42FqhD0bNB7VX+cW9Q5oPIoamRJeV9KhFv+QgB4zD\/v1GNQ1Hmc8ooZIzF\/hEhb5VRhmPGx1pAvbbwr0EJTfeEjjbjNpz98omz0C\/uEqaajjhlqkNyau4fZrJB1qMdM9rBfB\/0j4TYT\/17++Z7PnS9+Pzh3\/MUFDLXGPYtZDLdqk6p6iMOMR1FOkGfl7PMImA\/uHeaKGWoK4cKglLlv4vRCg8WAeVIxA1pNLoxruOOMRdW1B8x\/CjEecjnRL6\/kafvA2xkOOCVrVorX8KxPMjbdcJpfqLnfzGDnvzTff7PXGBH2EU9D92U4uFU1txvyGJ2zipXmMWUbOed9996l77rlnwURYOcZvPORvYRNV9b3GGd2gYYi4HieTY9g9RpkGs6G\/7bbbQnMrSkOuIWjSqe3kUpNLXI9fxSobnqjQBGg8Ch2ecl6c7XJacyls3HLaoAmrLkMtQjqqGz9u8qa5k2mUjpzHv\/Ty05\/+tDpy5MjcZMqgHg+zUQqbIGtq++eeBBkTswE2j5X\/1sYj6LwPP\/zwvB04ZVMzcz5JkuW0poEwG8KgpdZhy3j9XM05J6Ip3Hbt2qW2bt3q4TB7rvTqpLDluXH7b0Qtp5Vz2fYEhM3NkF6voEY9rJdHGAUtZQ7qNQkzrfJ3\/06pYXNltIZNz1w5azXelQsBGg8XWiybOYG4FQSZXwBPkIpA0JCOf+VS2D4q5omTbCCW6sJr+GDTKGqDFbTSJQ4RNxCLI8TvNYHSGA+pqGSPAdn0KKhii9pwiulQHAI0HsWJRZIriRpqihoiCjpXki3Tk1wzjwkeahEucZvuBZnFsrxbh3mRHYFSGI+4yWj6+5aWFu\/lSvrf8lC5vqgpu1BQWQjQeFR\/HvhNvtyRq+kwc4ENWWVywj8E6mI65AppFCsTpzKcpRTGQ8Y5JenlE9bj4Q+WjFWOjY1V9O2fZUgY3gMJkAAJkAAJpCFQ9cZDfl3dfffdqrOz0zMfNB5p0oHHkgAJkAAJkEC2BKreeEjPhXxkZ72oOR4mRt0V3NHR4Q298EMCJEACJEACJFAZAlVtPGQ+wCOPPKK2b9+u5MVgNsZDz+8QvObbTv24r7zyyspEgGchARIgARIggQgC8ibhK664ojSMqtp4yNCK7CEguzDGrWqRiNmaDikrxmNycrI0gc77RsgTGwHyJE8sAawa85M8owhUrfEImjmvbzTo9dWuK1n44PDBwRLAqjE\/yRNLAKvG\/CTPUhoP\/03F9XhI74js5hc1vGJq8sHhg4MlgFVjfpInlgBWjflJnjVpPHQPh6x2WbFihfcmUb0tsgYStfU0Hxw+OFgCWDXmJ3liCWDVmJ\/kWRPGAxtmzvFA8zx27FipJkeh+bjqkacrsejy5EmeWAJYtbIZuaqd44EN60K1sgU6a15x+qzY4wi5fU+ebrziSpNnHCG378nTjVdc6bK1RzQeIREvW6DjEjvr71kRYQmTJ3liCWDVmJ9YnmVrj2g8aDywT0iIGisiLGbyJE8sAawa8xPLk8YDy7OwamULdN6gWRFhI0Ce5IklgFVjfmJ5lq09Yo8HezywTwh7PMizIgSwJ2FDSZ5YAli1pdf9qjr5jb\/EiuaoRuNB41GR9GPFjsVMnuSJJYBVY37ieI4+fUrdMvqcmrn3wzjRnJVoPGg8KpKCrIiwmMmTPLEEsGrMTxxPGg8cy8IrlW1MLW\/grIiwESBP8sQSwKoxP3E8aTxwLAuvROOBDRErIvLEEsCqMT\/JE0sAp9b\/+HHV\/\/gxDrXgkBZXicYDGxtW7OSJJYBVY36SJ5YATo3GA8ey8Eo0HtgQsWInTywBrBrzkzyxBHBqNB44loVXovHAhogVO3liCWDVmJ\/kiSWAU6PxwLEsvBKNBzZErNjJE0sAq8b8JE8sAZyaLKWVCaZcTotjWlglGg9saFixkyeWAFaN+UmeWAI4NRoPHMvCK9F4YEPEip08sQSwasxP8sQSwKnReOBYFl6JxgMbIlbs5IklgFVjfpInlgBObeODz6qDL5zmUAsOaXGVaDywsWHFTp5YAlg15id5Ygng1Gg8cCwLr0TjgQ0RK3byxBLAqjE\/yRNLAKdG44FjmavSxMSE6u7uVgMDA6qpqSnwWmg8sCFixU6eWAJYNeYneWIJ4NRoPHAsc1OanZ1Vvb296vDhw2p4eJjGo0KRYMWOBU2e5IklgFVjfuJ4rr7n6+rkzFuc44FDWnmlQ4cOqf7+fu\/E7PGoHH9WRFjW5EmeWAJYNeYnjmf97U96YtzHA8e0okozMzPq7rvvVp2dnZ75oPGoHH5WRFjW5EmeWAJYNeYnhqf0dEiPB40HhmcuKnv37vXOu2bNGs7xqHAEWBFhgZMneWIJYNWYnxieB58\/rTY+9CyNBwZn5VVkQukjjzyitm\/frqampqyMh3mVBw4cqPxFl+iMwryxsbFEd5TvrZAnlj95kieWQDq11tZWT+Dti65Wb67rpvFIhzO\/o2VoZf369aq5uVlxVUvl48BfQFjm5EmeWAJYNeYnhqd+QZyocY4HhmnFVGRuR1dXlxofH19wzpGREc+M+D9cTosNDysi8sQSwKoxP8kTSwCjppfS0nhgeOaqwh6PyuNnxY5lTp7kiSWAVWN+YnjqpbQ\/9eNX1at\/9AmMaAFUzjtz5syZAlxHRS+BxqOiuL2TsSLCMidP8sQSwKoxP9PzNFe0vOOVb6uX\/\/SW9KIFUahJ42HDnkMtNpTsy7AismdlU5I8bSjZlyFPe1Y2JcnThlJ0GXNFy\/kHB9TJb\/xletGCKNB4hASCxgOboayIyBNLAKvG\/CRPLIH0aub8jgv+qkcd\/4dvpBctiAKNB41HRVKRFTsWM3mSJ5YAVo35mZ6n3rF03fLF6h92f0xNTk6mFy2IAo0HjUdFUpEVERYzeZInlgBWjfmZjufo06fULaPPeSIPdq5U29tbaDzSIa2OoznUgo0TKyLyxBLAqjE\/yRNLILmaTCqV3Url\/5fWL1JH7rxela09Yo8HezySPyEOR7Jid4BlUZQ8LSA5FCFPB1gWRcnTAlJIEbO3o2fDFapnwzIaj+Q4q+vIsjnMvOmzIsJGgDzJE0sAq8b8TMbTXEKreztEqWztEXs82OOR7AlxPIoVkSOwmOLkSZ5YAlg15qc7TzEdt44+pw6+cNo7WOZ2dF57qfffNB7uPKvyiLIFOu8gsCLCRoA8yRNLAKvG\/HTn+fmvTanexya8A2Uly75brpkTKVt7xB4P9ni4PyEJjmBFlABaxCHkSZ5YAlg15qcbz6eOvq5+44+OeAfJEMu+T13j\/b\/+0Hi48aza0mULdN6BYEWEjQB5kieWAFaN+WnP82vPn1Y3PfRsqOmQL8rWHrHHgz0e9k9IipKsiFLACziUPMkTSwCrxvyM5ylzOmQFS\/\/jx+ZMhyydDfrQeMTzLEWJsgU676CwIsJGgDzJE0sAq8b8jOZp7tUhJWVY5YGOlWrdVYtpPLCpWF1qNB7YeLEiIk8sAawa85M8sQTC1cyXv2nT4Z\/T4T+6bO0Rh1pC8qNsga7UQxV2Hlbs2AiQJ3liCWDVmJ8Lefp7OaTEB5uWqC\/+19Wx8MvWHtF40HjEJj2iACsiBMVzGuRJnlgCWDXm5zme\/v05dC+H7Eqq9+mIo0\/jEUeoJN+XLdB5h4UVETYC5EmeWAJYNean8t61Ym4Ipgn\/t9b3q7v+3ZVOwMvWHrHHgz0eTg9A0sKsiJKSCz6OPMkTSwCrVqv5KWZD\/jfw+LG5HUg12aD9OWyp03jYkqrycmULdN7hqNWKKCvu5IklS57kmYaAmI0D35lRd\/z5dxfIpDEcWqxs7RF7PNjjkeZ5sz6WFbs1KquC5GmFyboQeVqjsipYCzzjejd+afliJfM4zB1IreAFFKLxSEoug+NmZ2dVb2+v2r9\/v6e+ZcsW1dPTE3qmvXv3qm3btnnft7W1qb6+PlVXVxdYvmyBzgC\/k2QtVEROQFIWJs+UAH2Hkyd52hKQ5bBBQylyvN6PQ\/4fYTjY42EblQqW6+\/v984mZmNmZkZ1dXWpjo4O1d7evuAqDh06pKT80NCQZzbEsDQ0NIQaFRoPbCBZsZMnlgBWjflJnlEE4szG+qYl6hNrLw3dACwt3bK1R6UaajGNiD\/Q0tsxNjY218vh\/7e\/fNkCnTbx0x7Pij0twfnHkyd5Yglg1ao5P\/UQyuSrP1Z\/fvj7CyaJ6p6NpUsWqe4NV2RmNsyIlK09Ko3x0D0e0vvR3Nxs1ePR0tIS2DsiB5ct0NhqxV2tmisi97vN\/gjyxDImz9rmKWZDPkHLXzUZGTqReRud115WEbNB44HNSbia9HQMDg7GztuYmJhQmzdvVtPT02pkZCTQoOiLE+Nhfg4cOAC\/7loSnJqaUo2NjbV0y5neK3li8ZJnbfGcfuNt74Y\/8+VX1eGXzpqOoE\/DBe9Qv3b1z6jr3len1l5+7jX1WFoL1VpbWxf8cXJyMuvTVky\/oj0euldifHzc6QZXrVqlHnvssdhjxICIqQiaNCpDK3v27PHmeNTX13vzPeQTNhmVPR6xuJ0K8BelE67YwuQZi8ipAHk64YotXDSeMkfjzX98Wz30lRcDh070DXmTQpcsUg90rvT+hJwgGgstokDZ2qNcjEfYcEgQdz0p1MZ4SI9Gd3e3GhgYUE1NTXNyevWLObQSVlYfVLZAp0l6xLFFq4gQ95SnBnli6ZNnuXjK0Ml9f31STXz\/R1ZG4\/7Oleq8AhkNfzTK1h6VyniYK1ekV0N\/aDywlUoSNVbsSaiFH0Oe5IklgFWrZH5Kb8aJmVm19+lTkSZD92DoSaHoJa9YgvPVaDyypOuobQ6XaHMRtkQ2aKglbFhGLqNsgXZECy9eyYoIfvEFFCRPbFDIszp4Sk\/GV47OhK428d+FmIvWD9Sr31h9iTdsUpShE1faZWuPcunxkDkecZt92QTGv4GYuSmY\/q6zs3NuEqmehCra3EDMhjCuDCt2HEtRIk\/yxBLAqqXNT73K5N4vn1CTr\/w4tifD35ux7qrF2BvKWY3GAxAA0wBID8Xw8PC8ORmAU6SWKFugUwNJKZC2Ikp5+tIdTp7YkJJnfjxlqOTd7\/wp9Zn9L1gZDNNk3Pvvr1anfvBPVd2bYUO+bO1RRXs8\/ID9q1wQvSA2QbQpU7ZA29xzlmVYsWPpkid5Yglg1cz8lN4LGeIQg3H8tVn19cnT6sWZt5xMhn7vidaq1iGTpJTL1h7lajzMIJh7bBShF6RsgU6a8Kjj2FCiSJ7VIU\/yxBLAqOkhkr\/5+0l1cPo8Z4MhV\/Hrq9+rPvKBC0vfi+FCvGztUWGMhxmEsNUpLoFKW7ZsgU7LI+3xbCjTEpx\/PHmSJ5aAm5o2GN+aflP94VMvqpOvv6X032yU9C6g8vbWWu3FsOGky5StPSqM8TB7PAR23M6iLkFLUrZsgU7CAHkMG0okTfZ4YGmSZxhPGR6Rzxee+b46ZjnJ09QSg\/FBeYHamkvU+y88+ybwWhsmQeRq2dqjXI1H1KoURLDSaJQt0GlYII6l8UBQPKdBnuSJIqB7Kob+5iX17Mk3EvVeyLXccNUS1f6Ll3qXdd6PXlG\/tOrcJo6oa61VnbK1R7kYD3NVSxF6N4KSuWyBzvuBZUOJjQB5kqcLAf3G1b89\/gP11HdnnM2F7qmQzbc+vvYSdeVF746cg8H8dIlOfNmytUcVNR7mKpa4fTTiQ5FtibIFOlta8eqsiOIZuZQgTxda8WXLwFOGRWQYY\/CrU+rvX\/phYnMhtGRy59r3X+C9jVWbjniK7JFzYeRStmztUUWNx\/PPP69uu+02ddddd0W+GdYMiMu7WlwCGVe2bIGOu9+svy9DxZ41Ixd98nShFV+2GnjqIRF52dmDX3nRacWISUDPsVh31RLV\/dFlmUzurAae8VlRnBJla48qajx0j0dWL4lDpknZAo1kk0SLFVESauHHkGf5eJqrQh566kX17ek3E\/VamD0U5ryLSu7myfzE5mfZ2qNcjIdsme7yWbVqlbJ5O62LZlzZsgU67n6z\/p4VEZYweVYnT20uvnjkZfXl517zbuLgC2dXjrh+9CvcpeeixRgSKcKqEeanazSjy5etPaqo8cCGIlu1sgU6W1rx6qyI4hm5lCBPF1rxZRE8zR6Lv\/zWq+ov\/u6V1MZCBH756nr1Kz9\/kXr3O396bilqEcxFFFUEz\/io1U6JsrVHNB4huVu2QOf9iLIiwkaAPCvP0zQWf\/bNU+qrE69DjIWsFPnoz1+oVjdeUDXGIo4+8zOOkNv3ZWuPaDxoPNyegISlWRElBBdyGHnief70ey6bE\/2DAyfU8y\/\/OPEcCxHSvRJiLK674j3qQyvqS2Ms4ugzP+MIuX1P4+HGq2pLly3QeQeCFRE2AuTpzlPvwjn9g39UTx2d8VaFuG71bZ7VNBYyx0LmWsz9rX6R+wWW6AjmJzaYZWuP2OPBHg\/sE8Jf6ORZEQILT6I3yfrS+Mvqu6d+lMpUmD0WspfFte9\/j7rqvWc3zTK\/y+lWC39aGg9siGg8sDwLq1a2QOcNmhURNgK1xNOcW7Hnm6fUwZRzK3Qk9KqQ99UvUjdcfkY1NjTQWIDStJbyE4QsUqZs7VHuPR579+5V27Zt86DLi+FOnDihxsbGVF9fn6qrO\/tSoTw+ZQt0HgzNc7IiwkagDDz1W0n1jpsHn39d7Xn6lAcq6RJT01R4PRNLFqlfXPYeb2WI2VPhXxVSBp7YDEunRp7p+PmPLlt7lKvxkHe2TE9Pq+7ubnXrrbcq2VhM9uzo7e1VDQ0N3r\/z+pQt0Hlx1OdlRYSNQDXw1D0V8v+Pf\/tVNf7iD+GmYsUlP6NuWn2xOk+dl6q3ohp4YjMoWzXyxPItW3uUm\/EwdzFdsWKF6urq8oxGc3Oz0tukDw0Nqfr6s79Ugj7+t9tu2bIl0qyI7qZNmzwpMThR+mULNPYxcFdjReTOLOqIvHmawx8jT39PjckmWGewPRXyGvXfWP1eb26F7h3Jav+KvHlisyN\/NfLExqBs7VFVGw\/pMZGPGBZtZDo6OlR7e\/uCqE9MTKjNmzerXbt2eeZGhniihnTKFmjsY+CuxorInVmexkOGPy5f\/C4lr0r\/uylMT4Xcj55X0XTJz6iPrqxXKy87P3NTYUOe+WlDyb4MedqzsilZtvYoN+MhsHXjbw616N6PMAMRFSTTiPjLybmOHz9uPXxTtkDbJHeWZVgRYekm5annUzxz8g31laOvq8lX0u1Voe\/KXFoqPRVt\/+bieTttatOBpYBTS8oTdwXlUiJPbDzL1h7lajwkNObwhw7Vzp07A3stokIZ9QI6PSTT0tJirVu2QGMfA3c1VkTuzFx7PPRy0iePzqinj\/3AOzztJE3TMMhETTEV\/yOjN5piCbmpMT\/deMWVJs84Qm7fl609yt14uOEPLi09HYODg6qtrS1wNYw2Hhs2bFAPP\/ywkpfUcY4Hgry9Bisie1a6pLnqQ\/72L\/96Rn3hme+rk6\/NqslX3lTTb7ztLuo7wuypaF15oXrvz75Tyb4V+pPVnIrUFw4WYH5igZInlieNB5YnVE2vkvEvxdXG4+TJk3MTSsPK6guSQJufAwcOQK+11sSmpqZUY2Njrd126P2apuHk6X9WXz85q557+Z+88odfeis1p4YL3uFpXPaz71Dy363L363e\/c6f8v629vLa3lUzCC7zM3XKzRMgz3Q8W1tbFwhMTk6mEy3Q0bn1eOihEel9iPrErVQxj5UJpDJfZGBgQDU1Nc19FTTUElbWNB5lCnTeOVdrv4D0XIovHP6+tz23fNJsz23GT3oh3n77bXXlxeerG5qWqOuvPNtDwV01k2d5reVnclJ2R5KnHSfbUuzxsCVlUU4mfO7Zs2feslZzdcrGjRud9vSIWoYrPRzLli2bm+MhxmPHjh1q9+7dgUt2yxZoi3BkWqQsFVHWhsIzEEsWqZ+\/\/Hz10ZUXquUXBy8lLQvPTJPOQZw8HWBZFCVPC0gORcrWHuXe46H37jBjYBqIo0ePKjENjz322IIwmatYdK9G2MZjflMStQJGTlS2QDvkeCZFi1gR6TkUelKm9Bh88Sfv+Uj7AjF\/D4U2FLI993\/4t5epf\/nXsyXWXXVuPoUL+CLydLn+opUlT2xEyBPLs2ztUVUbD\/8GYubkUv1dZ2ent2+HfMwVNGETUXW6lC3Q2MfAXS2PisjcOfObJ95QEy\/\/KPUbSf13br7vY83SC7wlpJWYnJkHT\/eoV88R5ImNFXlieZatPcrNeEhY4oZaZCMwvdfHfffdh41kjFrZAl1ReAEnQ1dEeshjbPK0+trE65kaisve8y71oRXn3vVRhJUeaJ5550fe5ydPbATIE8uzbO1RrsbD3wuhQyUvi9O7i95\/\/\/1qeHh43mRRbEiD1coW6EowizqHbUWkDcUTz72mnn3xh97SUdSkTLk+bRrWXbVELb+4Tn3smksWXHYRjEVcvGx5xunw+7MEyBObCeSJ5Vm29ih344END06tbIHGkXFXkiGPF198Ub3vfe9T3\/7em+qvvv2aev5lzI6Z+mrMIY+1Sy9Q8vIw+cgcCj3kUg2GwpYuK3ZbUnblyNOOk20p8rQlZVeubO0RjUdI3MsWaLv0divlf635G2+9rQa\/OqVOAHspzA2uPnR1vZJhj0rMoXAjUfnSrNixzMmTPLEEsGpla49yNR76xW3T09MLohS3syg2rAvVyhboNLx0j8FrP\/pn9b8OTnnzKdJuxW0aiuuXL1Y3XLXEu0TuRWEXKTaUdpxsS5GnLSm7cuRpx8m2VNnao9yMh7mpl96vQ1ag6JfEBS2ztQ0SolzZAh3FxOy5aFj8LrX7ieOpzIU2D9IzIS8LO\/9d71CnTn1PXfdzV8wzF4g41aoGK3Zs5MmTPLEEsGpla49yMx7+l7qZG3zJstfR0dHA965gwxmuVrZA++9UJnF+4fApdezVWefeCz2fYtl42s\/LAAAgAElEQVRFdWrrjcusXmvOih2bueRJnlgCWDXmJ5Zn2dqjwhgP87X1UTuQYsNZfuOhh0nkTm8dfc7aZGhz0fMrV6gzZ84OgaSZnMmKCJu55EmeWAJYNeYnlieNB5CnuXuoaTaeeOIJNTY2xh6PhKz1TpyjT39PjT59KlJFG4yb11yifvnqc3tVJDx16GGsiLBEyZM8sQSwasxPLE8aDyBP\/8vb9OvtZdvzPPbuMG+t2gKtzcbA48dCezW0yfjY2kvUh1fUp+rBcE0DVkSuxKLLkyd5Yglg1ZifWJ7V1h7F3X1uQy1xF5b399UU6EeffVl98k+\/FYhMzMa+T13jfZdmqCRtPFgRpSU4\/3jyJE8sAawa8xPLs5raI5s7z814+CeXmhfLOR42oVPq1j3fUSPf+N6CwuuWL1bdG65I\/AIyu7O7lWJF5MYrrjR5xhFy+5483XjFlSbPOEJu39N4uPEKLU3jkQykDKnIvI3+x4\/NE5DejAc6VhbKbJgXyIooWbzDjiJP8sQSwKoxP7E8aTxS8pTVK9u2bYtV2bJli5K9PPL6FC3Qeg7HxoeerSrDoS+WFRE2k8mTPLEEsGrMTyzPorVHae+ukEMtaW8KcXyRAi2mQwyHuTS26D0c\/hiwIkJk5TkN8iRPLAGsGvMTy7NI7RHiznIzHoiLz1KjKIGWjb7MXo5qMxzs8cgmS1mxY7mSJ3liCWDVitIeoe6KxiOEZN6Blt6NP\/vmKbXj\/52by7G55XK1++MrULGvqA4rdixu8iRPLAGsGvMTyzPv9gh7N0pV1HjoCaXj4+Ox91HLL4kT02HuMlqtvRxmkFkRxaa8UwHydMIVW5g8YxE5FSBPJ1yxhWk8YhGVo0BegfbP59D7cOS5BwcioqyIEBTPaZAneWIJYNWYn1ieebVH2Ls4p1bRHg\/0TeidT\/fv3+9J266EmZiYUN3d3WpgYEA1NTUFXlYegS6r6RDArIiw2U+e5IklgFVjfmJ55tEeYe9gvlruxiNoee3OnTtVe3t77H2b73rRwzgdHR2Rx2qzcvjw4cht2Ssd6KDhlSN3Xh\/LoFoKsCLCRoo8yRNLAKvG\/MTyrHR7hL36hWq5Gg8xHXv27FFDQ0Oqvv7sC8psDUQQGNOIhIHTu6LK90Xp8RDTIRuC6Re6ybBKmUwHezzwjzErdixT8iRPLAGsGo0HiCd659IoPX3JUubuu+9WnZ2dSkxKUYzHyNOnvMmk8inLnA5\/mrBiBz04P5EhT\/LEEsCqMT+xPGk8QDyRxkO\/1batrU319fWpurq6wKuUHhb5rFmzpjBzPKS3Y\/U9Xy+16WCPB+ihMWRYsWOZkid5Yglg1Wg8gDyzGGqZnp4ONB8yofSRRx5R27dvV1NTU1bGw7zVAwcOAO\/8rNT0G2+rLY+d8v5fPp+\/+VK19vJF8PMUQVCYNzY2FuFSSnEN5IkNI3mSJ5ZAOrXW1tYFApOTk+lEC3R0rnM8hEOayaV+jlGrVaRXZP369aq5uVkVZVXLxgefVQdfOO3dxq6Pr1C\/3XJ5gVIDeyn8RUmeWAJYNeYneWIJYNXY44HlCVXTE0fNyapygqiNy0ZGRjwz4v9kHWj\/EEvZJpP6ebJih6Y6lydjcZIneYIJYOWybo+wVxuvlluPR5rVK\/q2zFUseplsQ0ND7Ftt8+7xMPfrKOtkUhqP+IcvTQkauTT0Fh5LnuSJJYBVo\/EA8vQPs4T1PoSd0r+BmDm5VH8nK1j8PRp5G49bRp+bWzr7QMcH1KbrLgNSLaYUK3ZsXMiTPLEEsGrMTyxPGg8szzk1vTJF\/iC9FsPDw6G7imZ0CfNkswp0rQ2xaKisiLBZS57kiSWAVWN+Ynlm1R5hr9JeLbehlqhLFBMi8zX8czXsbyt9yawCbU4ofbBzpeq89tL0F1sFCqyIsEEiT\/LEEsCqMT+xPLNqj7BXaa9WGONh9njk\/WZawZdFoGu1t0N4siKyfyhtSpKnDSX7MuRpz8qmJHnaULIvk0V7ZH92fMlcjUfRhldMvFkE+mOD4+rJ7854p5FVLNX+xlmXdGRF5EIrvix5xjNyKUGeLrTiy5JnPCOXElm0Ry7nR5fNzXjYbHGOvlkXvSwCXX\/7k94lrFu+WO275RqXy6n6sqyIsCEkT\/LEEsCqMT+xPLNoj7BX6KaWm\/Fwu8zKl0YH+tf\/8Ij66sTr3o3s+9Q1at1Viyt\/UzmekRURFj55kieWAFaN+YnliW6PsFfnrkbjEcIMHWjd21HGN8\/apB0rIhtK9mXI056VTUnytKFkX4Y87VnZlES3RzbnzLIMjUcFjMf\/\/tvvqdv2fsc7Uy2tZDHRsiLCPsbkSZ5YAlg15ieWJ40Hlmdh1VCBruWVLDQe2aU3K3YsW\/IkTywBrBqqPcJeVXK13Ho8oiaXhr1zJfltuh+JCvTB50+rjQ89W9O9HXLzrNjdczDqCPIkTywBrBrzE8sT1R5hryq5Go1HxkMtesMwmdvxQMfKmptUqvGyIkr+kAYdSZ7kiSWAVWN+YnnSeKTk6X8\/S5jcli1bYl\/2lvJSIg9HBNocZqnFJbQcaskuQ1mxY9mSJ3liCWDVEO0R9orSqRWyxyPdLWGORgS6\/\/Hjqv\/xY94F1eISWhoPTC6yxyM7juyRy4YtjRyWK6I9wl5ROrXcjEe6y87+aESgzWEW2am0lj+siLDRJ0\/yxBLAqjE\/sTwR7RH2itKp0XiE8EsbaHNSqbwITpbR1vKHFRE2+uRJnlgCWDXmJ5Zn2vYIezXp1SpqPMyVLCtWrFBdXV1qfHw88C7yflFc2kB\/8cjL6rf\/5FvevdX6MIswYEWU\/mE1FciTPLEEsGrMTyzPtO0R9mrSq1XUeKS\/3MoppA00h1nmx4oVETZ3yZM8sQSwasxPLM+07RH2atKr0XhkMNRiDrPc\/pH3qzt\/7cr0kapyBVZE2ACSJ3liCWDVmJ9YnjQeIJ562KWMQy2jT59St4w+x2EWI1dYEYEenJ\/IkCd5Yglg1ZifWJ40HlieC9TEkNxxxx3q937v91RTU1PGZwuXTxNoDrMs5MqKCJvK5EmeWAJYNeYnlmea9gh7JRi1Qg61yJbpo6Ojqq+vT9XV1YXe6ezsrOrt7VX79+\/3ykRtOubvYWlra4vUTxroeZuGXbVE7fvUakykqlyFFRE2gORJnlgCWDXmJ5Zn0vYIexU4tcIaj\/7+fjU0NKTq6+tD71bKyKenp0dpY9HR0aHa29vnHaMNSktLi\/ed\/ndDQ0Po7qhJA81hluBwsSLCPbSiRJ7kiSWAVWN+YnkmbY+wV4FTK6TxEEMxPT0d2+Phx2AakThEsnX72NhY6DmSBprDLDQecbmH+J4VO4LiOQ3yJE8sAaxa0vYIexU4tdyMR9TkUumJGB4edprjEfW22yBcWRgPvpslPDFZseMeWvZ4YFmSJ3niCWAVaTyAPMOGQPSQiO2ppKdjcHBQxc3b0HpRwzK6jATa\/Bw4cCD2cg6\/9Jb6z4+e8sp95iMXqbaV58ceUysFpqamVGNjY63cbub3SZ5YxORJnlgC6dRaW1sXCExOTqYTLdDRufV4CIOgIRUbUxDGz2aIRpsd0YiavJrEYR74zoz6xOfP7sTK3UrnR4k9HtinnjzJE0sAq8b8xPJM0h5hrwCrlpvxiBoasV3V4kcxMTGhuru71cDAQOAwja3pEN0kgeb8jvDkZEWEfXDJkzyxBLBqzE8szyTtEfYKsGqFNR42q1r8KMSwhB1ns5LF1HMNNOd3RCcmKyLsg0ue5IklgFVjfmJ5urZH2LPj1XIzHv75HeatxU381GXNVSxxxsJmGCaN8TC3Se\/ZcIXq2bAMH60qVmRFhA0eeZInlgBWjfmJ5UnjAeQpPRRbt26dt4JFhks2b96sdu3apZqbmyPP5t9AzJxcqr\/r7OxUYW\/CjXoDrmug+x8\/rvofP+Zd75E7r1dL6xcBSVW\/FCsibAzJkzyxBLBqzE8sT9f2CHt2vFpuPR76VsR8bNq0ad6djYyMxJoOPIr5iq6B5vyO6IiwIsJmLHmSJ5YAVo35ieXp2h5hz45Xy9144G8Jo+gSaM7viGfOiiiekUsJ8nShFV+WPOMZuZQgTxda8WVd2qN4tfxL5GY8zKGQuCGVPDC5BNo0HpzfERwtVkTYLCZP8sQSwKoxP7E8Xdoj7JmzUcvNeLjuNJrN7YerugTanN\/B\/TtoPCqRq6zYsZTJkzyxBLBqLu0R9szZqOVmPOR2bFevZHPr0aougeb8jvgIsWKPZ+RSgjxdaMWXJc94Ri4lyNOFVnxZl\/YoXi3\/ErkZj6h3tQiWqBUnlcDmEuj625\/0LklWssiKFn4WEmBFhM0K8iRPLAGsGvMTy9OlPcKeORu13IxHNreDU7UNtDm\/495PXK3+0\/UNuIsokRIrImwwyZM8sQSwasxPLE\/b9gh71uzUaDxC2NoG2tw4jPM7whOVFRH2ISZP8sQSwKoxP7E8bdsj7FmzU6uo8TAnlIZt6qVvtVqGWobHptUdf\/5d77JpPGg8sntU5yuzYseSJk\/yxBLAqtF4YHkWVs020JxYahdCVux2nGxLkactKbty5GnHybYUedqSsitn2x7ZqeVfqqI9Hv7b9b+vJer9LZVGZRtoPbF03fLFat8t11T6MqvmfKyIsKEiT\/LEEsCqMT+xPG3bI+xZs1PL1XgEvbhND8d0dHSo9vb27O48Rtkm0Nw4zD48rIjsWdmUJE8bSvZlyNOelU1J8rShZF\/Gpj2yV8u\/ZG7GI2oDMXl\/y+joqOrr61N1dXW5ULIJtGk8OL8jOkysiLBpTJ7kiSWAVWN+YnnatEfYM2arVljjIb0hQ0NDqr6+PlsCIeo2geaOpfahYUVkz8qmJHnaULIvQ572rGxKkqcNJfsyNu2RvVr+JXMzHlHzOYqwo6lNoDmx1D6BWRHZs7IpSZ42lOzLkKc9K5uS5GlDyb6MTXtkr5Z\/ydyMh9y6DKls3bpVDQ8Pq6amJo\/GxMSE2rx5s9q1a5fK8+VxNoHmxFL7BGZFZM\/KpiR52lCyL0Oe9qxsSpKnDSX7Mjbtkb1a\/iVzNR7afGzatGkeiZGRkVxNh1xMXKA5sdQteVkRufGKK02ecYTcvidPN15xpckzjpDb93HtkZta\/qVzNx75Iwi+grhAc8dSt8ixInLjFVeaPOMIuX1Pnm684kqTZxwht+\/j2iM3tfxL03iExCAu0JxY6pa8rIjceMWVJs84Qm7fk6cbr7jS5BlHyO37uPbITS3\/0jVjPPRk1v3793vUt2zZonp6ekIjEBdoPbFUBGbu\/XD+kSz4FbAiwgaIPMkTSwCrxvzE8oxrj7Bny16tZoyHLM+Vj5gNm03K4gLNFS1uycmKyI1XXGnyjCPk9j15uvGKK02ecYTcvo9rj9zU8i9dM8bDj9o0IkFhiAs0V7S4JS8rIjdecaXJM46Q2\/fk6cYrrjR5xhFy+z6uPXJTy790TRqPqF1TdUiiAs0VLe6Jy4rInVnUEeRJnlgCWDXmJ5YnjQeQp96zY3p6eoHqqlWrMtm5VHo6BgcHVVtbW+SW7FGBNle0PNi5UnVeeymQSjmlWBFh40qe5IklgFVjfmJ50niAeOrJng0NDZGTPEGnWyAT9II6s5AE2vwcOHBg7p\/7n3tTfebLr3r\/\/vzNl6q1ly\/K6jJLozs1NaUaGxtLcz953wh5YiNAnuSJJZBOrbW1dYHA5ORkOtECHZ3bUIvNcEeWnKS3pbu7Ww0MDMztmuo3HmGB5ooW98jwF5A7s6gjyJM8sQSwasxPLE\/2eIB46h6Pzs7OXHYple3ao15EFxVormhxTwJWRO7MaDywzMiTPCtHAHsmGg8gz7jGH3gqz2TIR5bT2gzzRAWaK1rcI0Pj4c6MDSWWGXmSZ+UIYM9E4wHiqYdaxsfHAxXRk0v9G4glnVzKFS3JEoDGIxm3sKPIkzyxBLBqzE8sTxoPLM\/CqoUFmsYjWchYESXjRuOB5Uae5FkZAtiz0HhgeRZWLSzQfEdLspDReCTjxoYSy408ybMyBLBnofHA8lR79+5V27Zt81RHRkbUiRMn1NjYWOQeG+BLCJQLC\/Qto8+p0adPecccufN6tbSeS2lt4kHjYUPJvgx52rOyKUmeNpTsy5CnPSubkjQeNpQsy+i9NGRZ66233upN\/JS5Hb29vSqv\/T30pYcFmitaLIPrK8aKKBk3\/kLHciNP8qwMAexZaDxAPM19PFasWKG6uro849Hc3Kwqudol7HbCAr36nq8rmechPR3S48GPHQEaDztOtqXI05aUXTnytONkW4o8bUnZlaPxsOMUW6oajYc5sXTd8sVq3y3XxN4nC5wlwIoImwnkSZ5YAlg15ieWJ40HkKfM75D5HOZQi+796OjoUO3t7cCzuUkFBdo0HnxHixtPVkRuvOJKk2ccIbfvydONV1xp8owj5PY9jYcbr9jSMqyyadOmeeV27tyZq+mQiwkKNF8OFxvO0AKsiJKzCzqSPMkTSwCrxvzE8qTxwPIsrFpQoLmUNnm4WBElZ0fjgWVHnuSZPQHsGWg8sDwLqxYU6M\/83xfU\/\/zrk941cymtW+hoPNx4xZUmzzhCbt+TpxuvuNLkGUfI7XsaDzdesaXNfTx0YdnPQ1a35PkJCjSX0iaPCCui5Oz4Cx3LjjzJM3sC2DPQeAB5iunYs2ePGhoaUvX19Z6yXu1SxMmlXEqbPPg0HsnZsaHEsiNP8syeAPYMNB4gnuZyWn\/vRlH38eBbaZMHn8YjOTs2lFh25Eme2RPAnoHGA8Sz2owHXw6XLvA0Hun4+Y8mT\/LEEsCqMT+xPGk8gDylZ2Pr1q1qeHhYNTU1FXqoxVxK27PhCtWzYRmQRPmlWBFhY0ye5IklgFVjfmJ50niAeOoej\/Hx8VhFeX\/LY489FlsOWcAfaNN47PvUNWrdVYuRpyu9FisibIjJkzyxBLBqzE8sTxoPLM\/CqvkDzT080oWKFVE6fhxqwfIjT\/LMlgBWncYDy7Owav5A3zL6nBp9+tTZ4aB7P1zY6y7qhdF4YCNDnuSJJYBVY35iedJ4YHmqoH08irhlOvfwSBd4VkTp+PEXOpYfeZJntgSw6jQeQJ5p9\/HwzxNpa2tTfX19qq6uLvAqTZMTV9YfaL2HB99KmywBaDyScQs7ijzJE0sAq8b8xPKk8QDxTLucdnZ2VvX29qqWlhbvhXL63w0NDaqnp2fBVZp7g4gxkWPDysrBZqDNpbQ0HskSgBVRMm40Hlhu5EmelSGAPQuNB4hnWuMRdBnSozE2NhbY6+H\/LqpslPHgUtpkCUDjkYwbG0osN\/Ikz8oQwJ6FxgPIM+1Qi\/9SosxEUI+H7i0JuqWwHg8aj2QJQOORjBsbSiw38iTPyhDAnoXGA8sTNrnU5h0vExMTavPmzWp6elrFvYjODLSsZpFVLfLhHh7JEoDGIxk3NpRYbuRJnpUhgD0LjQeWJ0RNz+8QsbDJpf7elf7+fu\/cQfNB5O8SaP156wM3qbc+sNH75+dvvlStvXwR5LprSWRqako1NjbW0i1neq\/kicVLnuSJJZBOrbW1dYHA5ORkOtECHX3emTNnzhToepwvxcZ0+Ceiykmk96O7u1sNDAzMbdduntx0mOYeHkfuvF4trafxcA0UezxciUWXJ0\/yxBLAqjE\/sTzZ44HlmUotbiWLFk9rPLiHR6oweQezIkrP0FQgT\/LEEsCqMT+xPGk8sDxTqclwiczXiNq7Q58gaKgl6lgz0DQeqcJE45Ee3wIFVuxYqORJnlgCWDUaDyzPxGphL5mTF8oNDQ15m4jJXh2dnZ2qubnZO48YlcHBQe+\/XTYQq7\/9Se8Y7uGROFzs8UiOLvBINpRYoORJnlgCWDUaDyzPwqqZgdbGo\/PaS9WDnSsLe81FvjBW7NjokCd5Yglg1ZifWJ40HliehVXTgTZ3LeUeHsnDxYooObugI8mTPLEEsGrMTyxPGg8sz8Kq6UAffP602vjQs951Sm+H9Hrw406AFZE7s6gjyJM8sQSwasxPLE8aDyzPwqrpQHPzMEyIWBFhOGoV8iRPLAGsGvMTy5PGA8uzsGo0HtjQsCIiTywBrBrzkzyxBLBqNB5YnoVV04E2Nw+buffDhb3eol8YK3ZshMiTPLEEsGrMTyxPGg8sz8Kq6UBzDw9MiFgRYThyqAXLkTzJMxsCWFUaDyzPwqrReGBDQ+NBnlgCWDXmJ3liCWDVaDywPAurpgPNzcMwIWLFjuHIX+hYjuRJntkQwKrSeGB5FlZNAv2Vb35brb7n6941cvOwdKGi8UjHz380eZInlgBWjfmJ5UnjgeVZWDW\/8eDmYelCxYooHT8aDyw\/8iTPbAlg1Wk8sDwLqyaB\/pO\/emZu8zAaj3ShovFIx48NJZYfeZJntgSw6jQeWJ6FVfP3eOz71DVq3VWLC3u9Rb8wGg9shMiTPLEEsGrMTyxPGg8sz8KqSaC3\/OFfq\/7Hj3nXSOORLlSsiNLx4y90LD\/yJM9sCWDVaTywPAur5jceR+68Xi2tX1TY6y36hdF4YCNEnuSJJYBVY35iedJ4YHkWVk0C\/auf+wsl72qRD3ctTRcqVkTp+PEXOpYfeZJntgSw6jQeWJ6FVZNA\/8Id\/0cdfOG019MhPR78JCdA45GcXdCR5EmeWAJYNeYnlieNB5ZnYdVoPLChYUVEnlgCWDXmJ3liCWDVaDywPAurJoE+\/etD3vWtW75Y7bvlmsJeazVcGCt2bJTIkzyxBLBqzE8sTxoPLM\/Cqi37hevUGx\/t966Pu5amDxMrovQMTQXyJE8sAawa8xPLk8YDyzOV2szMjOrq6lLj4+OeTltbm+rr61N1dXWBuocOHVKbNm3yvlu1apUaGhpS9fX1gWVN48HNw1KFyTuYFVF6hjQeWIbkSZ7ZEcAq03hgeSZWm52dVb29vaqlpUW1t7cr\/e+GhgbV09OzQHdiYkJt3rxZ7dq1SzU3N6u9e\/eqsbGxUKNiGo8HO1d6vR78JCdA45GcXdCR5EmeWAJYNeYnlieNB5YnVC3KTMh3x48fDzQlQRex9LpfVW+u6\/a+ovFIHyZWROkZ8hc6liF5kmd2BLDKNB5YnlC1MOPh7x2xOWnjh35T\/XjNb3tFuWupDbHoMjQe6RmyocQyJE\/yzI4AVpnGA8sTpqbne3R0dHhDL+ZHG48NGzaohx9+2JsTEjfHo+HX\/rt66wMbPZnzDw6op\/Y+BLvWWhSamppSjY2NtXjrmdwzeWKxkid5YgmkU2ttbV0gMDk5mU60QEefd+bMmTMFup5El6KNhRwcNLlUf3\/y5Mm5CaX9\/f1qeno6dI7HpR+\/R\/3T0l\/yrofbpScKy7yD2OORniF\/oWMZkid5ZkcAq8weDyzP1GpxpkNOEDTUIpNNu7u71cDAgGpqalpwHe\/9zQfV2xf\/nPd3bpeeOkxc1ZIeIY0cmCGNR3ZA+UMDy5bGA8szlVrcShZTXHo4li1bNjcMI8Zjx44davfu3YFLat\/7H\/9IvX3R1dwuPVWEzh3MiggE8icy5EmeWAJYNeYnlieNB5ZnKrW44RJTXPbwkPJ67w75b\/kELb2Vv1\/0X76g\/vXdF9F4pIoQjQcI3wIZVuxYsuRJnlgCWDUaDyzPxGr+zcO0kJ40KpuIyT4fnZ2d3r4d8jE3EIvbbKz+9ie9Y7hdeuIQzTuQFTuGo1YhT\/LEEsCqMT+xPGk8sDwLq6aNB7dLx4SIFRGGI40HliN5kmc2BLCqNB5YnoVV08aD26VjQkTjgeHIhhLLkTzJMxsCWFUaDyzPwqrReGBDQ+NBnlgCWDXmJ3liCWDVaDywPAurpo0Ht0vHhIgVO4Yjf6FjOZIneWZDAKtK44HlWVg1bTy4XTomRDQeGI5sKLEcyZM8syGAVaXxwPIsrBqNBzY0NB7kiSWAVWN+kieWAFaNxgPLs7Bq2nhwu3RMiFixYzjyFzqWI3mSZzYEsKo0HliehVXTxoPbpWNCROOB4ciGEsuRPMkzGwJYVRoPLM\/CqonxWFq\/yHtBHD\/pCdB4pGdoKpAneWIJYNWYn1ieNB5YnoVVo\/HAhoYVEXliCWDVmJ\/kiSWAVaPxwPIsrJoYD26XjgsPK3YcS1EiT\/LEEsCqMT+xPGk8sDwLq0bjgQ0NKyLyxBLAqjE\/yRNLAKtG44HlWVg1MR58TwsuPKzYcSzZ44FlSZ7kiSeAVaTxwPIsrJoYD76nBRceGg8cSzaUWJbkSZ54AlhFGg8sz8Kq0XhgQ0PjQZ5YAlg15id5Yglg1Wg8sDwLqybGg+9pwYWHFTuOJX+hY1mSJ3niCWAVaTywPAurtuwXrlPH\/+Ebhb2+arswGg9sxMiTPLEEsGrMTyxPGg8sz8KqlS3QeYNmRYSNAHmSJ5YAVo35ieVZtvbovDNnzpzBIiqHWtkCnXdUWBFhI0Ce5IklgFVjfmJ5lq09qmrjMTMzo7q6utT4+LgX5ba2NtXX16fq6uoioz4xMaG6u7vVwMCAampqCixbtkBjHwN3NVZE7syijiBP8sQSwKoxP7E8y9YeVa3xmJ2dVb29vaqlpUW1t7cr\/e+GhgbV09MTGnVd7vDhw2p4eJjGA\/t8hKqV7cGpEDbyrBBo5icWNHmSZxSBqjUeQTe1d+9eNTY2FtnrcejQIdXf3+8dzh4P7MMRpcaKCMuaPMkTSwCrxvwkTxqPnxCQoZm7775bdXZ2euaDxgP7cNB4kGflCGDPxIaSPLEEsGply8\/S9Hjo+R4dHR3e0EtYj4j8fc2aNVZzPLCpQzUSIAESIAESSEZgcnIy2YEFPKoUxkPP2xC+YZNLZULpI488orZv366mpqZijUcBY8VLIgESIAESIIGqJ1D1xsPGdEiUZGhl\/fr1qrm5Wdmsaqn6yPIGSIAESIAESKCABKraeNiuZPEvuzXjMDIy4pkRfkiABEiABEiABLInUNXGQ3oxpqenrfbuMFGyxyP7xOIZSLhSoScAAAiKSURBVIAESIAESCCIQNUaj7BejFWrVqmhoSFvEzHZ50NWsPh7NGg8+DCQAAmQAAmQQD4EqtZ45IOLZyUBEiABEiABEkhDgMYjDT0eSwIkQAIkQAIk4ESAxsOHS+aNDA4Oen\/lxFP7XJLhq82bN3tzbuLemWMyli3uo7aut7+CcpV04anv3P8agXIRSX83Lkz9Q7msCxbyd+FpluUz757L+tkOmjrgrpb\/ETQeRgz0duoyR+To0aPeElz57\/r6+vwjVeArMBu8jRs3znuHjv+y\/dvay7\/37NlDzgYoF54mX2G5bds2tXPnztBN9AqcRplemgtT\/2o5zglbGBoXntrEyTu0ZL4dn3m3VNes9+\/fX5ofwzQeRg7od7jIA1I2h+mW6m6l\/RWzGLjR0VGr1Uas1IN\/SZpvT7bhKZX7HXfcoU6fPq2idu91i2x5SrvkqJTdsWOH2r17N390hKSAK08zn\/nM2z9Xuqdo7dq16uTJk94LUMuw\/QONx09yIOxtt\/rtt\/apUnslzZ4i6R3y\/zuKCCuhhXSS8BTTfO2116ovfelLc29srr1MDL9jF6Y2L5usdbYuPIN6POJe5lnrfPX9v\/TSS95\/yirNrq4uGo+yJUZQD4dU5suWLWO3dUyw\/b\/IXX4xJt2LpWz5Z96PK0\/9OoDbb7\/dewkizXKwmTN74aJyVIzH8ePHPRHO9wp+0lxz1Bwu2LJli9eA8mNPwG\/e7I8sZkn2ePh6PMzJOzQedknrWglpVang77\/\/fk4u9WF24SkV+uc+9zn1W7\/1W6qxsTFyfo1dNMtZyoWpniujJ5TKsVu3bmWeGqnhwlMPF+zatcsbJnDpES1nNrrfFY2HO7OqOIJDLcnD5NLtStMRz9mFp5R96qmnvF+QXNUSztaFqX+ohVyDe5DMyfdRZoI845\/5uBI0HnGEqvh7s4eDk0vtA+nvto6bDMlZ7dFsXXiaS5NNVXZnz2fswtSfv6wLFuarC08aD\/u6NKwkjUd6hoVV4HLaZKFxWVrHbut4xi48TTX+Mg9n68LUX8lzaGAhVxeeQUMtHLqKrwfMEjQebryqrjQ3EEsWsqjNhPRkPRkOCPuFzg2aFv5CD9uQzeRJ42Gfr7Y5KormBmLc8CqYsQtPMW+bNm3yhMjTPmd1SRoPd2Y8ggRIgARIgARIgAQ8AlzVwkQgARIgARIgARKoGAEaj4qh5olIgARIgARIgARoPJgDJEACJEACJEACFSNA41Ex1DwRCZAACZAACZAAjQdzgARIgARIgARIoGIEaDwqhponIgESIAESIAESoPFgDpBATgReeOEFtWTJEutXr8ta\/tdff10tX748syvW+6ysWrVKDQ0NWV9btbxlWN9fpfaSqPT5MksMCpMAkACNBxAmpUjAloDrbpiV2EAojXlIc6wtM0Q5MQLyqeTbUauFDYIvNUjAhgCNhw0lliEBMIEiGg\/XazKRVEvjSuMBTmTKkUACAjQeCaDxEBKwIWBuuy3l9fDF0aNH57aPlr\/r7eL928nr4YALL7xQdXV1qfHxce+0+gVw+n0Z+\/fv9\/5uMzxinsMcbtCvgtf3tXPnTtXe3r7gNsPKaeOxceNG9dnPftY7zj+cYW6b7T+PsLrjjjvUBz\/4Qe94fS+vvfaa0lvHyzGf\/vSn1b59+9TAwIBqamryZMLuKShGfuMh\/\/7hD3\/o\/U9zjHrBnv+FZ3KOoL9VoymzyWmWIQEEARoPBEVqkICPQNAL28xGz9+7EPYGT5Ht6+vzXnkv5kOGCJqbm+feJdLR0TFnEKLe+quvR+vV1dV5Deb999+vhoeHvUY8rsfDX958+ZeYIzEIa9eu9a5X6+\/Zs8ebKyIGoru7e55hMPW0uVq6dOnc8dq46XvU\/37llVe8a25sbFS9vb2ewdFDJ3EvIQwyHoODg0obLf85\/YlN48FHnQTSE6DxSM+QCiSwgEBcAxbXyPt\/SfuNh\/\/V7VI+6u20QUMh\/vJR1xT35lv\/G0jleuKGX8zvtfHwG6mxsbE5IyKaprGQf+\/YsUPt3r173iTYqOGUIOMxPT294BxSLmhyLY0HH3YSSE+AxiM9QyqQQCABc1jCPwwS1sj7hyPa2toCezz8Qx7mBQQNk4Sdz3zTbZTxiJvcGmQygv7mH37yDyfpHh25nyADYWpKL4p+46k\/AGHDJUHGQ441J5tGGSYaDz7sJJCeAI1HeoZUIIFIAmZja87zMH9VayPhn3ehf\/H7ezzkWP8v9aiLCDMVUcM\/pl5a4yFaeq6GNkZBPR4uxuOZZ55Reiinvr7eKgtpPKwwsRAJZEqAxiNTvBQngXMEzMZb\/6KX7nyZDyFzFVpaWuZN6DTNhd94RM3nCGJeiaEW\/xwO85xiEqKGTfRQi2k8gnoXzKEW6fHYunXr3BwVm1zLYqglzgTGDTnZXDfLkECZCNB4lCmavJfCEAia42H2OpiTLYMmSeoeED3UIjdmmhOtLxNN9TBB0DwLDQQ1udTsYTDnfaxZs2bB5FG\/8TCP1dcq1ycTRYOMh+3kUtHQE1rj5taknVyqh8L0SiR9H+akWn8S0ngU5rHkhRSEAI1HQQLByygfAd0oyZCIfMxhFHMprAw93HjjjfOWzIrhuOmmm9Rdd90194veb0Z0L4heZivn0A1iGM2opae2E163bds2Jx80bKKXufobXP+5d+3a5c3jkAml+v7NHg85iZ+hfzmtf0mxHBO2FFj3Msn\/a7MWtJzWPD7INJjzayROq1evVkeOHPHMj98g6nvw9waVL9t5RyRgT4DGw54VS5IACeRMQIxA0EoW28uymeNhq2Vbjj0etqRYrlYI0HjUSqR5nyRQZQT881h074a5b4frLdF4uBJjeRLAE6DxwDOlIgmQAIiAfzfXqF1FbU7pf2nbo48+6h2W1btb+JI4m6iwTK0R+P92O82DVsYw9gAAAABJRU5ErkJggg==","height":327,"width":542}}
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
