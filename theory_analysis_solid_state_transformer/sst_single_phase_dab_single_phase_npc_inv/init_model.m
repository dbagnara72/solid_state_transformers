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
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"275"}}
%---
%[output:174c5e41]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"1875"}}
%---
%[output:6e4cc00d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"1.7756e-05"}}
%---
%[output:01f62bda]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"8.9163e-05"}}
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10XsV55x9SWllpQoxASEiOYwIy0J6uwS61MCQ0BeLd9shsk6aS2LNWXSV1DxC8B2xJDoFdGoIt2Wbr8rF1qeLKu\/VH0kKD+rGUsISGiPiwBrRpQoJNsI0lpAiMAwTjhER7nuuOGF3dj5l553nvfa\/+9xwf23pnnpn7e+ad+euZr1MmJycnCQ8IgAAIgAAIgAAIVBCBUyBgKshbqCoIgAAIgAAIgEBAAAIGDQEEQAAEQAAEQKDiCEDAVJzLUGEQAAEQAAEQAAEIGLQBEAABEAABEACBiiMAAVNxLkOFQQAEQAAEQAAEIGDQBkAABEAABEAABCqOAARMxbkMFQYBEAABEAABEICAQRsAARAAARAAARCoOAIQMBXnMlQ4rwSOHj1KnZ2dQfX6+\/uppqZGrKr79++nVatW0ZIlS2jjxo1UXV0tVpZuuJzvaPJCisNnP\/tZam1tNcmS6zS9vb20bdu2oI6rV6+m7u5uq\/ru2bOH1q9fTxs2bMglD36\/b33rW+LfDytoSFyxBCBgKtZ1qHjeCJRzcI8TMDxAXHHFFdTc3CyCJ+4dpcuNexmXAZEH0Mcff9xKHLjksXUAl3HttddOZSuigCma4LT1MdL7JQAB45cnrIFAJgSOHz9OPT09NDg4SDt37iybgOHITznKjYKqBsOWlhZjMaIiFDbiwCWPSyNQAsambuFy8h6BUe308OHDiMK4NBLkmUYAAgYNIpcE9FA6V1APievRh4suuoi+8IUvBO\/AA5k+naKiBcPDwzM+121cc8019OlPfzpI09DQQNu3b6empqZILrpQCKcPRyf4czWldOWVV9Jdd91FixYtCjpufeBPs8NTUarcffv2BfXjR00h3XbbbfQnf\/IngXhRT5gF\/1wx1QWOGjT19GoQVLb0AVV\/x3vuuYf6+voiyz1y5EhQv9HR0ak66T7UOTLze++9l770pS+Rej\/mH2at2KmpuajBWvlVL1e9b\/i9lK\/nzZs3JcLC\/B566KFgSkY9evsI20sTjuH2mGQrqR0mRWp0JlxnVfewKAp\/v3S2ysZNN91Ejz76KPH3R\/lOf2f+mSpD920al6h2mMtOCJXKPQEImNy7aHZVMDxo6W+vOuGoQSo88LAdFg9KvIQ\/jxpgkwZ\/\/iyubmqwOeOMM6atgVECRq8DC4UowaGLmLAdXwIm6jf88GASHtjiuPLP4wRMV1cX3XDDDTPYJwkG\/qy2tpYmJiYCgRYlKrhMfaAN1z2uXahyn3766Ugx8sADD0ytO9Hbmz5AhwVM2Jb6PE7EJLVZznPo0KFYoaTXKSxewiJTiYePfOQj9I1vfGNa5xElQqK+X2EBwmmi6sg\/V+Wk2da55D1KNLt63Mp+WwiYyvZf4WqvOmj9N1DV+fPL6tEH\/i1bdYz6AKF3tvpvnvqAxyJBRQiUDVV2+Dd9BTnqc70zvvrqq2MFjP4bqq2dNAHDUSd+0qZykiJEHBV69dVXZzDRowbMaeHChdPe0WQKKTy9pdgrf3K0Jex3rguvB4mKDDHLFStWBO+rR2xMFjabTAeF04T\/r5goscX1Tytbtb2o91E\/Y6HL7xw3haRzVO0p7NNHHnkkEEJREZU4u+EonIo66TaSylYRGtX+07j4mCorXMeHF3IiAAHjhA2ZpAjE\/XamBgDuuBcvXhy5A0dPc\/Dgwcjfqrneug3+rV\/tGEpbhJv2m2OcQNA7dC7f1o4vAaOXzWKEH33AjBtY9AH8M5\/5jLGAiYpYRZXL9QhPkcVFODgtD8SqHjrbqPLCU2lJAiZu6iycJymaEiV+w++mpifDQkiJtjihkdY+df\/qNuL8Go7mKFZKwMRNHeo77PS2rL6X+vSd6id0LlHTllL9CewWmwAETLH9W3FvV24Bo29DThsgbIUHw4\/aVm1qRx+cw4Md29a3UZtEYDiNvvCV\/89bdsMRqPAAaitgwhzDUZqwcPIlYFRjjxNOvDMrSsCEp6LSIjCVIGCiIn7Kr+H2FxeB0W3EfTcgYCquiy1UhSFgCuXOyn8Z1ymk8FSHWlMQ99tsVMg\/TcAkTf3oUQH2Av+WGidgTO1waD4sLtTUWljAsEgwWRwZHtz1CEV4Go4H\/LQpJI4OpQmAsA3XKSS9dcdFNcLfgLAYCUcj1DvrkTj1PqrthPNETSGlffOkp5CU2FWRqzgBExW5UozCEZi4Rdfh6aukKaQoLphCSmst+NyUAASMKSmkKwsB6UW8SQIgTcAk1S1qfUicgEmzw+F2tZ4lDN1EwHCeqF1IylZ4J4l+AJzNIl41laDn4XI\/8YlPBNGhqIc5Rb2f6SJetqlEXVg4xS1w1fPoabjMrVu30h133DFjwTHnCQsY\/lncgmD1rmmCOWp6JS0CpnOMe8ck8aELhhtvvDG2bSXZ4DpELe41XcSrc0mLQJalo0EhhSAAAVMINxbvJUy3UetboNO2UUctDLaZQmLKSdMTaYtk9ZN5k+xwOeEtt7feeis9++yzU4tWoyIw+uAWtxBZtx1emxMlcPSBXM\/L\/1YCJqrc+++\/f9qJsny4nr7exmUbtS5E9AE1aot93PbtMFd9TQ7bZG6bN2+mtWvXBjj0SJraTRa3LTvt\/JakbdRclmlkIm7tCkfhosRBXNSJGUVtYY+K4sSJX\/55+OTfuLVEyoZJpLB4PRreSIIABIwEVdgUJZC240O0cBgvmUDUVFV4p1ncOTx64S4H2ZVc+VlqQBecSqhF7UxKw4OD7NII4XMbAoUSMNyh8RkUfLhWVAeYdLCZDTSkzZYABEy2\/EstPWkKLWnqK6pcl6sESq3\/bM0fNYXELNIOf4wSnUW5u2q2toW8vHdhBEzaoj71+bJly4JLztT\/+ctne2FaXpw3W+sBAVP5ng\/\/MsFvZCteOA\/u1ilvWwhP7dqIF64pBGd5\/VX00gojYHiel78c\/MRFYMLO5N8ohoaGynqbb9EbFN4PBEAABEAABMpBoBAChn+bu\/3226m9vT0QMT4FDB85z3\/wgAAIgAAIgEClEuDrOfhPkZ5CCBiOpPDDJ0EmrYHRHadC2G1tbcGUUtTDwmXdunW0d+\/eIvkc7wICIAACIDDLCCxdupQ2bdpUKBFT8QKG58AHBgbolltuCSIlJgJGrX\/h9qvfXhxuz2r7ITu9sbFxljV32ddlUchncICtf85g65+psgi2YCtHQM6yardpN6bL1UDGcsULGJ4y4jMm+NTQtF1IjNBUvHBaJWCK5nSZpmRnFWzteNmkBlsbWnZpwdaOl01qsLWhZZe2qGwrWsBE7WRQbo0SHbY7j4rqdLumL5MabGW4QnjLcQVbsJUlIGe9qP1tRQuYsLvTIjAcreHTJ5OmjXSbRXW63NfE3DLfFs1Tfx0dHbRgwQLzjEiZSgBsUxE5JwBbZ3SpGcE2FZFzgqKOZYUWMCriwruTFi5cGNwMrI4DVy0h6cj1ojrd+VvgMePbb79NL7\/8Mp199tk0Z84cj5ZhCmzl2gDYgq0cATnLRR3LCiVgfLu\/qE73zcnFHgYCF2pmecDWjJNLKrB1oWaWB2zNOLmkKupYBgGT0BqK6nSXL4DvPOisfBN91x7Ygq0cATnLaLdybIs6lkHAQMDIfWsSLKOzksMOtmArR0DOMtqtHFsIGDm2ubVcVKfnATg6KzkvgC3YyhGQs4x2K8e2qGMZIjCIwMh9axCBAdtMCMgVikEWbOUIyFmGgJFjm1vLRXV6HoBjIJDzAtiCrRwBOctot3Js9zz+r7R+1Qoq2qGsiMAgAiP3rUEEBmwzISBXKAZZsJUjIGP58NG36aI7nqS5f9cJASODOJ9WEYGR8wsGArCVIyBnGe0WbOUIyFh+4sAxWnHfMxAwMnjzaxUCRs43GAjAVo6AnGW0W7CVIyBjGQJGhmvurULAyLkIAwHYyhGQs4x2C7ZyBGQsQ8DIcM29VQgYORdhIABbOQJyltFuwVaOgIxlCBgZrrm3CgEj5yIMBGArR0DOMtot2MoRkLEMASPDNfdWIWDkXISBAGzlCMhZRrsFWzkCMpZ3PTVG1+96Dot4ZfDm1yoEjJxvMBCArRwBOctot2ArR0DGshIwp\/1zN+3+yz+j5uZmmYIysIpzYBKgQ8DItUgMBGArR0DOMtot2MoRkLGMCIwM19xbhYCRcxEGArCVIyBnGe0WbOUIyFjuffhF6n34IKaQZPDm1yoEjJxvMBCArRwBOctot2ArR0DGMgSMDNfcW4WAkXMRBgKwlSMgZxntFmzlCMhYhoCR4Zp7qxAwci7CQAC2cgTkLKPdgq0cARnLEDAyXHNvFQJGzkUYCMBWjoCcZbRbsJUjIGP5tsEDdM9jL2ENjAzebKzu37+furq6qK+vj5qamiIrAQEj5xsMBGArR0DOMtot2MoRkLHccu8z9M0XjkHAyOAtv9Xjx49TT08P7du3j7Zv3w4BU34XEAYCOehgC7ZyBOQso93KsIWAkeGamVWOrPT29gblIwKTjRvQWclxB1uwlSMgZxntVobtRXc8SYePvo0IjAze8lo9evQo3X777dTe3h6IGBMBs2bNGlq6dOlURevr64n\/4HEnwJ3V2NgY1dXVUXV1tbsh5JxBAGzlGgXYgq0cAX+WuW\/lP\/z89pePB3\/P\/btO2rlzJ07i9Ye5\/Jb27NkTFLp48WLjNTDhWnZ0dNDKlSvLX\/kClXjixAmamJig2tpaqqqqKtCbZf8qYCvnA7AFWzkC\/izv2LGDBgYG6OfvPZNe\/\/jJ2QYIGH98M7HEC3fZqbfccgsdOXLEWMBs2rSJGhsbEYHx6DVehzQ+Ph5EsubMmePRMkyBrVwbAFuwlSPgz7KKwPDi3S\/uO\/kLIgSMP76ZWOIpoyuuuCIIoWEXUiYumCoU891y\/MEWbOUIyFlGu\/XP9okDx2jFfc8Eht\/3RB99+c\/+K6aQ\/GOWt8hrXzo7O2l4eHhGYXHzgthGLecXdFZgK0dAzjLaLdjKEfBv+fpdzxFf5sgPbqP2zzczi4jAZIY+KBgDgRx\/sAVbOQJyltFu\/bNVW6jf89YrgYDBIl7\/jDOxCAGTCfapQtFZyfEHW7CVIyBnGe3WL1veOs1bqPk59ZXvB1NIEDB+GefaGqaQ5NyDzgps5QjIWUa7BVs5An4t6+tfrvjAOA0PfA4Cxi\/ifFuDgJHzDwYCsJUjIGcZ7RZs5Qj4taymj9jqLUtO0L23XgcB4xdxvq1BwMj5BwMB2MoRkLOMdgu2cgT8Wdanj+bXzKE\/v+oUuvbaayFg\/CHOvyUIGDkfYSAAWzkCcpbRbsFWjoA\/y7zziHcg8fPQdRfTqa98DwLGH97KsAQBI+cnDARgK0dAzjLaLdjKEfBjmaMvfPYL\/83Rl2c\/fykVdSw7ZXJyctIPtuJZKarT8+ApDARyXgBbsJUjIGcZ7dYP296HX6Tehw8Gxu5uvYD+09KzIWD8oK0sKxAwcv5CZwW2cgTkLKPdgq0cgdIth9e+cPSFn6KOZYjAJLSZojq99K9J6RYwEJTOMM4C2IKtHAE5y2i3pbFl8cLrXvj+I37ubb+Q2i+ph4ApDWvl5oaAkfMdOiuwlSMgZxntFmzlCJRmecvXDtEX\/\/EHgZHLzp1Lg9dfPGWwqGMZIjCIwJT2rXHMjYHAEZxBNrA1gOSYBGwdwRlkA1sDSDFJ\/vbpcfrM\/\/pu8KlauKsnhYBxZ1uxOYvq9Dw4BJ2VnBfAFmzlCMhZRru1Z8vTRg9\/9xXqfmD\/lHjhbdMsYiBg7HkWKgcEjJw70VmBrRwBOctot2ArR8DOMouXXU+9PLXjKCryoiwWdSzDFFJCmymq0+2+JjKpMRDIcGWrYAu2cgTkLKPd2rHlixpZxPDD4uWetgvp8vPmRhop6lgGAQMBY\/et8ZQanZUnkBFmwBZs5QjIWUa7TWfLguWJF47RDf92yq4SL1HTRro1CJh0toVLUVSn58FR6KzkvAC2YCtHQM4y2m0yW75d+obdz01FXTh1z78\/h7o+viDVKUUdyxCBQQQmtfFLJEBnJUH1pE2wBVs5AnKW0W7j2eo3S6uoS\/fyc6bOeUnzCgRMGqECfl5Up+fBVeis5LwAtmArR0DOMtrtdLY8XcTXAvDFjPrzn5eeTTdfvWDGTqMkzxR1LEMEBhEYuR4pwTI6KznsYAu2cgTkLKPdUjA9pISLOlFXEeeFumlrXeK8AwEj125za7moTs8DcHRWcl4AW7CVIyBneba3W\/0WaZ1yKcJF2SnqWIYIDCIwcj0SIjBgmwkBuUJn+yArR3Z2rt3ihbk8TRSOtjBnvseI17mED6Vz8QEEjAu1MuQ5fvw49fT00ODgYFDa6tWrqbu7O7bkPXv20Pr164PPW1paaOPGjVRdXR2ZvqhOL4NbUovAQJCKyDkB2DqjS80ItqmInBPMFrYsWr46\/EPq\/+bIDFYsVtp+vZ6u\/Y2zvQgXRGCcm2N5Mvb29gYFsWg5evQodXZ2UltbG7W2ts6oAAsSTt\/f3x+IFhY+DQ0NsYIHAkbOh7Ols5IjGG8ZbOWogy3YuhBg0fK3z4zTwJOjkdnTDqJzKVPPU9SxrHBTSLqgCTudoy9DQ0NTUZfw\/8Ppi+r0Ur8MPvJjIPBBMdoG2IKtHAE5y0Vrt0nTQ0xRiRb+28c0UZJnijqWFUrAqAgMR2Oam5uNIjDLli2LjNZw5qI6Xa4LMrdctM7K\/M3lU4KtHGOwBds4ArwIl6eFeHpIHfEfTltO0YIIjFxb9W6ZIy\/btm1LXdeyf\/9+WrVqFY2OjtLOnTsjhY6qnBIwa9asoaVLl07Vub6+nvgPHncCPBCMjY1RXV1d7Bokd+uzOyfYyvkfbMGWCYy+\/k4gUn76zs\/ozx4fjVyEy+kaTjuVzn7\/qdS9nM9tqRaPtCjvcN\/Kf9QzMjJC69atSx3z5LwrY7lQERhGxEKGxUnU4lyeMtq9e3ewBqampiZIy0\/col8lYMLoOzo6aOXKlTIemSVWT5w4QRMTE1RbW0tVVVWz5K3L85pgK8cZbGcvWxYtX\/726\/Td8Z\/QvpGTlyhGPSxafvdX30+Lzq6iJY1z5IAlWN6xYwcNDAzMSJH2S3smlS2h0MIJGI6wdHV1UV9fHzU1NU2hUbuV9CmjuLQqkxIwmzZtosbGxilbiMCU0OL+LSv7Y3x8PIhkzZmTzZe89LfIpwWwlfML2M4etk8ceI3+9ukf0jd+8HoQcYl7eFrosnPn0qcurg0iLCxgsn7CEZi9e\/fS1q1bEYHJ2jFp5es7jTjKop5SBEzRVGsaw3J8jrUEcpTBFmzlCMhZzrLd8oLbH\/\/kZ3TPY4djp4PUmyvB0n7Jya3O0gtwfRAv6nrOTCIwarHt8PCwlW8WLVpEDz744LQ8+jSQEilxW6OjppDippu4kKI63Qq6UOIsOyuhV8qNWbCVcwXYVj5bFis\/Ov5T+vN\/OZIqVvhtWaD85sIauumqDwUvXwmCJeyloo5lmQqYuN1CUV8RFVkJC5jwQXb64XTqs\/b29qnFumqxL5eBg+zkOqM0yxgI0gi5fw627uzScoJtGiH3zyXYslg58c7P6U8fPWQsVj54+hzqWr6APlTGRbfu1MxyQsCYcTJKlbbd2UbAGBXomKioTnfE4TWbRGfltYIVbAxs5ZwHtvljy7uBOCrCf\/\/3Rw\/RgR++ZSVWVjY30NkfqKqY6SAXDxR1LMskAsOREf6jr1FxcYp0nqI6XZqbiX0MBCaU3NKArRs3k1xga0LJLY0pW46qvPLmT4IzV6LuEIoqnQUOR1Y+2nQ6XfrhuXT5eXPdKlmhuYo6lmUiYPQ1MGl3F2XZXorq9CyZqrJNO6s81LXS6gC2ch4D2\/Kw\/eFbFERUnnnpdfrn775KL732duzhcOEaKbHCFyHyUykLbeXIFnc9ZyYCRjlKX4\/CC2+3b98+beuzpENNbEPAmFByS4OBwI2bSS6wNaHklgZs3bjF5WKRwn9efPU4ffn\/jtGLE28mblnW7ajFtK2\/Xk8fOe90CJUE1xR1LMtUwCje4V1JeYnKFNXpfrsgN2sYCNy4meQCWxNKbmnA1o0bT\/uw4Pgfj79E\/zr6pvHUj4qg8N981orauqz\/3K1GsytXUceyXAgYvSnpR\/3zz7M8g6WoTs\/DVxcDgZwXwBZs5QjEW1bRlF94zyl05z\/9wGrah62qA+Cu+pWzgi3LanFuJW5bzoJ\/UplFHctyJ2B0J8QdSleuxlFUp5eLX1I5GGTlvAC2YCtFQF1SuP+Hb9EDz4wHIsN0Ia2qk1qjctWFZ9CS+adNnaty1nuJXn75ZTr77LNxOrdnBxZ1LMudgEEExnPLzak5DLJyjgFbsC2FgIqkfH\/8x\/T04dedRYqa9vnUkjo69T3vSV2jgnZbiteS80LAyLENtlT39PTQ4OBgUEraAXOCVZlmuqhOLxc\/RGCyIY2BQI57kdjyupQ3T7xD9379pQCYbSSF86hoyqcvn0dn\/PIvTkVTXKZ9isRWrgW6WS7qWJZpBEbfhcRuyXK9S1SzKKrT3b4CfnOhs\/LLU7cGtmCrpnr47\/\/9nVdo+Mgb1mtSwlM+n7y4js7jeR6hrclot3LttqhjWSYCRt91lJdoCwSM3JcnyjI6KzneYDs72HIEhZ93fj5JWx45WFIUhTPzQW9KpKgIikskxZU+2q0rufR8EDDpjIxTHDhwgG688Ua67bbbpu4oSsscdxdSWr5SPi+q00th4isvOitfJGfaAdvisGWR8r6qX6C\/\/OYIHXr1uNM0j4qY8N+\/dX4N\/Vrj++nKC2qmIJVTpCR5Bu1Wrt0WdSzLNALj4zJHOZcX9\/RCSWamttFZmZKyTwe29sxMc\/hkq6Z5fvDKcXrwmXHiv21OnNXrrEQIR1GuWHg6NZ9z8qj8SjqF1idbU3\/OlnQQMB49HT64ztT0okWLKHwbtWlel3RFdboLC9950Fn5JvquPbDNlq06v0RN8Tx75A167uU3nXbzhEUKC5Rza6vp9xbXV5xASfMK2m0aIffPizqWZRKBcXdDeXMW1enlpRhdGjorOS+ArTzbn1adPnVWyT98e4L+6Tuv0OSk204eVVu1o4f\/\/v0l9XTOmdVTL5KXaR45skRot3J0izqWQcAktJmiOl3ua2JuGZ2VOSvblGBrSyw6PUdQGudW0d2PHSY+uM11eicsUBacUU2fXHzW1Nko\/PlsEChpXkG7TSPk\/nlRxzIIGAgY929FCTnRWZUALyUr2CYDUmtPONXfc\/TkX18JMricg6KLE\/43T\/Fw5GTt1QumjsKHQDFr62i3ZpxcUkHAuFCr8DxFdXoe3ILOSs4Ls5mtfv7JxJs\/oa8992ogJHxET9hjSz\/0PmqaO0krlsynH75FdPl5JxfL4imdwGxut6XTS7ZQ1LEMERhEYKS\/O5H20VnJYS8i2\/DC2O++\/CZ9e+RNOviq+86dqOgJT+W0X8ILZE+JPFW2iGzlWqKdZbC142WTGgLGhlZB0hbV6XlwDzorOS9UIlt1\/w5TOev9vxSsO\/EhTtT0DU\/tsDi5+sIz6Mz3\/VIA3yV6Uols5VqaX8tg65enbq2oY1luIjB79uyh9evXB8z5SoFDhw7R0NAQbdy4kaqr312NH3Zx+B6l1atXE58vE\/coR\/LnvC27v7+famrePdRpNjhd7mtibhmdlTkr25R5Y6vECQuI\/3fkDfo\/3z\/qZVGsEif892XnzqXF80+j8+t+edqCWN+LY\/PG1rZt5Dk92Mp5BwJGji3xnUijo6PU1dVFN9xwQyBAWFzwBY8NDQ2JgoTz8sN51PkybW1t1NraOqPG6qbrzZs3BycAs2hKEklFdbqgK41No7MyRmWdsJxs9TUnL\/\/oBH39+aNe1pzo4oRFyEXz3k98YaCaStI\/twZUQoZysi2hmhWZFWzl3FbUsSzzCIwSHSxAFi5cSJ2dnYEYYYGhrg9IipKEXa4LmvBnLFgOHjyYKIgQgZH7EumW0VnJcfbBVl9zwgLiS0MjtO\/Q60GlS9mto95aRUY4cnLFwhpq+EBVSTcZy9GcbtkH23LVtdLKAVs5j0HACLH1KWB0WyyA9EdNNS1btiwyOhP1ekV1upArrcyis7LCZZU4iW1YmDx9+HV6xNNOHV2cqDUnH206nS798NypyInvKR0rMB4So916gBhjAmzl2BZ1LMs8AsMuU1M5+hSSisbETQdFRV62bdtGcbdbKwGzfPlyuv\/++2l4eNh4DcyaNWto6dKlU0XW19cT\/8HjToA7q7GxMaqrq0tc4+RewuzKOfr6O9Rw2qnEf+9\/+Rj94olj9PjoqfT0yHFvURO2z8\/Z7z+VFta9l37l7PfR+XXvDX42v+bkOrVKFyhprQbtNo2Q++dg684unJP7Vv6jnpGREVq3bl2wvjT8y72\/UstvKRcChl9bX1yrMGzYsME4WqLyqPU04cW\/SsAcPnx4auFuXFplK6pO\/FlHRwetXLmy\/N4qUIknTpygiYkJqq2tpaqqqgK9md9X0YXJy2+8E4iH7038hP7lxbcCscI\/479LfXRxckHtL9EVHz4pTLg8ftTnpZZT6fnRbuU8CLb+2O7YsYMGBgZmGISA8cdYxBIv1OVITl9fHzU1NU2VETWFFJc2LGA2bdpEjY2NU7YQgSnddeyP8fHxIJI1Z86c0g1WmAU1lXNyh87xIIJx9Mc\/pf5vHqGDr77tTZjotxTzvy9Z8AFacPrJbcT8fwgTu4Yz29utHS271GBrxyspdTgCs3fvXtq6dSsiMP4Qy1hKWvjLEZcFCxZMRXVYwNx55520ZcuWyK3URZ03lCFvZ7Wo893ThcnbgUj46c9+Tvd8\/SU6wEe3eloEqwQI\/80LYXk6p+Xf1QZrTc56LwXRrSXnf3BWikO7lmiXuqjt1o6CTGqwleHKVos6lmU+haQW3vKalKQn7nwXfdeRirLEbb0Oi5ukHUtFdrrc18TccqV2VvqZJj+fnKR7v\/4SfW\/sxyLChBfC\/lrj++mPP3py+7By+I4MAAAgAElEQVQSLWnrTCqVrXnryS4l2MqxB1s5thAwcmyDRby7d++edqicfqbLihUrYs+ECR9kpy\/iVZ+1t7dPLVzS17XELfhVr1pUpwu60th03jor\/STYeafPofsef4mee\/lNMWHy4dpquuE359P46z8xFiamcPPG1rTelZAObOW8BLZybIs6luUmAqPOftFdqEdMnn\/++eDAuwcffFDOyyHLRXV62QAmFFSuzmq6MKmi+77+Ej0nGDFZcEY1ffryRnrj7Z95FyamfisXW9P6FCkd2Mp5E2zl2BZ1LIOASWgzRXW63NfE3LKPzuqJA8eCAuedXvVvERO5qZwPnVFNf3BpA739059nJkxM6fpga1rWbEsHtnIeB1s5tkUdyzIXMOyytCkkvhZAnRXDK6nL9RTV6eXil1RO2mFrajHsXz05Qk8dPHkC7Euv8Y6dk2tBSnn0nTkcMVnZ3EAn3nlXmLDttHUmpZQvnRcDgRxhsAVbOQJylos6luVCwLDbos5cUXvWWbzcfffdtH379mlbo+XcfdJyUZ0uzS3Ovr4Adt+Lr9LfP\/syjb45GWwZ9i1MWICsWtZYERET3\/7AIOub6Lv2wBZs5QjIWS7qWJYbASPnOnfLRXW6O5H0nDyt0zi3iu75+mF6fvwtL1GT8Fkm1\/7G2TQ5+W6UpJKjJelE7VNgkLVnZpoDbE1J2acDW3tmpjmKOpZBwCS0gKI63bTRh9PpNw9\/48BrNPTCsZIFii5OrrnoLLqg7pcr4lI\/V4blyIeBQI4y2IKtHAE5y0Udy3IhYPhAuVWrVtHo6OgMDy5atGja9mo5F8+0XFSnmzDkSMokTdLup8aC6R2XG4j1G4eXzD+NrrrwjMDW5efNJQwEJl5wSwO2btxMcoGtCSW3NGDrxs0kV1HHsswFjH7Evzrvhc9tUZc5Rm2vNnGYjzRFdbrORkVV+Bj7Wx86YCVUlED5rfNr6OO\/ckZwGqx60qZ10Fn5aKHRNsAWbOUIyFlGu5VjW9SxLHMBow6sU0JFP+6foe\/atYvCFzPKuXm65SI6XS2kfeLAa4FYMYmssBjh4+p\/\/UMfoKaz3htM8aQJlDQfobNKI+T+Odi6s0vLCbZphNw\/B1t3dmk5iziW8TvnTsDwjqODBw8SC5qke43SHObj86I4XYmW3odfTBQsLEr4+HqOqHxycV2AsFShEucHdFY+WigiMHIUwRZsy01ArryijGVhQpkLGK6QfieRLloeeeQRGhoaQgTGsV3zOpYk0cLi5MoLauh3L6rzElWxqSYEjA0tu7Rga8fLJjXY2tCySwu2drxsUkPA2NCyTKuvg+FD61jQbNu2jfhSxnKf\/aJXvRKdztEWFi27nhqb4QU1FdS9\/BzR6IqJ+9FZmVBySwO2btxMcoGtCSW3NGDrxs0kVyWOZSbvlYsIjElFs0hTSU5n4XL9rudmTBHpokVqOsjFN+isXKiZ5QFbM04uqcDWhZpZHrA14+SSqpLGMpv3y1zAhBfxhiMgHI3p7++nmpoam\/fykrYSnJ4kXB667mKxNSylAkZnVSrB+PxgC7ZyBOQso93Ksa2Esczl7SFgEqjl2eksXB793lG6+W++P+0NOMqSZ+GiKovOyuXrapYHbM04uaQCWxdqZnnA1oyTS6o8j2Uu76PyZCZgeLfR+vXrU+u+evXqYEdSFk9enc7iZcV9z0y7P6hShAsEjHxLxkAgxxhswVaOgJzlvI5lpb5xZgJGVTxpCqnUlys1f96crrZDs3hRDwuXe9ouDE63raQHA4Gct8AWbOUIyFlGu5Vjm7exzNebZi5gfL2IhJ08OZ3Fy66nXqbehw9Oveo9bRcQX2xYiQ86KzmvgS3YyhGQs4x2K8c2T2OZz7eEgEmgmRenh6eMOOrCW6HbL6n32RbKagudlRxusAVbOQJyltFu5djmZSzz\/YaZCBg1bTQ8PJz6PrP9Msco8VIJi3TTHIvOKo2Q++dg684uLSfYphFy\/xxs3dml5YSASSNUwM+zdnqUeHn285cWgjQ6Kzk3gi3YyhGQs4x2K8c267FM6s0yicD4fBl1iu\/g4GBg1nTX0v79+6mrq4v6+vqoqakpskpZOr3I4oVho7Py+S2YbgtswVaOgJxltFs5tlmOZXJvlYPLHNXLRW2r3rBhA\/HVAkmPfo+Smppqa2tLzKdEz759+xKvKsjK6eHD6XjNS1EiL8qX6KzkvtZgC7ZyBOQso93Ksc1qLJN7o5OWcxGBYfGye\/fuaSfumoqRMCBd0MTBUxdG8ud5jMDc9bVDdMc\/\/iCofhHFCyIwsl9rDARyfMEWbOUIyFmGgBFi6\/MqAZMzZTjN7bffTu3t7cGlkSYCZs2aNbR06dIpAvX19cR\/JJ6vPDNBa75yIDDdcNqp9MAf\/VpurwMo5f15IBgbG6O6ujqqrq4uxRTyhgiArVyTAFuwlSPgzzL3rfxHPSMjI7Ru3TrauXMnNTc3+ysoY0uZR2B8CRh1g3VLSwtt3LgxdlDkaA8\/ixcvNl4DE\/ZRR0cHrVy50rvrRl9\/h1oGjkyJl\/921Zm0pHGO93LyYPDEiRM0MTFBtbW1VFVVlYcqFaYOYCvnSrAFWzkC\/izv2LGDBgYGZhiEgPHHeMqS7ymk0dHRSBHDC3fZqbfccgsdOXLEWMBs2rSJGhsbp+orEYHhdS9rvrKf9h56Myhn66fOo09dXCtAOx8meR3S+Ph4EMmaM6eYIi0r0mArRx5swVaOgD\/L4QjM3r17aevWrYjA+EM83ZLrIt5wfZJ2F3GU5oorrghCaHnbhdT78ItTp+xedu5cGrz+YinUubCLtQRybgBbsJUjIGcZ7VaOLdbAyLH1alkt0O3v76eampop20mH58WF1crp9JqbHgvqWmmXMro6D52VK7n0fGCbzsg1Bdi6kkvPB7bpjFxTlHMsc62jS77crIFJ2\/oc93L6riO1PbqhoSH1Bus8RWBa7n2GvvnCseAVebs0i5iiP+is5DwMtmArR0DOMtqtHFsIGDm2FJ4+slloFD7ITl\/Eqz7jHUfhldd5ETC7nhqj63c9F9CdDVNHqhmhs5L7QoEt2MoRkLOMdivHFgJGju00y2o3Ef+QIynbt2+PPSlXukrlcLqKvsyWqSMIGOlWi1OOJQljkJWjC7ZybMsxlsnVPt5y5lNISS\/NYobBh9ezlAuUtNN559FFdzwZvM5nPzafbm85t1yvlnk56KzkXAC2YCtHQM4y2q0cW+mxTK7myZZzJ2D0CEyWN1EzNkmn63cdzbboC7NFZyX3lQdbsJUjIGcZ7VaOreRYJlfrdMu5EDB5mjbSkUk6\/YkDx2jFfc8Exd31qfPpDy5tSPdWgVKgs5JzJtiCrRwBOctot3JsJccyuVqnW85cwJgc\/5\/+GjIppJyuX9Y4G6MviMDItFdlFQOBHF+wBVs5AnKWpcYyuRqbWc5cwJhVM5tUUk7Xdx71faKJPn35vGxeMMNSMRDIwQdbsJUjIGcZ7VaOrdRYJldjM8sQMAmcpJw+W3ce6ajRWZl9QV1Sga0LNbM8YGvGySUV2LpQM8sjNZaZlS6XCgKmzAJGX\/ty+Xlz6aHrin1lQBxedFZyX2qwBVs5AnKW0W7l2ELAyLHNrWUJp8\/GU3ejHIzOSq7Zgy3YyhGQs4x2K8dWYiyTq6255cwjMEmLeOPuNTJ\/vdJS+na6fu4LL97lawNm64POSs7zYAu2cgTkLKPdyrH1PZbJ1dTOMgRMGaeQdAEzW+48whSS3RfSR2oMBD4oRtsAW7CVIyBnGQLGM9vw\/Udx5levXp16MaPnqk2Z8+10ffHubI6+MGAMBFKtFmzlyIIt2EoSkLPteyyTq6md5VxHYOxexX9qn07Xoy+z6dJGRGD8t8s0ixCHaYTcPwdbd3ZpOcE2jZD75z7HMvda+M+ZuYDx\/0r+LPp0eu\/DL1LvwweDyvHOI96BNJsfdFZy3gdbsJUjIGcZ7VaOrc+xTK6W9pYhYBKY+XQ6po+mg0ZnZf9lNc0Btqak7NOBrT0z0xxga0rKPp3Pscy+dLkcmQgYfefRwoULqbOzk4aHhyPfMssLHX05XZ8+ar+knu5tv1DOoxViGZ2VnKPAFmzlCMhZRruVY+trLJOroZvlTASMW1XLn8uX0\/Xpo9m++0h5EZ2VXHsGW7CVIyBnGe1Wjq2vsUyuhm6WIWDKMIWE6aOZkNFZuX1hTXKBrQkltzRg68bNJBfYmlBySwMB48YtNZeaTirqFBKmj6KbADqr1K+GcwKwdUaXmhFsUxE5JwBbZ3SpGSFgUhH5TcDC5uabb6bPfe5z1NTU5Ne4oTUfTn\/iwGu04r5ngxKx++hd8OisDBuhQzKwdYBmmAVsDUE5JANbB2iGWXyMZYZFlTVZrqeQGPquXbto48aNVF1dHQnm+PHj1NPTQ4ODg8HnSQffhaM9LS0tibZ9OB3TR4jAlPUbjUMCRXFjkJXDC7ZybH2MZXK1c7ecewHT29tL\/f39VFNTE\/mW\/Dk\/3d3dpARKW1sbtba2TkuvhM6yZcuCz9T\/GxoaYk\/6LdXpOLwuvmGis3L\/0qblBNs0Qu6fg607u7ScYJtGyP3zUscy95Jlc+ZawLA4GR0dTYyShPHogiYNHV9nMDQ0FGu\/VKfrAoa3TvMWajwnCaCzkmsJYAu2cgTkLKPdyrEtdSyTq1lpljMXMEmLeDk6sn37duM1MEk3W0dhMhUwa9asoaVLl06ZqK+vJ\/6T9nyq\/zn65gvHgmTfWnsx8Q3UeN4VMGNjY1RXVxc7PQhWbgR4IABbN3ZpucA2jZD752Drzi6ck7\/\/\/Ec9IyMjtG7dOtq5cyc1Nzf7KyhjS5kLGH7\/uOkdNd1jwogjL9u2baO0dS3KVtJ0k0qjVGu4\/I6ODlq5cmVqtZbcffLqgIbTTqXBjnmp6WdTghMnTtDExATV1tZSVVXVbHp18XcFWznEYAu2cgT8Wd6xYwcNDAzMMAgB44\/xlKWoqSITgRFVFZNpJyWYOH\/SAmElYDZt2kSNjY1TxZlEYHj6qHnzM0Gem6+cRzdf+UEBcpVrkn0wPj4eRLLmzEFkyqcnwdYnzem2wBZs5Qj4sxyOwOzdu5e2bt2KCIw\/xCctJU37mOxCCtdn\/\/791NXVRX19fZFTT6bihe2WMm+IyxuTWwrmu31\/k961B7ZgK0dAzjLarRzbUsYyuVqVbjnzKaQ0AZO2CymMgB0Vl8dk55FurxSnY\/s0BEzpX083CxgI3LiZ5AJbE0puacDWjZtJrlLGMhP7WaXJXMCE17\/oINIW2XJafddRmkAxmV7yIWCwfTq9OaOzSmfkmgJsXcml5wPbdEauKcDWlVx6PgiYdEbOKRju2rVrp+044qmgVatW0ebNmxNXTYcPstMX8arP2tvbKe7W66Tbrl2d\/sSBY7TivpPrX7B9OrpZoLNy\/rqkZgTbVETOCcDWGV1qRrBNReScwHUscy6wTBkzj8Co94za8ZP1imlXp2P9S3rrRWeVzsg1Bdi6kkvPB7bpjFxTgK0rufR8rmNZuuVsU+RGwGSLIbp0V6dj\/Uu6N9FZpTNyTQG2ruTS84FtOiPXFGDrSi49n+tYlm452xSZCxh9midvB+y4Or3mpscCr1527lwavP7ibD2c09LRWck5BmzBVo6AnGW0Wzm2rmOZXI38WM5cwNienuvntc2suDhdX8C7veNX6ZpFZ5kVNstSobOSczjYgq0cATnLaLdybF3GMrna+LOcuYDhVzHZbeTvlc0tuThdX8D70HUX0+XnzTUvcBalRGcl52ywBVs5AnKW0W7l2LqMZXK18Wc5cwGTdBcSv2bSLiF\/GKItuThdrX9hi89+\/lLcfxTjJHRWcq0XbMFWjoCcZbRbObYuY5lcbfxZzlzA+HsV\/5ZcnI4FvGZ+QGdlxsklFdi6UDPLA7ZmnFxSga0LNbM8LmOZmeVsU0HAJPC3dToOsDNvzOiszFnZpgRbW2Lm6cHWnJVtSrC1JWae3nYsM7ecbcpMBIy+cDfugDmFpZKmkHQB0718AXUvPydb7+a4dHRWcs4BW7CVIyBnGe1Wji0EjBzb3Fq2dbp+gB3WvyS7FZ2VXLMHW7CVIyBnGe1Wjq3tWCZXE7+WM4nAhF8hfB9S0v1Ifl8\/2Zqt0\/UFvEfv+lg5q1pxZaGzknMZ2IKtHAE5y2i3cmxtxzK5mvi1nAsBE3XJoppmamtro9bWVr9vbWjN1ukX3fEk8TTS\/Jo5wQ4kPPEE0FnJtQ6wBVs5AnKW0W7l2NqOZXI18Ws5cwGTdJAdQ9+1axdt3LiRqqur\/b65gTUbp2MBrwFQLQk6KzteNqnB1oaWXVqwteNlkxpsbWjZpbUZy+wsZ5s69wKGozP9\/f1UU1NTdlI2TscN1HbuQWdlx8smNdja0LJLC7Z2vGxSg60NLbu0NmOZneVsU2cuYJLWu2R9Qq+N03EDtV1DRmdlx8smNdja0LJLC7Z2vGxSg60NLbu0NmOZneVsU2cuYPj1Ge7atWtp+\/bt1NTUFBDZv38\/rVq1ijZv3kxZXfJo43Qs4LVryOis7HjZpAZbG1p2acHWjpdNarC1oWWX1mYss7OcbepcCBglYq699tppNHbu3JmZeNHrZFIPnMBr15DRWdnxskkNtja07NKCrR0vm9Rga0PLLi0EjB2vQqS2cXrNTY8F73zZuXNp8PqLC\/H+ki+BzkqOLtiCrRwBOctot3JsbcYyuVr4t5ybCIz\/VyvdoqnTsYDXnjU6K3tmpjnA1pSUfTqwtWdmmgNsTUnZpzMdy+wtZ5sDAiaBv6nTdQHz0HUX0+Xnzc3WqxVQOjorOSeBLdjKEZCzjHYrx9Z0LJOrgYzlWSVg1I6nwcHBgObq1aupu7s7lqyp0\/\/qyVG66SvfD+xAwJg1VHRWZpxcUoGtCzWzPGBrxsklFdi6UDPLYzqWmVnLT6pZJWD4TBl+WLSYnPRr6nQs4LVv0Ois7JmZ5gBbU1L26cDWnplpDrA1JWWfznQss7ecbY5ZJWDCqHVBE+UGU6erKwSwgNe8MaOzMmdlmxJsbYmZpwdbc1a2KcHWlph5etOxzNxiPlLmQsCoM19GR0dnUFm0aJHISbxJVxioSpg4Xb9C4LfOr6G\/Wb0oH57NeS3QWck5CGzBVo6AnGW0Wzm2JmOZXOlyljMXMGpdSkNDQ+J6FJ8IOPKybds2amlpSbxnSTl9zZo1tHTp0qkq1NfXE\/\/h51sH36RP\/MW3g3\/ffOU8uvnKD\/qsamFtcWc1NjZGdXV1mdxzVViwRAS2ct4FW7CVI+DPMvet\/Ec9IyMjtG7dOjI508xfLeQtZS5gTCIhUhiibsHWy1ICJlx+R0cHrVy5MvjxvpG36Y8eONlQ\/uIT9bSkcY5UdQtl98SJEzQxMUG1tbVUVVVVqHfL+mXAVs4DYAu2cgT8Wd6xYwcNDAzMMAgB449xYElFYNrb28t+6i5PXXV1dVFfX9\/UFQZRAmbTpk3U2Ng49ZEegdny6Eu05dEjwWffWnsxza+BgDFpIuz38fHxIJI1Zw6YmTAzTQO2pqTs04GtPTPTHGBrSio9XTgCs3fvXtq6dSsiMOno7FNwpCOLW6fTyjWZN8QOJHt\/cw7Md7txM8kFtiaU3NKArRs3k1xga0LJLY3JWOZmOdtcuZlCGh4ejiThcxGvvuvIZO2NidPVDiSOvDz7+Uuz9WYFlY7OSs5ZYAu2cgTkLKPdyrE1GcvkSpeznLmAkXu1mZbDB9mZLuKNmzfUdyBhC7WdJ9FZ2fGySQ22NrTs0oKtHS+b1GBrQ8suLQSMHa9CpE5zun6FQPfyBdS9\/JxCvHc5XgKdlRxlsAVbOQJyltFu5dimjWVyJctazk0EZs+ePbR+\/frgbTnicejQIRoaGkrc5iyLhijN6bqA4ekjLOA19wg6K3NWtinB1paYeXqwNWdlmxJsbYmZp08by8wt5StlLgSM2s7MO4JuuOGG4DwYXvvS09ND5TwfJuyaNKf3Pvwi9T58MMiGO5DsGjY6KzteNqnB1oaWXVqwteNlkxpsbWjZpU0by+ys5Sd15gJGPwdm4cKF1NnZGQiY5ubmIAKSxe4k5Z40p39y2zA99v2jQfKjd30sP16tgJqgs5JzEtiCrRwBOctot3Js08YyuZJlLUPAJPBNczq2ULs3TnRW7uzScoJtGiH3z8HWnV1aTrBNI+T+edpY5m4525yZCxh+fV7\/wutd9CkkFY1pa2uj1tbWTCilOb3mpseCemEHkr170FnZMzPNAbampOzTga09M9McYGtKyj5d2lhmbzEfOXIhYBhF1LH9GzZsyEy86HWK20atBEz7JfV0b\/uF+fBohdQCnZWco8AWbOUIyFlGu5VjCwEjxza3lpOcji3UpbkNnVVp\/JJygy3YyhGQs4x2K8cWAkaObW4tmwoY7ECydyE6K3tmpjnA1pSUfTqwtWdmmgNsTUnZp4OAsWdmlUM\/B0ZlzPrmzCSnYwu1lXtnJEZnVRo\/RGDk+IEt2GZDQK5UCBg5tsEi3t27d1N\/fz\/V1NQEJant1XldxHv9rudo11NjJ+uKLdTWrQMCxhqZcQawNUZlnRBsrZEZZwBbY1TWCSFgrJGZZdDPgeGzX\/Qnz+fAYAu1mX\/jUqGzKo0fogRy\/MAWbLMhIFcqBIwQ20oVMOoWamyhdmsYEDBu3Exyga0JJbc0YOvGzSQX2JpQcksDAePGzSgXw127di1t376dmpqagjx5nkLCLdRGbk1MhM6qdIaIbskxBFuwLT8BuRIhYITYKqEyPDycWgLfj\/Tggw+mpvOVIM7puoDBLdRutCFg3LiZ5AJbE0puacDWjZtJLrA1oeSWBgLGjVtF54pzun4GDB9gxwfZ4bEjgM7KjpdNarC1oWWXFmzteNmkBlsbWnZpIWDseBUitYmAwRkwbq5GZ+XGzSQX2JpQcksDtm7cTHKBrQkltzQQMG7cjHNFnQOT16sE9DNgnv38pTS\/Zo7xeyLhSQLorORaAtiCrRwBOctot3JsIWDk2FbcOTBqCzUjwRkwbg0DnZUbN5NcYGtCyS0N2LpxM8kFtiaU3NJAwLhxS81ViduocQZMqltTE6CzSkXknABsndGlZgTbVETOCcDWGV1qRgiYVERuCUoVMOFdTC0tLbRx40aqrq6OrJA+VZWWNs7p6hZqnAHj5nPOhc7KnV1aTrBNI+T+Odi6s0vLCbZphNw\/h4BxZ5ea0\/UqgePHj1NPTw8tW7aMWltbSf2\/oaGBuru7Z5Srn+zLAofzxqXlzFFO17dQ8+4j3oWEx54AOit7ZqY5wNaUlH06sLVnZpoDbE1J2aeDgLFnZpXD1yJetjM0NBQZhQl\/lpTWRMDgDBgrF09LjM7KnV1aTrBNI+T+Odi6s0vLCbZphNw\/h4BxZ1fWnEmiJCoCo6I3UZVMi8DgDBh316KzcmeXlhNs0wi5fw627uzScoJtGiH3zyFg3NmVLafJ9QP79++nVatW0ejoKO3cuZPCF0jqlVVOX7NmDS1dujT46K+\/8w799Xd+Gvz7y394AV1+3ulle78iFcSd1djYGNXV1cWuVyrS+5bzXcBWjjbYgq0cAX+WuW\/lP+oZGRmhdevWpY55\/mpQHkunTE5OTpanKNlS1PoXLiVuEW94rU1vb29Qqaj1MvxzJWD0mr99wQp6+4Jrgh8NdsyjhtNOlX2xglo\/ceIETUxMUG1tLVVVVRX0LbN5LbCV4w62YCtHwJ\/lHTt20MDAwAyDab+0+6tBeSwVQsCYiJfwgl\/Gy9GYrq4u6uvrm7pEUseuBMymTZuosbEx+Oi6hybo4NvvC\/49euel5fFSAUthf4yPj1N9fT3NmYODAH26GGx90pxuC2zBVo6AP8vhCMzevXtp69atiMD4Q+zHUtrOI1VKKQJGV604A8aP3zDf7YdjlBWwBVs5AnKW0W7l2GINjBzbkizzNBCvZ0k6+0UVEDWFlJQ3yukX3fEk8VZqnAFTkttwDkxp+BJzYyCQgwu2YCtHQM4yBIwcW2fL4UPslKFFixZRf39\/sDiUz3ppb2+fWqzLgmfbtm1BUpeD7HCInbO7pmXEQOCHIyIwchzBFmzLS0CuNAgYOba5tRx2un6IHc6AKc1tEDCl8UvKDbZgK0dAzjLarRxbCBg5trm1HHb6EweO0Yr7ngnq+9B1F9Pl583Nbd3zXjF0VnIeAluwlSMgZxntVo4tBIwc29xahoCRcw06K7CVIyBnGe0WbOUIyFmGgJFjm1vLYaf3Pvwi9T58EBEYDx7DQOABYowJsAVbOQJyltFu5dhCwMixza3lJAHz7Ocvpfk1OL\/E1XnorFzJpecD23RGrinA1pVcej6wTWfkmgICxpVcBecLO12dAcOvdPSuj1Xwm2VfdXRWcj4AW7CVIyBnGe1Wji0EjBzb3FqOEzAceeEIDB53Auis3Nml5QTbNELun4OtO7u0nGCbRsj9cwgYd3YVmzPsdBxi58+V6Kz8sQxbAluwlSMgZxntVo4tBIwc29xaDjsdh9j5cxU6K38sIWDkWIIt2JaPgFxJEDBybHNrWXd6w8KLiCMw\/PyXK+fTbb9zbm7rXQkVg4CR8xLYgq0cATnLaLdybCFg5Njm1rLu9HfOvGDqEDucwlu6y9BZlc4wzgLYgq0cATnLaLdybCFg5Njm1nKcgLm3\/UJqv6Q+t\/WuhIqhs5LzEtiCrRwBOctot3JsIWDk2ObWsu70F39hAV2\/67mgrrhGoHSXobMqnSEiMHIMwRZsy09ArkQIGDm2ubUMASPnGggYsJUjIGcZ7RZs5QjIWYaAkWObW8u60\/\/nix+gXU+NBXXFKbyluwwDQekMESWQYwi2YFt+AnIlQsDIsc2tZd3pX9xXRd984VhwfQAOsSvdZRAwpTPEICvHEGzBtvwE5EqEgJFjm1vLEDByroGAAVs5AnKW0W7BVo6AnGUIGDm2ubWsO\/2PvzZJh4++TZedO5cGryPPS+UAABCISURBVL84t3WulIphIJDzFNiCrRwBOctot3JsIWDk2ObWsu703\/7y8aCeEDB+3IXOyg\/HKCtgC7ZyBOQso93KsYWAkWObW8vK6b33folWPzIZ1JPPf+FzYPCURgCdVWn8knKDLdjKEZCzjHYrxxYCRo5tbi1HCRicwuvHXeis\/HBEBEaOI9iCbXkJyJUGASPHtiTLR48epc7OThoeHg7stLS00MaNG6m6ujrSrnIkf7ho0SLq7++nmpqaxLTXf+E+4l1I\/OAU3pLcNZUZAsYPRwyychzBFmzLS0CuNAgYObbOlo8fP049PT20bNkyam1tJfX\/hoYG6u7unmF3\/\/79tGrVKtq8eTM1NzfTnj17aGhoKFbwKKfrAgZnwDi7a1pGCBg\/HDHIynEEW7AtLwG50iBg5Nh6tZwkSvizgwcPRoqbqEoopy\/quJMe\/1FdkATXCPhxFwSMH44YZOU4gi3YlpeAXGkQMHJsvVqOEzDhaI1JocrpZ151HR1435Igy7arT6FLLlxA9fW4zNGEYVwaFjBjY2NUV1cXO91Xiv3ZnBds5bwPtmArR8CfZe5b+Y96RkZGaN26dbRz585g9qEozymTk5Mnt9cU4FHrYdra2oIpJf1RAmb58uV0\/\/33B2tmTNfAvLX4D+kn8y8LzM39u07q6OiglStXFoBYdq9w4sQJmpiYoNraWqqqOrm+CI8fAmDrh2OUFbAFWzkC\/izv2LGDBgYGZhiEgPHH2KslJVDYaNQiXvX54cOHpxbu9vb20ujoaOoamDcv76J3zjw\/uEbgz686JYi+IAJTmvvYH+Pj4wHHOXPmlGYMuWeIdbCVaRRotzJc2SrY+mMbjsDs3buXtm7digiMP8T+LKWJF\/Xl0Bf88s94UW9XVxf19fVRU1PTjAqpKSRdwOAeJD9+wxoYPxyjrIAt2MoRkLOMdivHFmtg5NiWZDlt55FunCMuCxYsmJpeYgFz55130pYtWyK3Uiunv\/7xXvr5e8\/EKbwleWp6ZnRWHmGGTIEt2MoRkLOMdivHFgJGjm1JltOmgXTj7EROr85+4X\/zE7Xlmn+unH7sP\/YH6XCNQEmumpYZnZU\/lmFLYAu2cgTkLKPdyrGFgJFj62w5fIidMqQW5\/Jhdjxt1N7ePrXyWj\/IzuTQu7ZP30gcgQmEzvIF1L38HOf6IuO7BNBZybUGsAVbOQJyltFu5dhCwMixza1ldjoEjIx70FnJcGWrYAu2cgTkLKPdyrGFgJFjm1vL7PTfv\/F24kW8\/OAaAX+uQmflj2XYEtiCrRwBOctot3JsIWDk2ObWMjv993ruJT4Hhh+cwuvPVeis\/LGEgJFjCbZgWz4CciVBwMixza3l8CJeCBh\/roKA8ccSg6wcS7AF2\/IRkCsJAkaObW4thwUMLnL05yoIGH8sMcjKsQRbsC0fAbmSIGDk2ObWMju9vaOTfvQ79wR1PHrXx3Jb10qrGASMnMfAFmzlCMhZRruVYwsBI8c2t5b1NTB8jQBO4fXnKnRW\/lgiSiDHEmzBtnwE5EqCgJFjm1vL+hQSBIxfN0HA+OWpWwNbsJUjIGcZ7VaOLQSMHNvcWtYFDE7h9esmdFZ+eULAyPEEW7AtDwG5UiBg5Njm1jIEjJxrIGDAVo6AnGW0W7CVIyBnGQJGjm1uLesCpv2S+uAgOzx+CGAg8MMxygrYgq0cATnLaLdybCFg5Njm1rIuYP7098+nlc0Nua1rpVUMnZWcx8AWbOUIyFlGu5VjCwEjxza3lnUBg4sc\/boJnZVfnro1sAVbOQJyltFu5dhCwMixza1lXcDgHiS\/bkJn5ZcnBIwcT7AF2\/IQkCsFAkaObW4t6wIG1wj4dRMEjF+eGGTleIIt2JaHgFwpEDBybHNrGQJGzjUQMGArR0DOMtot2MoRkLMMASPHNreWldP\/wxf\/gbqXn0N8mB0ePwQwEPjhGGUFbMFWjoCcZbRbObYQMHJsc2u5qE7PA3B0VnJeAFuwlSMgZxntVo5tUceyUyYnJyflsFW25aI6PQ9eQWcl5wWwBVs5AnKW0W7l2BZ1LIOASWgzRXW63NfE3DI6K3NWtinB1paYeXqwNWdlmxJsbYmZpy\/qWFbxAubo0aPU2dlJw8PDgTdbWlpo48aNVF1dnejd\/fv3U1dXF\/X19VFTU1Nk2qI63bzZy6U8ePAgDQwMUEdHBy1YsECuoFloGWzlnA62YCtHQM5yUceyihYwx48fp56eHlq2bBm1traS+n9DQwN1d3fHtgaVbt++fbR9+3YIGLnvTazlon6hMkA5o0iwlfMC2IKtHAE5y0VttxUtYKLcvWfPHhoaGkqMwrAze3t7g+yIwMh9aZIsF\/ULlQ3N6aWCrZwXwBZs5QjIWS5qu511AoannG6\/\/XZqb28PRIyJgFmzZg0tXbpUrnXNQssjIyO0bt06Alv\/zgdb\/0yVRbAFWzkCcpZVu925cyc1NzfLFVRmy4USMGo9TFtbWzClFBeh4Z8vXrw4dQ3MkSNHgkF27969ZXYLigMBEAABEAABfwT4l\/BNmzbRvHnz\/BnN2FJhBIxa18I84xbx8sJdXjh6yy23EIuTtEW8bIvT8R88IAACIAACIFCpBFi4FEm8sB8KIWBMxAu\/LE8ZXXHFFUEIzWQXUqU2VNQbBEAABEAABIpOoOIFjOnOo\/B2a92xRZsXLHqjxfuBAAiAAAiAQMULGI6qjI6OGp39orsbERg0fhAAARAAARCoXAIVLWDioiqLFi2i\/v7+4DA7PieGdxyFV15DwFRuo0XNQQAEQAAEQKCiBQzcBwIgAAIgAAIgMDsJQMDMTr\/jrUEABEAABECgoglAwFS0+1B5EAABEAABEJidBCBgEvzOC4S3bdsWpMBOJbcvCK81WrVqVbDQOu2iTZ0332eVdE+VW22KlcuGrXrz8P1hxSLi521suIbX4aGfSPaBDVs9LfqD0tq2+t5HrQctzXK2uSFgYvir+5J4MfDzzz8fnCHD\/66pqcnWYxVUuj5YrlixYtrFm+HXCN9hxf\/fvXs3mMf424atboK5rl+\/njZs2BB7WnUFNTHvVbXhGj7CARsDkt1hw1YJQ76UlzdgoD9wb+qK++DgYOF+EYeAiWkX6rJH\/gIVVb26fyXMcoY7dBaFu3btMtryjsEg\/TdZ\/SRpE7Y8KNx888107NgxSrpuw8y7xUxl02Y57Z133klbtmzBLzYGzcGWrd6+0R8YAI5IoqJYS5YsocOHD5MShG7W8pcLAibCJ+EwO8Lubg1Xj2Jx5Cr8\/ySr6LCSmbuwZVF+ySWX0Fe\/+lVatmwZIjARiG24hqOGbt+S2ZPLhm1UBGZoaMjol5\/ZQzT9TfkSR374SJHOzk4ImHRklZ8iKuLCnf+CBQvQ6Vu4NxwVsPmN1fWAQovqVXRSW7bqHrCbbropuI0dAiba\/TZcWcAcPHgwMIS1culfJxu2bE2f+li9enUw+OJxIxAWhG5W8pcLEZiECIy+4AkCxr7x2nZYqgQeGO6++24s4k1AbsOWB4IvfvGL1NHREVzmxoc7QsD4ETC8nkgt3GWfrF27Fu02pt3atFk19bF58+ZgDYxN9Na+pyp+DgiY4vt46g0xheTH2TYhY4gXO+Y2bDnt448\/HvwGi+nQZM42XMNTSGALtnbf4vKlhoApH+tclKRHXLCI180l4SmjtIWm2GlgztmGrb49XS8BYfmZvG24htsz+onk9mvDFuLQvC8wSQkBY0KpQGmwjbp0Z9psm0T43Y63DVvdMqIEyZxtuIYHBUxz+GMbNYWE6Tm7PkJPDQHjzq5ic+Igu9Jdl3RwlVoEyVMbcVECHAwW7wNTthAwdu3Yhqt+kB0OW0vnbMOWBeG1114bGAXbdLZJKSBgSuOH3CAAAiAAAiAAAiDgjQB2IXlDCUMgAAIgAAIgAALlIgABUy7SKAcEQAAEQAAEQMAbAQgYbyhhCARAAARAAARAoFwEIGDKRRrlgAAIgAAIgAAIeCMAAeMNJQyBAAiAAAiAAAiUiwAETLlIoxwQcCDwwgsv0Omnn2582zFvl3zttdfo3HPPdSjNLIva8r5o0SLq7+83rlulXNCp3q9cW3fLXZ6Zl5EKBPJPAAIm\/z5CDWcpAduD0cpx1kMpIqSUvOVsAiwo+Cnn5YGVwqacfkBZIJBGAAImjRA+B4GMCORRwNjWSUdXKYM0BExGDR7FgoAlAQgYS2BIDgI+CegnubJdNS3z\/PPPT51Cyj9XJxKHTyxW0xxnnHEGdXZ20vDwcFA9dc+ROhp\/cHAw+LnJtI9ehj6Nwicn8+3L6tmwYQO1trbOwBGXTgmYFStW0Be+8IUgX3iaRj99NVwOs7r55pvpox\/9aJBfvcurr75Kq1atotHR0SDLrbfeSg899BD19fVRU1NT8LO4d4ryZVjA8P\/feOON4I\/imHSPVPgeHy4j6meVKO58tn3YAoFSCUDAlEoQ+UHAkUDUvUT64BmOdsRdcMfFb9y4MbhpmkUMT300NzeTEkdtbW1TQiPpwkxVH2Wvuro6GHjvvvtu2r59eyAG0iIw4fT6nTYsslhoLFmyJKivsr979+5gLQ0Lka6urmnCQ7enRNr8+fOn8offUf1\/YmIiqPO8efOop6cnEEpqSijt3q0oAbNt2zZSgi2Kq94EIGAcvxDIBgKWBCBgLIEhOQj4IpA2EKaJhfBv9mEBE3X7d9JljlFTPOH0SXVKuygyfEEf1z9tWkn\/XAmYsCAbGhqaEjRsUxco\/P8777yTtmzZMm2xcdI0UZSA4eiOEl2qDE4XtYgZAsbXNwR2QCCZAAQMWggIZEhAn24JT+\/EiYXwNEtLS0tkBCY8laO\/ZtT0T1x5+qWbSQImbRFxlFiJ+ll4Wi08TaYiTPw+UUJEt8lRHXUhYNjNcdNAUQKG8+qLepOEFwRMhl8oFD2rCEDAzCp342XzSkAftPV1MPpv+UqQhNelqAhEOALDecORg6T3jxMnSdNaur1SBQzbUmtZlMCKisDYCJinn36a1BRVTU2NkfshYIwwIREIZE4AAiZzF6ACIPAuAV0EqAgDT1PwehFey7Fs2bJpC2d1kRIWMEnrXaKYl2MKKbzGRS+TxUbSdJCaQtIFTFS0Q59C4gjM2rVrp9bwmLQ1iSmkNDGZNpVmUm+kAYHZRgACZrZ5HO+bGwJRa2D0KIi+qDVqMaqKyKgpJH4xXeQo+7ygV01\/RK1DUUB8LeLVIx76upjFixfPWKQbFjB6XlVXrh8vyI0SMKaLeNmGWsOStvao1EW8aopP7RxT76EvXg43QgiY3HwtUZEKIgABU0HOQlWLR0ANbmoLsD49pG+B5imVq6++etpWaRYu11xzDd12221TEYawqFFRGbW9mgmqgTWOZtKWY9OFxVHbrU3WwITL3rx5c7DOhRfuqvfXIzD8DmGG4W3U4a3knCduC7iKevHfSvRFbaPW80e9l77+iP100UUX0bPPPhuIqLDQVO8Qjk4Vr7XjjUDALwEIGL88YQ0EQCBjAiwoonYemVbLZA2MqS3TdIjAmJJCOhB4lwAEDFoDCIBAxRIIr\/NR0Rb93Bfbl4OAsSWG9CCQDQEImGy4o1QQAAFPBMKnEyedkmtSZPhyxQceeCDIJnU3Ei5zNPEK0oDATAIQMGgVIAACIAACIAACFUfg\/wP\/1YSgJiFQawAAAABJRU5ErkJggg==","height":337,"width":560}}
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
