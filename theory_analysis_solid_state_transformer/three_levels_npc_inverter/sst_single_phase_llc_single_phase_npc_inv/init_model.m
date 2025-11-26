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
kp_i_dab = 0.0025;
ki_i_dab = 0.25;
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
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     1.775568181818182e-05"}}
%---
%[output:01f62bda]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     3.566505664210290e-06"}}
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUUAAADECAYAAAAI7zOUAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ2QV1d1P9FUs6kgLJBGssCSZMlEoyREnXUTCoqRdKYL1laBbQ1d0MEKpDMuZT\/SmQyV74ZIEqJSXRGnYWFaYVimdijZ2NS4Ek1IqEaUyLIhm00MWSQQQ2qpdM4j95+7d9\/Hve9\/3333nT1vJhPgf++59\/zOOb937ue75MKFCxeAH0aAEWAEGIEAgUuYFNkTGAFGgBF4CwEmRfYGRoARYAQkBJgUPXCHZ599FhobG6G+vh6am5ut9+jUqVOwePFimDhxIqxfvx4qKiqstyEL3LBhA+zbtw+2bdsGNTU1mbalCj937hy0tLTA+PHjM8EyS2UOHjwIDQ0NQRNLliwx6n+emKuYCH+bP38+zJs3L0vIMpHNpJgJrGZC8yBFbHP\/\/v2wbNkys85qlFYDNMu21O4IYtmxYwfU1tYm9ta0b7t27YJJkyZpyU5sXCogyPzEiRPQ3t4OlZWVJtXBJ1LEjmN\/0BZpdDFSPIPCTIoZgOq7yKxJWA5QxCLLLFjF2iQYTXFAQmxtbQVdwjXxA2qkaPpyMsEq67LkSRGDZOvWrQGOOKSSh3QieGfOnBk4Oj7r1q0blPLL9XF4K4afIqCw7tmzZ4PhoipfNZ4IKvHvIrjU4BTlcAiFfRdDKVGuv79\/0BBLHR7jjziEFFkH\/l0Mn1euXBlkh4cPHw5kTJ06ddDbXAQn\/qbqKg\/vdXC97777YPXq1UPaEv0J64NoH\/HER2Ag20UeZsqYCxwwQxTTEOLf1Lai+hD170ePHi0NbUW\/sI2ovoQFrtoX4U\/CXkJn\/HsY8UbVx+kQ4cvjxo0r4S1jpuIq4yb86uabbw58Bh\/M8GSdZ8yYEfz76dOnS\/6i+qPcZ9MXTtZEZyKfNCmqQwr1TS8CWzhP2O9ibmzMmDEBsYiAE0ZHJ0QHGhgYiM2I4mSr2ZRMiiK4VScTwYh9v+222wbNGcaRIhJdX1+fUV+xPw888EDphaKDq8BN1U0lXbUvMk5I2EjuKEvYSNYb56vkzFDYYPny5aUXW9jvgtxVTE36hn4Q1xd1+Jv04kJik19kSfVV3FRfVm0k4yC\/JGV\/EL6Mbav9xZcKzneKl6jq76qPuJ7HNiG9pLJkSTEuaxDEFjb3JYZ6n\/\/854csTsQFWJwTJA2NojJF+c0bN3RLCrioIIha2JH7c+eddwbBKjJH1EV+OeC\/q1jrDJ\/VrAczQtGWPK8WRjzyIo48TMO+YODKGZKc0arZV1Q2E9Y3fDnFvdhwQSluyBj2m\/xv4gUQNaeo4qAGdtKLCsur2aLIVMNekmp7qg8fOHBg0FSCwFK8kJJ8PomY8vydLCmGObxKHvfff\/+gVVL5d3WYKYwkhh1qBhRHiklvTR1SjJtIt02KqJt4ASAZrFixAoSzm+IalSmK7G\/atGmlrDXsRRRGimI6RA4cJEJcAFFJUR3iYR1BmlGZYljfokgxqi\/qqmvYS03Wbc6cObGZYtJ8ZhIpipcDvnxUnMNIUW0vihRV8hJTPUyKedJ6RNtZZIpyU2pAUcoUUU8RsNOnT4djx46Vhs6muKqkqOIWlpWaZIqyTZKyKRHoUS+2uL7pZIpxYZBnpjhlypRBox6R7YstWjYyRVV3JkUPSVFkO\/JQK2pOUbzddOYUoxwpKRtUZctzMFFzinET1\/JwRc0yxHyPmCNSh89hQ2DVhPIQUt0zp5OVJM3F4qQ+zmdhti4vJqWZU1TnL+OGcGFza+o8cVTfVGJLGtrLmCZl86ZziqoN42wiSBH7g\/PfYugbN3xOM6cor8wnxYOnlBF0i+zwWYCus0qKc2Rf\/vKXgypxq8\/ySq1Jpij6Yrr6HDUHpq4+y5kd\/hmHkLgiHrb6LFaUBS5xK+aiTNhKqA6uYqVfbevQoUPBfBQ+YlVz5MiRAUniIxZXsG\/CNlGrz1he9C8si41bdRUvTiRlgUNc3wQR4aKDIBSxACFsHLddJ271WCez0ll9FpirL2F5lRz9GF\/uwj+iFgnlOqpP4WKMOjUh24hXnxNoH4MCn7DTGiqw6haRLN8ocfN0WbbLsvURMN3vJmeCphug9Xs1\/EqGbdWKQ8HUbj4hmnmmKMCJOraEv3d0dDg5fqYCz6Tokyte7EvYNil5O1BSj9GfcGEojyOGSX0r0u\/qljPsu7rrIE6fIr+cMiVFTPdXrVoVYBd1FhXT\/N7eXqNznraci0nRFpL25KhDRHl4rNNKkc8+6+jnsow63SMfXojrh7Ahn30OQQlJp7q6OiC9qOGzPDdlGgAuHYTbYgQYgeGBQGaZIqbf27dvh7vuuitYYQwjRfFWr6urC04g8NBneDgda8kI+IxAJqSIZLdmzRpYuHBhcHVU3EKLDI5KkvJvV199tc84ct8YAUbAEgJdXV0wefJkS9LMxWRCimGnCLBrSXfECVJcsGDBkKuZkBR7enrMNfSsBuvhmUEAgG3il03ytkcmpKhCHJUp4oRsU1MTtLW1BRklDp+xbNgdbHkDZcttWA9bSNqTwzaxh6UNSXnbwzkpqkQoZ5VxG4nzBsqGsVHG8ePHcx0asB5DEWCb2PIKO3LyjnUnpGgDqryBsqEDk6ItFO3KYVK0i2e50vKOdSbFci1oWJ8D0BAwB8XZJg5ANmjCO1IM2zybpA8ezduzZ09SsbJ+zxuosjovVeYAtIWkPTlsE3tY2pCUd6wPyRTVOb8kJXFOcO3atcGxqiyfvIGypRsHoC0k7clhm9jD0oakvGOdh882rGgggwPQACxHRdkmjoDWbMY7UvT1HrS8gdK0Z2IxDsBEiJwXYJs4hzy2wbxjPTRTVA+Cq3cM5gFh3kDZ0pkD0BaS9uSwTexhaUNS3rGeOHyWL2xwedehCm7eQNkwNsrgALSFpD05bBN7WNqQlHesJ5KiUDLqC2c2QNCRkTdQOn3UKcMBqIOS2zJsE7d4J7WWd6xrk6KsiFhx3rRpE7i63ThvoJIMqfs7B6AuUu7KsU3cYa3TUt6xrk2Kcd+H0FG03DJ5A1Vu\/0V9DkBbSNqTwzaxh6UNSXnHeiwpqrfdxJ1NtgFGnIy8gbKlHwegLSTtyWGb2MPShqS8Yz108zZ+iwG\/UiaeuC+U2QBBR0beQOn0UacMB6AOSm7LsE3c4p3UWt6xHkmK4lu4SQq4+j1voGzpyQFoC0l7ctgm9rC0ISnvWOdjfjasaCCDA9AALEdF2SaOgNZsxktSVIfPSbrwhRBJCL31OwegPlauSrJNXCGt1453pKjXbfel8gbKlsYcgLaQtCeHbWIPSxuS8o517S05NpQtR0beQJXTd7kuB6AtJO3JYZvYw9KGpLxjnUnRhhUNZHAAGoDlqCjbxBHQms0wKRYEKM1uJhbjAEyEyHkBtolzyGMbZFLUtEfeQGl2M7EYB2AiRM4LsE2cQ86kaANyJkUbKNqTQYVIEBEqulDRI+9Y5zlFezyhJYmK41LRg0lRy22dFXrsV6fhM3eugv7vfcVZm2pDiaR47tw5aGlpgX379kF9fT0sWrQINm\/eDC5vyMFO5\/32sGUhKmRCRQ8mRVuebUdOx09egqUdR+DUvR+1IzCFlFhSFIRYV1cHkyZNgo6ODli\/fj10dnZCd3d38OeKiooUzZpXYVI0xyzLGkyKWaKbTjYFm3hPivKX\/QYGBkqk2NfXF3zBz2W2yKSYLlCyqkUhAAU2VHShoMeG\/b2wYf9xfzNFdBr8HEF\/fz\/MnTsX9u7dGwyfly5dGgylm5ubrcYctoVPmFwmRatQly2MQgAyKZbtBtYFFIIUUeuDBw9CQ0NDCYAsPmQl2liyZAmTonVXsy+QSdE+puVKpGCTwpBiucZKqo\/D9FWrVgXF8CJbzhSTEMv\/dwoByJli\/n6k9uAf\/q0HNnc95\/fw2QVsOGyurq6G3t5eHj67ANxCG0yKFkC0LIKCTeY8+BQ8duy0v6SofpclzIblfqIAP3mwfft2uOuuu+D+++9nUrQcKFmJoxCAnClm5R3p5XpPiqiayOLmzZtX0nTXrl1BVofDXPwzbs+57777jJHALT9r1qyBhQsXQk1NTdAWPlHDZ\/ytq6vLuB2fKuDKfVVVlU9dStUXKnqg8lR0Kboes2bNgtOfbA\/80dt9ivKWHCQt8cifOMWtOrg9Z9u2bcbBpX4YSwgIW2zh1WdjeDOtwJlipvCmEl50m5w49QbcuPpHfpOiyBS3bt0K4uNV6ipxOZmiavmkTLGnpyeVs\/hUqeiOS23IifqwTfyIEDziN+erT\/lPithDOaOT5xDljLGysrJsZJkUy4bQmQAqRMKk6MxlYhvCLHFZx5FgkeVtr78Cr3z907l1LPHsc249Uxrm4bMvlrjYDyZFv+xRdJvIWeI7jx2AF\/euzQ1gJkXH0FMhEyp6FJ1MZPctqk3kucSJlZfB6Z1\/C70\/+7HjyHyruURSVE+ziKr4Bb\/29nawMWzW0Z4zRR2U3JUpagCGIURFlyLqgYSI84j4f3w6v3gT3PGJaZDn+kEsKYp9isuXL4dHHnkk2DqD20nwKjG8OUfeppN1ODIpZo2wmfwiBmCUhlR0KZIeSIKCEIVd\/u4T1dB6++TcrwlMJMWmpiZoa2uD3bt3B6dOkAhtL7DohCOTog5K7soUKQCTUKGiS1H0wPnDZTuPlLJDtM\/3lk+D2snvDkyVd6xr3aeIK84zZswILoX45je\/GdyWc+LECR4+J0VbyO9Fcdwk1ajowXOKSZa297u8wiyk4hwiDpnx\/+LxmhRFJ7ds2QKzZ88G3KiNxIjXhrm8YNaHt4ct16BCJlT0YFK05dnRclZ\/rwfuffi5IQU+d+tVsGzmxEGE6EOsJy60ZA+ZXgt5vz30eplcigqZUNGDSTHZZ01LYEa48T96YcePXxxSFTPC2987FtZ\/6q0TcmqhvGOdSdHU4mWWp0ImVPRgUizToQEA5whfOP0GPPT4i8Hm67BHDJPxN3moHFbWa1LUOfvMW3LMnIoKmVDRg0nRzH+xNGaCOBz+zsH+2MpIflvmXw+3XjvKqBEvSTHqogZZM9fzinkDZWTVmMJUyISKHkyK0c4q9g7iN1N+eOz0oNXiqGzwY9dVwr2fvq6scMk71rW35Mi35JSlccrKeQOVsttDqlEhEyp6MClezPzw+UrXc3Ds5dcjh8CqM2Mm2Fh3FfzZjVckDolN4ifvWOc5RRNrWShLhUyo6DGcSBHJ7\/gr52DTgV448ZuLm6d1HyTAmVMq4Usfn2SVAAsxp6hz2zYqwsf8dN1pcDkqZEJFD0qk+MPDz8KECRPgB8\/+BnY98ZIx8SEWSH4TR18Gd\/\/pNTBuxDsyJ8BCkGK6UM++Vt4ptS0NqZAJFT2KRIois8P\/d\/zkRXj+1BvaQ13Zf8Xq7\/RrR8OdH5sI77z0bbmQX1RM5R3rPHy2xXaacqiQCRU9fCJFQXrf\/+Up+O6hXwceFbXFJcndBPFh1td8+2SYMPoyr4gvrv+FIEW8Xbu1tbWkRxbffU4yct5AJfVP93cqZEJFD1ekKAjv4PFX4Z\/f3MqSlvCEr4mh7rRJI2FR3VXw\/PPPwy1TozdF6\/po3uXyjvXETBFvw8brw8Q1YWLOsba2NvQDU1kBmjdQtvSiQiZU9CiXFAXZvfG\/v4evPfo8HDv5eqq5PNW\/5EzvI9eMgr\/88HuCInEbn6nYJO9YT7Ulh2\/JSU+RVByXih5RpCjI7vcXLsA\/\/aAPfvbCa2UNZ8Pm9HBo+77x74I7PjIe\/vAdb08kvSSvo2ITr0kRjcCZYpIrmv1OxXGLqIcgOsy2nnjuDPzroV\/Dz\/tfg56Tr0H\/mfNmhowoLWd4UyeMgM\/dUgWXXJJ8tM1G40W0SZje3pMidprnFG247EUZVBzXJz3kIey3ul8IiA4f0714cVaWyW7y2Ar4bO14uOLNLSvYftJ5XnseFC3JJ5uUo28hSLEcBW3VzRsoW3pQcdys9RBE99r\/\/B98u\/sF+MVLv7VOdGKO7vz583D1uHfB+6tGwLwPXgmjKi4teyhry19M5GRtE5O+lFM271hPXGgpRzmbdfMGypYuVBzXVA\/59MTPX3wNHj5yCo7++rdWszlhIzmrm1B5GdxyzSi49drRkURnqostX7Ath4oeecc6k6Jtz0yQR8VxUY+3v\/viiujLZ38HD\/34xeDcrO1haxTRTa0aAbe\/b2yJ6OT5QlOTUrLJ5MmTTdX3rrzXpCi230ycONH5TduqpfIGypbn+ByAglhePXceOg+\/DI8ffzUzkhND1+D\/uLF4TAXc\/r4x8IGrRpSgdjVP57NNTPyOih55x3piphh2jRhv3jZx1cFlXTmuPFw9PnAO9jz1MvSczC6TU0kO\/37LtaPhMzf\/Ebz9bZdEDl3TI2mvpiub2OtxuCQqenhPimHw42r0zp07+cNVKbw8rePKJPfvz7wC\/913Njj7mtVwNYzkcIvJp6ddCaMuv5TM6QnUM61NUpg\/0ypU9PCeFMMyRfy637Zt2yDujkW5XtyNOqr8qLJ5A2XLm8VcHA4Nkehwc3Dn4ZPw8JGBTAkujORumjgSGj58JVT8gfnGYSoByKRoy7Ptyck71hNPtCxevDjQVhzz01H93LlzsGbNGli4cGFAnJhZdnd3h85L4hHCjo6OxDnLvIHS0VveRoKnIMRwtdwzrlFtq6usOB93+w1jYdKbhCsToU7\/TcowKZqg5aYsFZvkHeuJc4o2zBl3LBAJs7e3N\/Ecdd5ABcPUN4ereL71Kw8\/Z307iUxyePzr49ePgZorLi+ZwNXCg47NqQQgZ4o61nZbJu9Yd0KKcZkiHiPcunVrCfUdO3YAXjahPnkAhSSo+32KMLdRM7k5U6+Ad\/3vqeAi0CyzOBcuzKToAmWzNqjYJI9Yl5HOlBTlW7zDyA6H2S0tLVBXVwfz5s0LbuNZsWJF6HwlAoVPV1eXmacYlMbzr99+8lX47s\/OJtYaP\/LiqYf3jLgU\/uqmkfDHky8Pzs+Kf48S0NfXB1VVVYnyfS9ARQ\/EmYouRddj1qxZJbfv6enJLQQyJUWhlSDH5ubm0CxQlFNJUkYlq7eHGBLP+epTkd+swIxv9nvHwtKZE8o+40rlbU5FDx4+58Y9kQ1nFeu6miYutDQ1NUFbW9uglWbTq8PiyE7uqCi3YMGCIeSZBVA4N\/ihdY8PwQpJED\/TiJ9rtP1QIRMqejAp2vbw8uVlEesmvQolxXK\/+4yZoUymKG\/lypWwcePGQeSqlsPhM84xhq102wQKs0M1M0QifGjR+2HEZZeWnQ3GGYAKmVDRg0nRhC7clLUZ62l6nCpT1GlIJVYxpxhGmI2NjdDf3w9x+x9tAIVkeOjEGVj0nWdKKiAZbpl\/Pdx67SgdtcouQ4VMqOjBpFi2S1sXYCPWy+mUkznFcjoo6pYLlFhJ7vjJS6XubP\/rG6D+A+NsdE9bBhUyoaIHk6K26zorWG6sl9vRIaQoFkVOnjwJ99xzTzCcPXz48JB2ivTdZyTEZR1HSl9Gw+yw84s3ZTpMjjIMFTKhogeTYrkUYr++d6RoX0U7EtMCFRDizl\/AY7\/6TdCRPAmRA9COL9iWQoXgqeiRNtZt+QX54fOuJ16Cv9lxxAtCZFK05bZ25VAhEyp6eE2K8uZr1Q2LMHx+7Feng1VmHzJEgR8Vx6WiB7+o7L5gbEjzmhSjFMR5xhkzZsRuxLYBjizDFCgcNt+4+kdeESIHoG2vsCOPCsFT0cM01u14wVtSUg2fTTdv2+i0KVBzv\/o0\/ODNeURcVHG15SZJVyqOS0UPflEleaz7301j3XYPU5Gi75fMqlni03\/\/Edu4pZZHhUyo6MGkmNqVM6voNSnGzSlG3WaTFVImQM158KnS9hskRL5yy75VmBTtY1quRCo2MYn1cjELq58qU8yiI0kydYGSs8Tp146GvV+8MUm009+pOC4VPThTdOr+Wo3pxrqWsBSFEklRPZYXdzdiiva1q+gC5XOWyAGobW6nBakQPBU9dGM9KyeJJcWo2210b8u22WkdoOQtODOnjIbdX\/ArS2RStOkR9mRRIRMqeujEuj3rD5WU6kIIX1effc8SmRSzdOX0sqmQCRU9vCbFqPsN4674Su+a8TV1gKr80vcDIbiw4tOKs6wZFceloge\/qLKK2PRydWI9vfTkmolzijhUbm1tBbHajITY0NAA69atCz4h4OpJAkoeOj+44HpY8KErXXXNqB0qZEJFDyZFI\/d1Ujgp1rPuRCIpYgei7kbMunOy\/CSgxNDZ5yyRA9Clx+i3RYXgqeiRFOv6lk1XUosU04m2WysOKHkbzq3XjILOpTfZbdyiNCqOS0UPflFZdG5LorwmRXU7jiWdU4mJAwovjl3acfEmHN82a6vKUiETKnowKaYKx0wreU2KqDle\/lBdXe10\/jAM8TigijJ05gDMNJZSC6dC8FT08JoUi3J1mFh19n3ozKSYmrcyrUiFTKjo4TUpZuqJhsKjgJLnE30fOjMpGhrdUXEqZEJFDyZFTcePAkresH3q3o9qSsuvGBXHpaIHv6jyi4Wolr0jxSJ9uKpIq87CAaiQCRU9mBSZFFUECr0lRybF5tmToXl2tX8WVnpEhUyo6MGk6F\/IeJcpqhD5fkuOWGTx6XbtODejQiZU9GBSZFI0yhR9vyWnSFtxePjsX\/CxTfy0ideZYtTmbZu35MhHCOO+EBgGVJG24nAA+hmAnCn6ZxevSTHrW3JQ\/po1a2DhwoVQU1MDcRfYxpFiUeYTOQD9C0C2iX828ZoUES6Xt+TEZaAqUBv298KG\/ccDixZhfyJniv4FH9vET5t4T4oIm6tbckwyxaLtT+QA9DMAOVP0zy6FIMWsYZOPE0Z9JVAFqoiLLByAWXtSOvlUVtKp6MGkKPmxIMfm5maora0d5OEIFD5dXV3Qf+Y81G\/vC\/7+wasug62f8vNC2bAQ7evrg6qqqnTR61EtKnogpFR0Kboes2bNKnl4T09Pbt7u1ebtqC1AiI789ijipm0ePufm44kNU8mwqOgxrDNFdcsPzl2uXLkSNm7cGKxGy48MlLzIUpRN20yKidyUWwEqZEJFj2FNiiaLODJQRfhqX1SEU3FcKnrwPG9u76LIhoc9KeqaJIwUff8eS5huVMiEih5MiroR6K6c96Qovt6nQhJ3+iQL+GSginiShYfPWXiFHZlUCJ6KHl6TolgNnj9\/vjefIyjyIgtnJXZIzLYUKmRCRQ\/vSbGpqQna2tqGLHzYdswkeQIo+fvORTrJwplikoXz+50KmVDRw2tSRDfFUya9vb2AewfzfMJIsWgrz5wp5ulB0W1TIRMqenhNij5+uKqox\/s4U\/STEPlF5Z9dvCZFn+ASQBX1eB+Tok\/eNLgvVDIsKnowKWrGigCqyCvPnJVoGttxMSpkQkUP70lRHL3bt28f1NfXw6JFi2Dz5s2wadMmqKysdOa+CNR\/PvFzuHH1j4I2i3SHogwSFceloge\/qJyFsHZDXpOifBZ50qRJ0NHRAevXr4fOzk7o7u4O\/lxRUaGtbDkFVVJ8cMH1sOBDxbkIgofP5Vg\/27pUCJ6KHl6Tonw2eWBgoESKeBvH2rVrnWaLCNSSrz1Suli2iCvPnJVkS25ppVMhEyp6eE2K6GQbNmyA\/v5+mDt3LuzduzcYPi9dujQYSrvcpoNAtWz\/L2jZ82zg+0Xco8ikmJa2sq1HhUyo6OE9KaI7qkf91q1b5\/yECwJ1Q9N34bFjp6GIZ555+JwtsZUjnQqZUNGjEKRYjsPZqotAjVz0EOAxPyZFW6iml0MlADl7T+8DWdVkUtREFoE6\/cn2oPSt14yCzqU3adb0qxgVMqGiB5OiX\/GBvWFS1LRJ9Q0fhjOf2BCU3vyZ6+CO2vGaNf0qRoVMqOjBpOhXfBSCFOV9igK+JUuWOF1kwXZlUizqHkUOQP8CkG3in028zhQFIY4fP75EguLfEEqX+xSrZn4WXp+2KLBgUbfjcAD6F4BsE\/9s4jUpqt9QEfDFfbQ+K4iZFLNCNp1cHj6nwy3LWlRs4jUpogFxO444ySJOr+Dexerqaqfbcq78i9Xwu4m3BD516t6PZulbmcqm4rhU9OBMMVN3TyXca1KMuzpM1hY\/TbBnz55UAOhWuuKOr8P5sdcVejsOB6Cutd2Wo0LwVPTwmhTdumZ8a2O\/8C\/w+8vHFno7DpOiTx71Vl+okAkVPZgUNeOk6FeGCTWpOC4VPfhFpRmADosVghTxkwStra0lWPI45idI8R\/\/fAosvuUqhyay2xQVMqGiB5OiXf+2Ic17UsRFFVxsaW9vD+5PFPOMtbW1TvcqClIs8h5FDkAbIWNfBhWCp6KH16To05YcQYpFvR2Hh8\/2ycyWRCpkQkUPr0kRnc63TLHIG7c5U7RFY3blUCETKnp4T4rofmnmFNXrxnbs2AE45FYf3Aje2NgY3NmID27vEUN1uSxninaJoFxpVAKQX1TleoL9+oUgRVO1cdi9atUquPvuu4N5SCRIzDjDyC5sc3hYe4IUi7xxmwPQ1JPclKdC8FT0IEmKqiuLxRm8qVvNFjEL7e3tTVy0QVIs8j2KPKfohuDStEKFTKjoMSxIEYfIK1euhI0bN0JNTc0gv8UMcuvWraV\/ixpmIykW+R5FJsU0dOWmDhUyoaIHeVKUvwg4b968QV6u\/oZD6RUrVsC2bduGkCeS4qWv\/BIeXzXbTaRk1Ap+9Kuqqioj6e7EUtEDEaOiS9H1mDVrVsmBe3p63Dmz0tIlFy5cuJBV62FXj8W1FUegSIorbpsEbX9ydVbddSKXytucih48z+vE7Y0aIZspxhFcFEKizoIFC4bMPSIpFn3jNuqdt8GNvDOmMBU92Ca2PMKenLx9K5NMUZcQ1c3hcavUSIqXH\/oWvOPED+2hz5IYAUbASwTIDZ\/VvYcCdVxEmTJlCjQ1NUFbW1swbyiXxRu+w+YTvbQad4oRYARIIpBJpkgSKVaKEWAEhgUCTIrDwsysJCPACOgiwKSoixSrVF73AAAGGElEQVSXYwQYgWGBAJPisDAzK8kIMAK6CDAp6iLF5RgBRmBYIOA9Kcqr00uWLEk8I+2D1XT6LLYt7du3L+hyfX290+9o6+Kko4ssK+5Ip26bWZTT1UMuF3VjUxb9M5Gpq4t8U1VRYkfgkMcXQ0XbXpOifJEEOmhLSwvU1dU5\/bSqibNiWd0+40UY+ODRR919naZ9Kbe8ri6iHaHHk08+6dXWKl091ItLdC8rKRdnk\/q6usgvJzxWWoTYkQkR70PI47Mn2AevSRENu3btWti0aVNwBRk6aXd3t5cZlTBo2j77qJupLqjDT3\/6U3jmmWdCL\/8wCX6bZXX1wMzq0Ucf9Xo0YqKLfF0f\/hkfvKnK10c+Fox9dP1t+UJkiupdi3EnXnwxdNo+++i0JrqIYMVhGuoSdiNSXjbS1QNJ\/eTJk9DV1QWHDx+OvPA4Lz2wXV1ddDPKPHWJa5uHzxHoqNlTEUgxTZ991ctEF3TiGTNmwJgxYyKvicsrAHX1wHIPPPBAaejv44tKVxd5KgcJPupKvrxsktQuk2IEQrpvxSSAXf5u2ue469Jc9jusLV1d5GGnjwstunqoc4g+vqx0dVH77iPBc6aYIsJ1509SiM6sikmffQw6GRhdXdSLglGGT+fYdfXQJZzMnEdDsK4uKgn67muq6pwpRjhDEedFdPtcBCfV1UUl0qhb1jViPpMiunrItzaJFVskd58WJ3R1CcsU8eNw69evh4qKikxwtimUSTEGTd09WTYNUq6sqD7Lhg7Lrnzcq6iji++kiP3T1UMu56M9THSRv8Lp657LqFhjUiyXhbg+I8AIMAKWEPB6n6IlHVkMI8AIMALaCDApakPFBRkBRmA4IMCkOByszDoyAoyANgJMitpQcUFGgBEYDggwKQ4HKzvUUb79x+RmFh83F4tbZrLccymvdhft1IlDt3LaFJOiU7izbczkUgmTsia9Trv\/0ldS7OjoyHxvX9ynfU2w57J2EGBStIOjF1JMiM6krIly6qkQ3bpMii0Q9r1zXfy4nD0EmBTtYelEkno5rRiiyheKik3HfX190NjYCHiSAZ+4sih38eLFwe0w+MQN5cSpCrWs3IeoIWfUJ22RFM+ePRv8hxfvqvVl2dg\/cdeeOPY2cuTIoJ7ot7w5HvXG+u3t7cEVdLqf1VUJXu1j3IZoleTjXkKcKToJHe1GmBS1ofKjoHxpgRpMcuBhb\/FiUZF9qBc1hJUVF\/jGXeqgXoirXmgRN3xWL3GVy37jG98ISE189xtJRRxLwzblb4XL9QYGBgLiX758eenyYfl3PNKGOJw4cSIgRXyQ\/PHoXm1tbUCW8r2DspXDSBEvP5WJN+roHJOiH\/GSphdMimlQy7GOer2V3JW4bEQlL7ksZpTyZb4oM+qYlUqYYSQZRzJRv5mQCPZ9586dAckhKapnreMuQzh69CjI84RxWVoYKcokGPfyMNGHM8UcAyqkaSZFv+yh1Rv5TKs8zFRJUR5C4lAPH3H5q1wWh8ENDQ1D2g5bPVYzKxNSjCPtOBIRWa\/4ns306dPhzJkzoaQY9mkHuc8HDhyA1tbWIbqGXX0fRopYUVwQIV8gUVNTM0gmk6KWK3tZiEnRS7Pod0oeZnZ2dpY+14DZn5xBxQ2fwzLFqB7kkSkiacsZpjp8LidTjEOaM0V9P6RUkkmxYNYMy0B6e3uD7EUdEuNc2z333BPMnWE9ec4ubk5RzP3Nnz9\/yEfCbM4pygS7e\/fuwBIiC1Mz2RUrVgTzjeJKLzFHGDZ8NplTFIsuAqekOcWoeU\/1Oi55iC\/mNVF22NVdPHz2KwiZFP2yR2Jv1NVneQVUBPi4ceOCoSUuXuDCAD5btmwJ\/i4WGNSyWEZefY7beB21+owykvYpyiu\/WF5etIgiRXn4jNMFbW1tgS44FYBP2P2NYuoAy6NemEWHrT5j\/aivxkVlikjI6jdc1KG0jJHow9NPPx2QorpwxKSY6PZOCzApOoWbG8sDgbR7J5PmFG3pwqRoC0k7cpgU7eDIUjxCIGzbUpobtJkUPTKqw64wKToEm5tyg4A6vE97g7Z69lmd97ShDZ99toGiXRn\/D6xy5uDgAmL\/AAAAAElFTkSuQmCC","height":196,"width":325}}
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
