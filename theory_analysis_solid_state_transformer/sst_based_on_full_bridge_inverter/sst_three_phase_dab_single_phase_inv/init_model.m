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

model = 'sst_three_phase_dab';
rpi_enable = 0;
rpi_ccaller = 1;
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
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/2) %[output:31c2a36b]
f0 = fPWM_DAB/5;
Cs = 1/Ls/(2*pi*f0)^2 %[output:538b2e49]

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
Vac_FS = V_phase_normalization_factor %[output:23e72b9a]
Iac_FS = I_phase_normalization_factor %[output:2e20a02c]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:0cc65d7c]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:8c1b6188]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:995db2c2]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:13ed32a5]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:7b651ad6]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:38b72f5b]

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
figure;  %[output:9482b399]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:9482b399]
xlabel('state of charge [p.u.]'); %[output:9482b399]
ylabel('open circuit voltage [V]'); %[output:9482b399]
title('open circuit voltage(state of charge)'); %[output:9482b399]
grid on %[output:9482b399]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink settings
heatsink_liquid_2kW; %[output:3c0bacdf] %[output:5cff4622] %[output:87721deb]
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
dab.Csnubber = Irr^2*Lstray_module/Vdab2_dc_nom^2 %[output:607bd0d3]
dab.Rsnubber = 1/(Csnubber*fPWM_DAB)/5 %[output:9a2cb80e]

% infineon_FF1000UXTR23T2M1;
inv.Vth = Vth;                                  % [V]
inv.Rds_on = Rds_on;                            % [Ohm]
inv.Vdon_diode = Vdon_diode;                    % [V]
inv.Vgamma = Vgamma;                            % [V]
inv.Rdon_diode = Rdon_diode;                    % [Ohm]
inv.Eon = Eon;                                  % [J] @ Tj = 125°C
inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
inv.Eerr = Eerr;                                % [J] @ Tj = 125°C
inv.Voff_sw_losses = Voff_sw_losses;            % [V]
inv.Ion_sw_losses = Ion_sw_losses;              % [A]
inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
inv.Rtim = Rtim;                                % [K/W]
inv.Rth_mosfet_JC = Rth_mosfet_JC;              % [K/W]
inv.Rth_mosfet_CH = Rth_mosfet_CH;              % [K/W]
inv.Rth_mosfet_JH = Rth_mosfet_JH;              % [K/W]
inv.Lstray_module = Lstray_module;              % [H]
inv.Irr = Irr;                                  % [A]
inv.Csnubber = Csnubber;                        % [F]
inv.Rsnubber = Rsnubber;                        % [Ohm]

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
%[output:31c2a36b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     5.918560606060606e-05"}}
%---
%[output:538b2e49]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     1.857555033442859e-05"}}
%---
%[output:23e72b9a]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     5.633826408401311e+02"}}
%---
%[output:2e20a02c]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     3.818376618407357e+02"}}
%---
%[output:0cc65d7c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.345132058575099"],["81.455860923503209"]]}}
%---
%[output:8c1b6188]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:995db2c2]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:13ed32a5]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000250000000000"],["-24.674011002723397","0.996073009183013"]]}}
%---
%[output:7b651ad6]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.388772090881737"],["67.915215284996137"]]}}
%---
%[output:38b72f5b]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.091119986272030"],["4.708907792220440"]]}}
%---
%[output:9482b399]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUUAAADECAYAAAAI7zOUAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXX+QV9V1PyYmulYpLmIMrrCoi2Ni4q+ablYpGGJoZrKQSZos0DR0YRxSETOTZdgfdppS5adgUDEJNRuKnbAwTWFY\/miJrs0Pg5jEH8SoKcqy4mY14hIEIyahoXMeuV\/v3n0\/7n3vvvvuO3veTCa433vPvedzzvm8c3++006ePHkS+GEEGAFGgBEIEDiNSZE9gRFgBBiBdxBgUmRvYAQYAUZAQoBJ0QN3eOGFF6C5uRkaGxuhtbXVeo8OHz4M8+fPh\/Hjx8PKlSuhqqrKehuywFWrVsHOnTth48aNUFdXl2tbqvDjx49DW1sbjBs3Lhcs81Rmz549MGfOnKCJBQsWGPW\/SMxVTIS\/zZo1C5qamvKELBfZTIq5wGomtAhSxDZ37doFt956q1lnNUqrAZpnW2p3BLFs3rwZ6uvrE3tr2retW7fChAkTtGQnNi4VEGR+8OBB6OzshOrqapPq4BMpYsexP2iLNLoYKZ5DYSbFHED1XWTeJCwHKGKRZxasYm0SjKY4ICG2t7eDLuGa+AE1UjR9OZlglXdZ8qSIQbJhw4YARxxSyUM6EbxTp04NHB2fFStWDEn55fo4vBXDTxFQWPfYsWPBcFGVrxpPBJX4uwguNThFORxCYd\/FUEqUGxgYGDLEUofH+CMOIUXWgf8ths9LliwJssO9e\/cGMq688sohb3MRnPibqqs8vNfB9Z577oE777xzWFuiP2F9EO0jnvgIDGS7yMNMGXOBA2aIYhpC\/E1tK6oPUX\/ft29fZWgr+oVtRPUlLHDVvgh\/EvYSOuN\/hxFvVH2cDhG+PHbs2AreMmYqrjJuwq+uvfbawGfwwQxP1nnKlCnB348cOVLxF9Uf5T6bvnDyJjoT+aRJUR1SqG96EdjCecJ+F3NjY8aMCYhFBJwwOjohOtDg4GBsRhQnW82mZFIUwa06mQhG7PtNN900ZM4wjhSR6Pr7+436iv257777Ki8UHVwFbqpuKumqfZFxQsJGckdZwkay3jhfJWeGwgaLFi2qvNjCfhfkrmJq0jf0g7i+qMPfpBcXEpv8Ikuqr+Km+rJqIxkH+SUp+4PwZWxb7S++VHC+U7xEVX9XfcT1PLYJ6SWVJUuKcVmDILawuS8x1Lv55puHLU7EBVicEyQNjaIyRfnNGzd0Swq4qCCIWtiR+3PbbbcFwSoyR9RFfjng31WsdYbPataDGaFoS55XCyMeeRFHHqZhXzBw5QxJzmjV7CsqmwnrG76c4l5suKAUN2QM+03+m3gBRM0pqjiogZ30osLyarYoMtWwl6TanurDDz300JCpBIGleCEl+XwSMRX5O1lSDHN4lTzuvffeIauk8u\/qMFMYSQw71AwojhST3po6pBg3kW6bFFE38QJAMli8eDEIZzfFNSpTFNnfNddcU8law15EYaQopkPkwEEixAUQlRTVIR7WEaQZlSmG9S2KFKP6oq66hr3UZN1mzJgRmykmzWcmkaJ4OeDLR8U5jBTV9qJIUSUvMdXDpFgkrUe0nUemKDelBhSlTBH1FAE7efJk2L9\/f2XobIqrSooqbmFZqUmmKNskKZsSgR71Yovrm06mGBcGRWaKkyZNGjLqEdm+2KJlI1NUdWdS9JAURbYjD7Wi5hTF201nTjHKkZKyQVW2PAcTNacYN3EtD1fULEPM94g5InX4HDYEVk0oDyHVPXM6WUnSXCxO6uN8Fmbr8mJSmjlFdf4ybggXNremzhNH9U0ltqShvYxpUjZvOqeo2jDOJoIUsT84\/y2GvnHD5zRzivLKfFI8eEoZQbfIDp8F6DqrpDhHdscddwRV4laf5ZVak0xR9MV09TlqDkxdfZYzO\/w3DiFxRTxs9VmsKAtc4lbMRZmwlVAdXMVKv9rWk08+GcxH4SNWNUeNGhWQJD5icQX7JmwTtfqM5UX\/wrLYuFVX8eJEUhY4xPVNEBEuOghCEQsQwsZx23XiVo91Miud1WeBufoSllfJ0Y\/x5S78I2qRUK6j+hQuxqhTE7KNePU5gfYxKPAJO62hAqtuEcnzjRI3T5dnuyxbHwHT\/W5yJmi6AVq\/VyOvZNhWrTgUTO3mE6K5Z4oCnKhjS\/h7V1eXk+NnKvBMij654qm+hG2TkrcDJfUY\/QkXhoo4YpjUtzL9rm45w76ruw7i9CnzyylXUsR0f+nSpQF2UWdRMc3v6+szOudpy7mYFG0haU+OOkSUh8c6rZT57LOOfi7LqNM98uGFuH4IG\/LZ5xCUkHRqa2sD0osaPstzU6YB4NJBuC1GgBEYGQjklili+r1p0ya4\/fbbgxXGMFIUb\/WGhobgBAIPfUaG07GWjIDPCORCikh2y5Ytg7lz5wZXR8UttMjgqCQp\/3bxxRf7jCP3jRFgBCwh0NPTAxMnTrQkzVxMLqQYdooAu5Z0R5wgxdmzZw+7mglJsbe311xDz2qwHp4ZBADYJn7ZpGh75EKKKsRRmSJOyLa0tEBHR0eQUeLwGcuG3cFWNFC23Ib1sIWkPTlsE3tY2pBUtD2ck6JKhHJWGbeRuGigbBgbZRw4cKDQoQHrMRwBtoktr7Ajp+hYd0KKNqAqGigbOjAp2kLRrhwmRbt4ZpVWdKwzKWa1oGF9DkBDwBwUZ5s4ANmgCe9IMWzzbJI+eDRv+\/btScUy\/V40UJk6L1XmALSFpD05bBN7WNqQVHSsD8sU1Tm\/JCVxTnD58uXBsao8n6KBsqUbB6AtJO3JYZvYw9KGpKJjnYfPNqxoIIMD0AAsR0XZJo6A1mzGO1L09R60ooHStGdiMQ7ARIicF2CbOIc8tsGiYz00U1QPgqt3DBYBYdFA2dKZA9AWkvbksE3sYWlDUtGxnjh8li9scHnXoQpu0UDZMDbK4AC0haQ9OWwTe1jakFR0rCeSolAy6gtnNkDQkVE0UDp91CnDAaiDktsybBO3eCe1VnSsa5OirIhYcV67di24ut24aKCSDKn7OwegLlLuyrFN3GGt01LRsa5NinHfh9BRNGuZooHK2n9RnwPQFpL25LBN7GFpQ1LRsR5LiuptN3Fnk22AESejaKBs6ccBaAtJe3LYJvawtCGp6FgP3byN32LAr5SJJ+4LZTZA0JFRNFA6fdQpwwGog5LbMmwTt3gntVZ0rEeSovgWbpICrn4vGihbenIA2kLSnhy2iT0sbUgqOtb5mJ8NKxrI4AA0AMtRUbaJI6A1m\/GSFNXhc5IufCFEEkLv\/M4BqI+Vq5JsE1dI67XjHSnqddt9qaKBsqUxB6AtJO3JYZvYw9KGpKJjXXtLjg1ls8goGqgsfZfrcgDaQtKeHLaJPSxtSCo61pkUbVjRQAYHoAFYjoqyTRwBrdkMk2JJgNLsZmIxDsBEiJwXYJs4hzy2QSZFTXsUDZRmNxOLcQAmQuS8ANvEOeRMijYgZ1K0gaI9GVSIBBGhogsVPYqOdZ5TtMcTWpKoOC4VPZgUtdzWaSHvSfH48ePQ1tYGO3fuhMbGRpg3bx6sW7cOXN6QgxYpGihbXkGFTKjowaRoy7PtyHn0xSPwmX\/ZAq89+CU7AlNIic0UBSE2NDTAhAkToKurC1auXAnd3d2we\/fu4N9VVVUpmjWvwqRojlmeNZgU80Q3nWwKNun66auwsOt5OHz3jelAsFArlhTlL\/sNDg5WSLG\/vz\/4gp\/LbJFJ0YK1LYqgEIACDiq6UNBj1a4+WLXrgL+kiE6DnyMYGBiAmTNnwo4dO4Lh88KFC4OhdGtrq8UwO9UWPmFymRStQp1ZGIUAZFLM7AbWBZSCFFHrPXv2wJw5cyoA5PEhK9HGggULmBStu5p9gUyK9jHNKpGCTUpDilmNlVQfh+lLly4NiuFFtpwpJiFW\/O8UApAzxeL9SO2B93OKriDDYXNtbS309fXx8NkV6BnbYVLMCGAO1SnYZMb9T8Gj+4\/4O6eofpclzI5ZP1GAnzzYtGkT3H777XDvvfcyKeYQLHmIpBCAnCnm4RnZZHpPiqieyOKampoq2m7dujXI6nCYi\/\/G7Tn33HOPMRq45WfZsmUwd+5cqKurS1xowQZ6enqM2\/GpAq7c19TU+NSlVH2hogcqT0WXsusxbdo0OPLpzsAfS7ElB0lLPPInTnGrDm7P2bhxo3FwqR\/GEgLCFlt49dkY3lwrcKaYK7yphJfdJgcPvw1X3fmY36QoMsUNGzaA+HiVukqcJVNULc9bclLFQiGVyh6AMmhUdCm7HmKRxetMUc4Mm5ubg\/2K8hyinDFWV1dnDk4mxcwQOhNQ9gBkUnTmKloNYZZ4a9fzwSLLu956HV7\/5ue06uVRiC+EyAPVGJlUyISKHmgqKrqUWQ888zzj608FkXPGgUfgle13OI7Md5pjUnQMfZkdl2J2xaToOABCmpPnEsdXnwlHtnwZ+n7xk8I6lkiK6mkW0VP8gl9nZyfYGDbraM8LLToouStDhdyZFN35TFhLSIiYIeL\/49N9y9XwxU9cA729vYV1LPFCCPzc6aJFi+CRRx4Jts7gdhK8SgxvzpG36eStAZNi3gibyWdSNMPLReky2QRJUBCiwOa2j42Hf\/7UJYVfE5hIii0tLdDR0QHbtm0LTp0gEdpeYNFxGCZFHZTclSlTACahQkWXsuiB84e3bnm+kh2ifXbcchVMvvTcwFRFx7rWfYq44jxlypTgUohvfetbwW05Bw8e5OFzUrSF\/F4Wx01SjYoePHxOsrS93+UVZiEV5xBxyIz\/Lx6vSVF0cv369TB9+nTAjdpIjHhtmMsLZn14e9hyDSpkQkUPJkVbnh0tZ+V\/H4DV3zt1r4H8zGu4EHDILBOiD7GeuNCSP2R6LRT99tDrZXIpKmRCRQ8mxWSfNS2BGeFd3+uD7\/zklWFVkQBvunwM3PXZSZFii451JkVTi2csT4VMqOjBpJjRoQEA5whfPfo7ePCxgWDzddgjhsn4m5oZquW9JkX5cwRRZ595S46ZU1EhEyp6MCma+S+Wxkzw7odfggf3DMRWRvJbP+tyuOHS0UaNeEmKURc1yJq5nlcsGigjq8YUpkImVPRgUox2VrF3EL+Z8uP9R4asFkdlgzdeVg1f+9xlmcKl6FjX3pIjZ4qZNE5ZuWigUnZ7WDUqZEJFDybFU5kfPut6XoIXX3srcgisOjNmgnPrx8Fnr3lf4pDYJH6KjnWeUzSxloWyVMiEih4jiRSR\/PoGj8Oa7\/XBwd+c2jyt+yABTp10Lnzl47VWCTCsfe9IUee2bVSEj\/nputPQclTIhIoelEjxx3tfgIsuuggeffE3sOWnrxoTH2KB5Df+3DPhnz51CZx\/zntzJ8BSkGK6UM+\/VtFvD1saUiETKnqUiRRFZvfyb96GzT95BV4+\/Lb2UFf2X7H6e8Ol58KXPzYezjj9XYWQX1RMFR3rPHy2xXaacqiQCRU9fCJFQXrf33cYvvvErwOPitrikuRugvgw61syfeKpDFA6NZJUv8jfS0GKeLt2e3t7Bac8vvucZISigUrqn+7vVMiEih6uSFEQ3uN9b8C\/P3ZqK0tawhO+Joa6V48fBfOvvxBefvlluP7Kdz4bouuTvpUrOtYTM0W8DRuvDxPXhIk5x\/r6+tDvM+cFcNFA2dKLCplQ0SMrKQqye\/sPf4Rv\/PBl2P\/aW6nm8lT\/kjO9j14yGv72I+8PisRle1RsUnSsp9qSw7fkpKdIKo5LRY8oUhRk98eTJ+GBH\/XDM79600p2JxMbDm0\/MO7sYFvLn53x7kTSS\/I6KjbxmhTRCJwpJrmi2e9UHLeMegiiw2zrZy8dhe8++Wt4buBN6D30JgwcPWFmyIjScob34Zpz4OYbauC005KPttlovIw2CdPbe1LETvOcog2XPSWDiuP6pIc8hP327l8FRIeP6V68OCvLZDfxvCr4wl++H9436oxgOIvt+7CI4ZNNskRMKUgxi4K26hYNlC09qDhu3noIonvzd\/8H\/\/bYr+CXr\/zWOtGJoeyJEyfg4rFnw4cuPAearnsfjK56T+ahrC1\/MZGTt01M+pKlbNGxnrjQkkU5m3WLBsqWLlQc11QP+fTEc6+8CQ8\/fxj2\/fq3VrM5YSM5q7uo+ky4\/pLRgHvyBAmqtjTVxZYv2JZDRY+iY51J0bZnJsij4riox7v\/\/NSK6GvHfh\/cnYcrr7aHrVFE9+ELz4FPXnFehejk+UJTk1KyycSJE03V966816Qott+MHz\/e+U3bqqWKBsqW5\/gcgIJY3jh+Arr3vgaPH3gjN5KTszZchR0\/pgr++oNjAMlOJUJb2EfJ8dkmJrpT0aPoWE\/MFMOuEePN2yauOrSsK8eVh6t4CcC2p16D3kP5ZXIqyeF\/N1w6GpquvQDe\/a7TIoeu6ZG0V9OVTez1OFwSFT28J8Uw+HE1esuWLfzhqhRentZxZZL7r2dfh5\/3HwvOvuY1XA0jOdxi8vlrL4DRZ51O5vQE6pnWJinMn2sVKnp4T4phmSJ+3W\/jxo0Qd8eiXC\/uRh1VflTZooGy5c1iLk5s5cDNwd0\/PwQPPzeYK8GFkRweD5vzkQug6j3mG4epBCCToi3Ptien6FhPPNEyf\/78QFtxzE9H9ePHj8OyZctg7ty5AXFiZrl79+7QeUk8QtjV1ZU4Z1k0UDp6y9tI\/vVH\/ZXhatYzrlFtq6usuKUEFx8m\/GnvnEyEOv03KcOkaIKWm7JUbFJ0rCfOKdowZ9yxQCTMvr6+xHPURQMVDFP\/NFzdf+gt+NrDL1nfTiKT3AfHnQ0fv3wM1J1\/VsUEPmwQFp2hEoCcKdqIcLsyio51J6QYlyniMcINGzZUUN28eTPgZRPqUwRQSIK636cIcws1k5tx5flw9h8OBxeB5pnF2XXRcGlMii5QNmuDik2KiHUZ6VxJUb7FO4zscJjd1tYGDQ0N0NTUFNzGs3jx4tD5SgQKn56eHjNPMSiN5183PfEGfPcXxxJrjRt1elDm\/eecDl+4ehT81cSzgvOz4u9RAvr7+6GmpiZRvu8FqOiBOFPRpex6TJs2reL2vb29hYVArqQotBLk2NraGpoFinIqScqo5PX2EEPiGV9\/KvKbFZjxTf\/AebBw6kWZz7hSeZtT0YOHz4VxT2TDecW6rqaJCy0tLS3Q0dExZKXZ9OqwOLKTOyrKzZ49exh55gEUzg1et+LxYVghCd79ucvgY5dV6+KoXY4KmVDRg0lR23WdFcwj1k06H0qKWb\/7jJmhTKYob8mSJbB69eoh5KqWw+EzzjGGrXTbBAqzQzUzRCL8zrwPwTlnnp45G4wzABUyoaIHk6IJXbgpazPW0\/Q4Vaao05BKrGJOMYwwm5ubYWBgAOL2P9oACsnwyYNHYd6Dz1ZUQDJcP+tyuOHS0TpqZS5DhUyo6MGkmNmlrQuwEetZOuVkTjFLB0XdrECJleSun75a6c6mv78CGj881kb3tGVQIRMqejAparuus4JZYz1rR4eRolgUOXToEKxZsyYYzu7du3dYO2X67jMS4q1dz1c+FITZYfctV+c6TI4yDBUyoaIHk2JWCrFf3ztStK+iHYlpgQoIccsvgw+E41MkIXIA2vEF21KoEDwVPdLGui2\/ID983vqzV+EfNj\/vBSEyKdpyW7tyqJAJFT28JkV587XqhmUYPj\/64pFgldmHDFHgR8VxqejBLyq7Lxgb0rwmxSgFcZ5xypQpsRuxbYAjyzAFCofNV935mFeEyAFo2yvsyKNC8FT0MI11O17wjpRUw2fTzds2Om0K1MyvPw0\/+tM8Ii6quNpyk6QrFceloge\/qJI81v3vprFuu4epSNH3S2bVLPHpf\/yobdxSy6NCJlT0YFJM7cq5VfSaFOPmFKNus8kLKROgZtz\/VGX7DRIiX7ll3ypMivYxzSqRik1MYj0rZmH1U2WKeXQkSaYuUHKWOPnSc2HHLVcliXb6OxXHpaIHZ4pO3V+rMd1Y1xKWolAiKarH8uLuRkzRvnYVXaB8zhI5ALXN7bQgFYKnoodurOflJLGkGHW7je5t2TY7rQOUvAVn6qRzYduX\/MoSmRRteoQ9WVTIhIoeOrFuz\/rDJaW6EMLX1Wffs0QmxTxdOb1sKmRCRQ+vSTHqfsO4K77Su2Z8TR2gqr\/yP4EQXFjxacVZ1oyK41LRg19UeUVserk6sZ5eenLNxDlFHCq3t7eDWG1GQpwzZw6sWLEi+ISAqycJKHnofP\/sy2H2dRe46ppRO1TIhIoeTIpG7uukcFKs592JRFLEDkTdjZh352T5SUCJobPPWSIHoEuP0W+LCsFT0SMp1vUtm66kFimmE223VhxQ8jacGy4ZDd0Lr7bbuEVpVByXih78orLo3JZEeU2K6nYcSzqnEhMHFF4cu7Dr1E04vm3WVpWlQiZU9GBSTBWOuVbymhRRc7z8oba21un8YRjicUCVZejMAZhrLKUWToXgqejhNSmW5eowsers+9CZSTE1b+VakQqZUNHDa1LM1RMNhUcBJc8n+j50ZlI0NLqj4lTIhIoeTIqajh8FlLxh+\/DdN2pKK64YFceloge\/qIqLhaiWvSPFMn24qkyrzsIBqJAJFT2YFJkUVQRKvSVHJsXW6ROhdXqtfxZWekSFTKjowaToX8h4lymqEPl+S45YZPHpdu04N6NCJlT0YFJkUjTKFH2\/JadMW3F4+Oxf8LFN\/LSJ15li1OZtm7fkyEcI474QGAZUmbbicAD6GYCcKfpnF69JMe9bclD+smXLYO7cuVBXVwdxF9jGkWJZ5hM5AP0LQLaJfzbxmhQRLpe35MRloCpQq3b1wapdBwKLlmF\/ImeK\/gUf28RPm3hPigibq1tyTDLFsu1P5AD0MwA5U\/TPLqUgxbxhk48TRn0lUAWqjIssHIB5e1I6+VRW0qnowaQo+bEgx9bWVqivrx\/i4QgUPj09PTBw9AQ0buoP\/vsvLjwTNnzGzwtlw0K0v78fampq0kWvR7Wo6IGQUtGl7HpMmzat4uG9vb2FebtXm7ejtgAhOvLbo4ybtnn4XJiPJzZMJcOioseIzhTVLT84d7lkyRJYvXp1sBotPzJQ8iJLWTZtMykmclNhBaiQCRU9RjQpmiziyECV4at9URFOxXGp6MHzvIW9iyIbHvGkqGuSMFL0\/XssYbpRIRMqejAp6kagu3Lek6L4ep8KSdzpkzzgk4Eq40kWHj7n4RV2ZFIheCp6eE2KYjV41qxZ3nyOoMyLLJyV2CEx21KokAkVPbwnxZaWFujo6Bi28GHbMZPkCaDk7zuX6SQLZ4pJFi7udypkQkUPr0kR3RRPmfT19QHuHSzyCSPFsq08c6ZYpAdFt02FTKjo4TUp+vjhqrIe7+NM0U9C5BeVf3bxmhR9gksAVdbjfUyKPnnT0L5QybCo6MGkqBkrAqgyrzxzVqJpbMfFqJAJFT28J0Vx9G7nzp3Q2NgI8+bNg3Xr1sHatWuhurramfsiUN\/\/2XNw1Z2PBW2W6Q5FGSQqjktFD35ROQth7Ya8JkX5LPKECROgq6sLVq5cCd3d3bB79+7g31VVVdrKZimokuL9sy+H2deV5yIIHj5nsX6+dakQPBU9vCZF+Wzy4OBghRTxNo7ly5c7zRYRqAXfeKRysWwZV545K8mX3NJKp0ImVPTwmhTRyVatWgUDAwMwc+ZM2LFjRzB8XrhwYTCUdrlNB4Fq2\/RDaNv+QuD7ZdyjyKSYlrbyrUeFTKjo4T0pojuqR\/1WrFjh\/IQLAnVFy3\/Co\/uPQBnPPPPwOV9iyyKdCplQ0aMUpJjF4WzVRaBGzfsO4DE\/JkVbqKaXQyUAOXtP7wN51WRS1EQWgTry6c6g9A2XjIbuhVdr1vSrGBUyoaIHk6Jf8YG9YVLUtEntFR+Bo59YFZRe9\/nL4Iv14zRr+lWMCplQ0YNJ0a\/4KAUpyvsUBXwLFixwusiC7cqkWNY9ihyA\/gUg28Q\/m3idKQpCHDduXIUExd8QSpf7FGum\/h28dc28wIJl3Y7DAehfALJN\/LOJ16SofkNFwBf30fq8IGZSzAvZdHJ5+JwOtzxrUbGJ16SIBsTtOOIkizi9gnsXa2trnW7LueBv7oTfj78+8KnDd9+Yp2\/lKpuK41LRgzPFXN09lXCvSTHu6jBZW\/w0wfbt21MBoFvp\/C9+E06cd1mpt+NwAOpa2205KgRPRQ+vSdGta8a3dt6X\/gP+eNZ5pd6Ow6Tok0e90xcqZEJFDyZFzTgp+5VhQk0qjktFD35RaQagw2KlIEX8JEF7e3sFliKO+QlSvOuzk2D+9Rc6NJHdpqiQCRU9mBTt+rcNad6TIi6q4GJLZ2dncH+imGesr693uldRkGKZ9yhyANoIGfsyqBA8FT28JkWftuQIUizr7Tg8fLZPZrYkUiETKnp4TYrodL5limXeuM2Zoi0asyuHCplQ0cN7UkT3SzOnqF43tnnzZsAht\/rgRvDm5ubgzkZ8cHuPGKrLZTlTtEsEWaVRCUB+UWX1BPv1S0GKpmrjsHvp0qXw1a9+NZiHRILEjDOM7MI2h4e1J0ixzBu3OQBNPclNeSoET0UPkqSourJYnMGbutVsEbPQvr6+xEUbJMUy36PIc4puCC5NK1TIhIoeI4IUcYi8ZMkSWL16NdTV1Q3xW8wgN2zYUPlb1DAbSbHM9ygyKaahKzd1qJAJFT3Ik6L8RcCmpqYhXq7+hkPpxYsXw8aNG4eRJ5Li6a\/\/Lzy+dLqbSMmpFfzoV01NTU7S3YmlogciRkWXsusxbdq0igP39va6c2alpdNOnjx5Mq\/Ww64ei2srjkCRFBffNAE6PnlxXt11IpfK25yKHjzP68TtjRohmynGEVwUQqLO7Nmzh809IimWfeM26l20wY28M6YwFT3YJrY8wp6con0rl0xRlxDVzeFxq9RIimc9+W1478Ef20OfJTECjICXCJAbPqt7DwXquIgyadIkaGlpgY6OjmDeUC6LN3yHzSd6aTXuFCPACJBEIJdMkSRSrBQjwAiMCASYFEeEmVlJRoAR0EWASVEXKS7HCDB4WKX2AAAGFElEQVQCIwIBJsURYWZWkhFgBHQRYFLURYrLMQKMwIhAwHtSlFenFyxYkHhG2ger6fRZbFvauXNn0OXGxkan39HWxUlHF1lW3JFO3TbzKKerh1wu6samPPpnIlNXF\/mmqrLEjsChiC+Gira9JkX5Igl00La2NmhoaHD6aVUTZ8Wyun3GizDwwaOPuvs6TfuStbyuLqIdoccTTzzh1dYqXT3Ui0t0LyvJirNJfV1d5JcTHistQ+zIhIj3IRTx2RPsg9ekiIZdvnw5rF27NriCDJ109+7dXmZUwqBp++yjbqa6oA7PPPMMPPvss6GXf5gEv82yunpgZvWDH\/zA69GIiS7ydX34b3zwpipfH\/lYMPbR9bflS5Epqnctxp148cXQafvso9Oa6CKCFYdpqEvYjUhF2UhXDyT1Q4cOQU9PD+zduzfywuOi9MB2dXXRzSiL1CWubR4+R6CjZk9lIMU0ffZVLxNd0ImnTJkCY8aMibwmrqgA1NUDy913332Vob+PLypdXeSpHCT4qCv5irJJUrtMihEI6b4VkwB2+btpn+OuS3PZ77C2dHWRh50+LrTo6qHOIfr4stLVRe27jwTPmWKKCNedP0khOrcqJn32MehkYHR1US8KRhk+nWPX1UOXcHJzHg3BurqoJOi7r6mqc6YY4QxlnBfR7XMZnFRXF5VIo25Z14j5XIro6iHf2iRWbJHcfVqc0NUlLFPEj8OtXLkSqqqqcsHZplAmxRg0dfdk2TRIVllRfZYNHZZd+bhXUUcX30kR+6erh1zOR3uY6CJ\/hdPXPZdRscakmJWFuD4jwAgwApYQ8HqfoiUdWQwjwAgwAtoIMClqQ8UFGQFGYCQgwKQ4EqzMOjICjIA2AkyK2lBxQUaAERgJCDApjgQrO9RRvv3H5GYWHzcXi1tm8txzKa92l+3UiUO3ctoUk6JTuPNtzORSCZOyJr1Ou\/\/SV1Ls6urKfW9f3Kd9TbDnsnYQYFK0g6MXUkyIzqSsiXLqqRDdukyKbRD2vXNd\/LicPQSYFO1h6USSejmtGKLKF4qKTcf9\/f3Q3NwMeJIBn7iyKHf+\/PnB7TD4xA3lxKkKtazch6ghZ9QnbZEUjx07FvwPL95V68uysX\/irj1x7G3UqFFBPdFveXM86o31Ozs7gyvodD+rqxK82se4DdEqyce9hDhTdBI62o0wKWpD5UdB+dICNZjkwMPe4sWiIvtQL2oIKysu8I271EG9EFe90CJu+Kxe4iqXfeCBBwJSE9\/9RlIRx9KwTflb4XK9wcHBgPgXLVpUuXxY\/h2PtCEOBw8eDEgRHyR\/PLpXX18fkKV876Bs5TBSxMtPZeKNOjrHpOhHvKTpBZNiGtQKrKNebyV3JS4bUclLLosZpXyZL8qMOmalEmYYScaRTNRvJiSCfd+yZUtAckiK6lnruMsQ9u3bB\/I8YVyWFkaKMgnGvTxM9OFMscCACmmaSdEve2j1Rj7TKg8zVVKUh5A41MNHXP4ql8Vh8Jw5c4a1HbZ6rGZWJqQYR9pxJCKyXvE9m8mTJ8PRo0dDSTHs0w5ynx966CFob28fpmvY1fdhpIgVxQUR8gUSdXV1Q2QyKWq5speFmBS9NIt+p+RhZnd3d+VzDZj9yRlU3PA5LFOM6kERmSKStpxhqsPnLJliHNKcKer7IaWSTIols2ZYBtLX1xdkL+qQGOfa1qxZE8ydYT15zi5uTlHM\/c2aNWvYR8JszinKBLtt27bAEiILUzPZxYsXB\/ON4kovMUcYNnw2mVMUiy4Cp6Q5xah5T\/U6LnmIL+Y1UXbY1V08fPYrCJkU\/bJHYm\/U1Wd5BVQE+NixY4OhJS5e4MIAPuvXrw\/+WywwqGWxjLz6HLfxOmr1GWUk7VOUV36xvLxoEUWK8vAZpws6OjoCXXAqAJ+w+xvF1AGWR70wiw5bfcb6UV+Ni8oUkZDVb7ioQ2kZI9GHp59+OiBFdeGISTHR7Z0WYFJ0Cjc3VgQCafdOJs0p2tKFSdEWknbkMCnawZGleIRA2LalNDdoMyl6ZFSHXWFSdAg2N+UGAXV4n\/YGbfXsszrvaUMbPvtsA0W7Mv4fc0rj4GUWu+4AAAAASUVORK5CYII=","height":196,"width":325}}
%---
%[output:3c0bacdf]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:5cff4622]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:87721deb]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
%[output:607bd0d3]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"dab","value":"                   Vth: 5.500000000000000\n                Rds_on: 0.001040000000000\n            Vdon_diode: 4\n                Vgamma: 4\n            Rdon_diode: 0.001850000000000\n                   Eon: 0.077000000000000\n                  Eoff: 0.108000000000000\n                  Eerr: 0.009700000000000\n        Voff_sw_losses: 1300\n         Ion_sw_losses: 1000\n    JunctionTermalMass: 2\n                  Rtim: 0.010000000000000\n         Rth_mosfet_JC: 0.019000000000000\n         Rth_mosfet_CH: 0.006000000000000\n         Rth_mosfet_JH: 0.035000000000000\n         Lstray_module: 1.200000000000000e-08\n                   Irr: 475\n              Csnubber: 1.203333333333333e-09\n              Rsnubber: 2200"}}
%---
%[output:9a2cb80e]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"dab","value":"                   Vth: 5.500000000000000\n                Rds_on: 0.001040000000000\n            Vdon_diode: 4\n                Vgamma: 4\n            Rdon_diode: 0.001850000000000\n                   Eon: 0.077000000000000\n                  Eoff: 0.108000000000000\n                  Eerr: 0.009700000000000\n        Voff_sw_losses: 1300\n         Ion_sw_losses: 1000\n    JunctionTermalMass: 2\n                  Rtim: 0.010000000000000\n         Rth_mosfet_JC: 0.019000000000000\n         Rth_mosfet_CH: 0.006000000000000\n         Rth_mosfet_JH: 0.035000000000000\n         Lstray_module: 1.200000000000000e-08\n                   Irr: 475\n              Csnubber: 1.203333333333333e-09\n              Rsnubber: 6.944444444444443e+05"}}
%---
