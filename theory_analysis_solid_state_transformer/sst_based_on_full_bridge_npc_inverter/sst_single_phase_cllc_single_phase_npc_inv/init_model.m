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

model = 'sst_cllc_npc_inv';
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
fPWM_DAB = fPWM*5; % PWM frequency 
tPWM_DAB = 1/fPWM_DAB;
half_phase_pulses = 1/fPWM_DAB/2;

fPWM_LLC_pu = 0.65;
fPWM_LLC = fPWM_LLC_pu*fPWM_DAB %[output:8141aa1d]

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

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:673f9826]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:6e722844]
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
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/4) %[output:3903d2b4]
Cs = 1/Ls/(2*pi*fres)^2 %[output:0f15b374]

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
Vac_FS = V_phase_normalization_factor %[output:7b4951a3]
Iac_FS = I_phase_normalization_factor %[output:4bcf6517]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:5d7dfb85]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:9ae51091]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:9f83c7bc]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:8794d0c6]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:056316b8]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:885b2734]

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
figure;  %[output:035a3cd4]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:035a3cd4]
xlabel('state of charge [p.u.]'); %[output:035a3cd4]
ylabel('open circuit voltage [V]'); %[output:035a3cd4]
title('open circuit voltage(state of charge)'); %[output:035a3cd4]
grid on %[output:035a3cd4]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink settings
heatsink_liquid_2kW; %[output:80039208] %[output:965bb623] %[output:551aa7f8]
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

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":47.6}
%---
%[output:8141aa1d]
%   data: {"dataType":"textualVariable","outputData":{"name":"fPWM_LLC","value":"       13000"}}
%---
%[output:673f9826]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"     4.583333333333334e+02"}}
%---
%[output:6e722844]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"     9.375000000000000e+02"}}
%---
%[output:3903d2b4]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     1.278409090909091e-05"}}
%---
%[output:0f15b374]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     4.953480089180958e-06"}}
%---
%[output:7b4951a3]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.919183588453085e+02"}}
%---
%[output:4bcf6517]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:5d7dfb85]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.345132058575099"],["81.455860923503209"]]}}
%---
%[output:9ae51091]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"5","name":"Afht","rows":2,"type":"double","value":[["0","0.000010000000000"],["-1.421223033756867","-0.000188495559215"]]}}
%---
%[output:9f83c7bc]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.018661060362323"],["3.911916400415777"]]}}
%---
%[output:8794d0c6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000250000000000"],["-35.530575843921682","0.995287611019615"]]}}
%---
%[output:056316b8]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.466526509058084"],["97.797910010394418"]]}}
%---
%[output:885b2734]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.091119986272030"],["4.708907792220440"]]}}
%---
%[output:035a3cd4]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAcgAAAETCAYAAACyWJYGAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfXt0X9V15qYhrUWAGBkyoKiOjZELUzI29jhxHAhNXEKTGZlpZmUkOTPNckWqNjisVdtLsk1hjQOxZI3NlAHTerKElpPWj3SKB3vmD8pyCA1RTACD+kcDFn7WiIfBmEcQYQieta9zxNHRfZx9fvfe3z33fnetrCD\/9nnc73xnf3ef51mnT58+TXiAABAAAkAACACBCQicBYEEI4AAEAACQAAITEYAAglWAAEgAASAABAIQQACCVoAASAABIAAEIBAggNFQODkyZPU2dkZVGVgYIAaGxszq9bIyAgtW7aM5s+fT319fdTQ0JBZWXrGeb6jzQspHL71rW9RW1ubTZJC22zYsIG2bNkS1LGrq4t6enpE9d25cyetWbOGent7C4kHv9++ffsy7x8i0CpojAiygo1e71fOUzyiBJId0LXXXksLFy7MBI6od8y63KiXcXG47KAfeeQRkfi4pJE2AJexdOnS8WRlFMiyfdBI27go9hDIorQE6pELAmNjY7R69Wras2cPbdu2LTeB5Mg1j3LDQFTOtrW11VrsVIQlER+XNC6NrgRSUjeznKJHkIqnx44dQxTpQpKU0kAgUwKyXtnoQ01cB33ISI+e5s6dS7fffntQTXaU+nCjinaGh4cn\/a7nccMNN9CNN94Y2DQ1NdHg4CC1tLSEvrouRKa9GV3x72rIdfHixXTnnXfSnDlzAsegC0tSPjxUq8p98skng\/rxo4ZYb7vtNvr2t78diKN6TCz43xWmuoAqp6zbKyer8tIdtv6O99xzD\/X394eWe\/z48aB+o6Oj43XS21DHkTHfvHkz3XfffaTej\/E3sVbYqaHrMDFQ7aqXq97XfC\/V1s3NzeMib+K3e\/fuYMhSPTo\/zPySPkxMPsblFcfDuEhTx4TrrOpuiq7Zv3RsVR4rVqygvXv3Evcf1Xb6O\/O\/qTL0tk3CJYyH9fIzVS0XAulpy5tOUX8N1cnDnKDp2DgfFicljubvYQ48Tlz4t6i6KWc2bdq0CXOQSiD1OrAQhQmaLpJmPmkJZFiEYjor03FG4cr\/HiWQ3d3dtHz58knYxwkS\/3bRRRfRiRMngg+AMNHiMnVHbtY9iheq3P3794eK3f333z8+76fzTRcAUyDNvNTvUSIZx1lOc\/To0Ugh1utkiqP5EaPE6ZprrqEf\/\/jHE7xAmMiF9S9T4NgmrI7876qcpLx1XIoe5XrqOkXVhkCK4CqOsXIA+he0ci5cSz164ihBdTzdAemdWf9y1h0qi5CKcFQeqmwzUlHohP2ud\/brrrsuUiD1L2xpPkkCyVEzP0lDnXERLke1r7766iRM9KiHcZo9e\/aEd7QZYjWHfxX2qj05WjTbnevC83FhkS1juWTJkuB99YjTZuGSzXCpaWP+rTBRYs71TypbcS\/sfdS\/8YcUv3PUEKuOo+KT2aYPPfRQILRhEWFUvuYogoqa9TziylYRpuJ\/Ei5pDCUXx2P5WRMIpJ\/tRlFfl8rBsGOYN29e6ApO3ebIkSOhUQHDoufBUYtacZq0yCbpyzdKgHSHweVL80lLIPWyWez40R1ylOPSBeIb3\/iGtUCGRdxh5erRuRK8qAiNbdnRq3ro2IaVZw41xwlk1NCymSYuGgz7uDKHx9XwvSm06qMgSsiS+Km3r55HVLua0ajCSglk1NC6vkJb57Lql\/rwtnJDOi5hw\/qeuitvqw2B9LTp8hZIfZtEkgOSChs3Qdi2D9t8dOdvOlPOW9\/mYRNBso2+sIX\/5i0FZgRtOmipQJo4mlGmKcxpCaSifJQw88reMIE0h2qTIkgfBDJsxEK1q8m\/qAhSzyOqb0Ag\/XS0EEg\/223SUJs+fBU3xGoOBao5naiv8bAhsSSBjBsa1aMarid\/ZUcJpG0+PHRlipcaejYFkkXIZvGDKR56hGUOU7OgJA2xcnSbJDBmHq5DrDqlo6Iyk\/am2JnRlHpnfSRBvY\/ijpkmbIg1qbtlPcSqPqZU5B0lkGGRt8LIjCCjFlWZw7txQ6xhuGCINYkt2f8Ogcwe40xKyHqRTpzAJAlkXN3C5ueiBDIpHx6OUvOJJsg2AslpwlaxqrzMlYj6BnvJIh011Kan4XK\/8pWvBNFt2MM4hb2f7SIdzlN9NJjCHLWARU+j23CZd911F91xxx2TFhRxGlMg+d+iFvyod036IAsbfkyK4HUco94xTtx0Qbr55psjuRWXB9chbPGO7SIdHZekEZRMHAsynYAABNJzQthu89C3aCRt8whb+CMZYmVI44bvkhbB6CfrxOXD5ZhbAm699VZ6+umnxxelhEWQuvOMWmik523OjYYJqC4Uelr+byWQYeV+97vfnXAiDB9eoM93umzz0IVOd9hhW4CitpeYuOpzopwn47Zx40ZatWpVAIc+EqBWI0dtG0navxi3zYPLso2souYOeRQhTHyiombGKGyLTVgUGvVxxf9untwTNZer8rAZ6fDcfRW++hDIwjeRewWTVgy654yUeSAQNpRrrlSO2oeq18\/loIA83q+MZegfNOpDIGxla9K746CAJITy+b3QAskdm\/eK8SbrMEcQFl0kfZ3mA2sxSoFAFqMdXGsRN8QcNzQcVp7LUXOu9a56urAhVsYk6XCNsI+aspyd6ysnCiuQNosLeChm5cqVtHbt2sgTXXxtmDTqDYFMA8X65mEON3JtpOLIaXC2Z77taE59SMSRa4oPmnzbK6q0wgokzwcwSfiJiiC5069fv542bdqU6Y0QxWgq1AIIAAEgAATyRKCQAslfzevWraOOjo5AJKMEUolo1lcm5dkgKAsIAAEgAASKgUAhBZLH8PnhEyfi5iDNsf64FYnFgBu1AAJAAAgAAV8QKJxA8rDp1q1b6ZZbbiE+KDtOIDm65CXY6mYK82+zES699FJf2gX1BAJAAAiUDgG+9WTmzJnevFfhBFK\/UDZpFauJcpI9C+ShQ4e8aZy8KgpcopEGNsDGpR+CN+Go+YZLoQQybMWegjnpDjm2S1q041vjuHRMlzTABSIA3rggAN5IUfPN1xRKICURodoGsmjRIuIjwNTfvJy6p6cntN18axwp+VztgQscnQt3wBvwRsob3zjjlUAqEeTVrXxAdNyB0mEN51vjSMnnan\/48GGv5gVc39MlHbCJRg3YABtpn\/LNBxdaIKXgJ9n71jhJ75PW73B0cHQuXAJvwBspb3zzwRBIaQuX0B6ODo7OhdbgDXgj5Q0EUopYjva+NU5e0MDRwdG5cA28AW+kvPHNByOClLZwCe3h6ODoXGgN3oA3Ut5AIKWI5WjvW+PkBQ0cHRydC9fAG\/BGyhvffDAiSGkLl9Aejg6OzoXW4A14I+UNBFKKWI72vjVOXtDA0cHRuXANvAFvpLzxzQcjgpS2cAnt4ejg6FxoDd6AN1LeQCCliOVo71vj5AUNHB0cnQvXwBvwRsob33wwIkhpC5fQHo4Ojs6F1uANeCPlDQRSiliO9r41Tl7QwNHB0blwDbwBb6S88c0HI4KUtnAJ7eHo4OhcaA3egDdS3kAgpYjlaO9b4+QFDRwdHJ0L18Ab8EbKG998MCJIaQuX0B6ODo7OhdbgDXgj5Q0EUopYjva+NU5e0MDRwdG5cA28AW+kvPHNByOClLZwCe3h6ODoXGgN3oA3Ut5AIKWI5WjvW+PkBQ0cHRydC9fAG\/BGyhvffDAiSGkLl9Aejg6OzoXW4A14I+UNBFKKWI72vjVOXtDA0cHRuXANvAFvpLzxzQcjgpS2cAnt4ejg6FxoDd6AN1LeQCCliOVo71vj5AUNHB0cnQvXwBvwRsob33wwIkhpC5fQHo4Ojs6F1uANeCPlDQRSiliO9r41Tl7QwNHB0blwDbwBb6S88c0HI4KUtnAJ7eHo4OhcaA3egDdS3kAgpYjlaO9b4+QFDRwdHJ0L18Ab8EbKG998MCJIaQuX0B6ODo7OhdbgDXgj5Q0EUopYjva+NU5e0MDRwdG5cA28AW+kvPHNByOClLZwCe3h6ODoXGgN3oA3Ut5AIKWI5WjvW+PkBQ0cHRydC9fAG\/BGyhvffDAiSGkLl9Aejg6OzoXW4A14I+VN5QTy5MmT1NnZScPDwyKs5syZQ7t27RKlqdXYt8ap9X1t08PRwdHZckW3A2\/AGylvfPPBNUeQSiB7enpo4cKFVnjt27ePNmzYAIG0Qit7Izg6ODoXloE34I2UNxBIC8QgkBYg5WgCRwdH50I38Aa8kfKmcgIpBaie9r41Tl5YwdHB0blwDbwBb6S88c0HpzbEynOQXV1dxEOtRX18a5y8cISjg6Nz4Rp4A95IeeObD65ZIBVAPKe4ZcuW4M+mpiYaHByklpYWKX6Z2vvWOJmCoWUORwdH58I18Aa8kfKm+ff+Cx3\/0felyepmn5pAqjcwV7UWKaqEQIbzDI4Ojs7FA4E34I2EN9sff5Fu2v5zOnnn5yXJ6mqbukDqbzMyMkLLli2j0dHRQkSVEEgIpLS3QQQgAlLOsD14Mxk1CGQMk9TK1YGBAWpsbHThXM1pIJAQSCmJ4OggkFLOQCDDEdvw4BHa8OBhRJAKHj2C5H\/btm2b9V5JF1ImpYFAQiCTOGL+DoGEQEo5A4GEQEZyZmxsjFavXk179uwJbFpbW6mvr48aGhpceJZqGggkBFJKKAgkBFLKGQgkBHISAvoq1iJEi2FNBIGEQEqdHQQSAinlDAQyHDFeoMPzkJVapKOvWi1StAiBtO\/WEAGIgD1bPrAEb8AbCW8qKZDPPfcc3XzzzXTbbbdZzy9mcdQcz3d2d3dTf39\/5P5LRJCIICUdGpFAPFoQSAikpD9VUiCLcFi5mvd88sknYw8ogEBCICUdGgIJgZTyRdnj42Eycks2P0WPHjxV3SFWCZnSvO5KRaRcPiJISSucsUVnRiQgZw14E4cZ+tRkdObe8VM6dvKdagmkS8dKMw1HsOvWraOOjo7gCi0IpBxddGYIpJw1EEgIpIw1jSseDhJUapGODKL0rXfu3BlkOm\/ePKs5SFWDvXv3pl8ZT3M8fvw4NTc3e1r7bKsNbKLxBTbAxqb3LV68mN4\/50J644sbIJA2gKVlwwtztm7dSrfccgtxZ8UiHTdkEUEignRhDngD3tjy5tHnTtGSe5+CQNoCloYdD6lee+21wepZrGJ1RxSODo7OhT3gDXhjyxt1zByGWG0Rq9HOvDVEzy7qSDusYg0HHY4Ojs6lO4I34I0tb9QK1t94+xV65a+\/apus7naZ3uaR59shgnRHG44Ojs6FPeANeGPDG165yitY+Tn7lWfp5e\/9qU2yQthkIpC8cGbNmjXBC3I0d\/ToURoaGsr0TFYIpDuf4Ojg6FzYA96ANza8Uddcse05+++r9oXJPC\/I9z\/ygpnly5dTT08P8Z5HPsC8qakp+LteD4ZYMcQq5R5EACIg5QzbgzdnUOPokRfn8P9Pb5xCb9z3NTp06JALpHVJk2oEqZ+qM3v2bOrs7AwEkRfR4D7IurSvVaHozBABK6IYRuANeJPEGz167Ll+Jm35sy9AIFkUIZBJ1CnO73B0cHQubARvwJs43uhzjxw9Pv0XnyHfRvFSjSAZLJ5\/5PlGfYhViWV7ezu1tbW59MVU0vjWOKm8tEUmcHRwdBY0mWQC3oA3UQjoQ6tss7njCupYcDEEksHg4dSlS5dOwK63t7eu4siVgUCG0xmODo4OAumCAHgThoApjvd0XEFLF1wcmPrmg1OPINOlWbq5+dY46b49OrMLnvh4AG\/AG3sE9BNzOBVHjRw9qsc3HwyBtG\/70lpCBCACLuQGb8AbHYG\/3HuMvv1\/D47\/U9c1zdT7hy0TQKq0QMadbqOj1NXVVZftHr41jovTckkDRwdHB964IADe8HCqGlJVaPCCnHvar6CrL5s6CSDffHDqESQv0tmxYwcNDAxQY2NjAJASTl6ks2TJkrrtifStcdLtsujMLnji4wG8AW8mI8CiyI\/a46iL4+5vXhXseQx7fPPBqQqkvg+S9z7qj74P8sCBA8Hdjbt27XLhnnMa3xrH+UWFCSECEAEhZQJz8KZ6vGFh\/H+\/ep8W9D424eXjokbd0DcfDIF08QwlSwNHVz1HlwaFwZvq8IYX3\/Q\/eJgePXhqkjDevuQyav03F1lRqtICyQglDbHyPki1V\/Kuu+6yAjUtI98aJ633TsoHjq46ji6JC5LfwZty84ajxf4Hj9C2x1+Y9KIcMfJQKj9Rw6lh6Pjmg1ONIBUgYfsg1RVULI533303DQ4OUkvLxBVOks7pYutb47i8o0saOLpyOzoXTtikAW\/Kx5uoSFEJ4fQLplD39TNDF+DYcMY3H5yJQNoAVQ8b3xonL4zg6Mrn6PLgDnjjP284Stz40BH6m8cmR4nq7VyjRUSQefTCFMuAQIaDCUfnv6NLsZtYZwXe+MebuAhRF0QVKbI4SoZQk8jjmw9OPYLkexmXLVsWXHllPnztlb79IwnMtH\/3rXHSfv+o\/ODo\/HN0eXEjrhzwpti8YTH82ZHX6UfPnpy0uMasOYvgd25ooU9+\/NxUBdEsxzcfnKpAjo2NBXscFy1aNL7fsaOjY9LNHvXq3L41Tl44wdEV29HlxQNpOeBNcXjDYvj40dfp4WfsxJBrvvZLl9LCmR\/NVBAhkBoC5j5I3us4Y8aM4JByXrizfft26uvro4aGBmlfTMUeAokhVimRIALFEQFp29XTPivesBBytPeXe4\/Scy+\/nRgZMgZs\/9lZU4nvY1R\/1wsb33xwqhGkKZC8YvXIkSPBsXK4MLlelEwuN6vOnFxy8S2ADQTShaW18kYJ4aaHjtDhV8ashFCJH88f\/vf\/9Dv0wuvvBuKY5hyiCxZ6mkoLJAPBUSM\/pig+9NBDwT2RiCBrpVj66WvtzOnXqDg5AhsIpAsbbXijzjG96LzfpHsfOUaHT9gLoRkZcl5hZ5+61D3LNJUXSH0ekodWWTC3bNlCTU1Nddn76PPXS5ZE1fO26cx51aVo5QAbCKQLJ3XeKCHc8cSLdOxVmQjqUeHaL19KTR\/9rboPk7rgodJUXiBrAS\/rtL41TtZ4qPwhAhABF66BNx+gxiJ49OQY7Xj8RfqXk+9YD4nquAfDoRdMob6vzKZzf+tDXgthFJ9888GZzkHqIGEO0sUF5ZMGjg4C6cK0KvFG3V7xj8+9Rj94\/MUALvNcUhsMlQjO\/e3z6Marm8eTFGme0OY9XG0gkJ2dwfxj3G0e6hosV5Bd0\/nWOK7vKU1XJUcHbKQIVOPjgRfF8PPqL96l+34ySqdPn3YSQM6Dxe6iBqLrrryEFs2aWriFMukxQJ6Tbz44lQiSV6uuWbMmEa16XZSsKuZb4yQCmpIBBLIaIpASXcaz8Yk3KgL83mOj9LNDrztHgEoAg\/+\/YAr90Wea6OLzz8wL6otkfMImbV7E5eebD05FIBUgcfdB5tkIUWX51jh5YYbODIF04VpReKO2RLz0xru044kXaOSlt+nYa2duund51HAnC+Anm8+jrmvkQ6FFwcbl\/bNM45sPTlUgswQ2jbx9a5w03tkmD3RmCKQNT0ybPHijRO6tX75H9z5yPFgFWov4mRHgvOnn0+9fMW1SBOiCh54mD2xqrWM90vvmgyGQ9WBJwcpEZ4ZAulCyVt6o7Q\/v\/up9+vv9LwWrP2sVPyWAHP39duMU+tqnL6HmqVOCaDLPTfO1YuPSHj6kqZxAqmHV4eHhxPbBYeWJENXFAJ0ZAulCvCje6EOb9z\/1Ev3wmZNB9i6rPs166cOfX7h8Gv3bT5yfevTngkU9ous06pl3HpUTyLwBrqU83xqnlneVpIVAQiBt+aLE74Hhl+mJ516i1949O5WoTx\/6vHrWBfS7TR+hf\/fJiyZUy6etEOhT4YzyzQdjiNXWM5TYDp0ZAqmGOw+98jY9dvj11IY7deHjYc\/p06bQjZ9tprd++avxM0J9Ej5bN4A+BYGM5ErYto\/e3t7gVo96Pr59veSFFTpzOQVSiR4LEB9z9ujIa8GLpjHPpxBTG995vu8zl06lGdPO3NTjw7mgWfYv9CkIZCgCLI47duyYcDGymqdsb2+vq0hCIMNJi87sj0DqovfmO+\/R9x97gf559K3gBdKY49OFT0V\/POTZvuDi4CcWRLXgBbzxhzdZfgxI8vbNB6c6xBq3DxJHzUlolK8tHF19HZ2+qOX0aaJdT3+wsCXtaC8QuV+v8FzwiY\/SFy5vHBc8fTjUhoHgTX15Y9NGRbOBQOKouaJxMrE+cHTpOzpzk\/rfPfkS8fxeWlsZ9BrrKzunT2ugjgUXEwvt+L83TknkgIsBeJM+b1zawac0lRZIbigMsfpE1zN1haOzd3S68H1v3yi98PovMxc9nt+77GPnEEd8aoizCHN84I09b\/zzCtnUuPICqUTSPJsVi3SyIVwauVbd0SnR+9X7p+mv\/vFf6JkXfhHAmubwpmonPdr7103n0peuvJA+dNZZuW5iT4Mz+LCKR7HqfSoKHQhkWr0vg3x8a5wMIAjNskydWS0gUaI3dOhUsHpT\/Z216HGkx6LX8OEPjYueqlNe7ZlXOWXiTdqYAZtwRH3zwZks0qn3atWyfL2k3Wmj8vOhMyuB47m1waHn6cRb72YytMkY6VHe2Dvv0JfnNBHf3\/fhD\/1G6UVPwjkfeCN5nzRtgQ0EMhQBcw\/ktm3bJt0NmSYRJXn59vUiebdabPPuzOYClr3PnKQnjp7ZnJ7H0CbP6X165kfp0gvPmbCQJSzSyxubWtox77TAJhpxYAOBTOyPGzZsoC1btgR2TU1NNDg4SC0tLYnpsjKAQIYjm1ZnVsL3+th7dN9PnqeDJ97OTPDMSG\/GhQ20+PJpNO0jH54Q5el2LrxKCxuXsoueBthAIKUc9c0HpzrEGgcWiyXvhRwYGKDGxkYprqnY+9Y4qby0RSZhjs6M8n488hrxfF4W2xRUFfWhTT3KUyKnb1K3eK1UTCACEAEXIoE3iCATeaNHkPW+yYMrW3WBNEXvb3\/2Ah1\/7R06MHqKToyR8wWzUUTQBY9tLr\/kI8E5nC+\/+W6qUV4iEWswgKODQLrQB7yBQIYiUOuw6tjYGK1evZr27NkT5N\/V1UU9PT2hZZm2SfZlF0h1DNmRV8fopxlFexM2pTdOoT+48kKa2vDhoH3U3rwyrdqEo4NAQiBdEIBATkIg7qg5W4hZYPlhUUw6w5V\/X7lyJa1du9ZqbrMMAqmiwP\/2D0fo6KtjqZy\/2XT+2XT22WcHR5Bdfsm59CfXfJxefN2fKM+WWy52EEgIJHjjggAEMj3UYnLSBdM0GxkZofXr19OmTZus5jV9E0gWQ97S8OTRN5yEUN20cOlF59Cc5vOo5WNnVm2ac3kQAYiAS2cGb8AbKW9888G5LdKRAsn2SRGp9AD0ojcOC+Ly7T+3FkMlgPM+cT79\/uXTnM\/ehKODo3Ppn+ANeCPlTdF9sPk+hRVINZfZ2tpKfX191NBw5p45\/TH3XCYtBOLGUc\/evXulbZuJ\/fefep1+8E9v0ugb70Xmz0Og8z8+hf7kU1PphTffo0vOO5v439J6jh8\/Ts3NzWllV6p8gE10cwIbYGPT2RcvXjzB7NChQzbJCmFTWIFU6LBQjo6Ohoqk+VucLedXlK+XR587Rf0PHg6NFFVU2H39zNzO50QkgEjAxRuBN+CNlDdF8cG29U5VILO4D5LnGbu7u6m\/vz9xIU6SbT0bh4dPn33pF9T23X+a1DYsiv\/138+i\/zD3Y7btlqodHB0cnQuhwBvwRsqbevpgaV3ZvvACKZlnTFq0U4\/GUVsvltz71IT2YVH86rx\/Rbd8+YNhX5cGTCMNHB0cnQuPwBvwRsqbevhgaR11+1QE0pwLjKpQ3J5GfUiV\/5u3eah9jnxMnbkXUv22aNEiamtri7VVedejcebe8dMJG\/BZGO9pv2J8z2AtjZdWWjg6ODoXLoE34I2UN\/XwwdI6pi6QKsOkVac2FTU3\/+uLdNRvHR0dwQHocbZhZeXVOBw1jrz8Nn31fw6PV4OFcfc3rxpfaWqDRV42cHRwdC5cA2\/AGylv8vLB0npF2acSQaZVmazzyaNxWBw3PHiYtj\/+4vjr9P1hC\/3B715YSHHkSsLRwdG59D3wBryR8iYPHyytU5w9BDJFNFkcea5RnXZT5KhRf204Ojg6l24A3oA3Ut5UTiD1YdXZs2dTZ2cnDQ9\/MLSoA5i0T1EKttQ+y8bxVRwRQcazCCIAEZD6GfSpaMSy9MEu7ZSUBhFkEkIWv4eJ49N\/8RmLlMUwgQhABFyYCN6AN1LeQCCliOVon0XjmMfDfe1Tl9Dd7Zfn+Fa1FwVHB0fnwiLwBryR8iYLHyytg8Q+1QhSDbdWaYj1z3\/wLG3dNxpgznOOPkWOiihwdHB0EqcB3iSjhT4VjlGlBTKKNtJrqZLp52aRReM0rnh4XByLuo0jCS10ZghkEkfCfgdvwBspb7LwwdI6SOxTjSDjCuYTcbZv3x558Lik0q62aTYOD63yIQAqciza5n8JRnB0cHQSviCCTEYLfQoRZDJLNAvJkXGijAXGaQmkOe\/IkePVl00V1KRYpujMEEgXRoI34I2UN2n5YGm5rva5RZBJN224voAkXVqNw4cA3LT950HRHQsups0dV0iqUThbODo4OhdSgjfgjZQ3aflgabmu9qkKZNwiHT5PdXBwMPFGDtcXsUmXRuOYQ6u+zjvqeMHRwdHZ9B\/TBrwBb6S8ScMHS8usxT5VgeSKRB0irg4Vr6WytaZNo3HW\/Z+DdNcPjwVV4ciRI0jfHzg6ODoXDoM34I2UN2n4YGmZtdinLpBhQ6kqsmxvbw9u3qjXU2vjmNGjj1s6wrCHo4Ojc+mT4A14I+VNrT5YWl6t9qkKZNKFyb6vYuVzVh997lSAOYsj73sswwNHB0fnwmPwBryR8gYC2dkZ3N3I11Hpj++rWPXo8epZU2n3TVdJuVFYezg6ODoXcoI34I2UN5UWSHMpD\/KaAAAVnElEQVT+UQePL1UeGhrydh\/kks1P0aMHz0SPvm\/rMEkNRwdHJ3V0bA\/egDdS3lRaIBksjhRXrVo1YcXqyMgILVu2jDZu3DgpspQCXIu9a+OUde5RYQlHB0fn0q\/AG\/BGyhtXHywtJy37VOcgVaVYJJcuXTqhjtu2baurOHJlXBtnw4NHgkuQyxg9IhKI70oQAYiAi7MFb8JRc\/XBLm2QRppMBDKNimWRh0vjlD16hEBCIF37GkQAHw9S7rj4YGkZadqnKpBqDrKjo6Pu0WIYSC6Nw6tWefVqWaNHCCQE0tWhQCAhkFLuuPhgaRlp2qcqkHHbPNKstGteLo2jFufwlo4ynJoThh0cHRydS58Cb8AbKW9cfLC0jDTtUxVIrlgRVqtGASRtnDJv7dAxgqODo3NxKuANeCPljdQHS\/NP2z5VgSzbhcn68GqZDgYwSQRHB0fn4ljAG\/BGyptKC6QUrLztpY2jD6+W5Vg5DLHKWAcRgAjIGHPGGrwJR03qg12wTzNNqhFkmhXLIi9J4+jDq59ruYD+95\/NzaJKhcgTnRki4EJE8Aa8kfJG4oOleWdhX7NA6gtzZs+eTZ2dnTQ8PBxa1zlz5tDAwAA1NjZm8S6JeUoapyrDq\/jajacNRAAikOhYQgzAG0SQLrypaxqJQFZleBUCCYF07ZQQAXw8SLkj8cHSvLOwrzmCNCtVhvsgq7J6VbUdHB0cnYtzAW\/AGylvKi+QZbgPcvvjL9JN238etH3ZDiYPIzQcHRyd1NFh5AEjDy6cqbRAluU+SP3mjpN3ft6FB16lgUBCIF0IC96AN1LeQCA9vw+yasOriAQQCUidHIbmkxHDx0M4RpUWyDLcB6kLZM\/1M6nn+hnJvcFzC3RmRAIuFAZvwBspbyotkAyW7\/dB6ldblfn0HJ3YcHRwdFJHh5EHjDy4cKbyAqlE0tf7IKu0vQNDZcldHB8P+HhIZslkC\/AGQ6wuvKlrmqSvlyrOPyISQCTg2ikhAvh4kHInyQdL88vaPvV9kFlXuJb8kxpHPz2nKvOPEEgIpGufgkBCIKXcSfLB0vyytodAaghXcf4RAgmBdHUyEEgIpJQ7EEgpYjnaJzVOFecfIZAQSNcuCIGEQEq5k+SDpfllbY8I8tcIV3X+EQIJgXR1MhBICKSUOxBIKWI52sc1jj7\/+MA3r6JrLpuaY83qWxQcHRydCwPBG\/BGypvKC+TIyAgtW7aMRkdHJ2FX5OuudIGswvmreuPA0cHRSR0dRh4w8uDCmUoLpDpJp6mpiXp6elzwyzRNXONU7fxVCKQd1fDxgI8HO6ZMtAJvwlGrtEDGHVbuQrKkNEqQ9+zZE5h2dXXFCrONQE5vnEJ8gk6VHnRmiIAL38Eb8EbKm0oLpBKsjo4OWrhwoRQ7sT1frcUPR6tKnNvb26mtrS00r7jGaVzxcJDm6llTafdNV4nr4nMCODo4Ohf+gjfgjZQ3lRZIBovPYmXhGhgYoMbGRil+NdnrghmWUVTjVHn+kXGCo4Ojc+l44A14I+VNpQVSRXHDw8OhuGW5SMdmeDeqcap2QbLZOHB0cHRSR4cPq3jE0KfC8am0QLp0sjTScOS4ZcsWam1tpb6+PmpoaBANsVZ5gQ4cHRydax+ECODDSsodCKQUsRTtWSh5e0mUSHLjqGfv3r3j\/9269TiNvvEeNZ1\/Nu35enOKNfIjq+PHj1Nzc\/Xe26Z1gE00SsAG2Nj0ocWLF08wO3TokE2yQthkcpLOzp07ac2aNcELbtu2jY4ePUpDQ0Ox0V0aaPAezO7uburv76eWlpZJWYZ9vVT5BB0FECIBRAIu\/Q+8AW+kvKl8BKmiOBaq5cuXBytMee5x9erVlPX+yKQFQmGNU9UbPHRiw9HB0UkdHYbmMTTvwplKC6S+UGb27NnU2dkZCCRv+UgSLxew9VWrNocUhDWOfoNH1U7QQQSZzDp8PODjIZklky3Am3DUIJC\/FsU8BNI8KMBlkc7K\/\/UsDQ6dORaPDwjggwKq9qAzQwRcOA\/egDdS3lRaIBksnn\/k+UZ9iFWJZdwmfinQLvZhjVPVK64wxGrHIIgARMCOKROtwBtEkJG84eHUpUuXTvi9t7c38oQbFwK6pAkTyCqfoIMh1mQWwdFBIJNZgiFWW4wqH0HaAlUPO7Nx9BWsPdfPpJ7rZ9SjWnUvEyIAEXAhIXgD3kh5A4GUIpajfZxAbu64gjoWXJxjbYpTFBwdHJ0LG8Eb8EbKGwjkr+ch1T5IBSDvh8zjAPO4BjMbBytYz6AFRwdHJ3V04E08YuhT4fhUXiB5kc6OHTsmHFZuc9OGSweVpjEb5+adz9DfPPZCkE1VV7DC0cHRSfuRsocI4MNKyp1KC2TcgeFZ7IOstXGwghURZBKHIAIQgSSOhP0O3iCCnISAbwKJFawQyCTnB0cHgUziCATSHqFKR5AME0eKq1atosHBwfHzUIs4xKqvYP3jz36cNv7H2fatXDJLiABEwIXS4A14I+VNpQUy6T5IHUw+n3XXrl1SfGuy1xsHZ7B+ACUcHRydS8cCb8AbKW8qLZBSsPK2jxLIqp7BqvCHo4Ojc+mL4A14I+UNBFKKWI72euNgiwciSBvqQQQgAjY8MW3Am3DUIJAR+yCLdtQcVrBCIG0cHxwdBNKGJxBIO5QqL5C+7IOEQEIgbbo0BBICacMTCKQdSpUWSJ+2eWCLBwTSpktDICGQNjyBQNqhBIHULknWISvSQQH6Fo\/\/0XY5\/edPX2LXuiW1gghABFyoDd6AN1LeVFogGSwfhlixxWMireHo4Oikjo7twRvwRsqbygukEknzsPIiLdLRBbLqWzzg6OK7OEQAIiAVAfSpaMQgkC5syimNahxs8UAEaUs5CCQE0pYruh14E44aBNKFTTmlUY2jVrBysSfv\/HxOpRe3GHRmiIALO8Eb8EbKGwikFLEc7U2BnN44JbjmquoPHB0cnUsfAG\/AGylvIJBSxHK0V40z946fEq9kvXrWVNp901U51qCYRcHRwdG5MBO8AW+kvIFAShHL0Z4b50dP\/DOxQPIDgTwDPhwdHJ1LNwRvwBspbyCQUsRytDcFsuf6mdRz\/Ywca1DMouDo4OhcmAnegDdS3kAgpYjlaM+N871\/2E9L7n0qKHVzxxXUseDiHGtQzKLg6ODoXJgJ3oA3Ut5AIKWI5WjPjdP1Vz+kDQ8eDkrFHkgMsSbRDyIAEUjiSNjv4E04ahBIFzbllMYUSF7ByitZq\/6gM0MEXPoAeAPeSHkDgZQilqM9N86VK\/+eHj14KigVeyARQSbRDyIAEUjiCCJIe4QgkPZY5W6pCyT2QH4AP0QAIuDSGcEb8EbKGwikFLEc7blxzv\/jv8UeSANzODo4OpduCN6AN1LeQCCliOVoP+PKT9EbX9wQlIg9kIggbagHEYAI2PDEtAFvwlGDQLqwKac0ukBiDyQE0oZ2cHQQSBueQCDtUIJA2uFUF6vpn\/oSvXV1d1A2tnhAIG1ICIGEQNrwBAJphxIE0g6nulhBIMNhhwhABFw6JHgD3kh5A4GUIpajfdOX\/5zeuXwJIkgDczg6ODqXbgjegDdS3kAgpYjlaH\/JkjX0y8u+GJSIPZAYYrWhHkQAImDDEwyx2qEEgbTDqS5WH\/ujv6b3LvwdCCQiSGv+QSAhkNZk0QzBm3DUIJAubMopjRJIHBIwEXB0ZoiASxcEb8AbKW8gkFLEcrS\/8E\/\/jt4\/50LsgUQEac06iABEwJosiCAToYJAJkJUP4PGFQ8HheOQAESQtiyEQEIgbbmi24E3GGJ14U1d0yiBxCEBEEhbIsLRQSBtuQKBTEYKEWQyRnWzgECGQw8RgAi4dErwBryR8gYCKUUsR3slkJs7rqCOBRfnWHKxi4Kjg6NzYSh4A95IeQOBlCJm2J88eZI6OztpeHg4+KW1tZX6+vqooaFhUs5jY2O0evVq2rNnz\/hvXV1d1NPTE1oLJZA4Zg5DrLY0hQhABGy5giHWZKQgkMkYRVoowVu0aBG1tbWR+rupqSlU9FhMV65cSWvXrqWWlpbEkiGQGGJNJIlhAIGEQEo5w\/bgTThqEEgXNsWk2blzJw0NDYVGkSMjI7R+\/XratGkTNTY2JpasBBKn6CCCTCTLrw3g6CCQtlxBBJmMFAQyGSORRZxA7tu3jzZs2EADAwPWAolDAibDDxGACIg6JT4eEuFCn0IEmUiSWg3UfGR7e3sw5Go+LJ5r1qwZ\/+c5c+bEiiVHkBBICKSEl3B0+HiQ8EXZgjcQSBfeWKdR84+cIGqRDkePo6Oj47+bf5uFsUCe\/cqzdO6j\/bR3717rupTd8Pjx49Tc3Fz213R6P2ATDRuwATY2nWrx4sUTzA4dOmSTrBA2Z50+ffp0IWqiVcJGHMPqzHOS3d3d1N\/fH7pohwUSp+gggpTwHZEAIkgJXxBBxqOFOUgXNoWIY9TK1bjskxbtsEDiFJ3JCPpG2hopJkoObKLhAjbARtSZiMg3zhQugkwaJlUNIt0SwukgkOF09o200k5Ziz2wgQi48Ae8KYevKZRAmocEKIjV4hs+LIAPBujo6KCFCxeO75NUBwXEHSqgBPKc\/ffRbx77iQvnkQYIAAEgAARqRABzkDUCiORAAAgAASAABOqNQKEiyHqDgfKBABAAAkAACCgEIJDgAhAAAkAACACBEAQgkKAFEAACQAAIAAEIJDgABIAAEAACQMAOAUSQdjjBCggAASAABCqGAASyYg2O1wUCQAAIAAE7BCohkHz4wJYtWwJEtm3bFuyhrNrDpwwtW7YsOLs2ab+ojhefaDQ4OGh136avmEqwUe9oHlTh67vH1VuCi7mHuez9TIKNbluF\/hTHKdVv1F72oveb0gukfiXWgQMHRNdjFb3xbOunO\/MlS5YEhy2oS6nNPMzrxfjvHTt2WF8pZlunothJsNHrrG6S6e3tDb1ppijv51oPCS7mxeZJZyK71qko6STYqA+Hnp6e4MO87P3JRhz5YBdfPqBKL5AcDfHDBPXt6yUth2A6LP5o2L59e+QtKXq5ZXd2Ltiw01u5ciWdOnWKoq5iS6vt6pWPBJekM5Dr9Q5ZlSvFRr9Aoez9KQpzFUXPnz+fjh07FvhjH0bySi2QUee1RkVPWXWoeudrXiwtuWi67B3aBRv+6FqwYAE98MADkZF4vdu81vIluMRdal5rPYqYXoJNWAQ5NDRk9XFaxHd3rdPzzz8fJOXjQjs7OyGQrkCmmS4sYmTnNmPGjFIOi0VhZ0aMki9+28Pj02y3PPOSYsPYbd26lVasWEHr1q0rtUDqowxxnGGBPHLkSNBsVZjrl3JG+SEeWuzq6grEoaqP+cFQdBwqEUHqE8IQyAayFUh2fHfffXepF+lInB07uu985zv09a9\/PbhgOm4ut+gdP6l+ElzUfKyaV+K0q1atKi1vJNioocWNGzcGQ4qS0ZukNvLxdwhkgVoNQ6xnGkMyJKSarwriKMWGcXzkkUcmzGeXdbhewhlziLXsK3yBjbuTh0C6Y5dJSj1irPIinfXr19OmTZuosbExEMy4RTpVWmlnRtNx2OjbX3SylnHYTIKLiVnZ+5kEm6p9PCQ5cQhkEkI5\/45tHjS+epejnaRtHmUfHjPpJ1myr6cte5QkwcV0emUfRpRgEzbEWubh5yT3DoFMQqgOv+OgAArmHaMOClCLLHjxQFSU5Mu+JRd62WJTJYHkd5Xgoh8UUIXN8BJs+INh6dKlAX2qgE1cH4RAungopAECQAAIAAEgUDAESr2KtWBYozpAAAgAASDgEQIQSI8aC1UFAkAACACB\/BCAQOaHNUoCAkAACAABjxCAQHrUWKgqEAACQAAI5IcABDI\/rFESEAACQAAIeIQABNKjxkJVgQAQAAJAID8EIJD5YY2SMkTg4MGDdMEFFwQnBdk8vB\/rtddeo1mzZtmYO9moPaVz5swR3afpyw0q6v3y2tuXd3lOjY5EpUIAAlmq5qzmy0hPbsljs3ItIldL2jwZoN+1mle5vmCTFx4oJ1sEIJDZ4ovcc0CgiAIprZMOky8iAIHMgdwooq4IQCDrCj8Kt0VAP8qM06hhywMHDowf48X\/ro7EM4\/MU8OA06ZNCy5sHR4eDopWB43rd\/bp+ccN2epl6MOM6von9W69vb2h949G2SmB5HNzb7\/99iAbcxhTP77MLIexWrlyJX3uc58L0iusXn311fHjBjnNrbfeSrt376b+\/n5qaWkJsol6p7B2MgWS\/37zzTeD\/\/Hdhzq+YenDLlpOunzZl48HW17DrtgIQCCL3T6oHU08bL2trW3ckfN\/8PmxZrQWdYMC2\/f19QWHt+u3mivxbW9vHxeyuBtNlJiq\/PiWdPN6sKQI0rTXD7VmEedzc+fPnz9+87xeHxa67u7uCcKm56c+AqZPnz6e3nxH9feJEyeCexvV\/ZYsxOpC36SD68MEki9MVh8EYbjqhIZAonsXHQEIZNFbCPWjJEebJEYMoe6MTYEMu+Iq7raOsCjGtI+rU9JNIOYNEFz\/pMhJ\/10JpCn4Q0ND44LJeeoCyH\/rV6Ip2sUNo4YJ5Ojo6KQy2G5gYGDSAioIJDp30RGAQBa9hVC\/AAF9ONJcFRolRuYwZGtra2gEaQ516pCHDY9GlaffihInkEmLhMLEMOzfzGFncxiZI0G+xV4NnaqIW72fnidHperGCZNyUfddhglkXBlqGFflD4FE5y46AhDIorcQ6jcBAV0U9HlIPUpRgmfOC6oIyowgOa0Z+cTBHiV+umBkKZBcN3V1mRLwsAhSIpD79++nHTt2iLajQCDROcuOAASy7C1c0vfTozAVIfEwHs8Hrl69mvhyaDVfqX5XImgKZNx8Yxh8eQyxmnOMepksZnHDpWqIVRfIsGhNH2LlCFJ6kW8WQ6xJHytJQ80lpTteq04IQCDrBDyKtUcgbA5Sj9D0RSthi01URKmGWLlkXURV\/jwcqRaohM0DqhqntUhHj9j0ecl58+ZNWoRjCqSeVtWV68cLbsIE0naRDufBC5n4QyNp7rfWRTpqCFytPFbvoS9OMlkCgbTvN7CsHQEIZO0YIoccENBvcOfi9OFTfYsGDzled911E7ZysDDecMMNdNtttwUCwnNhpmiqqFJt\/+AylOOOer24LRG2C4fWrFkznn3YcKmatzOFwSx748aNwRYNXpij3l+PILkQE0Nzm4e51YXTRG1RUVE5\/7\/6qAjb5qGnDxM3ff6X22nu3Ln09NNPByJtfsiodzCj6xzohyIqigAEsqINj9cGAixYYStXbZGxmYO0zcvWDhGkLVKwSwMBCGQaKCIPIFBwBMx5VhUt6vsepa8AgZQiBnvfEIBA+tZiqC8QcETAPF0oavuGbfbm4eH3339\/kFQNudrmY2uHw8ptkYJdWgj8f00c25iMjpp3AAAAAElFTkSuQmCC","height":275,"width":456}}
%---
%[output:80039208]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:965bb623]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:551aa7f8]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
