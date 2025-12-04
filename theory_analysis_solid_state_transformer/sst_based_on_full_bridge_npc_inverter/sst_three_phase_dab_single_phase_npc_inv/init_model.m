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
application400 = 1;
application690 = 0;
application480 = 0;

n_modules = 2;
%[text] ## Settings and initialization
fPWM = 2.5e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM*10; % PWM frequency 
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
ubattery = 750;
margin_factor = 1.25;
Vdab1_dc_nom = ubattery;
Idab1_dc_nom = Pnom/Vdab1_dc_nom;
Vdab2_dc_nom = 750;
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

% danfoss_SKM1400MLI12BM7; % 3L-NPC Si-IGBT
% inv.Vth = Vth;                                  % [V]
% inv.Vce_sat = Vce_sat;                          % [V]
% inv.Rce_on = Rce_on;                            % [Ohm]
% inv.Vdon_diode = Vdon_diode;                    % [V]
% inv.Rdon_diode = Rdon_diode;                    % [Ohm]
% inv.Eon = Eon;                                  % [J] @ Tj = 125°C
% inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
% inv.Erec = Erec;                                % [J] @ Tj = 125°C
% inv.Voff_sw_losses = Voff_sw_losses;            % [V]
% inv.Ion_sw_losses = Ion_sw_losses;              % [A]
% inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
% inv.Rtim = Rtim;                                % [K/W]
% inv.Rth_switch_JC = Rth_switch_JC;              % [K/W]
% inv.Rth_switch_CH = Rth_switch_CH;              % [K/W]
% inv.Rth_switch_JH = Rth_switch_JH;              % [K/W]
% inv.Lstray_module = Lstray_module;              % [H]
% inv.Irr = Irr;                                  % [A]
% inv.Csnubber = Csnubber;                        % [F]
% inv.Rsnubber = Rsnubber;                        % [Ohm]
% inv.Csnubber_zvs = 4.5e-9;                      % [F]
% inv.Rsnubber_zvs = 5e-3;                        % [Ohm]

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

infineon_FF450R12KT4; % Si-IGBT
igbt.dab.Vth = Vth;                                  % [V]
igbt.dab.Vce_sat = Vce_sat;                          % [V]
igbt.dab.Rce_on = Rce_on;                            % [Ohm]
igbt.dab.Vdon_diode = Vdon_diode;                    % [V]
igbt.dab.Rdon_diode = Rdon_diode;                    % [Ohm]
igbt.dab.Eon = Eon;                                  % [J] @ Tj = 125°C
igbt.dab.Eoff = Eoff;                                % [J] @ Tj = 125°C
igbt.dab.Erec = Erec;                                % [J] @ Tj = 125°C
igbt.dab.Voff_sw_losses = Voff_sw_losses;            % [V]
igbt.dab.Ion_sw_losses = Ion_sw_losses;              % [A]
igbt.dab.JunctionTermalMass = JunctionTermalMass;    % [J/K]
igbt.dab.Rtim = Rtim;                                % [K/W]
igbt.dab.Rth_switch_JC = Rth_switch_JC;              % [K/W]
igbt.dab.Rth_switch_CH = Rth_switch_CH;              % [K/W]
igbt.dab.Rth_switch_JH = Rth_switch_JH;              % [K/W]
igbt.dab.Lstray_module = Lstray_module;              % [H]
igbt.dab.Irr = Irr;                                  % [A]
igbt.dab.Csnubber = Csnubber;                        % [F]
igbt.dab.Rsnubber = Rsnubber;                        % [Ohm]
igbt.dab.Csnubber_zvs = 4.5e-9;                      % [F]
igbt.dab.Rsnubber_zvs = 5e-3;                        % [Ohm]
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
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"     4.583333333333334e+02"}}
%---
%[output:174c5e41]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"     9.375000000000000e+02"}}
%---
%[output:6e4cc00d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     2.727272727272727e-05"}}
%---
%[output:01f62bda]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     3.715110066885718e-05"}}
%---
%[output:1bc16b1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.265986323710904e+02"}}
%---
%[output:9a535e97]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:26989b55]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"2","name":"Ldrso_pll","rows":2,"type":"double","value":[["0.005131660589376"],["1.166353750716398"]]}}
%---
%[output:75c58cb8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:6a06269e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:0c76ff32]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000400000000000"],["-39.478417604357439","0.993716814692820"]]}}
%---
%[output:1ba8c347]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"2","name":"Ld_fht","rows":2,"type":"double","value":[["0.006220353454108"],["1.086643444559938"]]}}
%---
%[output:16217f48]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.142908164917381"],["7.327264123984933"]]}}
%---
%[output:28c3084b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAgcAAAE4CAYAAADPZVDeAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10HtV55x\/atEUkECPsxAiF2IBcyNLY2CVVFAeXVYi36cp007SSnD3JepWse4KTcwJeSS4JZwkES6rhnCwfW5UVOt7T+qPNQmOd3S6hCqVQxYEYrG03JBbIwhGCABaGJBXbpdGeZ8xVrkbz\/Z+Zd955\/3NOTrDeuc\/c+T137vOf537MGfPz8\/PCgwRIgARIgARIgATeInAGxQHbAgmQAAmQAAmQgE2A4oDtgQRIgARIgARIYBEBigM2CBIgARIgARIgAYoDtgESIAESIAESIAF\/AswcsHWQAAmQAAmQAAkwc8A2QAKzs7PS1dXlgBgaGpL6+vrMoExMTMi2bdtkw4YN0tfXJ3V1dZldyzac5z1GuSHD4fOf\/7y0t7dHKVLoc\/r7+2VwcNCp4\/bt26WnpydWfQ8ePCi7du2S3bt3F5KH3t\/hw4czfz5iQePJuRFg5iA31LxQkQjkGTj9xIF2vps2bZLm5uZM0PjdY9bX9buZJMFGg9MjjzwSK\/AmKRPXAXqNrVu3LhQrozgom5iL6+NaP5\/ioNZbAO8\/dwJzc3PS29srIyMjsm\/fvtzEgWYs8riuF1ATaNra2iIHevNmHSfwJimTpAEYcRCnbu7rFD1zYNrpiRMnmD1I0kiqvAzFQZU7sAjVt9OrWh87TWq\/Na9bt05uueUWp8oaJOwUu3nLHR8fX\/K7bePaa6+Vz3zmM845DQ0NMjw8LE1NTZ4Y7CDsPt\/9Vq2\/m2GG1tZWueOOO2Tt2rVOp2gH1TA7OjxhrnvkyBGnfnqYYYWbbrpJvvKVrzjCwBxuFvp3w9QWDyYg2eebAGNs2cHKvse77rpLBgYGPK87PT3t1G9mZmahTrYPbY7K\/O6775b77rtPzP0pfzdrw84M13gFQuNX+7rmft33ZXzd2Ni4IHDc\/A4dOuSk6c1htw+3vTBR5m6PQbaC2mFQhsFmonU2dXcLDvfzZbM1Nq6\/\/noZHR0VfX6M7+x71r+Za9i+DePi1Q6L0OewDtkToDjInnFpr+AOCPaNmg7OKwC4O3W1o4HZCAP3717BKyiw6m9+dTMd+XnnnbdozoERB3YdNAh7BXNbILjtpCUOvN5M3R21O2j4cdW\/+4mD7u5u2bFjxxL2QcFYf1uxYoW8\/PLLjvjxCth6TTuIuevu1y7MdZ988knPQH\/\/\/fcvjPPb7c0Ofm5x4LZlfvcTCEFtVss899xzviLErpNbGLgFnAnMH\/7wh+XRRx9d1E94BXiv58sd3PUcrzrq3811wmzbXIqe3Sht51qAG6M4KIATqrUKpvOz35xMx6r3ZL8169uh6XTsztfuyOw3JjuYaAA2b7bGhrm2+w3VsPT63e7orrnmGl9xYL9ZxbUTJg40W6JHWHo\/KLOh2YyTJ08uYWK\/7SqnNWvWLLrHKMMK7iEPw974U7MEbr9rXXT83SujoSy3bNni3K+daYgySTPKEIH7HPe\/DRMjZLT+Ydc2bc\/rfszfVETqPfsNK9gcTXty+\/Shhx5yRIZXJsDPrjt7ZLIlto2ga5vMgmn\/YVzSGD6p1v6t1utNcVDrLQC4f7+3CtO5aqe4fv16z5n69jlTU1Oeb4NaNduGvq2alQVhEwrD3nj8gq\/dWer149pJSxzY19ZAr4cdjPw6bTs4fvazn40sDrwyLV7X1Xq4h0383sz1XA1yph42W6\/ruYdXgsSB33CKu0xQFsBLWLrvzQxZuUWGEUR+QTysfdr+tW34+dWdhTCsjDjwG06yV+LYbdk8l\/aQjukKbC5eQ1lAl8GiVUSA4qCKnFW0quYtDuylgGGdb9ygrmy9ljZGtWMHPncgUdv2UsYomQM9x57Ep\/\/WZXPuzIk7OMUVB26O7uyCW5SkJQ5MW\/YTJbqCw0scuIcnwjIH1SAOvDJVxq\/u9ueXObBt+D0bFAdF60GLXR+Kg2L7p9C1Szqs4E5\/mzFcv7cwrzRwmDgIGg6w32YVsL5d+YmDqHY0XesO3Ga4xS0ONABHmejlDpz2m7V7aEaDadiwgmY1woKr20bSYQW74fq9jbsbtzvQu9+izT3bGSRzP6btuMt4DSuEPVRZDysYIWkyLn7iwCvjYhi5Mwd+E0jdQxpBwwpeXDisENZayvs7xUF5fZv5nWU9ITEouIaJg6C6eY3H+4mDMDuagjXzB9zAo4gDLeO1WsHYcs84tzcPijMh0aSX7TJ63Y9\/\/ONOVsPrUE5e9xd1QqLaNILJLUr8JuvZZexz9Jpf+9rX5NZbb10yeVLLuMWB\/s1vcqO51zAx6pVyD8vc2Bz97jEosNvB+Atf+IJv2wqyoXXwmqgYdUKizSUsc5Z5J8MLVIwAxUHF0JfnwlGXMtrLEMOWMnpNcowzrKB0g1LWYRP+7B0Tg+zoddzL3r785S\/L0aNHFybgeWUO7MDhN6nStu2eC+ElHuwgaZfV\/zbiwOu6995776Kd\/nRjJnt+Q5KljHaQt4OV1zJXvyWUbq72HAi1qdz27NkjO3fudHDYGSCz6sRvaWTY\/gRBSxn1WlHfqP3mCmj2yCvw+mVLlJHXMlKv7IOfsNS\/u3dk9Ju7YWxEyXCVpyfjndgEKA7YHjIlEDYzPNOL0zhMwGv4wr0ixW+fCfviSTZBgitfowZsMWdEkNcKhjA83AQpjFC5f69qcaAdjq7T1s1dvDqooI11yu3W4twdxUFxfJGkJkHDKkHDIV7XSrJ9cpI6s4z3sIJyCds4zEvQleVbGGwX8QhUrTgIm+Rkfm9paXE+amL+rQ9H3A+kxEPKs706mLw\/OkQvpEfALbLVclxhoGW4V396PoliyT3cF0cYqH2KuSiUy3tO1YoDHZvTxquHX+bA7TYdXxsbG8v1y3jlbTq8MxIgARIggbISqEpxoG8yN998s3R2djoCgeKgrM2T90UCJEACJFAJAlUpDjQDoIfu8hU058AGalKjHR0dhfx2eiWcz2uSAAmQAAmQgBeBqhMHOm65d+9eufHGG0U\/yBNFHJj5BgrA\/hKgG8hFF13EVkICJEACJEACiQno1zFXr16duHxRCladONBhBF2HrTvCha1WUMhRhYGeq+JgcnKyKL6punqQH+4yMsQYkh\/Gj\/0g+RkCVSUOvGZNmxvx+vxq3BUK7FiwB4P8MH7smMkPJ4Bb4HOMMSwLv6oSB26XhWUONMugO4sFDSXYNsviVKxpJy9NfsnZmZJkiDEkP4wfBSr5VWXmIEwcmEyBrmIw37I326iaskFb1bJjwR4M8sP4sWMmP5wAboHPMcawLPyqOnOAuXBp6bI4NW0uUe0dP368FBNxot5vFueRIUaV\/DB+WpoMMYZliSMUB1Y7KItTsaadvDQ7leTsTEkyxBiSH8aP4gDnV5Y4QnFAcYA\/DW9ZYMeMoyRDjCH5YfwoDnB+FAc4w8JZKItTKwWWHTNOngwxhuSH8aM4wPmVJY4wc8DMAf40MHNAhqkRwAxRHGD8KA5wfhQHOMPCWSiLUysFlh0zTp4MMYbkh\/GjOMD5lSWOMHPAzAH+NDBzQIapEcAMURxg\/CgOcH4UBzjDwlkoi1MrBZYdM06eDDGG5IfxozjA+ZUljjBzwMwB\/jQwc0CGqRHADFEcYPwoDnB+FAc4w8JZKItTKwWWHTNOngwxhuSH8aM4wPmVJY4wc8DMAf40MHNAhqkRwAxRHGD8KA4wfvufeFE+P\/SovPLHv4cZKkBpigOKg9SaITtmHCUZYgzJD+NHcYDxU3Fw3f6nZfaOqzFDBShNcUBxkFozZMeMoyRDjCH5YfwoDjB+\/Q9OSf+DxykOMIzFK12WsaJKkWXHjJMnQ4wh+WH8KA4wfhQHGL\/ClqY4wFzDjhnjx46Z\/HACuAU+x8kZUhwkZ1fokhQHmHvYqWD8KA7IDyeAW+BznJyhzjfQeQecc5CcYSFLUhxgbmGngvGjOCA\/nABugc9xcoYUB8nZFbokxQHmHnYqGD+KA\/LDCeAW+BwnZ7jl7qfksWdPMXOQHGH+JScmJqS7u1sGBgakqanJswIUB5hf2Klg\/CgOyA8ngFvgc5ycIcVBcnYVKTk3Nye9vb1y5MgRGR4epjjIyAvsVHCwZIgxJD+MHwUqxo\/iAOOXe+nDhw9Lf3+\/c11mDrLDz44ZZ0uGGEPyw\/hRHGD81t36bTkx+waHFTCM+ZSenZ2Vm2++WTo7Ox2BQHGQHXd2zDhbMsQYkh\/Gj+IgOT8VBSoO9OBqheQccyt58OBB51rr16+PNOfArtjo6Ghu9SzDhaanp6WxsbEMt1KxeyBDDD35Yfy0NBnGY9ja2uoU+NlZy+X1j57OUFMcxGOY+9k6CXHv3r1y4403Og2eExKzdQHf2nC+ZIgxJD+MHzMHyfk99swp2XLPUxQHyRHmV1KHETZt2iTNzc3C1QrZc2fHjDMmQ4wh+WH8KA6S8zMfXWLmIDnDXErqXIOuri4ZHx9fcr19+\/Y5gsF9cCkj5hp2zBg\/dszkhxPALfA5TsbQrFSgOEjGr2KlmDnIHj07FZwxGWIMyQ\/jR4GanJ9ZqfAL\/\/iKvPLHv5fcUEFK1swnmykOsm9x7JhxxmSIMSQ\/jB\/FQTJ+9kqFXzn+sLzwwFeSGSpQqZoRB1GYc1ghCiX\/c9gxY\/zYMZMfTgC3wOc4PkPzNUYt+Y7HBuTE438V30jBSlAcWA6hOMBaJzsVjB\/FAfnhBHALfI7jMzTzDS6sP1Nev++TMjk5Gd9IwUpQHFAcpNYk2angKMkQY0h+GD8K1Pj87CWMGy9eJv9w++9SHMTHWOwSzBxg\/mHHjPFjx0x+OAHcAp\/jeAztVQqHPneFfOqj6ykO4iEs\/tkUB5iP2Klg\/CgOyA8ngFvgcxydoT0RUYcUjn7pg1KWOMJhBQ4rRH8SQs5kp4KjJEOMIflh\/ChQo\/NTYbBj\/9Py2LOnnEKaNdh4yTKKg+gIq+fMsii+ShFnx4yTJ0OMIflh\/CgOovP7s8dfkM8f+L5ToPPKlXJ352XOf5cljjBzwMxB9KeBmYPUWPkZYnDDEJMfxo\/iIBo\/93CCZg10WIHiIBq\/qjurLIqvUuDZMePkyRBjSH4YP4qDcH72NxRUENzVcZkznGCOssQRZg6YOQh\/GiKewY45IqiA08gQY0h+GD+Kg2B+tjDQM3UoQYcU7IPiAG+DhbNQFqdWCiw7Zpw8GWIMyQ\/jR3HgzU+HEZ6Yek0++6ffWzjBTEB0lyhLHGHmgJkDvDd5ywI7ZhwlGWIMyQ\/jR3GwlJ8Kgy33PCX6\/3roUELP5tVLMgamJMUB3gYLZ6EsTq0UWHbMOHkyxBiSH8aP4uDn\/FQM\/N2zp+S6\/U8v\/FGFgT350It2WeIIMwfMHOC9CTMHZJgaAcwQxQHGj+LgND\/dEnnHgacXsgX6N90a+a7OyxZWJfiRpjjA22DhLJTFqZUCy44ZJ0+GGEPyw\/jVsjjQTIH+b+DB4wsbG5lhBPeKhCDKZYkjzBwwc4D3JswckGFqBDBDFAcYv1oVB+55BUYUfPEj75Wr19SHZgts6hQHeBssnIWyOLVSYNkx4+TJEGNIfhi\/WhMH+x5\/QXa8tcuhIafzCswQQhKaZYkjzBwwc5Ck\/XuWYceMoyRDjCH5YfzKLg7MigP7mwi2KPjkB86X\/\/jRVRBEigMIX\/LCc3Nz0tvbKyMjI46R7du3S09Pj6\/BgwcPyq5du5zf29rapK+vT+rq6jzPL4tTk9PFSrJjxviVvWPG6YRbYBsMZxR2RhkZ6gRD91wCWxToCgQ9zBbIYYyCfi9LHMk8czA7OytdXV0yPj4ei\/fatWvlgQceWFKmv7\/f+ZsKAmO7o6ND2tvbl5x7+PBh0fOHhoYcQaCioqGhwVdMlMWpsUCneHIZO5UU8UQyRYaRMPmeRH4Yv7II1KAMgREBH7p4mbNfQRqCwKZeljiSmzjQYN7c3Byp5Zqg7iUO3AZsseD+TbMGY2NjC9kC97\/d55fFqZEgZ3ASO2YcKhliDMkP41et4kDFgLMHwfhL8l8fe37RagObiNe3EHBiiy2UJY5UtTgwmQM\/4eGVOWhpafHMMqh7y+LUtBt7VHvsmKOS8j+PDDGG5IfxqxZxYMRA0HCByRBceO6Z0v1WhiDtLIEX7bLEkczFAd5UvS1oxmBwcDB0HsHExIRs27ZNZmZmZN++fYHZC3WqfYyOjmZV\/VLanZ6elsbGxlLeW143RYYYafLD+GnpIjKcef1NaTjnbXLk+Tdk8DunnP\/3O\/S8qy86SzZddJZsuOD0Z5SzPFpbW5eYn5yczPKSudjOXBzYcw7CJg8muWMVCRr4vSYa6jDCgQMHnDkH9fX1zvwDPfwmMJZF8SXhmEYZvrXhFMkQY0h+GD8tXQSGJjPwtxOvyp5vTvkOE2h9NRug2QHdvdD8G6eQ3EJZ4kjm4sAgNm\/6+m+dFDg8PCxNTU3JPfBWSc0MdHd3y8DAwCJ7ZlWDPYzgd66pRFmcCkNNaKAInUrCqhemGBliriA\/jF+lxIGZQHjXwyfk+y\/+NFQMaD0HPr5GLl359tQnFKIEyxJHchMHBrh79QKaTbDnFWh2wBwUB2gTj1+eHXN8Zu4SZIgxJD+MX17iQMXA2OQp2fedFwKFgJ0ZuOGaVbJ6eV3hxICbOMUB3gbFng8QNZtgDw0YAeC3PNFrWMFvCEJvpyxOTcE1iUywY06EbVEhMsQYkh\/GL21xYDICX3\/yR\/I3P5gNFQJGDOgyw84rzz89ZFCf\/bwBnNrPLZQljuSeOfBzgl8GwH2+exMke2Mj81tnZ+fCxEN7OIObIKX5CCy1xY4Z50uGGEPyw\/gh4sB8uGj\/Ey\/ID2ffiCwEzHwBLb\/xkmX4DVTYAsVBCg6wMwdqLmw1QQqXDDRRFqdmzcnPPjtmnDwZYgzJD+MXRRyYbMBfPPkjeSRiNsBkBFQI\/M4V75KPXHqeU9FqywpEoVuWOJJ75iDozT8K+CzPKYtTs2QUZJsdM06eDDGG5Ifxs8WBEQEPPX1SvnH0pUiZAHN1DfpmaED\/VoaMQFSyZYkjuYkDO71fhCyBl6PL4tSojTjt89gx40TJEGNIfvH5GRHwp4+\/IIefPRVbBGg24GO\/tlw+dvmK0mYD4lAtSxzJXBzYqxPCxvzjOCCLc8vi1CzYRLHJjjkKpeBzyBBjSH7+\/IwI+KNvTslzJ+diiwC1\/OFLzl301cIyDgtgLbA8E9szFwfPPPOMfOELX5Cbbropk28roI60y1McYDTZMWP87JQubqk2LdR6G9TthPX4m2Oz8vjx12IJAC2nwX5FncgHLl4hv3X58qpcLVDpll+WOJK5OAj7\/oGXI+N8eCnNhlAWp6bJJI6tWu+Y47DyO5cMMYq1wM+sCnh86jVneeCJV98QkxWISs\/sKvj7v75SVp13eu8AkwWoBYZROSU5ryxxJDdxkNYnm5M4K2qZsjg16v2mfR47FZwoGWIMy8JPMwAarO99dFrGp3+cWAAozd++fIVsv6rRERBR9g0oC0OsJSUvXZY4krk4SI44\/5JlcWr+5E5fkZ0KTp4MMYbVws+86U++8o\/y9SdfkhMx5wAYSuZtX1cGNK9eJpvWnOv8hMwFqBaGWEvJrnRZ4gjFgdVGyuLU7Jp9sGV2Kjh5MsQYFomfGf9\/6oevy0PfO5no7d8O9CoA2n99pfzCGWc4SwNNJgAjtrR0kRimfW952CtLHKE4oDhI7Xlhp4KjJEOMYZ78TOr\/0WdelT\/\/7o9kfn4+9gRAOwOgSwLf33i2\/Kt\/sXzh7R\/JACQlmSfDpHUscjmKgyJ7J2HdyuLUhLcPF2OnAiPk0AyIMM02aIL\/4clT8sjEq5G3BPa6BTv9\/3sb3i0XLT\/LSf1n9faPYEyTIVKPai1bljjCzAEzB6k9g+xUcJRkiDGMw8+M+3\/zeyfl0PhLzoUfe\/b0UsAkh1kBsOG958iad7\/d2SFQj0q8\/SepvykThyFynbKWpTgooWfL4tRKuYadCk6eDDGGhp8J\/Pr\/3\/rBrHx36rXEY\/6mRib4X37B2fKxyyub+scoBZdmG8ToliWOVCRzoJ9S3rVrl+MB\/djSc889J2NjY9LX1yd1dXWYZ4DSZXEqgAAqyk4FwucUJsNoDE3K\/8iJ12X0+7PObP8k6\/3twO+85Z97pnxg9TvlU80NkZf+Ratx9ZzFNoj5qixxJHdxoN9YmJmZke7ubtmxY4f09PTI2rVrpbe3VxoaGpx\/V+ooi1MrxY+dCk6+1hmaMXjz5n\/nwyfkBy\/+FAr8dmpfg3\/rZefJv1n3Lif419IHgaK2zlpvg1E5+Z1XljiSqziwd0tcs2aNdHV1OWKgublZzK6IQ0NDUl9fj\/onUfmyODXRzadQiJ0KDrEWGJolft+ePCWPTrzqQEPG+k3w18C\/4sx\/lvYPrpazfvkXHbtRNv3BvVYuC7XQBrP0WFniCMWB1UrK4tQsG36QbXYqOPlqZmiP849NnpLHUgz8JuX\/0fedJ1vWnn7r9wr81cwPbz3pWCBDjGNZ4kiu4kCR63wDnV9gDyuYLEJHR4e0t7djngFKl8WpAAKoKDsVCJ9TuIgM3an+vd+ekSemXkvtjd8E\/l9d+Xa54j3nLKT6k8zyLyI\/vFXka4EMMd5liSO5iwPFrkMIW7duXeSB3bt3RxIGc3NzzvyEkZERp\/z27dsD5ynY19K5DUHDFmVxKta0k5dmp5KcnSlZCYbmQz4\/m5+XQ+Mvy7Ef\/TT1wP\/e8+rk41e8S37pF39hYWlfkuAfRrgS\/MLqVG2\/kyHmsbLEkYqIAwS9TmjUQ+cqmDkMfhmHiYkJ2bZtm+zZs8eZ12CyFn6rIsriVIQvUpadCkLvdNm0GZpZ\/U9Mve58xvc5cFa\/uUMT2HWcf+17zpauD10g06\/+30Vr+rMI\/mGE0+YXdr0y\/k6GmFfLEkeqThy43WaLBfdvKgampqYir4Aoi1Oxpp28NDuV5OziZg7s8f3Dx0\/J3x5LZ2Kf1sMO\/O9reIe8\/4KzoVQ\/TiW6BbbB6Kz8ziRDjGFZ4kiu4sC86Yd9vjlsqMC4zl79oJkB+zDDDy0tLZGGK7RsWZyKNe3kpdmpJGdni4Pn\/\/lcJ0BPnZyT+596SSZf\/kd4KZ\/XG\/+\/vLRefv2973R+WhAE9WfiN1FBC2yDOHwyxBiWJY7kKg4Uub7NHzhwYNHYvz08sGXLlkh7HmjGYHBwUNra2jw3TzLiYPPmzXLvvfeKCpIocw7sZjE6Ooq1khorPT09LY2NjTV219Fv98jzb8j5Z79Npl9\/U771zE9lcvb\/OYX17+jRcM7bHBNqX\/\/7qtVnyaUrflle+PGbsuGC6g74cdiwDcah5X0uGcZj2NrauqTA5ORkPCMFPDtXcRD0pm\/vc3Ds2DHR4P\/AAw+EIjObKrnnERhxcOLEiQUh4neuuUhZFF8otIxOqNU3DjvF\/9gzr8rYW\/vzIzv22S6y0\/wtlyyTjRef6\/yc5Wd7M2oimZut1TaYJlgyxGiWJY5UvTjQSYe6LHJgYECampoWvOo1rOB3LsUB9jCY0mXsVMyEvkP\/+2X55vdeEZmXTFL8F604S\/71ry2XUydfkpUrz+fmPQmbZBnbYEIUiYuRYWJ0TkGKg4T8woYVdJ8Ds6rga1\/7WuhVgnZW1EzBqlWrFuYcqDi47bbb5Pbbb\/fchbEsTg2FltEJ1dSpmKD\/50delOOvzDmf483iTf899WfKxSvOkk+sf7dDPWwGfzUxzKgZQWbJD8LnFCZDjGFZ4kiumQOD3GufA\/0Ak1lueOedd8rw8PCiTIApa69OMNkBv28yuIVD0MqGMik+rGknL12ETsWk+P\/y6Evy10+fdG4mi6B\/YX2drHvP2XLpyrcvCvphwT+MbhEYhtWxyL+TH+4dMsQYUhxg\/BKXdm+CZE9INL91dnY6QkMPW4j4TV40lSmLUxPDBQtm1amYt3wN\/LpBjwb+NIO+eaPXNfurltfJb6x+p7y3\/vTXQfOexZ8VQ9C1VVOc\/HBXkSHGsCxxpCKZAwx9dqXL4tTsCAVbjtup2BP5vjP1mjzyg9nMgr6m9993\/juk\/u2\/JB+6eNmiwF8pXl7XjcuwSHUvQl3ID\/cCGWIMyxJHchcHZtdC\/Wyz+whbaoi5LLx0WZwafqfZnGF3KubLexMv\/VQeeCrdN3179r4Gfd2P353et9\/6s7nbbKyyY8a4kh\/GT0uTIcawLHEkV3FgryAw+xnoEID7882Ya5KXLotTkxMIL2m\/7f+Pv39Z\/s\/MT5xC6Cd37WCu6X0N+h+8aJlc1XSu8wW+Wlm2x445vA0GnUF+GD+KA5xfWeJIruLAvc+BvZpA5wbs37\/fc0Mj3F3RLJTFqdHu1vssE\/xf\/sk\/yX1\/97wziz+twG+Cvv2mXytBP6pPGNyikvI+j\/wwfhQHOL+yxJGKigP72wdBSxJxd0WzUBanRrlbFQE\/+NFP5a6Hfyjz8\/OJBYCd4n\/v2T+T32+5SM6QM3KfyBflnqvhHAY3zEvkh\/GjOMD5lSWO5CoOFLu9nNAWBA899JCMjY0xc4C3zSUWdPz\/wHdflBMn52KLAA3++sb\/axecLduvagxM8bNjxp1HhhhD8sP4URzg\/CgOEjJ071xovpGgexX47W2Q8FKxi5XBqSoEHj42K08cfy2SELDf\/L\/02xfJynN+xXnr18xC3DX77JhjN7klBcgQY0h+GD+KA5xfGeKIUsg9c4Cjz85CNTrVzBHYsf\/pUDGgwV6X8fVsXu1AjBv8w8izYw4jFP47GYYzCjqD\/DB+FAc4v2qMI153nas4iPrhpfr6etxDCSxUk1P3P\/Gi7H\/8BV9BYAuBtEWAH1p2zAkanasIGWIMyQ\/jR3GA86umOBJ0txQHFp2iO1WzBP0PHhcVBu7DzA3o3rzaWfZXiYMdM06dDDGG5IfxozjA+RU9jkS9w1zEga5K2LVrV2idtm\/fLj09PaHnZXVCUZ2q8wgGHjy+JEuggmDz+5ZL2\/tXVEwQ2L5gx4y3TDLEGJIYME9sAAAgAElEQVQfxo\/iAOdX1DgS985yEQemUkHDCnErnsX5RXOqZgq85hKoKLir47JCCAKKg3RbIoMbxpP8MH4UBzi\/osWRpHeUqzhIWsm8yhXFqSoKHnvmVdlx4PuLbn3rB86XuzouzQtH7OuwY46NbEkBMsQYkh\/Gj+IA51eUOILeCcWBRbAITlVhsOWep5ylhObQTMHRL30Q9XXm5dkx44jJEGNIfhg\/igOcXxHiCH4XOSxlNEMJ4+PjofWt9Q8v6UTD6\/Y\/vUgUFHH4wM+R7JhDm3joCWQYiijwBPLD+FEc4PwoDnCGhbNQKadqluDgd1+U3f\/r+AKTO9svlU\/+xvmFYxRUIXbMuLvIEGNIfhg\/igOcX6XiCF7zxRY4rGDxqIRT3ZMOizrZMErDY8cchVLwOWSIMSQ\/jB\/FAc6vEnEEr\/VSCxURB15LG3fv3i3t7e1Z3GNkm3k71UsYHPrcFanvXBgZAHgiO2YQoIiQIcaQ\/DB+FAc4v7zjCF5jbwu5iwMVBgcOHJChoSExOyGaeQkdHR2hAsF8m2FkZMS5o6h7I0xMTEh3d7cMDAxIU1OTJ408nVo2YcBOJZ1HlMEN40h+GD8+xzi\/POMIXlt\/C7mKgzS2T7a\/6hhVVBhBceTIkcCPO+Xp1D0PTcltf3V6joEOJVRzxsA0L3bM+KNKhhhD8sP4URzg\/PKMI3htSyQO3LdiiwW\/2zSfhtbfi5A5sFcllEUYsFNJ5zFlcMM4kh\/Gj88xzo\/iICFDdFjBvmyUHRf1nJtvvlk6OztFhUSlxUFZhQE7lYQPhKsYgxvGkfwwfnyOcX4UBwDDNCYkaqAfHByUtrY26evrk7q6Os8a6bX0WL9+faQ5B7aR0dFR4C6XFp15\/U1p2zu98MN\/+shyabvsHaleo5LGpqenpbGxsZJVqPprkyHmQvLD+GlpMozHsLW1dUmBycnJeEYKeHaucw6yuH8VCTMzM54CQSch7t27V2688UanwVd6QuKWu59a+HjS3Z2XSeeVK7NAUjGbfGvD0ZMhxpD8MH7MHOD8mDlIwDDqBMI4poNWIahw2LRpkzQ3N0ulVyu4hxOqYTvkOH5gpxKXlvf5DG4YR\/LD+PE5xvlRHCRk6B5S2LdvnxO8kx5msqG9NFJtBW3b7HfNrJyqyxbX3fpt5xbLNAHR7TN2zElb8c\/LkSHGkPwwfhQHOL+s4ghes3gWKjqsYOYNaJUbGhoClxma27JXJ5glilq2p6cn8M4rmTm49X9Oyh1\/\/ZxTvzIOJxjw7JjjPXxeZ5MhxpD8MH4UBzg\/igOc4SILGvQ1C+DOALgv494EyZ6QaH7TlQnubESlxIE7a1DG4QSKg\/QeBgY3jCX5YfwoDnB+FAc4Q2dpoa440KPSX2TUOmThVHsSom50tPGSZSmQK6YJdsy4X8gQY0h+GD+KA5xfFnEEr1V8C7kPKyQZSoh\/W8lKpO3Ux545JVvuecqpzMaLl8mh665IVrEqKcWOGXcUGWIMyQ\/jR3GA80s7juA1SmYhV3EQZdOiZLeRTqm0nVpLWQN2Kum0QQY3jCP5Yfz4HOP80o4jeI2SWchVHCSrYn6l0nSqPdegFrIG7FTSaacMbhhH8sP48TnG+aUZR\/DaJLdAcWCxS9OptZY1YKeS\/CG0SzK4YRzJD+PH5xjnl2YcwWuT3ALFQQbioBazBuxUkj+EFAfpsGMbTIcjBRbGkeIA41fI0mk59UvfeEbueeSHzj3q0kXd+KgWDnYquJfJEGNIfhg\/CiycX1pxBK8JZiHXzEHQhES\/nQ6x24tXOg2n1tK+Bm667JjjtTevs8kQY0h+GD+KA5xfGnEErwVugeIg5WEF+xsKPZtXS8\/mVbiXqsQCO2bcUWSIMSQ\/jB\/FAc6P4iAGQ69PNHsV3759e+g2yDEuG\/vUNJxqJiLqUEKZd0PkW2\/s5hWpAINbJEy+J5Efxo\/iAOeXRhzBa4FbKEzmAL8V3ALqVHtI4TfXnCv3\/8E6vFJVZIEdM+4sMsQYkh\/Gj+IA54fGEbwG6VjIVRykU+XsrKBO\/at\/eEU+ed\/fOxUs+1bJzBxk0w4Z3DCu5IfxozjA+aFxBK9BOhYoDiyOqFNreUiBnUo6DySDG8aR\/DB+fI5xfmgcwWuQjoXMxYG9QmHNmjXS1dUl4+PjnrWv9MeXEKfaQwq3bLlErvvN96TjoSqywo4ZdxYZYgzJD+NHcYDzQ+IIfvX0LGQuDtKravaWEKf2Pzgl\/Q8er9khBXYq6bRPBjeMI\/lh\/Pgc4\/yQOIJfPT0LFAcpDSvU+pACO5V0HkoGN4wj+WH8+Bzj\/CgOEjA0QwxlG1awP83ceeVKubvzsgR0qr8IO2bch2SIMSQ\/jB\/FAc6P4gBnuGBBRcMNN9wgf\/iHfyhNTU2Blufm5qS3t1dGRkac84L2RnCLkba2Nunr65O6ujrPayR16re+Pyuf+JPT8yhqcZWCgcmOGX8oyBBjSH4YP4oDnF\/SOIJfOV0LhRlW0O2T9+\/fHxi89db7+\/sdAj09PWKCf0dHh7S3ty8iY0RES0uL85v5d0NDg+9GS0mdyiGF0+jZMeMPJxliDMkP48fnGOeXNI7gV07XQqHEgQb+oaEhqa+vj3yXtlgIK6Q7NY6NjfkKkCROrdUvMHqxZscc1gLDfyfDcEZBZ5Afxo\/iAOeXJI7gV03fQmHEgQb5mZmZ0MyBjSDoQ05eqLIQB\/a3FGp5SIGdSjoPJ4MbxpH8MH58jnF+FAcJGAZNSNR0\/\/DwcOicA3NZFRODg4MSNo\/AnB80BGHOSeJUM6SgNmbvuDoBlfIUYceM+5IMMYbkh\/GjOMD5JYkj+FXTt5B75sBvLoCZGxD3FqNkHMw11XbYhET7+qOjo4HVmXn9TWnbO+2cs+GCM+VPPr4ybvVLdf709LQ0NjaW6p7yvhkyxIiTH8ZPS5NhPIatra1LCkxOTsYzUsCzcxcHXsE8ylu9H7uJiQnp7u6WgYEBz6xDVGGg9uMqPnsJY619ntnLH3xrw59wMsQYkh\/Gj5kDnF\/cOIJfMRsLuYqDoDkCUVcruDFoOb+JjFFWKNj24jqVuyIu9gY7ZvwhJUOMIflh\/CgOcH5x4wh+xWwsFEocRFmtYK9OCAv+UYYcEHHAJYwUB2k\/lgxuGFHyw\/hRHOD8KA4SMHTPN7BNhK0kMOe6N0GyJySa3zo7O8XvI09BH3eK41QuYVzaANgxJ3goXEXIEGNIfhg\/igOcX5w4gl8tOwu5Zg70NnQYYOfOnYtWJui8gW3btsmePXukubk5u7sNsRzHqZxvQHGQRUNlcMOokh\/Gj+IA5xcnjuBXy85C7uLACIStW7cuuqt9+\/ZVVBhoZeI4lfsbUBxk8VgyuGFUyQ\/jR3GA84sTR\/CrZWehIuIgu9vBLMdxKucbUBxgrc27NIMbRpX8MH4UBzi\/OHEEv1p2FnIVB\/acgEoOH\/jhjOPU+usfdsxsvHiZHLruiuw8VEWW2THjziJDjCH5YfwoDnB+ceIIfrXsLOQqDuJud5zdbXtbjuNUIw64v8HPWbJjxlssGWIMyQ\/jR3GA84sTR\/CrZWchV3GgtxF1VUJ2t+xvOapTOd+AKfGs2ieDG0aW\/DB+FAc4v6hxBL9SthZyFQdB31bQ2wxaZpgthtPWozr12nuOyqPPvOqUOfqlD8qF9WfmUb3CX4MdM+4iMsQYkh\/Gj+IA5xc1juBXytZCruIg21vBrUd1KicjMnOAtzYyzIIhxQFOlQwxhlHjCHaV7EtTHFiMoziVmx\/5N0p2KvgDS4YYQ\/LD+DFzgPOLEkfwq2RvIXNxYE9C9Nu10NxmNQwrcPMjioMsH0sGN4wu+WH8KA5wfhQHOMPCWYjiVE5GpDjIsuEyuGF0yQ\/jR3GA84sSR\/CrZG8h88yB+xbc31cI+t5C9re\/+ApRnGrmG2jJ2TuuzruKhb4eO2bcPWSIMSQ\/jB\/FAc4vShzBr5K9hdzFgdeXEs3QQ0dHh7S3t2d\/1z5XiOJUTkZk5iDLBsrghtElP4wfxQHOL0ocwa+SvYVcxUHQJkj6Qab9+\/dLX1+f1NXVZX\/nHlcIcyonIwa7hR0z3mzJEGNIfhg\/igOcX1gcwa+Qj4VCiQPNKgwNDUl9fX0+d++6SphTbXHAnRGXuogdM95syRBjSH4YP4oDnF9YHMGvkI+FXMVB0PyCIuycGObU\/genpP\/B445nDn3uCtl4ybJ8vFQlV2HHjDuKDDGG5IfxozjA+YXFEfwK+VjIVRzoLenwwc6dO2V4eFiampqcu5yYmJBt27bJnj17KvrZ5jCn7vz6Mblv7HmnztwZkZmDLB5RBjeMKvlh\/CgOcH5hcQS\/Qj4WchcHRiBs3bp10R3u27evosJAKxPmVE5GDG6U7Jjxh5YMMYbkh\/GjOMD5hcUR\/Ar5WKiIOMjn1kTMMMbIyIhzye3bt0tPT4\/v5cOcys80Uxxk3XYZ3DDC5IfxozjA+YXFEfwK+VgotTjQCY56qCCIslwyyKn2ZMS9\/+5yaXv\/inw8VEVXYceMO4sMMYbkh\/GjOMD5URzgDHO3YIsFr4sHOdXeNvnuzsuk88qVude\/6Bdkx4x7iAwxhuSH8aM4wPlRHOAMc7UQtMeCqUiQU7ltcri72DGHMwo7gwzDCAX\/Tn4YP4oDnB\/FAc4wNwuaMRgcHJS2trbATZaCnMptk8PdxY45nFHYGWQYRojiACMUXpptMJxR0BkUBxi\/ipT22rrZrog61T5GR0cX\/vkf7n9Rjjz\/hjSc8zYZ+XRjRepf9ItOT09LYyPZIH4iQ4SeCPlh\/LQ0GcZj2NrauqTA5ORkPCMFPDv3CYlmT4OZmZklOLL+ZLNeu7u7WwYGBhb2WHCLAz+nmpUKH77kXPnG59YV0JWVrxLfOHAfkCHGkPwwflqaDDGGzBwk4GeWFjY0NAQuKUxgOlIR3YApaItmP6dy2+RIeNmpRMMUeBY7Zgwi+WH8KA5wfhQHCRhGmRSYwKxvEXt1QhRh4udUe6UCd0b09xA7Zrz1kiHGkPwwfhQHOD+KgwQMTYDu7OzMZTdE9yZISSck8psK0ZzNjjkap6CzyBBjSH4YP4oDnB\/FQUKGYan9hGZTKebn1C\/++Q9k7+HTcySYOWDmIJXG5mOEwQ2jS34YP4oDnB\/FQQKGZlhhfHzcs3TWExLDquznVH5TIYzc6d\/ZMUfjxMwBzsnPAtsgzpYMMYYUBxi\/Qpb2cyq\/qRDNXexUonGiOMA5URyQYXYEMMsUBxi\/Qpb2cqq9UqHrQxfIH\/3umkLWvQiVojjAvUCGGEPyw\/gxA4jzozgAGB48eFB27drlWNBPNT\/33HMyNjYWuHshcLnIRb2caq9U6Nm8Wno2r4psr9ZOZMeMe5wMMYbkh\/GjOMD5URwkZGh2KdTNiHbs2OHsd6BzDXp7e6VS+x+YWwkTB4c+d4VsvGRZwjsvfzF2zLiPyRBjSH4YP4oDnB\/FQQKG9j4Ha9aska6uLkccNDc3SxFWMXg51f7gElcqBDudHXOCh8JVhAwxhuSH8aM4wPlRHCRgWI3igB9ciu5odszRWfmdSYYYQ\/LD+FEc4PwoDhIy1PkGOr\/AHlYwWYSOjg5pb29PaBkv5uVULmOMzpUdc3RWFAc4Ky8LbIM4VzLEGFIcAPx0CGHr1q2LLOzevbuiwkAr4+VULmOM7mh2KtFZURzgrCgOyDAbAphVigOMXyFLu51qL2PsvHKl3N15WSHrXZRKURzgniBDjCH5Yfw4rIDzozjAGRbOgtupXMYYz0XsmOPx4psvzsttgW0QZ0qGGEOKA4Cfvc+BMaP7HeiqhUoeQeKAKxXCPcNOJZxR2BlkGEYo+Hfyw\/gxc4DzozhIyFCFwYEDB2RoaEjq6+sdK2YVQ9EmJPJrjPGczI45Hi9mDnBezByQYfoEMIsUBwn42UsZ3VmCIu5zcN3+p0X3OdCDmYNwh1MchDMKO4MMwwgxc4ARCi\/NNhjOKOgMioME\/KpNHHAZYzwns1OJx4uZA5wXMwdkmD4BzCLFQUJ+miHYuXOnDA8PS1NTU6GHFdbd+m3RFQsX1p\/pZA548K0t6zZAgYURJj+Mn5YmQ4whxUECfiZzMD4+Hlpav7fwwAMPhJ6X5glup3KPg3h02anE48XMAc6LmQMyTJ8AZpHiAOOXuLRbYLS1tQV+zdFeGRF2ru1Ue4+D\/9x+qfzb3zg\/cZ1rpSDFAe5pMsQYkh\/Gj5kDnB\/FAc4wtoW5uTnn640tLS3Oborm335fc7QnOdbV1YV++dF2Kvc4iO0epiPjI1tSgsENg0h+GD+KA5wfxQHA0Gufg6TbJ5tvNfT19YkKAPtw\/xZ0rpbzEwf8VHM0Z7NjjsYp6CwyxBiSH8aP4gDnR3GQkGHa+xwEBXyvzIHJOnhVn+IgoVPfKsaOGePHjpn8cAK4BT7HGEOKgwT80l7KGGXzpImJCdm2bZvMzMxI2C6MtlO5x0F8B7NTic\/MXYIMMYbkh\/GjQMX5URwkYJimODDzDbQaXkMK+nd3lqK\/v9+pdU9Pj2ft1anm+MnGbnlz+a9Kwzlvk5FPNya429orMj09LY2NZIV4ngwReiLkh\/HT0mQYj2Fra+uSApOTk\/GMFPDsM+bn5+fzrFcawwpRhIF78qLeo2YRuru7ZWBgYGGPBfvebcXHDZDitwq+tcVnxswBzsy2wDaI8yRDjCEzBwA\/ZEJi2AoFUy1UHHCPg\/gOZqcSnxnFAc6M4oAM0yWAWaM4wPglLq1DAzp\/wG8owTbsNawQVNY41d7joPPKlXJ352WJ61tLBSkOcG+TIcaQ\/DB+WpoMMYYUBxi\/RKX9dljU3RT1K49mL4POzs6Fzz+rmBgcHHSuF3UTJFsc9GxeLT2bVyWqb60VYqeCe5wMMYbkh\/GjOMD5URzgDAtnwStzwD0OoruJHXN0Vn5nkiHGkPwwfhQHOD+KA5xh4SwYp+pnmnUpox4UB9HdxI45OiuKA5yVlwW2QZwrGWIMKQ4wfoUsbZza\/+CU9D94nOIgppfYqcQE5nE6GWIMyQ\/jx8wBzo\/iAGdYOAvGqfYGSLN3XF24eha1QuyYcc+QIcaQ\/DB+FAc4P4oDnGHhLBinco+DZK5hx5yMm12KDDGG5IfxozjA+VEc4AwLZ4HiAHMJO2aMHztm8sMJ4Bb4HGMMKQ4wfoUsbZzKDZCSuYedSjJuzBzg3IwFtkGcJRliDCkOMH6FLO0WB9wAKZ6b2KnE4+V1NhliDMkP48fsFc6P4gBnWDgL6tS\/+e73ZN2t33bqxg2Q4rmIHXM8XhQHOC+3BbZBnCkZYgwpDjB+hSytTv1v33xSttzzlFM\/3TZZswc8ohFgpxKNU9BZZIgxJD+MHzMHOD+KA5xh4SyoU796cIwbICX0DDvmhOCsYmSIMSQ\/jB\/FAc6P4gBnWDgLFAeYS9gxY\/zYMZMfTgC3wOcYY0hxgPErZGl16vb\/8q2F3RGPfumDcmH9mYWsaxErxU4F9woZYgzJD+NHgYrzozjAGRbOgjr18hv+uzz27CmnbtwdMZ6L2DHH4+V1NhliDMkP40dxgPOjOMAZFs6CLQ40Y6CZAx7RCbBjjs7K70wyxBiSH8aP4gDnR3GAMyycBXXqOf\/+z+TE7BvOcALFQTwXsWOOx4uZA5yX2wLbIM6UDDGGFAcYv0KWVqee+p0hp24bL14mh667opD1LGql2KngniFDjCH5YfyYOcD5URzgDAtnYdXlH5DXP9rv1Iu7I8Z3Dzvm+Mz45oszsy2wDeI8yRBjSHGA8UtcenZ2Vrq6umR8fNyx0dbWJn19fVJXV+dp8\/Dhw7J161bnt7Vr18rQ0JDU19d7nmuLA+6OGN9F7FTiM6M4wJlRHJBhugQwaxQHGL9Epefm5qS3t1daWlqkvb1dzL8bGhqkp6dnic2JiQnZtm2b7NmzR5qbm+XgwYMyNjbmKyYu\/MBvyU82djt2KA7iu4jiID4zigOcGcUBGaZLALNGcYDxS610UMDX36ampjyFg1cFbHHAPQ7iu4jiID4zigOcGcUBGaZLALNGcYDxS620nzhwZxmiXLDhY1+UNy7d4px66HNXyMZLlkUpxnPeIkBxgDcFMsQYkh\/GT0uTIcaQ4gDjl0ppM\/+go6PDGWawDyMONm\/eLPfee68zRyFszoEtDpg5iO8idirxmTFzgDNj5oAM0yWAWaM4wPjBpU3wV0NeExLN7ydOnFiYhNjf3y8zMzO+cw5WfuJW+acLP+TUbdlfdsno6Chcz1oyMD09LY2NjbV0y6nfKxliSMkP46elyTAew9bW1iUFJicn4xkp4NlnzM\/PzxewXoFVChMGWthrWEEnKHZ3d8vAwIA0NTUtuca7PvXH8ubyX+UGSAkbBDMHCcFZxcgQY0h+GD8tTYYYQ2YOMH6JS4etULANa6Zg1apVC0MOKg5uu+02uf322z2XM1IcJHaLU5CdCsaPDMkPJ4Bb4HOMMaQ4wPglLh02NGAb1j0O9Hyzt4H+tx5eyx7178v\/4C\/kZ2ct5+6ICb3DTiUhOGYOcHBvWWAbxFGSIcaQ4gDjl6i0ewMkY8RMNNSNkHQfhM7OTmdfAz3sTZDCNkyqv\/5hpwy3Tk7kHmYOkmFbVIodMwaR\/DB+zF7h\/CgOcIaFs2DEAbdOTuYadszJuNmlyBBjSH4YP4oDnB\/FAc6wcBaMOODuiMlcw445GTeKA5ybscA2iLMkQ4whxQHGr5CljTi4u\/My58NLPOIRYKcSj5fX2WSIMSQ\/jB8zBzg\/igOcYeEsGHHA3RGTuYYdczJuzBzg3Jg5IMP0CGCWKA4wfoUsTXGAuYXiAOPHtzbywwngFvgcYwwpDjB+hSxtxAG3Tk7mHnYqybgxc4BzY+aADNMjgFmiOMD4FbI0xQHmFooDjB8zB+SHE8At8DnGGFIcYPwKWdqIg9k7ri5k\/YpeKXYquIfIEGNIfhg\/ClScH8UBzrBwFlQcXFh\/puiwAo\/4BNgxx2fmLkGGGEPyw\/hRHOD8KA5whoWzQHGAuYQdM8aPHTP54QRwC3yOMYYUBxi\/QpZWccCtk5O7hp1KcnamJBliDMkP40eBivOjOMAZFs4CxQHmEnbMGD92zOSHE8At8DnGGFIcYPwKWVrFAbdOTu4adirJ2TFzgLOjuCLDdAhgVigOMH6FLE1xgLmF4gDjx+BGfjgB3AKfY4whxQHGr5ClVRzwuwrJXcNOJTk7Zg5wdhRXZJgOAcwKxQHGr5ClGz72RTn8p7ud5Yw84hOgOIjPzF2CDDGG5Ifxo8DC+VEc4AwLZ6EsTq0UWHbMOHkyxBiSH8aP4gDnV5Y4csb8\/Pw8jqMcFsri1Ep5gx0zTp4MMYbkh\/GjOMD5lSWOVJ04mJ2dla6uLhkfH3e82NbWJn19fVJXVxfo1YmJCenu7paBgQFpamryPLcsTsWbdzIL7JiTcbNLkSHGkPwwfhQHOL+yxJGqEgdzc3PS29srLS0t0t7eLubfDQ0N0tPT4+tVc96RI0dkeHiY4gBv\/xRXZJgRAcxsWTpmjAJWmgzJTwlUlTjwctnBgwdlbGwsMHtw+PBh6e\/vd4ozc4A1\/KDS7FRwtmSIMSQ\/jJ+WJkOMYVn4lV4c6DDEzTffLJ2dnY5AoDjAGj7FQXb82DHjbMvSMeMkklsgw+TsyvQMV7U4MPMPOjo6nGEGv8yC\/n39+vWR5hxgzYKlSYAESIAEap3A5ORk1SOoWnFg5hGoB\/wmJOokxL1798qNN94o09PToeKg6r3JGyABEiABEiCBFAhUpTiIIgyUjQ4jbNq0SZqbmyXKaoUUeNIECZAACZAACVQ9gaoTB1FXKLiXPNqe2rdvnyMYeJAACZAACZAACSwlUHXiQLMBMzMzkfY2sG+XmQM2fxIgARIgARKIRqCqxIFfNmDt2rUyNDTkbISk+yDoygR3ZoDiIFqD4FkkQAIkQAIkUFXigO4iARIgARIgARLIngDFQfaMeQUSIAESIAESqCoCFAdvrWoYHBx0HMfJisHtV4dntm3b5sz7CPuuhc4PMVx1i+ugraur6qkBKhuHn7mMe9tw4PKlKBqHoXsoks+3OCu3oj7D9rl8hqM9PuZ59RrejmahGGfVvDgwWyvrnIVjx445yx\/1v+vr64vhoQLVwg5SW7ZsWfSdC3c13dta678PHDhQ02zj8LN5Krtdu3bJ7t27fTf7KlAzybQqcRi6VzZx3pEsfI9Gv08T9gwbYaXfrdE5XHyGw5u2aXMjIyNV\/6JZ8+LAfHNBH4CyKL7wJpzsDHfnqsJq\/\/79kVaOsGM+\/cZmfxk0Cj\/toG+44QY5deqUBO0Emsyj1VcqDkM997bbbpPbb7+dYv8tV8flZ7dXPsPRsqobNmyQEydOOB8DrOYl8zUtDvy+8mi++lh9XWe2NbazLJpZcf876OrsWGQJryj8VLxeeeWV8o1vfGPha6TZernY1uO0wSgfZSv23aZfuzj8vDIHYR+5S7\/G1WPx+eefdyqrq+a6urooDqrHdUtr6pUp0M541apVNZ++9fKr+003zptZ0v0pqrl9uesel5\/Z\/vv66693Ph5G0XpaYNnZqqA2qOJgamrKcQPnFJ1ujXH46fl2mnz79u1OwOMRTMAtqqqVFzMHrn0RKA78m3LcjsVY0k76zjvvrPkJiXH4aaf81a9+VT796U9LY2Nj4PyOau18ktQ7DkMzV8NMQtSyO3furOl2GIefmYy4Z88eJz0eJdOVxKdlK0NxUAKPclghnhPjpCQpDJayjcNPz33kkUecNzWuVvg5yzgM3cMK5BhvaIv84vWP5myKg2TcClfKzhRwQmKwe9wp3LAJdZzdvJhnHH72MlDbSq2nduMwdLdPPt+nJ8XakzSDnmGKg2ThiuIgGbfCleJSxuguibOMjCncpVzj8LNL84335zTiMHR30kyLx1vK6DWsUOvDMlF6S4qDKJSq5Bz7LY2bpIRnD\/w2UDETwDQV7vfmW+t8gzagscG+V14AAAVBSURBVPlRHPi3wzgM7U2QuInPaaZx+Kmg2rp1q1OO\/KIFNIqDaJx4FgmQAAmQAAmQQJURqOnVClXmK1aXBEiABEiABHIhQHGQC2ZehARIgARIgASqhwDFQfX4ijUlARIgARIggVwIUBzkgpkXIQESIAESIIHqIUBxUD2+Yk1JgARIgARIIBcCFAe5YOZFSIAESIAESKB6CFAcVI+vWNOCE3j22Wfl3HPPjfx5YF0P\/eqrr8rFF1+c2Z2Z\/SbWrl0rQ0NDketWLV\/RNPeX1xr8vK+XWcOgYRIIIUBxwCZCAikQiLv7Xh4bpSABHimbAs7IJjRY65Hn1wKrhU1kiDyRBDwIUBywWZBACgSKKA7i1snGUC0BkOIghcZLEyRAccA2QALJCdhb8aoVk6o\/duzYwhaz+nezRbR7C2mT+j7vvPOkq6tLxsfHncqYjymZ7waMjIw4f48yFGBfw06tm88Vm7vdvXu3tLe3L7l5v\/OMONiyZYvccsstTjl36t7eWtd9HWV1ww03yFVXXeWUN\/dy8uRJMdtva5kvf\/nLcujQIRkYGJCmpibHjN89eXnOLQ703z\/+8Y+d\/xmOQR+rcn9cSK\/h9bdqFE7JWzpLkoAIMwdsBSQQgYDXx4\/swOR+S\/f7op1eqq+vz\/kMswoETYc3NzeLER4dHR0LQTzoq5amPsZeXV2dE9TuvPNOGR4edgJtWObAfb79oR0VMBrEN2zY4NTX2D9w4IAzd0GDfHd396KgbtszAujCCy9cKO++R\/Pvl19+2alzY2Oj9Pb2OiLEDBOEfcDLSxwMDg6KEUNeXG13UxxEaPw8pSYJUBzUpNt503EJhAWZsEDsfiN1iwOvT+cGfY3RK+3vPj+oTmFfenR\/kU\/rHzbUYP9uxIFb7IyNjS2IBbVpB3\/9t\/05YeOjoKEDL3EwMzOz5Bp6nteETIqDuE8Cz68VAhQHteJp3idMwE7Bu1P+foHYnXpva2vzzBy40\/t2Zb2GBPyuZ3\/ZMUgchE2I9BICXn9zD7W4h05MZsQMF+j\/25MHbZuajTBfAHQ7y29owEscBF3DDF0Y+xQH8GNBAyUlQHFQUsfytrIjYAdEe96B\/XZqgr17HoB5c3ZnDrSs+4036A78An\/QUIdtDxUHasvMHTDixStzEEccPPnkk2KGLerr6yM5kOIgEiaeRAKxCVAcxEbGAiRwmoAdYM2bsaaudXxex85bWloWTQK0BYBbHATNL\/DincewgntOgX1NDeRBQwRmWMEWB15v6fawgmYOdu7cuTBnIko7y2JYIUyohQ2vRKk3zyGBohOgOCi6h1i\/QhDwmnNgv73bE\/S8JtaZTIIZVtCbsgWEsa+TE03a3Wvc38BIa0Ki\/aZuz0NYv379kgmHbnFglzV11frp5EIvcRB1QqLaMJMgw+Z6oBMSzbCPWWFi7sOeiOlugBQHhXgkWYmMCVAcZAyY5stDwAQOTf\/rYQ8Z2MsQNc1+zTXXLFquqKLg2muvlZtuumnhzdgtGEw2wSxx1GuYoOVHMWjZX9RJkrt27Vow7zVEYMbp3UHRfe09e\/Y4yxB1EqK5fztzoBdxM3QvZXQv59QyfsswTbZG\/98IKq+ljHZ5r8Buz\/dQP61bt06OHj3qCBS3iDP34M6qlKeV805I4DQBigO2BBIggYoR0GDttUIhaoWizDmIaivqecwcRCXF86qZAMVBNXuPdSeBKiLgnldhsgT2vgZxb4fiIC4xnk8C0QhQHETjxLNIgARSIODeNTJo98Iol3N\/COn+++93imX1rQV+eCmKV3hOGQj8f9y6SaE92\/DoAAAAAElFTkSuQmCC","height":186,"width":309}}
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
