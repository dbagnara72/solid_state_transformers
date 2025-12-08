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

model = 'sst_three_phase_dab_three_levelt_inv';
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
% Ls = (Vdab1_dc_nom^2/fPWM_DAB/Pnom/3)
Ls = 42e-6; % from TR design script
f0 = fPWM_DAB/5;
% Cs = 1/Ls/(2*pi*f0)^2

m1 = 6;
m2 = 6;
m12 = m1/m2;

Ls1 = Ls/2;
% Cs1 = Cs*2;
% Cs2 = Cs1/m12^2;
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
Vac_FS = V_phase_normalization_factor %[output:540d1256]
Iac_FS = I_phase_normalization_factor %[output:96e25287]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:82e29d43]

%[text] ### PLL DDSRF
use_advanced_pll = 0;
use_dq_pll_ccaller = 0;
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*f_grid;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega0 = 2*pi*f_grid;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:8d092f26]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:17f14063]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:2cc3b7d6]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:012657e6]
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
kp_pll = 314;
ki_pll = 3140;

Arso = [0 1; 0 0];
Crso = [1 0];

polesrso = [-5 -1]*2*pi*10;
Lrso = acker(Arso',Crso',polesrso)';

Adrso = eye(2) + Arso*ts_inv;
polesdrso = exp(ts_inv*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:78b0eed0]

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
figure;  %[output:0b49bf74]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:0b49bf74]
xlabel('state of charge [p.u.]'); %[output:0b49bf74]
ylabel('open circuit voltage [V]'); %[output:0b49bf74]
title('open circuit voltage(state of charge)'); %[output:0b49bf74]
grid on %[output:0b49bf74]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink settings
heatsink_liquid_2kW; %[output:178ec249] %[output:7b55c0ed] %[output:50dd316e]
%[text] #### DEVICES settings
% danfoss_SKM1700MB20R4S2I4; % SiC-Mosfet full leg
wolfspeed_CAB450M12XM3;
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
dab_mosfet.Cgs = Cgs;                                  % [F]
dab_mosfet.Cds = Cds;                                  % [F]
dab_mosfet.Cgd = Cgd;                                  % [F]

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

wolfspeed_CLB800M12HM3P; % common emitter half bridge
parallel_factor = 1.75;
invt_mosfet.Vth = Vth;                                           % [V]
invt_mosfet.Rds_on = Rds_on/parallel_factor;                     % [Ohm]
invt_mosfet.Vdon_diode = Vdon_diode/parallel_factor;             % [V]
invt_mosfet.Vgamma = Vgamma/parallel_factor;                     % [V]
invt_mosfet.Rdon_diode = Rdon_diode/parallel_factor;             % [Ohm]
invt_mosfet.Eon = Eon/parallel_factor;                           % [J] @ Tj = 125°C
invt_mosfet.Eoff = Eoff/parallel_factor;                         % [J] @ Tj = 125°C
invt_mosfet.Eerr = Eerr/parallel_factor;                         % [J] @ Tj = 125°C
invt_mosfet.Voff_sw_losses = Voff_sw_losses;                     % [V]
invt_mosfet.Ion_sw_losses = Ion_sw_losses;                       % [A]
invt_mosfet.JunctionTermalMass = JunctionTermalMass;             % [J/K]
invt_mosfet.Rtim = Rtim;                                         % [K/W]
invt_mosfet.Rth_mosfet_JC = Rth_mosfet_JC/parallel_factor;       % [K/W]
invt_mosfet.Rth_mosfet_CH = Rth_mosfet_CH/parallel_factor;       % [K/W]
invt_mosfet.Rth_mosfet_JH = Rth_mosfet_JH/parallel_factor;       % [K/W]
invt_mosfet.Lstray_module = Lstray_module/parallel_factor;       % [H]
invt_mosfet.Irr = Irr/parallel_factor;                           % [A]
invt_mosfet.Csnubber = Csnubber;                                 % [F]
invt_mosfet.Rsnubber = Rsnubber;                                 % [Ohm]
invt_mosfet.Csnubber_zvs = 4.5e-9;                               % [F]
invt_mosfet.Rsnubber_zvs = 5e-3;                                 % [Ohm]

% danfoss_SKM1400MLI12BM7; % 3L-NPC Si-IGBT
% igbt.inv.Vth = Vth;                                  % [V]
% igbt.inv.Vce_sat = Vce_sat;                          % [V]
% igbt.inv.Rce_on = Rce_on;                            % [Ohm]
% igbt.inv.Vdon_diode = Vdon_diode;                    % [V]
% igbt.inv.Rdon_diode = Rdon_diode;                    % [Ohm]
% igbt.inv.Eon = Eon;                                  % [J] @ Tj = 125°C
% igbt.inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
% igbt.inv.Erec = Erec;                                % [J] @ Tj = 125°C
% igbt.inv.Voff_sw_losses = Voff_sw_losses;            % [V]
% igbt.inv.Ion_sw_losses = Ion_sw_losses;              % [A]
% igbt.inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
% igbt.inv.Rtim = Rtim;                                % [K/W]
% igbt.inv.Rth_switch_JC = Rth_switch_JC;              % [K/W]
% igbt.inv.Rth_switch_CH = Rth_switch_CH;              % [K/W]
% igbt.inv.Rth_switch_JH = Rth_switch_JH;              % [K/W]
% igbt.inv.Lstray_module = Lstray_module;              % [H]
% igbt.inv.Irr = Irr;                                  % [A]
% igbt.inv.Csnubber = Csnubber;                        % [F]
% igbt.inv.Rsnubber = Rsnubber;                        % [Ohm]
% igbt.inv.Csnubber_zvs = 4.5e-9;                      % [F]
% igbt.inv.Rsnubber_zvs = 5e-3;                        % [Ohm]
% 
% infineon_FF450R12KT4; % Si-IGBT
% igbt.dab.Vth = Vth;                                  % [V]
% igbt.dab.Vce_sat = Vce_sat;                          % [V]
% igbt.dab.Rce_on = Rce_on;                            % [Ohm]
% igbt.dab.Vdon_diode = Vdon_diode;                    % [V]
% igbt.dab.Rdon_diode = Rdon_diode;                    % [Ohm]
% igbt.dab.Eon = Eon;                                  % [J] @ Tj = 125°C
% igbt.dab.Eoff = Eoff;                                % [J] @ Tj = 125°C
% igbt.dab.Erec = Erec;                                % [J] @ Tj = 125°C
% igbt.dab.Voff_sw_losses = Voff_sw_losses;            % [V]
% igbt.dab.Ion_sw_losses = Ion_sw_losses;              % [A]
% igbt.dab.JunctionTermalMass = JunctionTermalMass;    % [J/K]
% igbt.dab.Rtim = Rtim;                                % [K/W]
% igbt.dab.Rth_switch_JC = Rth_switch_JC;              % [K/W]
% igbt.dab.Rth_switch_CH = Rth_switch_CH;              % [K/W]
% igbt.dab.Rth_switch_JH = Rth_switch_JH;              % [K/W]
% igbt.dab.Lstray_module = Lstray_module;              % [H]
% igbt.dab.Irr = Irr;                                  % [A]
% igbt.dab.Csnubber = Csnubber;                        % [F]
% igbt.dab.Rsnubber = Rsnubber;                        % [Ohm]
% igbt.dab.Csnubber_zvs = 4.5e-9;                      % [F]
% igbt.dab.Rsnubber_zvs = 5e-3;                        % [Ohm]
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
%[output:540d1256]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.919183588453085e+02"}}
%---
%[output:96e25287]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:82e29d43]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.345132058575099"],["81.455860923503209"]]}}
%---
%[output:8d092f26]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"5","name":"Afht","rows":2,"type":"double","value":[["0","0.000010000000000"],["-1.421223033756867","-0.000188495559215"]]}}
%---
%[output:17f14063]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.018661060362323"],["3.911916400415777"]]}}
%---
%[output:2cc3b7d6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000250000000000"],["-35.530575843921682","0.995287611019615"]]}}
%---
%[output:012657e6]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.466526509058084"],["97.797910010394418"]]}}
%---
%[output:78b0eed0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.091119986272030"],["4.708907792220440"]]}}
%---
%[output:0b49bf74]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAaUAAAD+CAYAAACa7BouAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnX1wn9V15w+EpggwawQUUISRDDKBaasSChWuGZJ4CHRnbTo7yUpyWzxGcZ2JyUyLHUk2YRjPYkt2pGQJuNR1hNbZXUve3YSJPZ3FoWoCOI5JaoPaBjeIyKoRgsHYOIQgJgNo5zzKka+unpf7vP2el\/t9\/gH59zz35dx7z+eec8+996ypqakpwgMJQAKQACQACeRAAmcBSjloBRQBEoAEIAFIwJEAoISOAAlAApAAJJAbCQBKuWkKFAQSgAQgAUgAUEIfiCyBU6dOUVtbm\/N9X18fVVdXR07L78M9e\/bQhg0bqKuri5qbm51X+d\/4kb+TzNitXpOTk7R582ZauXIlNTQ0JJmdZ1ojIyO0atUq+tKXvmRUzyhlPHToED399NPU0dGRSp1ElsPDw076u3fvpqamJuO83Nre+OOUXmQ5d3Z2Uk1NTWpyS6nohUgWUCpEM6GQqgTSVlQ6lKqqqhwldPjwYerv768YlLZu3UoMDRPgi6IMU0ZOe8WKFbRmzZrUlKvkoU4owvTmtNs6TFn0PvjII49UtD9ELWvRvgOUctZirIh27NjhlIpnYqoSlAF633330dDQEPHsU39Hn5mqCkdm3jfeeCPNmzfPmbXyE6QwJF+9TLryPnnypDOzF0uCZ+CStmkabG3pikxVTFwGtprkWbZsGXV3dxODgx9RzsePH59R5m4KW2QxMTHhfKfKSa3Xo48+Stu2baN9+\/bN5Ml1Wr58uQMq\/d9Vy03aktuop6eH+O8FCxbMlFcvg9oO8hvXT6wYvW2l7Wtraz3LosqdKyDy4r7DQJKnsbHRkRc\/bP2KZRMELJGtyEHS4XbU81Z\/U4edX59V235sbMwZG3qfl\/6i14XLIHLU++Ttt98+U0+WyV133UWf\/\/zn51jj0tf0PN3aJ2eqpLDFAZRy1HQqkNRiictDH+RBCkV+F2WnK0H5XR9w+oxQhYAKposvvniW+06gJIpe0j1y5MgskPilERdKnLbISeSmwpgBNj4+7sBTyqkDjhWtuCW9oCQKUpWVKkc3hXzixAniCYFfGfS2lrbTlb\/a9l5lvOqqq2aBR+0P+m8MjK9+9av05S9\/eQZIev\/Rh4pXmbza3Q1KOpAkD4GhV58XuHq1pXyv93ku22OPPUbf\/OY3Z00obr31Vnr22WddJ1FusKuU6zpH6qliRQGUKiZq\/4xk8Fx66aUzM3yZAcoA3Lt3r6Pc5W9OUWbrYvXw7FdXZGI1CDT4O7bA1Bm2m69fBh4rU7HY1JmrzDY5PZ5l6+nz7DRsGkFQYkskyKWjz2L19wX+bgqf5bBo0aJZsDVx30ma6vdsbeiQ0dtSfhc5iSX1jW98w7EK5Hc3C1DtUSbuO91d5\/W3V\/\/R1wz1\/slyElnrUPGyxvX3dWX\/1FNPzerz6oTBza3pNQGRPs99UsotkJT2ZWtPtYJVa9vNDcltzt9U0qWbE3WVajEApVTFa564m6LVFZEMUBUg6mDh3HSrRrVK+P\/ZQpDZuqpE3KCkD3BxkUmtvNx3avph00gCSqrcxIoQBcNldwvOUOWow9YPSvpMnuXIFqQuZy\/o6D2EFaWUWV8f8nLFcfn8oOTlqtSh5GWVeFnSKogleEGvp0ykvKDkloabpR4ESt3i0i0ptz6vlsmt\/cWFqZZHdWcGld189ONNVQKAUk76g9tMzAtKXoPJC0r8717KUnd1qeIICxSxlOJCSQd00N9uTSjfPPDAA44VJ2szXhZHWCjpCkn9Ow6UVPeSV9CC27qjWL3qN6aWUZCrTPqPHjXn1ncqDSW9z4k7T3eTJgUldQ0TUEpHeQJK6cg1dKppuO\/0QrhBxg9K6uxTLClV0a1evdp1TUlVAKZpiItQdSnqQRJef7sJW7cOVEswrvtOX0tT3T9R3XdhXXH8vgprCbxQoaQrTd1VFuS+C+rESbrvdJe01EPWI70sJfEeyO96mXRIcVtFcd+5yQJQCuoh0X4HlKLJLZWv0gp0MHFleO0fcXPpiDvHK9BBhZKqPFWh+UWOyXtBUOL3vCK6+DeRp\/6OV8CHyElft1Chw+nefffdTjCAm3vHKyiFy2AS6CBWi67wvAICvOTI6fAjkZxuLig1ao3Tefjhh+mhhx6aUy89wlHSCgp04PWboPU\/00CHICjpA9Kvz7uV2yTQQbcYsaaUihrEMUPpiDV6qkmHhKsKOaylJLVwWzdhV47JmlJQGvy7CgkuL1tg995775xIKFFMqiLzg5LfPhzTkHBZTFcVOCv82267bSayjRXgPffcQ2vXrp1xE+pQ5JDw9evX+4aEq8rfzZ3rpsDd1hc5by6jWLKydWD79u30+OOPk6yvqbDVJxoCXD\/5cj5+IeG6Nee10dlrPUhd8\/SCkj5hYHnwVgQJQOAy6Ot7\/G9qnmp76hu01TVa9TfdTamvt0bXAPgyd5YSK4v29nZnb4jbznl9T4JfOHPZmjdo1lm2+ha1Pqqi1sPxdSvSq46i9Bj+aZ22UFT5xi23TEg4HbeoUpNTQrBPKW4reH+fKyiZhLWyYuY9IjYOVEApvYGQdMpertigjcpqOcKc6JB0+cucXpAr1OQYKR6LONEhnV6SKyixFcQDkR8vS4l\/r6urMzoLLB2RZZcqoJSd7MPm7LZuEXQ6gp6HzMbZ9RfmvLiwZbXxfbd1RdNz+XD2Xbo9JjdQ4tnLpk2bqLW11QGTG5SkMyxevNhKKKXbFZA6JAAJQALZSyA3UGIrgJ9PfOITnmtKbmZ3GHdI9uJGCSABSAASgAT8JJALKLEpvWvXLrr\/\/vudc8m8Ah10dwbcG+jckAAkAAmUSwK5gBK76zjElv3mQdF3uvhlDcot8GHhwoXlai3UBhKABCCBGBLg2wXq6+tjpJD+p5lDySsShqtusvDoF\/jAUBodHU1fijnOATIgggwgAx6i6AfFkEHmUNL1uZ+lJNF56oZG3pDodUovOmExOmHaTEc\/QD8AlKZHWRHGQu6hxCAaGBiYuRhN3zzrZ00VoQGgkNOWQDEGYtpSwFhAPwCU0h5lBuljIBIdO3Ys9z5kg6aM9QpkgH7AHQj9oBhgzp2lFEv7aB8DShiIUEbTgwIKGTKApZQkXSKmBShhIEIhA0qiPgBmWEoRUZLcZ4ASoAQoAUqA0hmdWgSdCPddcgzMZUqYHQLMADPALMoJUMpYTRehAdIWEaAEKAFKgBKglLamNUwfUIJChkKGQob7Du47Q2Sk\/xqgBCgBSoASoAQopU8bwxwAJUAJUAKUACVAyRAZ6b8GKAFKgBKgBCgBSunTxjAHQAlQApQAJUAJUDJERvqvAUqAEqAEKAFKgFL6tDHMAVAClAAlQAlQApQMkZH+a4ASoAQoAUqAEqCUPm0McwCUACVACVAClAAlQ2Sk\/xqgBCgBSoASoAQopU8bwxwAJUAJUAKUACVAyRAZ6b8GKAFKgBKgBCgBSunTxjAHQAlQApQAJUAJUDJERvqvAUqAEqAEKAFKgFL6tDHMAVAClAAlQAlQApQMkZH+a4ASoAQoAUqAEqCUPm0McwCUACVACVAClAAlQ2Sk\/xqgBCgBSoASoAQopU8bwxwAJUAJUAKUACVAyRAZ6b8GKAFKgBKgBChZBKVTp05RW1sbDQ8PGxOmsbGRnnjiCeP347wIKAFKgBKgBChZBqV169bRxo0bqaGhIZAfIyMjtGXLFurv7w98N4kXACVACVAClAAli6CUBDjSTANQApQAJUAJULIISuK+4yr39fVRdXV1mowJnTagBCgBSoASoGQRlLiqk5OT1NnZSfv27XNqvnv3bmpqagoNkDQ+AJQAJUAJUAKULIOSCpNDhw7RihUrnH9atmwZdXd3U1VVVRq8MUoTUAKUACVACVCyGEpSddV6qqmpcQIbTAIhjEgT4iVACVAClAAlQAlQmoUNibjr7e2t+JoToAQoAUqAEqA0LYEDL5+mz3Y+Sq\/\/36+EmNpX\/tWzpqamppLOFpZS0hKNnt6xY8eovr4+egIl+BIywOQEkxOigZ+8TmsHjtKpr30q16M6UShhTSl\/bQ2FDIUMhQxrkSVgDZT06Luuri5qbm7OhXaG+w4KGQoZChnuu2kJbN0\/Rlv3Hyu3pST7lE6cOJFZMIMf\/QAlQAlQApQAJYuglAtzyKcQgBKgBCgBSoCSRVBiSynLs++2bt3qSLujo8MVTYASoAQoAUqA0rQEOMiB15VKHeiQ5SnhElSxZs0aQMnHWkSgA8AMMAPM1kApK\/cdB1ds3ryZfvrTnzpHGsFS8m4JQAlQApQAJZbA8u3P04Gfny63pZQVlPbs2eNkPTY2BvddQCMASoASoAQoAUop0opdhps2baIHH3yQdu7cCSgBSoG9DWAGmAFmoj946Ed0\/NR7sJQCNUbIFzi44bbbbnPcdiaBDpL80NBQyJzK8fr4+DjV1taWozIRawEZEEEG9spg6dKl9OF5l9Dbn5kODCt1oENEHRH5Mz5Hb9euXXT\/\/fc7p4+bQGl0dDRyfmX4EFYCrARYCXDfsYXElpJ1UOK1ng0bNjgV53uV+BkYGEjsCgs1fRUYfE3Gww8\/PIchCAmHQoZChkIWxWDzBE1Oc7AKSmy1TExMOGs9vObT2tpq5GKLY43AUgqWns0DEcroTP9AP7B7giaRd2e\/+ya9+befC1YcGb6RyIGs6iZaXr\/gm2gFSmleXQEoBfccKCO7lRHADDCzBKrv+74jiHPe\/Bm98a0vBCuODN9IHUq8yZXh0dfXh\/uUMmhoQAlQggvTbhfmzgPj1PGdEUcIFxzYRsd\/\/P8y0ETmWSYCJc6O13sOHjw4y323aNEiamtro5aWlkxODseaEhQyFLLdCllVhTZO0NQAhwXV59Lbj\/8Z5T34KzEoceOr9ylJZ8jyKgtACVAClAAlW12YDKR7B446pzjws731Orq\/ebFdUDI30CrzJqAEKAFKgJKtUHpm5C3608decKrPVtILX7mFiqATE7WUKoMa81yK0ADmtYn2po0uC11SkAEmJzZNTthCeumNd+m\/\/N3wDJD2fvEGB0xF0ImJQMn0tHC\/E72jqVz\/r4rQAGnU23Y\/OqA0t1cBzHaAmYHE11PwDbNiIT3ach0tuWa+83cRdGIiUOLKcqDD4ODgrCg7gZUEOgSFcCetoIvQAEnXGQoZCtmtTwFK5YfSgZdP072DR53z7VSXndofiqATE4GSwIevkOAz6dRHDQk\/efIkbdmyxbk6vRJPERogbTlAGZVfGZn0IfSDcvcD2RwrfUHWkPS+UQSdCCiZjOgCvwNlVG5lZNo10Q\/K1w\/06Dqxjno\/ey0t\/Xi1a9ewBkqm7jvZy+R2Tp3p4ArzXhEaIEx9orwLZVQ+ZYR+EEUC5egHDCIHRoqbTrWOJKDBS0JF0ImJWEoiALd9SnwwK7v0+Lf169c7rruGhoZovSrkV0VogJBVCv06oFQOZRS64bUP0A+K3Q\/crKIwMJJ3i6ATE4VS3IGT9PdFaICk66ynB2VUbGWUVP9APyhePwgC0YY76+mWhfOdUG\/Tpwg6EVAybc2CvgdlVDxllEZXQz\/Ifz\/g6Lm9\/\/wGfe\/FkzMRdGpfYPgsuOhcar+jfibEO2xfsQpKbq47EVhjYyMOZA3bexJ6H8oo\/8oooab2TQb9IH\/9gC2h7T94hY6+9s7MUUB6IyYBIjVNa6A0OTnpXFexePFiWr58+czVFTiQtRLqxj8PKKP8KaMsegX6Qbb9QPYOqWfRefUDBtGKm6+gxQvnR7aIvNK2BkrqfUocxMCbZOvq6pyTwdmCSvL22TADuggNEKY+Ud6FMspWGUVpszS+QT+oXD8QAPGpCj\/8+WlXV5zulrttUTV97hOXJQ4hvS8VQScmsqakQ4lDv8fGxog306Z5yV\/Q4C1CAwTVIe7vUEaVU0Zx2yrN79EP0ukHDKDDx9+m\/h++6umC0wHEf\/\/X5ddQY+28UEEKSfSPIujERKDEwlKPEFKto7179zr3LHV3d1NVVVUScjVOowgNYFyZiC9CGaWjjCI2R2afoR9E7weyN+jo67+ifcNv0PG3pvcKBT0SFXfH9ZfQ2k9eWXEAuZWvCDoxMSjpRw0xpHbs2EE1NTUV3ZtUtEW9oI4d93coo+jKKK7s8\/Q9+kFwPxDQfOvQBP342C+MLB9pYwlIWH1rrWMB8RMmVLtSfcUqKFVKqGHyKUIDhKlPlHehjIKVURS5Fu0b9IPpfvDqBxc5Tff9n52in4z9wtjqUeHD\/\/\/pj1fTX336qlyCx69vFkEnJmIp6WtKqlCwppSt+oIyApS4B9rUD8Ti6T\/4Kh3+97dDg0esHN4T1HT1fPrzm68oHHy8tA6g9Jsr0tmV19fXR9XV7ocEpqW2i9AAadVd0rVJGXnJEjIoD5RkfYfb+rljp+npl95yml2u\/A4znsS9dus1F9HnbryMzj7rLAc+eXS7hamX1ZaS34ZZVTCVvtxP8gaUyqOM4gxKQKlY\/UAsnYM\/P027f\/xaZOioFs+VvPfn+nPoyivzEXAQpz\/H+bYIOjF1910cAcb9tggNELeOQd9DIRdLIQe1Z9Tf89QPGDq\/fv9D+m\/\/+O90\/OR7kdxr+hoPu9r+oqmG\/qj+P8yISLd48iSDqO0Y97si6MREoBRXUGl9X4QGSKvucN+dkSyUUWXArIZJ\/4\/nJui50V\/EsnLE0nH+e9G5tKThImr5w8udNKO42NAPLLgOXcLAh4eHfXUrzr5LGz3e6WMgVkYhZ9fCZjnH7QcCnKmpKfr60HEaPfFuLAtHt3T++Or5jpXzyUWz152jwMdLInFlYCbpfL9VhIk6LKV896HYpcNABJS4E\/n1AwkeeOmNXznRaq\/wRXKGG0T9OqgAha2cxivn0eoltZGtnNgDwbIIRC95AUpJ9KQYaRShAWJUz+hTQMlOKKmutJffeJe+\/dwxeuWdsxOBje5Wu\/GqC2npxy+eBZwkLRyjjm7wEsaCBe47vR\/wmXcbNmyY9c9dXV3OwaxZPICSnQpZ72tlU0YCnF\/9+gPn6oPjJyedKkcJi3YblwKUJddcRDfXXUgLLzmvFKHSZesHUXRqEXRiYu47BtLg4OCs\/Uiy5tTS0pIJmIrQAFE6VphvMBCLA2aBzb9OvEN\/\/y8nEnOj6es37E67+tLz6EufXkDnnH1Wpi61MH057rsYCxZZSvq5d2rn4b1M2DwbdzhF\/x4DMTsoyVoNWx5TU0T\/+8jr9OxvNnsmsWbjBpv6S8+jO6+\/mOade86sCLUPfvEa1dfXR+9IJfgSYwFQcroxoJTtaMZATB5K6nrNP7\/6S3rqxZN07M3JxNZr3GDD6zYcmVZ\/yfRJ+2HXbNAPku8H2Y7saLkXwXsE9120ti3MV1BG5spo5iSB0dP07MhbibvPVJiwC41PGbj+igto2e9fGgk0YToh+oF5Pwgj16K9axWUuHEQ6JC\/LmqzMhLI\/PjFY\/TKe+fRyyfeTQU0Omyuu+J8JxrtvI9+ZBZswlo3SfYmm\/uByBEysNB9l1VAg9fgLcKsIEnF45ZWmQaiukbz4dQUfef5N+gHPzvlVDvJNRo399m1lzNoqumC3z5nBjRZQiZsvylTPwhbd0DpjMSKoBMTc9\/ph7Pu3r2bmpqaovafRL4rQgMkUlGfRPKsjBgyrNjFojnxy1\/TnsOv07+99quKgGZBdRW13nw5XXnRuam7z9Ju56D089wPgsqe1O+QgUWWkt5pVDcebp5NakhFSyeLgSiQef\/DKfrvB1+lF175ZWqQ0V1niy47n9r++GN0\/m+fcZ1lIYNorZXeV5AB1pS4dxVhop6YpeQ1nDgcnK0o3KeUnsLxSzkJZaRGmw3920n6pwSPonEru3o8DQcD1F1cRbcsnD\/HmjF1nyUhg2xaL7lcIQNAyWooMYh27NjhjCiTw1gnJyeps7OT9u3b53zjd\/+S7ib0s8SKMCtITu24p+SmjNSL0t799Qf0g5feon99tXLWzO\/VzqNPLrqIqn7rjDVjCpgo8oJChkLmfoN+YJmlFMdlxxDjp6Ojg4JOgeB8xsbGnHeDHpugpFozfPzMrh9N0IsT79Dke+\/R4VffCxJV6N9Va6bhsvOJ99EsuXramlFdaqETTuEDKCMoZEBpemAVQScm4r7zO9Ehio5RIaV\/z7\/V1dUZHVtUhAYwlY9A55sHxmfWaJI660yFCO+f4ecPr7qQ7r6lxrkiOm+QMZWZvAcoAUqAkmVQCqsk\/N73A5y4+RYvXlxKKDF4Pvhwinr\/gW\/knIx9wCZbM++\/\/z4tvPQCZ6PmZ66\/hG64cp4T7aa6y9J0nSXZN6KmBSgBSoASoBRJf8ha1LJly6i7u5uqqqaPVJHH7VJBv1PI82wpMRj2v\/gm7Rs+ERo+quuMo82+cFstffQjZ8+SlbwDhQyFDIU8PTQwFixy30UikM9HDKeJiYk5YBoZGaFVq1ZRT0+PswdK\/1tPMk9QYgj9z+deo0Ojp40gxFBhVxqf5HztZedHdqFhIEIZQSEDSqIb86QTvRCQyJpS0lBi2LS3t9O2bduooaHBN3m\/9SduAHmGhoaSLqZRej3PnKKB4bc936258By6Yt459MDSi+kjZ51F\/HeSz\/j4ONXWTt\/4aesDGRBBBvbKYOnSpbOG\/ujoaK5VQSJQYrfaunXraOPGjXMgwoDZsmUL9fb2UnV1tZEwwpws7hf4kNWs4MDLp2nb\/mOuFhFbQLy584YrL6zIxWmwlGApwVKCpWSdpeQHJRPAqNaOBDPw\/iM97FtPi\/9ev3499ff3u1pUlYSS7P25d\/DozLE50hEYRI+2XEdLrjkTMm1E5wReApQAJUAJULIGSvpGVi8d6rcZlr\/RN8+qgQ6cx8DAwMz6Upgz9ioJpeXbn59lGYlFdFfj74S++yYBFs0kASgBSoASoGQNlKSifpZSkgo2bFppQ0mso+V\/8\/xM0RhGD911Df2n35u+IyfrB1AClAAlQMk6KGWteL3yTxNKDKSBn7xOW\/cfm8meTzR4tPW6TC0jXRaAEqAEKAFK1kBJ9g2tXr2adu7cScPDw658MDn\/Lg2wpQUlBhJbR3LKQpZrRkFyA5QAJUAJULIGSkEKMevf04CSG5D2fvGGXFlHqtwBJUAJUAKUAKWsafSb\/JOGkhuQXvjKLTmprXsxACVACVAClKyDktvxP6qKLIP7joF078DRmQg7dtnlHUhQRlBGMg4xOcHkhPtC0hP1NGbkiWye9SoYh3pv3ryZVq5cGXgyQxqVS7IBvvq9Mep6cjqooShAApQAJUDpjGYBmAElpzfo+4zSgI9XmklBiaPs1g4cnQFSnteQdFlgIGKGjMkJJifWue\/8QBPlmKGkwJUElNht9wcP\/aiQQIIygjKCpQRLSdWnSejEpPSzVzqpuu84U68Tv9OuWFL+U\/WkBraQsjgqKI6sYCnBUsLkBJMT6ywlv0AHPsPO62y6OMrW5Nu4swLdbVeEwAa47+b2DIAZYAaYp8dFXJ1oonfjvpO6pRS3gHG+j9MAavh3njfHBskHChkKGQoZlpJ1lhJXWL+q3O+07yBFmtTvcaEka0nfXtNIn7rW7NqNpMqeVDqAEqAEKAFKVkLJ7bK9rMEUFUp6cEMR3XbSCQElQAlQApSsg1LSl\/wlZSUkASUGErvvivoASoASoAQoWQmltrY251K+pqamWfrb5JK\/tBR+FCiVyUqCMoIygsV8RrtggmZZoMOePXtocHCQ+vr6Zq49l6i8lpYWam5uTos9nulGgZIacVfEEHBdGBiIsJQwOcHkxDpLSSrsdhNtV1dXJkDiMkWBkuxLKtJRQn60B5QAJUAJULIWShU3hQIyDAulAy+fdu5J4mfV4o9R72cX5a1KocsDKAFKgBKgZB2UJMqutbV1zppSaC2a4AdhocRAYjDxU\/QAB6wlYC1BHUqYnGByEtV7lKBKNkoqkc2zftF3RqVI6aUwUCpbgAOgBCgBSrMVC8AcbUkjJfXsmWwiUOLUOdBhbGzMicDLyxMGSmqAQ8cd9dRxR11eqhGrHBiImCHDfQf3nXXuuzJc8le2AAdYSrCUYCnBUtJntGEm6rFmwzE+TsxSilGG1D4N0wDV933fKceSq+fT3rU3pFamSicMSwmWEiwlWErWWUqVVrSm+ZlCqWx7kzBDxgxZHyOYnGBywn3CVCea6tg03otlKYnbbvXq1bRz504aHh52LWNjY+OsTbVpVMQtTdMGKKvrDjNkzJDhxoUbV9WNpjqxUjraLZ9YUMqy4CZ5mzSAGnVXNtcdoAQoAUqAkrVQKurVFeqG2e2t11HrTZeb8K4w78BtA7cNJieYnFi5plTUqytW9P0LPfnTN502K8NZd1hLmDtfAJgBZoB5elyYeI+ynnEn4r4r8tUVfJEfu\/DKctYdoAQouSkVgBlgtg5KRby6Ql1PYrcdu+\/K9kAZQRnBSoD7zkr3XRGvrihzKDgWuLHArU6wMDnB5MQqS0k6f9GurihzKDigBCgBSrP9HgCzRWtKeXV5+S3qlT0UHFAClAAlQEnXzdYEOhQRSmooeJkOYEWgAwIdEOjgrpFgKcFSypxVfrMCG9aTsMCNBW5YzLCYVUUMSyljLPk1gKwncRFPfe1TGZc0vewxO8QCNyYnmJyIhgGU0tO1Rin7NUDZ9ydhhowZMtaUsKZk9ZrSyMgIrVq1iiYmJuYAI48HsspVFWXdnwQoAUqAEqBkLZTk3LuamprUb55Vw845v\/7+fmpoaHC1nLwspbKfdwdlBGWkDwi4ceHG5T5hjfvO75ghIz+b4UtsjbW3t9O2bdscELlt2DVZ1LMlyAFrCVhLgMUMi9lEJxqq4Iq8lsjZd2Iptba2UlNTU0UKzpnokDI1VW3YNAtlBGUEixkWs6lOrJjSNsgoEShxPmy1HDx4kLq7u6mqqsog6\/ivBOXpZaqW9epzN4nCbQO3DSxmWMyiG6xy3\/GBrJW6eVYNqti9e7endebWAOpJDmW8PwlrCXOFIAPOAAAO0klEQVTRDDADzADz9LiwBkrxbZ5oKQicenp6XMHEDSDP0NCQ87+HX32P\/vI7rzv\/\/3f\/+XK68WPnRsu8IF+Nj49TbW1tQUqbTjEhAyLIwF4ZLF26dNbAGh0dTWegJZRqYu67hMoTOhm3ywX9TNWt+8do6\/5jzisvfOUW5x6lMj+wEmAlwEqA+846951UmNd4NmzY4PzJbjV+BgYGUltn0q9g1+HiZqraFOQAZQRlJGMCkxNMTrgvWOW+Y4uFN84++OCDtGnTJpJIPD9LJqyFokfb8Z6l9evXe+5VcmsAOclhydXzae\/aG8IWoXDvQxlBGWFygsmJdZaSuk+J1y86OztnoMQg2bJlC\/X29lJ1dXVspa7f2RQ20MGWkxwwQz7T1QBmgBlgnh4P1lhKflBiiLC11NfXlwiUwlBNbwBbrqtQZQSFDIUMhQxLyTpLiSsse4ZU992iRYuIQ8VbWlqoubk5DE8SedcPSnu\/eAMtuWZ+IvnkORFACVAClAAlK6HElc77dehrB44SHzHEjw2Rd1BGUEZw48KNq06arXHf5dVS0BvAtsg7QAlQApQAJUApR4TSoWTT8UJQRlBGWFucrYzgyrYo0EGaXt2nJP\/mFx2XNr9UKKnHC\/V8dhHds\/hjaWefi\/QxELGmBIsZFrOVa0pu10hwVF5eAh1sjLyDMoIygsUMi9lK953Ap6OjY84ZdHkJCVehZEvkHaAEKAFKgBKgpN2nlPTm2TA+MdV9Z9uZd1BGUEZYU8Kakq4vrYq+czvyJ0\/uOxsj72ApwVLC5ASTE6stJa\/7lFShNDY20hNPPBHG4In8rjorkDPv+FRw3qNky4NABwQ6YHKCyYmVgQ55VPIqlGwMB4cygjKCpQRLyUpLKY9A4jIJlNRw8I476qnjjrq8FjnxcsFSgqWEyQkmJ9ZaSm77lLq6ujI5906Fkhp5Z8MV6FjgxgK3PrvB5ASTE1UnJj77TTDBxG6ezfM+JT7vjs+948eWM+\/gtoHbBpMTTE50VlgTfZf3fUo2HsQKKAFKgBKgBCjldPOsreHgWEvAWgImJ5icWBvokGf3na3h4IASoAQoAUrWQokrntdAB1vDwQElQAlQApSshlKCARiJJMWLej\/4pxeJLSV+vv65a2nlLTWJpF2URBB1hagrTE4wORF9ZU2gQ14VNDfAt753hJb\/zfNOEW3bowRlBGUESwmWEiylHBFKt5RsOh0cygjKSB2KsJhhMXN\/gKWUMaC4AdY89o+0df8xpySAUsYNklH2UMhQyPAaTA8+QCkjJaT6T\/9k898Tb57l59TXPpVxiSqfPRQyFDIUMty4WFOqvO51zZFnBb+77tt04OenybbTweG+g\/sO7rvZagETNFhKmaOJoXThPf+L+EBWQCnz5sisAFBGsBZhLcJ9l5kCUjNmKJ3+0z7nn5ZcPZ\/2rr0hF+WqZCGgkKGQoZDhvoP7rpJa1yevut+9md7+zFbnDRvDwaGMoIzgxoUbV5+oj46O5kRDuxcjsVPC81jLBTf\/Cb2zpB1Qqq\/PY\/NUrEywFmEtYoIG913FFI5fRqqlZGM4OAYiLCVYSrCUYCnlAkfThaj5j39N7318ufP\/gFKOGqbCRYGlBEsJEzRYShVWO+7ZqVCy7XI\/zJAxQ1ZHBcAMMHN\/wObZjNH0O3f\/Lb1\/ybVOKWzcOIvZIdx3mJxgcgL3XcYgUrMXKNm6RwlQApQAJUAJUMoRlC75wv+hD8+7xNo9SoASoAQoAUqAUo6gZPPlflBGUEZYU5qtjLCuhjWlzPEkULJ14ywsJVhKmJxgcgJLKXMUnSkAoISII4AZYAaYz+hERN9lDCiB0vbW66j1psszLk022cNlATADzACzaB9AyVAPnzp1itra2mh4eNj5YtmyZdTd3U1VVVVzUjh06BCtWLFi5t9ramqov7+fGhoa5rwrULJ14yyUEZQRrAS47+C+MwSRvDY5OUmdnZ20ePFiam5uJvmbYdPR0TEntT179tDY2Jjrb\/rLgBKsBIAZYAaY4b4LiaW5rzN4Dh486Gotbd26lerq6hyABT0CJVs3zkIhQyFDIcNSgqUURAqD372gpFtVQUkxlGzeOAsoAUqAEqAEKAWRIuB3WV9qaWmZYw3pa0+cVFdXl6fVBCjBfQcwA8wAM9x3kbEklhAn4BboMDIyQqtWraKenh5qamoi\/W+3NaVz3vwZXXBgGw0NDUUuV5E\/HB8fp9ra2iJXIXbZIQMiyMBeGSxdunTWGMIlf4YqJQhIXsnwGhM\/bkERbCnZeg06Zodw26hjBlsD4DXg\/oCQ8JBA8oq480vGL\/CBoWTzaQ5F6YSG3STya0UYiJErZ\/ghZFAMhWzYnJFfK0I\/yMV16AyWiYkJz71J0gK8R4nf7evro+rqauK\/169f77tPCVBaSHk31yOPMMMPizAQDasS+TXIAFAqyiQ1cyi5BS+w8BobGx34vPTSSzQwMDADLH3z7O7du531JbeHLaXzjjxOHz3+w8iDGR9CApAAJFAmCeR9kpo5lMrU2KgLJAAJQAKQQDwJAErx5IevIQFIABKABBKUAKCUoDCRFCQACUACkEA8CQBK8eSHryEBSAASgAQSlACglKAwkRQkAAlAApBAPAkASvHkh68hAUgAEoAEEpRAKaHEB7pu2LDBEZPf2XgJyjHTpHjv1o4dO5wy+IXIq+\/53UOVaWUiZm4qA0k+7OG+EYtV8c9M5aBvxfDrNxWvRMwMTWUgx5TxHsmyjQc\/EYa5aSFmU0T6vHRQ4o7W3t5O27ZtcwQi\/+92CWAkieXsI3VDMe\/pUjcXq0XVT17nvwcHB2c2IuesWqGKYyoDXR48cSnTpMVUDvqdZeqYKfo4MZWBQJmPJ+N9jmUaD0FA4glsnvt96aCkK9+8zwpCaV+Xl9Wz\/0TZtLa2em4oliTKpIjCyoAV0rp16+j06dPkdhp93DbJ6ntTOXDbb9myhXp7e52TUcr0hJGBOmEt03hwa0+B8Pz5852f77zzTqM76bLoG6WDkn5Aq9+BrVkIPMk8vW7tlVt8\/fIqyyCMIgPuEzfddBN997vfnbnxOMl2ySKtMHLwu0Qzi7InlWcYGbhZSl4XiyZVvizT4fq+9dZbjptSvek7yzJ55V1KKKk304a5Pj2PDeRXJjfLyNQyND1vMO8yCSsDhvGuXbvovvvuo02bNpUOSqqV7NUXZExw25qsRea9D0j5wvYFeX\/fvn20Zs0a15sGilJ303IWYS0VUDJtzRy+F3YQShVYKT3yyCOeB9nmsKqeRQojA3538+bNtHLlSueOqbzPGMO0Qxg5SCCQBDcEHWwcphxZvhtGBvpdbPphz1nWI828AaU0peuRNtx3nb6z\/zIBibtAGJcNK56nn37amREXYXCGGT5h5KC778oiC8gguMcUoa1LZynp7jpTd1Zwc+bzDbV+QYEOZY0wMpWBGiqstmZZXDemcmA4qyfvB\/WbfPZ891KZyqCsYA5qK0ApSEIp\/I6Q8DP3TaniLYuLxq3LmIYBq98WYXCGHR6mctAX+cvkujKVgZv7zu9utrBtkdf3i9DvS2cpcWfA5tnp+6VUq9HLSijLpkmvDZNegS5FGJxRFJupHNTNs2XbOGoqA\/VutrLJwKvvFKHflxJKUQYzvoEEIAFIABLIXgKAUvZtgBJAApAAJAAJ\/EYCgBK6AiQACUACkEBuJAAo5aYpUBBIABKABCABQAl9ABKABCABSCA3EgCUctMUKAgkAAlAApAAoIQ+kAsJ6Bs6\/QqV9mZPNVx62bJl1N3dTVVVVYFy0vf\/BH5Q4RckVLqxsTHVK0vUM+XCyK\/C4kB2OZUAoJTThrGtWHmCUpiyqO1UBChxefmYpUo8ZT2NvBKyszkPQMnm1q9w3fXbTuV4H3UTo8zi2TLhA1P5BGd5+GKy5cuXz\/p3uaxMnZ3z+0EzdK+Nk+rGa07HbXOxVz3k3\/nSODl9W7dK1Hw5ffV3zvvEiRM0NDREw8PDTt78uyqHBx54gPbu3etcYskX8oWpt34uZJjTwt2AGwSdoN8r3P2QXUEkACgVpKGKXsygwzJV64TryoqYd9nLrF49SFZO+JZrGtx2qfvdo6UfueT2t3o2nCp7v3rcfvvt1NbWRgsWLHBcfno99HxUiHE93Q7LVa8YkfQOHz7snPDudtK5X73doKTevqsfvRNkBQZBJ+j3ovdplD8dCQBK6cgVqWoSCDreJMhlpp5pqEPJ7Vu5XXbjxo2ORSGPVzlUhe1XFr9z4qJYE2q+uhJ3q4MKtpMnT846WJXr6FVv\/s0NSvrFdl5Qi1I3QAlqIIoEAKUoUsM3kSSguq5095oXCNRzzOR8Mh1KustNCud2npnXuo96Rp4flPwUraniFotkYmLCKaq4MfW03a4tV+F85MgRYktHf7zOcfNy36lrTF71M62bWhZAKdIwsf4jQMn6LlB5AahKWV1XUl1mAiOB1\/j4OLW3tztrKW5QMr3KOksocR1WrVpFDCNZq\/KzlEygZFpvL0tpbGxsVuADoFT58YAcZ0sAUEKPyEwC+jUDAiV2sa1bt45U11uQ+46Ve19fH1VXV\/vWJ0v3HQco6BCI674zrTfcd5l1c2QcUgKAUkiB4fVoEggKRlBdZvwuBwywW4kj2cS64cg0dYFfD3RQAyP81n6SDHRQlf3q1atnlZt\/Uy0PhpJq2Yjb0ct9J2mzZaUGTuiBDqb19gp0EKtNBb+6DsflkPaTvKRNJKjDbR8X3HfRxortXwFKtveACtZfX0tR15V08PAi\/ooVK5zSsSLs6elxFupbWlqoubl55s4sUeh6mHbQBlG\/u3SCgi70vKQeOkx1KPHfang3l72uro4GBwcdK++pp56aBS0VBhIazyHhzzzzDPX29jpWYZh6u0HpySefdGTM18Tzo4bAu61xifuR5csQ3r9\/vwPMoLqbbD6uYFdEVjmWAKCU48ZB0SABXQJu60ymUjKJvjNNy+Q9WEomUsI7ugQAJfQJSCCnEnALyvDbhxRUDUApSEL4PQ8SAJTy0AooAyTgIQH9BAhxV0YRmH72nZu7MEq6+jc4+y4JKdqbxv8HOafKqtv18LMAAAAASUVORK5CYII=","height":254,"width":421}}
%---
%[output:178ec249]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:7b55c0ed]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:50dd316e]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
