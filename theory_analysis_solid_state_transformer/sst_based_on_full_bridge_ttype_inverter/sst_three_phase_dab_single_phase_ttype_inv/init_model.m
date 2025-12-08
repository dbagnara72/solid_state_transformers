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
three_phaseDAB_TRsizing; %[output:87bfeb36]

Ls = L_d_eff; % from TR design script
% Ls = Lsigma; % from TR design script
% Ls = (Vdab1_dc_nom^2/fPWM_DAB/Pnom/3);

m1 = n1;
m2 = n2;
m12 = m1/m2;

Ls1 = Ls/2;
Ls2 = Ls1*m12^2;

mu0 = 1.256637e-6;
mur = 5000;
lm_trafo = mu0*mur*n1^2*S_Fe/L_core_length * 1e-2;
rfe_trafo = V1^2/P_Fe;
rd1_trafo = R1;
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
wolfspeed_CAB450M12XM3; % SiC-Mosfet full leg
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
%[output:87bfeb36]
%   data: {"dataType":"text","outputData":{"text":"--- INITIAL ELECTRICAL PARAMETERS ---\nNominal Power (Sn): 110.00 kVA\nNominal Primary Voltage (Vn): 400.00 V\nNominal Primary Current (I1n): 275.00 V\nNominal Secondary Current (I2n): 275.00 V\nNominal Frequency: 4.00 kHz\n----------------------------------------------------\nCore Section Area (S_Fe): 46.9219 cm^2\nPrimary Turns (n1): 6\nSecondary Turns (n2): 6\nPrimary Copper Area (A_Cu1): 91.67 mm^2\nPrimary Copper Band Length: 18.33 cm\nSecondary Copper Band Length (L_b2): 18.33 cm\nCore Height (AM-NC-320C AMMET): 22.00 cm\nCore Width (AM-NC-320C AMMET): 5.00 cm\nCore Window (AM-NC-320C AMMET): 6.00 cm\nCore Length (AM-NC-320C AMMET): 72.00 cm\nCore Dept (AM-NC-320C AMMET): 9.38 cm\nSpecific Core Loss (AM-NC-320C AMMET): 8.53 W\/kg\n----------------------------------------------------\n--- LOSS ESTIMATION ---\nCore Mass (M_Fe): 26.35 kg\nCore Loss (P_Fe): 224.86 W\nCopper Loss (P_Cu): 48.99 W\nTotal Losses per Phase (P_tot): 273.85 W\nTotal Losses (3P - P_tot): 821.56 W\n----------------------------------------------------\nEstimated Efficiency (Eta, cos(phi)=0.95): 99.74 %\n----------------------------------------------------\n--- LEAKAGE INDUCTANCE AND REACTANCE ESTIMATION ---\nCalculated Leakage Inductance (Ld_calc): 0.000020 H (19.74 uH)\nEffective Leakage Inductance (Ld_eff): 0.000026 H (25.66 uH)\nLeakage Reactance (Xd): 0.645 Ohm\nEstimated Short Circuit Voltage (Vcc): 44.34 %\n----------------------------------------------------\n","truncated":false}}
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPcAAACVCAYAAACEsGwCAAAAAXNSR0IArs4c6QAAGqBJREFUeF7tXX2MltWVP7QuARSrM+DqFMcBBYurqxVLLet2UKu4saAh2QUm2aVACEtFdlMoMIALJMsIBPwD0JY1lGA2M2PIuoXJZsPOsjKLRaKVSlJhCzgMHx2NfAgIQje0bM4D593z3vc+z3Ofj3ufd97nPElDnfd+nd89v3vOPfer19WrV6+CfIKAIFBxCPQScldcn4pAgoCHgJBbFEEQqFAEck3uPXv2QENDAzQ3N8Ojjz6aaherZV+6dAnWrl0L06dPh6qqqlTqwjIXLFjglbVixQro27cvYL1Hjx6FCRMmpFKHXyFvvvkm7N69u1BvUGXr16+HMWPGwNChQ0PbFCVtaGHXExBObW1tUFNTA5s2bTJqiw5f0zrTTHfmzBmYNm0azJ8\/P5Ke5prcaXZAWFkrV670iLdx40Zr5D5x4gRMmTIFXnzxRavkJmWbOHFiaD04CKxbt86IUFHShuHNfz906JCHy9ixYz2CmH7lQm5sL+pPd3e30WBK8lknNwGLDcOPrCQBd\/78ee\/vHR0dJaMqWT\/8\/cEHHywQg\/7+0ksveX\/Dsl9++WVfRfNrA7euWD5aQWoP5sERftCgQd7fcdTHb8aMGZ6CUJlEJNVS8\/9GS9rY2OjlVy2HToHUgSAMQyx33rx5MGvWLNi3b19RO5EwfnVjPRs2bPDaNHr0aNi5c2eBhNzaYYEcX5WERHaqm9Ly\/qO+HzZsmGeF1Hbq0qI3xduP5CQPRSWoXxvUv+vKUGWlPtbpKNdDIh1i6KejWBb+TmUGYa62lXuUcbxMq+RWCcCVgkjzwQcfeApVXV3tdXptba3XgdwKjRs3rsj9RMVAd5oD5mcVVSvDiXPw4MGCW07kpvaQC8lHTKoXQcf2cisZRG5U0iDLjbi0trZ6AxV+iAPm0Q0iOgwxj4oZuuWIf1NTE6xZs6ZQLuFLsiARCV8uux9OJAu3Ijxte3t7kaVWBwJMW1dX5w3ERFxSYjUtx5QGBcKFk5v6mH5T+yLMcvv1saoTWKfa5y0tLUXYk3dAbSAdxbz0Nx3mxAfqy23bthXhGMVbKrLc6ogR5LaoI1dQWr8RHjth9uzZJfNFnn7v3r0lSkIEIFKShQhy5xDQuXPnat1CneUmcHH+GgRoFMsdRm4qa\/Xq1R6cPA4QBUM\/t1xn\/XD+j94IzT95PTTQElk4DupAiziRNdLphq5v\/CyUbiDgg7afa6qLb\/BYBOGic8uD+pgs97Fjx7QDL+k+yc89O52lxXR+mKsDB9cJ7Ad1AAviXQm5ly1bBkuWLAmcD2IFJum40LxzeAPDyP3WW2957gz\/yKU9ffp0IAF4njDiE5FolObkVgnMy02T3KREKB+N8DQ3Vy1+EIYquUnBUKmxb7HvSD4kN+8bjhMpGk2lSG60QhgQ5B4WkludNnCS6zwNVHC01kEDmTodojaYDCDqVCeI3EF9rJaD\/829Kho0OS5+3gO2X+1Ljg3ptEpaMmDU7+R5Ie5hn1W3PIrVwcYGWW4uiDoqmhJYjYibWm6dK5gmuVE2kmHgwIEFl5z\/nSxsFHJzZUJ8uTWLYrk59kFBJj53JXdUN2j4xSnCLLefMqdhuXV9HERu1TipxE9quVVZE1luHJHwSzOaazLnplGc5lRR5tx+czUOjAqKbrTEcnSWWx1tcXSlOddTTz1VNIqTa0ZtUjs3LFrOrR8PpJhgSNZYtdw6WSmgxOfcJMvJkycLbnrYnJusvjpoBLWBu\/tEDup\/Cp7xyLrLOTfJw\/tYnYKoBFZjDRi4pEFNR24+51YxtzbnRuVQI4ZR1gOD3AOutLxMPir279\/fc9NUlyssWm5CbmxblGg5d8t1uPAACZ\/LUlTUj9xcFt26ujq\/42vhJhhiQA0\/iuwjiXkEHbFHr4AP4Lai5TwizduOLiZ+FL3H\/sYBhSy5mpYH3TBflGi5boD0WwoLi5aTTqjkVvsF8VUDlmpfl0W0nCsuChe01ETkRmFRoVatWlWySUAF8IEHHvCWYHr37h1p7S5sniG\/myEQ1bPQeUNRN1WYtaxyUyXBHPloummIEDSac5sE0oi86lISVYRlzJkzBxYuXOgRv5w2CFSuOvlLpg62mDLKTr04ypZHnLnMcTFPdYeaulRh4qJjZ6OLheQmAnPB+JprWtsv864sIr8gEISAZ7njkFkl7ubNm2HmzJmwePFiLbnjuPnSdYKAIBAfAY\/c6C7g\/+JYVMy3fPlymDx5srdri7vefs2K62bEF1NyCgL5Q6BguU02p+jm3mokGiEMc+Np7jFq1Kii\/eBDhgzJXw+IxLlFYMeOHTB48GBr8mvd8qDagrafqkEzXg665fjRnmddVB3J3dnZaU1YKViPgOCejWbYxt0oWm4qukpuTuigU0ZUvm1hTeXIW7ojR45YtSB5w9NUXtv6niq5TYXyS2db2KTtq9T8Qu5seta2vgu5s+nXsqpVyJ1Ndwi5s8E9V7UKubPpbiF3NrjnqlYhdzbdLeTOBvdc1Srkzqa7nZObR7XpZJHfrrO0IbEtbNrtrZTyhNzZ9KRtfS8KqPEbQcaPHw+4pXTRokWA9zlFPZESBy7bwsZpUx7yCLmz6WXb+l5Ebr4DDQ+tE7mR9CY72JJCZFvYpO2r1PxC7mx61ra+lyyF0SV0U6dOhS1btniHQfDKXN01NGlDYlvYtNtbKeUJubPpSdv6rl3nVu+QNrmoIQ14bAubRhsrsQwhdza9alvfZRNLNv1aVrUKubPpDiF3NrjnqlYhdzbd7ZTcYY8ThB3lTAqRbWGTtq9S8wu5s+lZ2\/pe4pbjSa6urq6iB9Pob\/X19UDPp5hcih4VMtvCRm1PXtILubPpadv67rsUxm9loSUyvGsaL7MPe5kkLlS2hY3brkrPJ+TOpodt67t2Ewu\/wZRuWhkxYgQ899xzsHXrVmtXEdsWNpsuLP9ahdzZ9JFtfTdaCsMrb\/FFEJP70ZLAZFvYJG2r5LxCbve9+87hs\/D86nY49dO\/tFZ5JkthuFEGP\/UhdCG3tX4OLFjI7R73lvc\/hRdaDsCZVx63VrlzcuveLSbphNzW+lnInQ20vrWu3N4FK7cfcUtu9X5xal2Ud7n9JKLAHL44cvHiRbHcZaJwYrndd4RzcvMLDvF9bFz6oofj6+rqiq4hjgMHuuNY5tGjR0uW27A8sdxxUE2eR8idHMOoJTh3y\/mpsPb29gIBTd4KCxMO3fGOjg7PWuvW0onceJezfG4RwGd38QlZ+dwh8O0l2+HKgHvdueX8sYCHH3648GLn3r17obW1NdHb3fR0KYePP4crltudYqk1ieV2j33Vj972KnUaUFOtN72lHOUFyDCogiy3PEoQhl76vwu508c0qMRjZy7DQ\/\/4rntyuxBTyO0CZfM6hNzmWCVNicSe1XIA3vn4rFty+82t05hzm4AiATUTlNJPI+ROH1NdiUjsvcfOw9Q3PvJ+vuHUb+CzN\/7WWuXGb4XhZYkrVqwAGwdGSDoht7V+DixYyO0G93Gv\/qpgsWur+sDZ1r+Drl+\/Z61yo4Mj1mpXChZyu0K6uB4ht13cN+w6AY3\/eqhQCRJ72w+\/CaMfuc\/qw5fOd6gFwSjktqtkfqULudPHXZ1bUw1EbPzXtr4bu+Vp7FALg9C2sGH15\/V3IXfynkcyd576El5pP1pwvXmpnNSupqFiuZP3a48vQcgdvQuRzK\/uPA4HPrmgJTOWiIT+s7tvgfljBnv\/X\/1sGzMhd\/R+rbgcQu7gLkUir27vgq5Tl3yJzN3uMfdVwwuja7WE5jU5Jzd\/Toga4iJSjnXZFrbiWJmSQELua0C2HzgN6\/7rGBz7\/DIgoU0+tMhP3FsFf\/\/kXaFkztRy8+eE+FlreqhAlsJMurvnpckDuYmsr3Uch\/3dFyIRmFvlP7\/nVvjx03WRiazTCtvGzGgpTDax9DzCRmlxJZAbyfvJud\/BG3u64fiZy7HIS\/Nk\/JesMf9bFExN0jolNzYIrXRbWxts2rQJ8Nw13aGGrrl6c4qJAFHS2BY2SlvylLacyU0Wd8+Rc\/DPe7q9boniNuv6kYJb3\/tGNcx6\/E74Sq9eqVjiqDpjW9+1ATX1wgZ5Tihqt\/Ws9FmQm0i76\/DnsPvjs4msrYo2krf21j4wZGA\/+Idnh8CF3\/0+E\/KGaUEm5A5rlK3fbQtrq909vdw0yY2kxf+9+ctP4ejpS4mtrB9x76zqA3814nYYPKBvWRLXRCds67sshZn0QoWn8SM3WVckUvN7n6RuYTms5CqjxX3+m7fB1FFf9wYJ3fpwpXSHU3LTc0K1tbXWD4lkET2sFKVIIgcR9t3Os\/Bu5znoPPklXLp8GU5eAuPlH9P6OWHrBvSFv\/52DXyr7uaKJ60pPk7JjY3SrXOrN6aYNj5qOtvCRm1Puacnolbf9Efws1\/8Ftr3n04l4BQkNyfs3QP7waSRt8PIuq8JYWMoi219N3LL8f4zjKJv3LgR+DNDMeQJzGJb2LTbm2Z55ILiv2\/\/5gy813XOCzLhlzQ6HEbWK1euwJCBN3nz1\/phVfDIXTd7WSrZJU6z7+KWZVvfS8its9xhr3vSXeQopN9uNrVc3UEU28LG7YQ4+ZCkv\/\/DVfj3j07BR90XnBCVExLnrndV94XvPzAAxvzJgILLrSNsmgG1OFjlNY9tfS\/ZxDJt2jQPa1Mrza9Dxhs0FyxYAKNGjSq5Bpmnw\/Vz3Wdb2DhKRK4vEnXrvs\/g8GeX4NiZa1Fgz6oablOMUzd3gTGoNfyOG+HZ+wfCV7\/SK1XLKuSO0zvJ89jWdyO33FQMss6TJk3y7jvnH26GaWpqgjVr1vi69raF5e0hUm7e0w3vHznn\/UT3WpnKGyUdJ2ptdV8Ydls\/ePq+aujX+6uZu79C7ig9mV5a2\/qeGrnJNfdzy002xtgQFkl86sL\/wtK2j1MjLycqbpS457Z+8Oz9A7w5a09cvhFyp0fYKCXZ0Hdef2rkpkKRxLt37w5cSqMlN9zOyi08CpvGowTd56\/A0v88BR\/81uxkT83NN3jNv6P\/DTDoazfAuOE3wUM1fQDLod+idFpPSyuPErjvsSeffNKr1OZV3qmT2ySyzh8\/mDBhQgHZJCMZWsx\/+\/VJWPTzw9qeoi2J4x\/+Y\/jBd2p6pIW1pYJiuW0hG1xuEn03aXHiU2HqiTG\/53nRouOHZMb597x582DVqlXe4RT64giLpL7yh6vwSNOeInnDbsEwAScvaYTc2fR0HH2P0lLjO9SCLmzwWwrjhFaXwnSHUeIIiy838Ig1knr9xOHw2D23RMEh12mF3Nl0fxx9j9JSI8sdpcAkaaMIi4Qe99qvitZv8bpY2XgRvQeE3NExSyNHFH2PU1\/qc+44jYjqlguxk6BcmlfInS6epqU5ITfNm2fPng1z586Fffv2lbSvXK42Vok9\/bFBsHK8flOMKch5TyfkzkYDnJA7G9FKazUR9icdx2HR1msR8e8OvRV+PvOhcml+j22HkDubrjPR9yQt61FuOX\/6FOfWHy7+ThLZJe91BITc2aiCU3LT5pJydcv5Q2oYPJOIeDpKKeROB8eopTglt1\/jcO26vr6+ZL94VGHC0gcJ+87hs150HL\/lz98DM797Z1hx8rshAkJuQ6BSTlYW5C6Hq43JauveXEoZ89wVJ+TOpsvLgtwmW0rTgMdP2Jb3P4UXWg54VbzWMBwmPnJ7GtVJGTLnzlQHnJI7aM7d3NycmVsuVtuuDorltouvX+lOyZ2NiP9fq5+wVT9620uEATQMpMmXLgJC7nTxNC3NObnVQx24P7y1tdX4ZhZTwXTpdMLilbqzWv\/HSy4R8iTo+ucVctvBNaxUp+T2u0nF5Ix2mCAmv+uE5S65rGuboBg9jZA7OmZp5HBKbr+oeFbRcr5pZen374bZT9SmgamUoSAg5M5GJZySG0VEK71u3brCQ4AUZMMbU1w\/BMij5Gi15cSXHSUUctvBNaxU5+TGBtHLnt3d115VzOohQHHJw9Qjnd+F3OngGLWUTMgdtZFppVeFpSg5Pni+9YdyQCQtnNVyhNy2kA0u1ym5bc6t1ZtYdOvmXFg+35YouV3lE3LbxdevdKfkxkbgPvK6urqSRwWSio9z+a6uLm\/e7rfjjQv7WsdxWHz9aKfMt5OiH5xfyG0X37Igt6tTYUjulpaWkuuPObllvu1O4YTc7rDmNTm33DbF5K55mFte2JV29y2w7QXZlWazX4TcNtH1L7uiyE1ihj1KgI8BjN18wkv+T+NvhxFf75MN+jmpVR4lcN\/Rzh4lcH2HWtijBPzstsy37SueWG77GOtqqBjLHeVRAplvu1U2IbdbvKk25+S2dXAkylIYPTQg96S5UTohtxuc1VqckrscDo7s\/OV+QHLj95gE05xonZDbCcwllTgldzkcHOHknj9mMMwfU5cN8jmqVcidTWc7JTeKmPXBkTf+Y2\/hIkTZmeZG6YTcbnDO1C2nyrM8OPKnP34L\/vvQ515TzrzyeDao56xWIXc2He7ccmcj5rVaUdj75\/wLvPPxWe94p1zO4KY3hNxucC4Ly52NqNfIffb5jRJMc9wBQm7HgF+vLleWu+7+kXD+6ZWe6OsnfgMaRt6RDeo5q1XInU2H54rctSP\/Ai48Ns9D+tVJw2HSt+R+chdqJ+R2gXJpHbklt0TK3SmckNsd1rwm5+TG45gNDQ0l0rp4n3vQEz+ALx+a7NUte8rdKZyQ2x3WmZHb77SWK9Fv+5ufwpUB90qk3BXg1+sRcjsGPIuAms1rlkzgE3KboJR+GiF3+pialOjcLefXIZk0MM00ckFDmmialyXkNscqzZROye3qmiU\/gIjcsqc8TRUKL0vIHY6RjRROyW1DgChlErklUh4FteRphdzJMYxTgpA7DmqSJxICQu5IcKWW2Dm5+aUKY8eOhXnz5sHixYth4cKFMHTo0NQE0xUkltsqvL6FC7mzwd0puYnYNTU1MH78eNi8eTMsWrQItm3bBrt37y65ipggwSBcY2Oj959+6+HqTSy6dERuOQ3mVtmE3G7xptqckpsvhZ0+fbpAbiTmsmXLYMmSJVBVVVWEBB4PbWpqgjVr1ni\/4aMG+MbYihUroG\/fvoW0WPacOXMCPQAkt5wGc69oQm73mGONTsmNFRI5p06dClu2bIGZM2fCrFmzwPSVT78HB9RBwM8tF3K7VzQht3vMMyE3VqpuQY3yyqffc0Tcdcc6dGWi5ZZ709wrmpDbPeaZkTuuqKYbYPy2uSK5xw6\/CZZ+b0DcJki+GAjIowQxQEuYxdmjBAnb6WWP8oCg36MESG5Z406jN6KVIZY7Gl5ppXY+51aj2igILompATIuIBK7vr7em5f7fSaPEiC5++39GfQ+9ou08JNyBIGyRqCzs9Na+3pdvXr1KpXOl8LwqV386G\/4\/3UE1x0RpcEAl9DwmzBhQqGctrY2729R5vHWpJeCBYEKRqCI3FnfW17BOItogoBzBIrIjbXrlrKizKedSyAVCgKCgBaBEss9bdo02Ldvny9cLm5kkb4SBASB5AiUWO7kRUYvga+BNzc3BwbmopcuOQgBHh\/xi3moD1LMmDEDKP4iSKaLgMmuzSQ1Zk5uvnPt4MGD0NLSEhiZTyJsnvNyRUIc+JZhjovfDsM8Y2dDdhpEsexNmzZZOZRVQu44S2FJhEerTYdSsO6w\/edJ6spzXiQtxk42btzo7flfsGABTJo0qcRLMt2IlGcsk8qOA+3rr78OzzzzDCxduhRWrVpln9y6pTAUxO8wSFIhMT9XpqwvaExDnnItg1tkbCOSe9SoUd4yJX3qwI6nA21ZlXLFyWW70HrjkWon5M5iKUzI7UadTMittoRbe\/U0oJtWV3YtTslNVho3mtCITXMD3JhiI7AibrkbBTZ1y3lrbCufG8nLtxbb+GoDaiYnuNKCTAJqaSEZXI5JQA3d8uXLl8PkyZO9OSAfePnZfDctrvxaMiG3a1hpMJE5nl3k+VIYX3JU9\/1PmTLFu3BD+sNuf+SC3HYhlNIFgXwikPk6dz5hF6kFAfsICLntYyw1CAKZICDkzgR2qVQQsI+AkNs+xlKDIJAJAkLuTGCXSgUB+wgIuRNibLoX2+Rq56hN4Se4TE\/TldPBEFqas3WMmMoPuyYsKu49Jb2QO2FPZUluVN6Ojo5IOwfLjdy2TwHiAEgv5+RtI46Q25DcfAMIWRo8otrQ0OCVQOee1Tvl0KIOGzYM6BIMyksns+hOuSDLy8skK4QXalDdfpaJ7zSk89tE7v79+wPWqVpNnodvYsHDQ\/v374ddu3YV7r\/jm49Gjx4NWCZtUda1WSWXOtBgHTfeeCPs2LHDuzDE7yy56gUFDVhCbkMFz2sy9VA939HFLbe644hv38S7wdVnlxBPJAMq59y5c7UnsMj1Xr16tUdEPM2FpKN8fpaPKzw\/SovPROGgQMRWy6NjofQ0FLVRPRnIZa2urvYGL3qVhv82aNCgojZzHdKRm841UJkop3qrrpDbjIliuQ1wCroxI8gt58rLyY1VIhlIcemope58tUoAfgAk6HILv3vv1JNeQe3nv2F5RHT8V83H\/1vdk+5nWXXkDqqDukrIbaC0ACDkNsMJePCKu8GqkiMJNmzYUCiV0urIrd5Vp7v6SCWKyUEbv0cfsFEqoXj7dRd1kGusDhZBZFcPHmG9uqCZjtx1dXWFM+Z+A4+Q20xphdxmOBWlUq1UV1dXwU3mbm2Q5Ta9ccaG5eaufJDFVS13EPH8MAmCN8xyqwOIn+UOOr0mc+4YCp6nLKql8Jtz685MI074mEPQnJvPq3XzSzylFXXOrZ6Tp2kAtseE3GjF+Txatdymc248Oup3k4+O3Pg3vApKnbpwfdPFIQhnNWgn5M4TU2PKyl1N7pZTVBjd19mzZ3vBIwwKYdBr4cKF3jPI+HY5KSv+y+8xM3mBxS\/yHLasxacIarScXo\/hFpeuucLpArrR06dPh+3bt3uD09q1a4Fbbpp3NzY2emnxYbuLFy9qo+V+69g6cn\/xxRewc+dO78gpn6bo5vhYN+KMg9CHH36oHUSF3DEVXrIJAohA0Bw\/qluuDiBJERZyJ0VQ8ucOAXU9P8795lQGWXa8ETRNclP5skMtd+opAgsClY3A\/wHNnj\/I+i0fqAAAAABJRU5ErkJggg==","height":149,"width":247}}
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
