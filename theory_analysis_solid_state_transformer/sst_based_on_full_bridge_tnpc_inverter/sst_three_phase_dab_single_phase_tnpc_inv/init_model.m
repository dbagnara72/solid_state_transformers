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
Ls = (Vdab1_dc_nom^2/fPWM_DAB/Pnom/3) %[output:0ce1dfb1]
f0 = fPWM_DAB/5;
Cs = 1/Ls/(2*pi*f0)^2 %[output:8ab657aa]

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
%[output:0ce1dfb1]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     2.840909090909091e-05"}}
%---
%[output:8ab657aa]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     3.869906319672623e-05"}}
%---
%[output:540d1256]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.919183588453085e+02"}}
%---
%[output:96e25287]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:82e29d43]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["3.451320585750986e-01"],["8.145586092350321e+01"]]}}
%---
%[output:8d092f26]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Afht","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-1.421223033756867e+05","-1.884955592153876e+01"]]}}
%---
%[output:17f14063]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Lfht","rows":2,"type":"double","value":[["1.866106036232337e+03"],["3.911916400415777e+05"]]}}
%---
%[output:2cc3b7d6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000e+00","2.500000000000000e-04"],["-3.553057584392168e+01","9.952876110196153e-01"]]}}
%---
%[output:012657e6]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["4.665265090580843e-01"],["9.779791001039442e+01"]]}}
%---
%[output:78b0eed0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["9.111998627203048e-02"],["4.708907792220440e+00"]]}}
%---
%[output:0b49bf74]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjcAAAFVCAYAAAAT\/M5iAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQm4FcWVxw+76EMWQZ8sCnHB5wa4JlGJKyZuQUcREVAJsjwkcZBNgzEmRAdBIqIswiAiyjKoiFtAEw0IBMQBdBQ0IGgQQVBWIUCU+U6RevZr+t6uqu5zb99+\/\/4+P5Bbdbr7d+pW\/e+pU1WV9u\/fv59wgQAIgAAIgAAIgEBKCFSCuEmJJ\/EaIAACIAACIAACigDEDRoCCIAACIAACIBAqghA3KTKnXgZEAABEAABEAABiBu0ARAAARAAARAAgVQRgLhJlTvxMiAAAiAAAiAAAhA3aAMg8G8Cc+bMoR49elCtWrVo0qRJ1KJFi5yz2blzJ3Xt2pUWL15Mbdu2peHDh5c9Az\/f4MGDafr06VRcXJzTZ1u+fDl17tyZduzYQWPGjKE2bdqo++vnbd26NZWWlub0mcJu1qdPH5o5c2a55w2roz+P471GjRpFc+fOpfHjx1NRUZHprWMr5\/UZG23cuHGktpOtbcb20DkwpLmUlJTkzTc5eM0KfwuImwrfBABAE0iyuOGBctiwYZEHKFdvB4mbDRs2ULt27WjdunXUt2\/fRIkb\/by1a9e2HtDjeC8trM4555y8DKBeIaJ9HvVZ0iJuvO\/hFequ3w3USyYBiJtk+gVPBQLlCEDc2DUIzcsf\/TKxkjZx48IgiFNaxA2\/W5T2YdKGUCb\/BCBu8u+DgngCHdXQDxs0deP9tXrjjTfSXXfdVfZumX7Z6zq6oL+cv0P96U9\/qqaOMpXPBNM7YGWqGxS58b5Tq1ataOzYsaq69zl1R6nt+sP\/mYSJl6n+Bel\/39\/97ndl01Ted8v0KzxTxMD7\/v5fqya+9Udu+Fm8ftDP5rXt9y2XCfql7J8+4TKrVq0KjFSZTLXYvCs\/k3fw97Owfa+gdua\/h8k7ZOsUwvzlt2\/6XQmqFzQFqadMTb6L\/u+G\/7vD\/2\/yHfO2JW77DzzwAN16662BUcOwPoXvqd+V\/56vKeiC6PgL+CEhbgrYebl69KBBKqjDzFbO38EHhc21Te9gk61clE476F7ZxI2XtVfYZXpnb5lciptMU2v63\/3Cy9S3tuImm13vgJlJTAQJxUxl\/UI7jEHQ90a3uTBxE\/Zep59+etlUnfc+YfZN87xM\/OUibrL5QQt5k++i17dBwsa039A8jjvuuEBx72Vr8nz+6FUc0blc9b+4jxsBiBs3bhWmlrfT80YrdCebaaD3diZBZXXH560fNCj5O1TdeXo78Gy5BN763oE96JnCxI0\/qhTExvtcmkEUcaMTik2npYI67UzTCTa+tcm5CfpV7H0uzSWTb7zPpX3GXzid3xNU39veMrEKimoFtcNMA5\/pe\/mjETqhOIxB2PSRjb9sppC8z6W\/S\/wOOrFd+4CTovW\/8ef6uxj0XkHRM+8zeb+zXsFm8h3z9wm6jmmfws9uw6fCdPYpe1GIm5Q5NO7XMZnm0J2LLuuPDvgHC151E7QiyNvhBP0a84sYk6TNTKt8glYeZRM3QStNgu4ftMoml+ImaGBdvXp14EonG9\/aiBt\/G\/T\/gteDuNemf1Dzt6X33nsvcCVbUEQq03t5B9FsQsL0V32m98okbsIiSmGrmWz8ZTN4Z3ou\/2qvTOIk0\/t624E\/MhQkbrJ9x\/yf+duOTZ+in8uk\/4i7P4W93BGAuMkd64K7U7YOMuizTJ2Fv2z\/\/v0DQ\/dBYfxsz2DSOWUSN0HOCMu58S\/pNbk\/3yfX4sYfYZg\/f\/5B+Su2vrUVN9mmJILEjT8Xx8\/s+eefV++Q6QqaxvALmKDpmqDpoGzixuS9Mg322epynWxTU7b+ikPc+FnbfhezTXUFiZugCKypoLvuuuuM+xT9XqbR0ILruPHAigDEDRpCRgK2HSrETfBeJrkWN16\/de\/enZYuXXrQvjm2vrURN95BTQ\/YRx111EHTStmEp4S40Q09aND1RgYyiRvT94K4GU\/eaCHzYPH64x\/\/uCxiC3GDgUeaAMSNNOECt2\/6y4k3cIs6LRWEyvbXot9GpqkP\/e9Dhw4t25DONXITlKS7fv36sv1Nci1uvNGiRo0a0fbt2xUW\/6oQG9\/aiBv9vt4BLCgvI45pKZvoQlD7CnqGTOLG9L0yiZtM0z+mXYSNv1wiN1qE6A0a+Xn79etX1m5svosLFixQ04je70ZYzk22yI3rtFQ2tkH+NPUFyiWfAMRN8n2U1ye0SWLMlNNgmlAcNIDadKhBu8BmSlr1ThHoKRFbcRPEJig5U3f07EidW+JfMpxpKbhtQrFuLP4pmKCBw8a3LuImaMUYP19cCcWZRES2XChexhwmusLETdh7ZXquIIGXqWzQl97GXzbiJqjN8nfJ\/731rlzyT\/n5mXvbvP\/7xe9mGrkJemebhOJs0UHTaeW8dsC4uTMBiBtndBWnYtjyV\/1LL1s576DGf8+0H4i\/44sqbthepqWx\/nvZihvvwBTUGoJWdmVqNWHiJltCZpDNTAOAv6ypb8OEp7bL78FTUPqohqBnM9lX5qSTTqKVK1eW++WfLWclaAmy\/9d+thwQr2Dxs9MRDZv3ypRsbPoOmdqJqb9sxA3fKxubsNVpQQKN769XtwW9i6m4CfIF29MRST4OJNMPBu99\/eLelk\/F6e3T86YQN+nxpeib+Du\/sE38fvnLX1LPnj3VWUR8mW7i5\/9FGIe4ySSmgs5u8p8tZfLrzj9gBbEJiqR4NzoMEzf+Tj5sZY13wArbQ8XEt9lWnQVtqui3aboxn37WoCToIKGajTWX90\/FBYnqIJb+59ft1\/S9\/PfxDq7+thDmH\/8X28RfLoN30I8A7\/fW9rvot8e2jj\/++INWvZl8x\/xRYe+ihEwr7TS3oJVx2TZ6FO1IYTxnBCBucoY6\/Tcy6aTSTwFvGAeBKCtZMHDF4YFk2zBdrp\/pLYL2LEr2G+PpbAmkQtxwRzh16tTQA\/IyTU9EPVDOFnpay0PcpNWzMu\/lHaAyJZ6GbWyX6cm0OHKtL\/PGsGpLwBsV9UZooiZno33YeqLwyhe8uNGCxeT0X3\/2f+G5K9lPDHGTbP8k8enC8rRcT2226ReSyAXPdIBAtnw5\/tzlh6l3es21fcE\/ySdQ0OIm00qYbL\/m5s6dW7ZEN\/nuKawnhLgpLH8l5WmDkmxt81CC3kW3RwxgSfG023OYHEZqY1kLppKSEowFNuAKrGxBixu9PTif1vzKK6+ETktxZ8eXXl5bYL7C44IACIAACIAACBgQKFhx451i4pUVYTk3Wv3z\/g28xFRfmJM3aCUoAgIgAAIgAAIFRKAgxY0WKu3btyfeGdckoViHIi+55JKyyI2207Bhw6zhyXXr1hWQS\/GoIAACIAACIGBGgJP503gVpLjh6SX\/9vZhkZtMztNZ95nm5VnY8BbkixYtSqP\/8U4gAAIgAAIVmMC5555LfAxN2kROwYmboBVPJpGbTG1XR3T4gEGOAvmvv\/3tb9ShQwflfN4VE1e8BFg0jhgxAnzjxVpmDXyFwP7bLPiCrywBWeu6\/fJCG4gbWdah1sOWjmbaCTequEmj80Nh56CAFo\/gKwMbfGW4aqvgC76yBGStp7n9FlzkJsjVJpGbTNNPYXvfpNn5sl8bM+vga8bJtRT4upIzqwe+ZpxcS4GvKzmzemnmW2HEjd4Th3N1pk+fTsXFxWUbRHmTjP1NIs3ON2v+sqXWrl1LEydOpEGDBlHVqlVlb1YBrYOvrNPBF3xlCchaT\/P4llpxw9GcsWPHHnRwnn9aK2waK83Ol\/3amFn\/5z\/\/SV988QU1adIE4sYMmVUp8LXCZV0YfK2RWVUAXytc1oXTPL6lQtxYe9SiQpqdb4FBrCg6LzG0yjD4gq8sAVnraL+yfNM8vkHchLSdNDtf9mtjZh2dlxkn11Lg60rOrB74mnFyLQW+ruTM6qV5fIO4gbgx+xYIlULnJQT232bBF3xlCchaR\/uV5QtxI8s30dbT7PwkgEfnJesF8AVfWQKy1tF+ZfmmeXxD5AaRG9lvT4h1dF6y+MEXfGUJyFpH+5XlC3EjyzfR1tPs\/CSAR+cl6wXwBV9ZArLW0X5l+aZ5fEPkBpEb2W8PIjfgm1cCsjfH4Au+sgRkrUPcyPJNtPU0Oz8J4DE4yHoBfMFXloCsdbRfWb5pHt8QuUHkRvbbg8gN+OaVgOzNMfiCrywBWesQN7J8E209zc5PAngMDrJeAF\/wlSUgax3tV5Zvmsc3RG4QuZH99iByA755JSB7cwy+4CtLQNY6xI0s30RbT7PzkwAeg4OsF8AXfGUJyFpH+5Xlm+bxDZEbRG5kvz2I3IBvXgnI3hyDL\/jKEpC1DnEjyzfR1tPs\/CSAx+Ag6wXwBV9ZArLW0X5l+aZ5fEPkBpEb2W8PIjfgm1cCsjfH4Au+sgRkrUPcyPJNtPU0Oz8J4DE4yHoBfMFXloCsdbRfWb5pHt8QuUHkRvbbg8gN+OaVgOzNMfiCrywBWesQN7J8E209zc5PAngMDrJeAF\/wlSUgax3tV5Zvmsc3RG4QuZH99iByA755JSB7cwy+4CtLQNY6xI0s30RbT7PzkwAeg4OsF8AXfGUJyFpH+5Xlm+bxDZEbRG5kvz2I3IBvXgnI3hyDL\/jKEpC1DnEjyzfR1tPs\/CSAx+Ag6wXwBV9ZArLW0X5l+aZ5fEPkBpEb2W8PIjfgm1cCsjfH4Au+sgRkrUPcyPJNtPU0Oz8J4DE4yHoBfMFXloCsdbRfWb5pHt8QuUHkRvbbg8gN+OaVgOzNMfiCrywBWesQN7J8E209zc5PAngMDrJeAF\/wlSUgax3tV5Zvmsc3RG4QuZH99iByA755JSB7cwy+4CtLQNY6xI0s30RbT7PzkwAeg4OsF8AXfGUJyFpH+5Xlm+bxDZEbRG5kvz2I3IBvXgnI3hyDL\/jKEpC1DnEjyzfR1tPs\/CSAx+Ag6wXwBV9ZArLW0X5l+aZ5fEPkBpEb2W8PIjfgm1cCsjfH4Au+sgRkrUPcyPJNtPU0Oz8J4DE4yHoBfMFXloCsdbRfWb5pHt8QuUHkRvbbg8gN+OaVgOzNMfiCrywBWesQN7J8E209zc5PAngMDrJeAF\/wlSUgax3tV5Zvmsc3RG4QuZH99iByA755JSB7cwy+4CtLQNY6xI0F323bttHo0aOJ\/3S9ateuTQMHDnStHmu9NDs\/VlCOxjA4OIIzrAa+hqAci4GvIzjDauBrCMqxWJrHt9gjNxs2bKB27drRunXrHHETNW7cmObOnetcP86KaXZ+nJxcbaHzciVnVg98zTi5lgJfV3Jm9cDXjJNrqTSPb2LiZtCgQdSmTRtr5nPmzKHBgwdD3FiTK8wK6Lxk\/Qa+4CtLQNY62q8sX4gbC76bNm2iO+64Q\/13wQUXWNQ8UHTevHn02GOP0bRp06zrSlRIs\/MleNnaROdlS8yuPPja8bItDb62xOzKg68dL9vSaR7fYo\/cfPvtt8T\/Va9e3ZZzIsun2flJAI7OS9YL4Au+sgRkraP9yvJN8\/gWu7jhnJsOHTpQkyZNqLS0lM466yyqUqWKrIcErafZ+YLYjE2j8zJG5VQQfJ2wGVcCX2NUTgXB1wmbcaU0j2+xixteJfX73\/+eXn75Zdq7dy81aNCAbr31Vmrfvj3VrVvXGHpSCqbZ+UlgjM5L1gvgC76yBGSto\/3K8k3z+Ba7uNGu2LlzJ7355ps0YcIEeu+999Q\/n3766dSzZ0+68MILC2baKs3Ol\/3amFlH52XGybUU+LqSM6sHvmacXEuBrys5s3ppHt\/ExI0XLScZz5gxgyZOnEj896KiIrrhhhuoS5cu1KhRIzMv5KlUmp2fJ6TlbovOS9YL4Au+sgRkraP9yvJN8\/iWE3Gj3bN\/\/376+OOPady4cfSnP\/2Jdu3aRc2aNVPRnGuuuSaR0Zw0O1\/2a2NmHZ2XGSfXUuDrSs6sHviacXItBb6u5MLrDZm9hqa8s4G2T7hZbb3C+8ul6cqpuPGC43ychQsX0r333qv+efr06VRcXJw4thA3si5B5wW+sgRkraP9gq8sATnrvaasUOKmzsxfQNzEgVmLGp6iYnHDy8Z5s78HHniA+NiFpF0QN7IeweAAvrIEZK2j\/YKvLAE56xA3MbBlAbNs2TKaMmVK2XRUoayigriJoQFkMYHBAXxlCchaR\/sFX1kCctavfnwpzV+9FZEbW8ScX8PnS3GEZtasWfTVV1+pnJorrriCunfvTieeeCJVqlTJ1mzOy0PcyCLH4AC+sgRkraP9gq8sATnrEDeWbHfv3k1PP\/00TZo0idavX68ETElJidrr5qc\/\/alaKVVIF8SNrLcwOICvLAFZ62i\/4CtLQM46xI0lW30q+NatWwtmuXe2V4S4sWwAlsUxOFgCsywOvpbALIuDryUwy+LgawnMonjLwQvps6\/\/SYfPGUBv\/+l5rJYKY8eRm7Vr19Jxxx2XyKXdYc\/v\/xzixpaYXXl0Xna8bEuDry0xu\/Lga8fLtjT42hIzL6\/FDVZLGTLTkZtBgwapVVC215w5c2jw4MFqaVoSLogbWS+g8wJfWQKy1tF+wVeWgJx1iBtLtoUkbkaNGkVTp07NuscOxI1lA7AsjsHBEphlcfC1BGZZHHwtgVkWB19LYBbF6\/V5U5VG5MYQmhY3vFLK9eKdEqUjN8uXL6fOnTurvXWybSAIcePqRbN66LzMOLmWAl9Xcmb1wNeMk2sp8HUlF14P4iacUbkSfCr46NGjif90vVhwDBw40LV6aD0+1LNr1660ePFilUQFcROKTKwAOi8xtMow+IKvLAFZ62i\/Mnw5kZinpRC5keGbN6s8HcWRoVatWtErr7wCcZM3T2DwlUaPwUGWMPiCrywBGetvr9pK14xaCnEjgzc\/VjlhuV+\/fmofnvnz5yPnJj9uKLsrBgdZB4Av+MoSkLWO9ivDF+JGhmverOp8oPbt21NpaSnZJBQ\/++yz5fYBSOIhn3kDG+HG6LwiwDOoCr4GkCIUAd8I8Ayqgq8BJIsiPAbyNe+LqsRnS\/GFhGILgEkt2qdPH7Vr8vjx49VOyTbixv9OnIx8yy23JPVVC+a59uzZQ5s2bVInwletWrVgnrtQHhR8ZT0FvuArSyBe60899ZSatdh5fn\/6V\/3mEDfx4s2PNe90VIsWLdRD2IiboUOHUqNGjcoengdjRG+i+5I3fdy4caOKikHcROfptwC+8TP1WgRf8JUlEK91jtzwf6WzNtHafx44CgmRm3gZ59waR21mzpyZ8b59+\/ZVU1X+C0vBZV2FsDP4yhKQtY72C76yBOK37l0pxdZx\/EJExnxK+JYtW+jbb7+levXqUeXKlWnfvn15PaLBJnLDq6s4uoArXgIYHOLl6bcGvuArS0DWOtpv\/Hy9ycSI3ETgy6Lm9ddfp3vvvVflVuh9ZWrUqEG\/+MUv6KyzziKOmlSvXj3CXdyqQty4cYuzFjqvOGkebAt8wVeWgKx1tN\/4+f5i0gf0wrIvleHKuzaryE0af7xX2s\/qQ\/D661\/\/St27d6czzjiDTjjhBHrzzTfVvjKc0Hv33XfT7Nmzic+h4gTdXF8QN7kmjsE318QxOMgSB1\/wlSUQv3W9MzFbPq\/WBvrg6V9D3Nhi5pUEPXr0oO+++47GjBlD8+bNU4di6h2B9+7dS5wLs3nz5rIVTLb3kC6PnBtZwhgcwFeWgKx1tF\/wlSUQr3V\/vs2vz9xDj99bCnFji1nvK9OtWzfq2LEj6RO\/vccdTJ48mZ544omsuwTb3jfO8hA3cdJE5EaWJviCb64JyN4P4jE+vixseFdi\/lNFbY6rQyxuOnToAHFji1mLm06dOtHtt98eKG54amjGjBk0bdo0atCgge0txMtD3MgiRucFvrIEZK2j\/YKvLIH4rE95Z0PZxn1sdVZpK6q6eSXEjQtiPS21a9cuGjduHLFQ8E5L8cnht956KzVp0kRNW3GScdIuiBtZj2BwAF9ZArLW0X7BV5ZAPNb901HH1DuElg36kRqTEblxZLx06VLiaSlOJub\/Xn31Vbrzzjtp1apV9NxzzxHn3YwdO5Z+8pOfON5BtlqanS9Lzsw6BgczTq6lwNeVnFk98DXj5FoKfF3JfV\/PPx3Fwuax9iV0\/vF1IG6i4l2wYIFaCr5mzZpypnga6g9\/+ANdeumlUW8hVh\/iRgytMozOC3xlCchaR\/sFX1kC0az7hQ1be\/ymErrp7GJlOM3jm\/hScO0aXnHOq6I+\/PBD4umqU045RR1fUKVKlWjeE66dZucLozMyj8HBCJNzIfB1RmdUEXyNMDkXAl9ndCpx2JtAzJZuPudoGtn+pDKjaR7fciZu3F2U35ppdn5+yR64OzovWS+AL\/jKEpC1jvbrxpd3Ib5j6oqylVFsRefZeC2meXwTFTfbtm2j0aNHE\/8ZdtWvX19NT5166qmJiuak2flhPsnF5+i8ZCmDL\/jKEpC1jvZrz\/eNFV9Ru3HvlasYJGy4QJrHN1Fxw8ct3HHHHfTBBx8Qr5jii0UMXzxFFXRdcMEFNHLkSDr88MPtvSpQI83OF8BlbRKdlzUyqwrga4XLujD4WiOzqgC+5riCojVce0LnU6htyyMDDaV5fBMVN0xz8eLF1KtXL3W8wm233aaOXeCLV0n9z\/\/8D\/E+N4899hidfPLJNGvWLLVUnMvedddd5l4VLJlm5wtiMzaNzssYlVNB8HXCZlwJfI1RORUE33BsnFvTa8oKmr96a7nC3lVRmaykeXwTFTc7d+6krl270oknnkj3338\/VapUqRxjTjK+77776NNPP1X73NSsWZOGDBmiQmUvvPBCuFdzUCLNzs8BvtBboPMKRRSpAPhGwhdaGXxDEUUqAL6Z8XGkZsjsNQeJGq7BwoY36eM\/s11pHt9ExY3eoZgjNzfeeGMgY96Z+PHHHy87fuH555+nRx55RG0HnYQrzc5PAl90XrJeAF\/wlSUgax3t92C+maaftKjRe9iYeCbN45uouNmyZQt16dKFjjvuOHrggQeoevXq5Xjz1NQ999xDq1evpgkTJlDdunVVvg1v9Pfaa6+Z+Ea8TJqdLw7P4AbovAwgRSgCvhHgGVQFXwNIEYqA7wF4LGimvPMF8REKQZfJFFRQvTSPb6LihmHyoZjDhg2jG264QSUX8942fHFUh3Ntpk6dqvJrevbsqfJz7r77brUHDoucJFxpdn4S+KLzkvUC+IKvLAFZ6xW5\/bKgeX7pRpq4cH1GyK6iRhtM8\/gmLm44OsPi5sknn6Rvv\/22nJN4Az9OMu7bt6\/a74QP11y\/fr0SRCUlJbLfGkPraXa+IQLRYhW58xIF+2\/j4CtLGXzBN04CYREavhcLmiZ1D1E7DYfl1IQ9W5rHN3Fxo+HysvA\/\/\/nPtHz5cvVPHJ256KKLqFGjRur\/d+\/eraI5DRs2TNQBmml2fljDz8XnGBxkKYMv+MoSkLWe9vbLK53m\/n0LTVuyITAx2EtXR2n4z6iiBpEb2XZbENYhbmTdlPbOS5ZeuHXwDWcUpQT4RqEXXjdtfFnM8H+ZVjn5ibCIGXZ9czrxyENjEzTee6R5fBOP3PBy708++YRmzpwZuHEfnzP1+eef04gRI8ryccKbfO5KpNn5uaOY+U5p67ySwNT7DOAr6xHwBd9sBFjIrN60i4a\/8WloZEbbMV3GHQf5NI9v4uKGVz716dNHbdoXdPEKqrPOOkslEPNqqaRdaXZ+ElhjcJD1AviCrywBWeuF1H5ZyPA1dt46em\/dDisxc83pDajNyfXp\/OPryAL1WU\/z+CYqbr755hvq3r27ShJ+9NFH6dhjj1VJw61atVK7Fk+ZMoUmTpyoNvA77bTTcupU05ul2fmmDCTLFVLnJclByjb4SpE9YBd8KyZfPb30yv9tov\/7fKexkGFaHJk577g6dNPZR6u\/x5U\/4+KJNI9vouJGb+LHy8B79+6t2P\/2t7+lL7\/8Uk1D8cVRnRo1atDQoUMP2sHYxVlx10mz8+Nm5WIPg4MLNfM64GvOyqUk+LpQM6+TBL46IjPiL5\/Sxxt3WQkZLWYuLTmCfnnRMerF8ylm\/OTTPL7lRNzceeeddN111ymukydPppdeeonGjRunDsfk\/3\/mmWfo6aefLjtU07zpy5dMs\/Pl6YXfIQmdV\/hTFm4J8JX1Hfimiy8vxf7nvm9pxF8+sxYxWrjwMu1rWx5JLGiSJGSCPJXm8U1U3Gzfvl1NQ51xxhk0YMAAxfaNN95Q0RsWNU2bNlUih4XN9OnTkVAs208k0joGB1m3gC\/4yhKQtS7RfvWU0lff7KXxb39O\/9hyYAWT7aWFS782TenYejXzPsVk+\/xcHuLGhdq\/6zz88MNqA7\/+\/ftT+\/btVf5Nx44d1Y7FP\/vZz2jgwIHEK6r08QsRbiVSNc3OFwFmaVSi87J8hFQXB19Z94JvcvlqwTJz2Zf0+oqvnCIx+u30xnm9LzqGDq1epSCFDCI3MbdVjt5wvg3\/yQKmTp06NHjwYJVIzKKGTwrnk8E7d+4c853jMQdxEw\/HTFYwOICvLAFZ62i\/+eWrBczKjd\/QrOWb6NOvdkcWMfxGFzevR9e1Oio1IiaTl9I8volOS2mgLGJ27NhBtWrVUmKGj2FYsGABzZs3jy6\/\/HI1bcX\/nsQrzc5PAm8MDrJeAF\/wlSUga1233\/2HNaA1X++lF5d\/SWs2RxMw\/MR6SonzYtq2OFK9RK6XYcuSM7Oe5vEtJ+ImG2YWOtu2baPatWsTnzWVtCvNzk8Cawy+sl4AX\/CVJRCPdR2BmbfqwFEE+\/dTpAiMfio9nXRR83p0TtPaZaIm6Ym+8VANt5Lm8U1U3Oil4IMGDaI2bdoEkn7llVdoyJAhSCgOb4epLIHBV9at4Au+sgTMrGvx8t3+\/fToXz6jv39pv6Q60520ULnwxHr08xYNqFqVyhUyCmPmifKlIG4sqPFxCjzltGvXLtq6dSs98sgjdO2111KLFi0OssJlOQ+Hy06bNo0aNGhgcafcFE2z83NDMPsubfPQAAAgAElEQVRdMPjKegF8wVeWwAHrWrzsJ6InF3xO73663XkVUtDzegXM6Y2K6MSjDkt9Pkwu\/Jbm8U0kcjNp0iS6\/\/77VcJw2MVTUf369VNLxpOYd5Nm54f5JhefY\/CVpQy+4BsXAd4D5ojDqtG4t9epyIvrEupsERjeI6Zp\/Zr0o2Z1VPSF22+1PVuoSZMmVLVq1bheBXb+TSDN45uIuOFzpDiPZuPGjdSjRw\/61a9+RRdccMFBDYrPleLzpJIoavTDptn5SfiGY\/CV9QL4gm8YARYtOjIyZ8VX9NLyTcTTR\/NXbw2ravW5vgf\/eW3Lo+gEz0nXmXJg0H6tEFsXTvP4JiJuNGFOFv7666\/VTsR8xEIhXml2fhL8gc5L1gvgC74sXjjKUlSjCs358Cta+9Xu2KMuTFkLFI6+XNOiAZUUFyn4UVYhof3Ktt80j2+xixstaPhP04unpurVq4fVUqbAUlQOnZesM8E3vXy9u+q+9\/kO+uvHW2jlhm9EhItfvHQ4u5ia1Kup4Eoe\/oj2K9t+IW4s+OoVUuvWrTOu1bhxY6yWMqaVroLovGT9Cb6Fy1dPF839+xZa+MlWlbQbd56Ll45eNt2sfk26\/oyjqHKlSnlfOo32K9t+IW4s+O7evZvmz59PvBLK9OIpq\/POO49q1jzwSyBJV5qdnwTO6LxkvQC+yeLrjbbsp\/00ccF6WvLpdvWQcee4+IUL\/\/8Fx9eli0+qRw2KqudduJh4Bu3XhJJ7mTSPb7FPS7ljTmbNNDs\/CcTRecl6AXxzy1cfyrh+6x6avHi92oxOMtrCb6dzXc47rg7dcGYxVa1cSeW58LMU+mZ1aL+y7TfN41vOxM2WLVvo7bffpnfffZd4lVSrVq3ohz\/8oVotleQrzc5PAnd0XrJeAN94+OZ6ikg\/tZ4qOr7BoXT+CXXoqFo1ygmWQhcvYd5B+w0jFO3zNI9v4uKGE4ufeOIJGj58uDpTyntxInGvXr2otLRUCZ4kXml2fhJ4o\/OS9QL4ZubrnSJa+o8dNOfDzeJ5LV7Rwn\/nlUWnNSqiHq2blIu0pF20mLZ6tF9TUm7l0jy+iYubGTNm0IABA6h169Z01113UdOmTZUX1q5dSw8\/\/LDazXjo0KF0zTXXuHlHuFaanS+Mzsg8Oi8jTM6FKgpfPQWjp4VYHPAWonxO0durtuRkesg7RcSi5aTiw1R+yylHFynhEmVJtHMDKPCKFaX95stNaR7fRMXNzp07qWvXrlRUVEQjR448KGGYk4979+6tIjpjxoxJ5F44aXZ+vr5Q3vui85L1Qlr4ekXLsnU7iFcQfSS47NnvFe8eLj+oX1MdwnjsETXpyEOJNm3aREcffTT94MgD+7rgio9AWtpvfETitZTm8U1U3Ohl4d26daOOHTsGemXy5Mlq2mr69OlUXFwcr+disJZm58eAJ7IJdF6REWY1kGS+XsHy6de7af6qrWUrhqSTcDU0r2g59ohD6OSji+iq0xoYTxElma9sy8qNdfCV5Zzm8U1U3PAvmhtvvFEdnMkRmqCLIzovvPACDs6UbcOJtY7OS9Y1uebrzWPhv\/Pmch+s35mzXJYg0XL8kYdS6U+a0Mbte2Nf\/pxrvrKtJXnWwVfWJxA3jnz37dtHffr0oRUrVtC4ceOoWbNm5SytWbNGHZhZUlKiEo6rVavmeCe5aml2vhw1c8vovMxZuZSMg69XsPAzzP7wK\/pw\/U5atSn+wxOzvaM3ysJ\/b9G4loq06CsfOS1x8HXxa0WpA76ynk7z+CYauWG3vP\/++9SlSxe1qd9ll12mNuvjizf6e\/3111WeDU9L8dLwJF5pdn4SeKPzkvVCJr7eKSF+gpff36S27l+zWebcIVPR0rJJLbqoeT2qXqWyquI9bFGWlJt1tF83bqa1wNeUlFu5NI9v4uKGkX\/00Ud0zz330LJly2g\/72pFpE4Cb9myJT3wwAPUvHlzN8\/koFaanZ8DfKG3QOcViii0QNBKoW+\/209Tl2xQ2\/bv3v1P2rSb1NRQri5vlIWnhS48sS4dcdiB7R6SLlhsGKH92tCyLwu+9sxsaqR5fMuJuNGwOXrDm\/nxxZv3FcJJ4Wl2vs2XQKosOq9gsn7BwqUOqVaZpr6zgT7a+I2qlKukW\/2E\/hVDfP4Q\/0hJm2Cxaetovza07MuCrz0zmxppHt9ExQ0nFE+YMEHtYXPiiScm8tTvsIaQZueHvXsuPq+InZeeEmpctwa99N6B6SDpQxEz+dIfYWl3ZjFx1CdN0RXJdlwR268kT79t8JWlnebxTVzc8Gop3rDviCOOUCLn1ltvJT4FXP\/ik3VddOtpdn50OtEtFHrn5V8d9N3+\/TTrvU1qD5Z8RFe8URTeSK5Zvep01QlVqXrREWX7sGD32+jtVlso9PYbHwkZS+Arw1VbTfP4JipuGODevXtp4cKFNHHiRPUn\/z+vmtJLxBs0aCDrvYjW0+z8iGhiqZ6Uzst7yKBXsPCKIM5bWbRmW852ufWD9UZX+FDE0xrVojYnH6GKhUVYksI3lsaSQCPgK+sU8JXlm+bxTVzceF3jFzq8VJyXgfNqqquuuiqR50ul2fmyXxsz65KdV1DeSpN6h9DL731JKzfsorVf5X5lkKbiFSwlRx9GjescQmccc3gZtDDRYkaXSJKv6TOkuRz4ynoXfGX5pnl8y6m48bpp+\/bt9OSTT6qITq1atbBDsWwbTqx1187Lu5SZIysLPtlKn\/CSoDwk2nojKDwVxMKEzxU64rBqdEy9muUiLN6yuXCKK99cPFsa7gG+sl4EX1m+EDcx8eWzpt58800lZBgqX6effrqK3PAeOEk8GTzNzo\/JrZHMeDuv9dv\/pRJrG9WpQS8s+5Le\/OjrvIkVv2A5t1ltOrvp4VRS\/P0hiHr6Ksk5LBgcIjXP0MrgG4ooUgHwjYQvtHKaxzfxyI1f0PAhmZxzc9ttt9GVV16ploS7XLzz8cyZM8uq8sGbbdq0yWpq+fLl1LlzZ9qxY0e5cueccw6NHz9eHfDpv9LsfBfuLnXeXrVVRTNYrCxeuy0RK4P40MN2Zx5FlStVUs+W1lObMTi4tFjzOuBrzsqlJPi6UDOvk+bxTVTc6IMz161bR5w43KlTJ3XOVKNGjczpB5RkYbNkyZKyqaw5c+ZQjx49qG\/fvlRaWprRNpfr168fTZo0iVq0aGH0DGl2vhGALIV05OKzr3fTq\/\/3Fb3\/+Y6c7r3i33fliKLqdHHzemVPHFfeSlRO+ayPwUGWPviCrywBWetpHt9ExQ1v2DdlyhS6\/PLL6Qc\/+EEsy7919GXo0KHlIjUseNavX58xAsNNZNSoUTR37tysZRC5+Z6Ad9XQpL+tVyuGpDeO8wqWUxsWqagKrw7yT\/0keSpItjuys47B146XbWnwtSVmVx587XjZloa4sSWWh\/Im4obL8MWHdJpeaXa+l4FO0J3yzhdqimb+6q2miELLeQXLcQ0OpbOOPZyaHlFTCRbuvHizx6OPPrpsH5ZQgyhgTACDgzEqp4Lg64TNuBL4GqNyKpjm8U00cuNE26ESR2SGDRtG2fJu9BQZ59WsXLmy7C5t27bNKnbS6nzOg5m57Et6Y+VXkc4c8k79nHVMbbr1xw2VPdMpIXReDg3eogr4WsByKAq+DtAsqoCvBSyHomkd3xhFQYsbnWvDLxImUvR01iWXXFImZrTgadiwYWhC8bPPPqt2VtZXcXGxQ1PKXxUWM7NXbKGx89ZZP0TDw6sSL3G+6rQGdErDA0nXLF7imBpC52XtDqsK4GuFy7ow+Fojs6oAvla4QgvzmOe9OB+2Q4cOKl3DO76FGiqAAgUtbjRfXpHVtWtXlXPDy8xthIcWSJmiPlrZ+n3Jq65uueWWxLqYl1V\/seNfNHbRVnr3c7PToFnEHF2rKrU54TBqVq+a+jv\/m+TFh6nytBT7rGpV2XtJvkdSbYOvrGfAF3xlCcRr\/amnnlILavwXxE28nGO1piMz3bt3z7piyn\/TsHpa3HACs3eVFw\/GNiIq1pcNMdb\/uY9p8rtfZS3FUReOxvznxY3LlkNLC5mgB9q9ezdt3LhR\/WqAuIm\/lYBv\/Ey9FsEXfGUJxGudIzfe6M2iRYtoxIgRiNzYYt62bRuNHj1aHa1w6qmnBlZ\/99136ZFHHlFTRVHOmQoTKZmePaxeIcxJ6mTgIbPXZE0EZkHTr01Tuvmco21dKVYeYWcxtMow+IKvLAFZ62i\/snwLYXxzJSA6LaVzWgYNGhS4wR5v6MeJwK+88orxdFKmvWrCppcyfR62903Snc+5NHdMXZExKZgFzWPtS+j84+u4thHReui8RPFC3MjiBV\/wFSYgaz7p41uUt49d3PDhmPfccw89\/\/zzxs918cUX08iRI6lmzQPn8GS7dH4Nl9G7CuvoCx\/CmWmn4aC8nKAkY\/+9k+p8jtb0mrIiMFLDgmZEu5OoWf0Dy62TfEHcyHoHfMFXloCsdbRfWb5JHd\/ieOvYxQ0\/1Jo1a4gTl3gTPz5LqmXLloG7EvOBma1ataLzzz9fHZ5pc\/mPX\/DvTszLw8eOHXvQbsRh9ZIubljUzFu1hXpP\/X45u37mpEdpgvyLzsum1duXBV97ZjY1wNeGln1Z8LVnZlMD4saGlqesSc6No+mcVUuS81nYXDNq6UFTUIUoarQD0XnJNmXwBV9ZArLW0X5l+SZpfIv7TUUiN3E\/ZD7tJcX5nFvDwsZ7sai56exiGnB5s3wiinRvdF6R8IVWBt9QRJEKgG8kfKGVwTcUUaQCSRnfIr1EhsqxixsdreH73XzzzfTMM88Q\/1u2q3bt2tSzZ0\/iP5N2JcH5U97ZoPJr\/MJmVmmrxOfUhPkTnVcYoWifg280fmG1wTeMULTPwTcav7DaSRjfwp7R9fPYxY1eIcUPxEnCvXv3Jt4FMdvFe5zYbr7n+sK29fLt\/CcXrKe7ZnxU7rGn3X46XVZyhO2rJLI8Oi9Zt4Av+MoSkLWO9ivLN9\/jm+TbxS5uJB82H7bz6fy3V22ha0YtK3vtQs6tyeQ7dF6yrRp8wVeWgKx1tF9Zvvkc32TfrMDPlpKGw\/bz5fyJC9ZTH0\/EJo3Chvmi85JtxeALvrIEZK2j\/cryzdf4JvtWB6yLRm50\/g1ybuxc6c+xYWGThvyaIArovOzahm1p8LUlZlcefO142ZYGX1tiduUhbux4lZXW+TfIuTEH6F8VlWZhg8iNebtwLYnBwZWcWT3wNePkWgp8XcmZ1YO4MeNkXGr\/\/v20efNmmjp1Kr344ovqbKlMZ08ZGxUqmEvn8z42LQcvLHuTtAsbiBuhRusxi8FBljH4gq8sAVnruRzfZN\/kYOui01JhL8Mi57777lM7GfPBmdWqVQurkvPPc+n8qx9fWu44BZ6KSuqZUHE5AoNDXCSD7YAv+MoSkLWO9ivLN5fjm+ybJEzc8ONMmzaNHn\/88Qq\/FHzBJ1vpqse+36Rv+A3N6dYfNcx1e8j5\/dB5ySIHX\/CVJSBrHe1Xli\/EjRBffcjmBx98QE8\/\/TTVr19f6E7uZnPh\/KA8m2WDfuT+0AVUE52XrLPAF3xlCchaR\/uV5ZuL8U32DTJbF52WClst9cknn9CSJUuoc+fO9Jvf\/IYqVaqULw4Z7yvtfP95URUhz8YLG52XbJMHX\/CVJSBrHe1Xlq\/0+Cb79Nmti4qbsNVS1atXp+uuu44GDhxIhx9+eD455E3cDJm9hobMXlt2\/8dvKlHnRVWUC52XrKfBF3xlCchaR\/uV5QtxI8s30dYlnR+0OqqiTEdpp6Pzkm3+4Au+sgRkraP9yvKVHN9knzzcumjkJtPteZXUjh07qFatWomcivI+t6Tzrx29jP769y3qdmndgTisCaLzCiMU7XPwjcYvrDb4hhGK9jn4RuMXVltyfAu7t\/Tn4uKGk4b\/+Mc\/EicNjxo1ioqKitTS7y5dutD69evp17\/+NV199dWJFTlSzvdHbYZdfyJ1+XEjaX8nzj46L1mXgC\/4yhKQtY72K8tXanyTfWoz6+Li5oknnqBhw4ap3JpBgwYpcbN7926aPXs2jR8\/nlatWqX2uLniiivMnjjHpaSc793ThqM2FW06SrsRnZdsgwZf8JUlIGsd7VeWr9T4JvvUZtZFxY2O0Jxwwgn0hz\/84aBN+jiq06dPH7VbMQsdFj5JuySc71\/6zQnEnEhcES90XrJeB1\/wlSUgax3tV5avxPgm+8Tm1kXFjV4t1atXL7rxxhsDn+r5559Xxy9Mnz6diouTt0pIwvmI2nzfFNB5mX9ZXUqCrws18zrga87KpST4ulAzryMxvpnfXbakqLjhiEynTp3owgsvpAEDBgS+yZAhQ+itt96qMJv4+XNtKtrSb38jQOcl+wUHX\/CVJSBrHe1Xli\/EjSNfXhX1u9\/9jl5++WV66KGHlMjRG\/XxZyxq+vfvT1dddVWF2cTv3lmr6PG3\/qGIVuRcG92k0Hk5frkMq4GvISjHYuDrCM6wGvgagnIsBnHjCI6rrVu3jrp160YrV65UG\/Xx8m++9uzZo3JtTjrpJOKk48aNG0e4i1zVOJ3vj9qM73QyXdfqKLmHLwDL6LxknQS+4CtLQNY62q8s3zjHN9kntbcuOi2lH4dXR3FODf+3fft29c8sdNq1a6f+q1mzpv2T56hGnM6f8s4G6jVlRVnUhk\/95uhNRb7Qecl6H3zBV5aArHW0X1m+cY5vsk9qb11U3PDU04svvkinnHIK8YqpQrzicj5HbVjYzF+9VWE477g69FKvVoWIJNZnRucVK86DjIEv+MoSkLWO9ivLN67xTfYp3ayLiptNmzapVVLXX389lZaWuj1hnmvF5Xz\/8m\/e16aiR23Ytei8ZBs4+IKvLAFZ62i\/snzjGt9kn9LNuqi40auleAfiii5usPw7uIGi83L74prWAl9TUm7lwNeNm2kt8DUl5VYO4saNm6r1xhtv0G9\/+1sVwbnyyivp0EMPPchalSpVqF69esR\/Ju2Ky\/n1+rxZ9mqca3P+8XWS9qp5eR50XrLYwRd8ZQnIWkf7leUb1\/gm+5Ru1kUjN3oTP14xle3ilVJp3sTPm0jMHCBuvm8N6LzcvrimtcDXlJRbOfB142ZaC3xNSbmVg7hx46bOkJo\/f75a9p3tqlGjBp133nmJXDUVh\/O9U1JIJC7fEtB5OX65DKuBryEox2Lg6wjOsBr4GoJyLBbH+OZ4a\/FqopEb8afPwQ2iOh87Emd3Ejov2UYMvuArS0DWOtqvLN+o45vs00WzDnETwi+q872rpLAj8cGw0XlF+wKH1QbfMELRPgffaPzCaoNvGKFon0cd36LdXbZ27OJG59nwY48cOZJ69+6tdinOdqU55wZTUojcyH6FwRd880lA9t4QN7J8IW4s+G7bto1Gjx6tatx88830zDPPEP9btqt27drUs2dP4j+TdkVxPqakwr2JziucUZQS4BuFXnhd8A1nFKUE+EahF143yvgWbj2\/JWKP3OT3deK\/exTn+49b4I37cJUngM5LtkWAL\/jKEpC1jvYryzfK+Cb7ZNGti4sbPoLhtddeo7lz59K9995Lhx12mDpfijf1q1atGg0cOJCaN28e\/U2ELERxPqakwp2CziucUZQS4BuFXnhd8A1nFKUE+EahF143yvgWbj2\/JcTFzauvvkp9+vShFi1a0JgxY6hu3bpqmuqhhx6il156iXgZOJ8K3qpVMs9ZcnU+pqTMGjY6LzNOrqXA15WcWT3wNePkWgp8XcmZ1XMd38ys57eUqLjZuXMnde3aVe1fw8nFRUVF5d52y5Yt1KNHD7VrMQsfFjpJu1yd7z9LChv3BXsWnZdsiwdf8JUlIGsd7VeWr+v4JvtU8VgXFTd65VS3bt2oY8eOgU88efJkFblJ2w7FQ2avoSGz16p3xhLwzI0VnVc8X+RMVsAXfGUJyFpH+5XlC3HjyFefCn7ttdeqJeFB16hRo2jGjBk0bdo0atCggeOd5Kq5Ot+bb\/ObK39Ad15yrNxDFrBldF6yzgNf8JUlIGsd7VeWr+v4JvtU8VgXjdzs27dP5dusWLGCxo0bR82aNSv31GvWrKHbb7+dSkpKaPjw4SrBOGmXi\/P9+TaYkkLkJl\/tGoODLHnwBV9ZArLWXcY32SeKz7qouOHHfP\/996lLly60Y8cOOuOMM+jYYw9EMDZv3kzz5s2jWrVq0YQJE+i0006L761itOTifCwBN3cABgdzVi4lwdeFmnkd8DVn5VISfF2omddxGd\/Mree3pLi44df7\/PPP6cEHH6Q33niD9u7dq964evXqdOmll9Ldd99NjRo1yi+FLHd3cf5\/jF1Ob370tbKKgzKzuxadl2zTB1\/wlSUgax3tV5avy\/gm+0TxWc+JuInvcXNvycX59fq8WfagAy5vSgMuLz8dl\/u3SO4d0XnJ+gZ8wVeWgKx1tF9Zvi7jm+wTxWcd4iaEpa3zkW9j1zjRednxsi0NvrbE7MqDrx0v29Lga0vMrrzt+GZnPb+lIW5iFjc4BdyuQaPzsuNlWxp8bYnZlQdfO162pcHXlphdeYgbO16pKm3rfBy5YOd+dF52vGxLg68tMbvy4GvHy7Y0+NoSsytvO77ZWc9vaURuYo7ctBy8kHhqii\/k24Q3bnRe4YyilADfKPTC64JvOKMoJcA3Cr3wuhA34YxSW8LG+ci3sW8G6LzsmdnUAF8bWvZlwdeemU0N8LWhZV\/WZnyzt57fGojcxBi58Z8n9fXwi\/Lr3QK4OzovWSeBL\/jKEpC1jvYryxfiJiJf3on4T3\/6E3322WeBlmrXrk09e\/Yk\/jNpl43zcZ6UvffQedkzs6kBvja07MuCrz0zmxrga0PLvqzN+GZvPb81xCM3r776qjqCQW\/eF\/S6jRs3TsXBmd5k4pvOLqbHbyrJr3cL4O7ovGSdBL7gK0tA1jraryxfiBtHvjt37qSuXbvSl19+SY8++iidcsopVKlSJUdr+alm6nzk27j5B52XGzfTWuBrSsqtHPi6cTOtBb6mpNzKmY5vbtbzW0s0crNhwwZq164dderUSR2QWYiXqfP9+TY4LNPM2+i8zDi5lgJfV3Jm9cDXjJNrKfB1JWdWz3R8M7OWrFKi4mbLli3q0MwrrrgiMeKGp8hmzpxZ5oUxY8ZQmzZtMnrF1PnIt3Fr2Oi83LiZ1gJfU1Ju5cDXjZtpLfA1JeVWznR8c7Oe31qi4oZf7YknnqDXXntN\/dmgQYO8vi0LmyVLlpTl98yZM4d69OhBffv2pdLS0sBnM3X+NaOW0durtigbOCzT3M3ovMxZuZQEXxdq5nXA15yVS0nwdaFmXsd0fDO3mJySouJm9+7d9NZbb9GTTz5JK1eupNatW1OtWrUOevtcrJZavnw5de7cmYYOHVouUsOCZ\/369TR+\/HgqKio66NlMne9NJsbmfeYNHJ2XOSuXkuDrQs28Dvias3IpCb4u1MzrmI5v5haTU1JU3Oicm3Xr1mV943yulopL3HhPAke+jXkDR+dlzsqlJPi6UDOvA77mrFxKgq8LNfM6EDfmrAqq5KhRo2jYsGGULe9GO\/\/ZZ58lFmH6Ki4uLvv739bupGtGLS37\/yUDz6Zj6h1SUCzy9bDovGTJgy\/4yhKQtY72Gy9fDjh4Lw48dOjQgebOnVtufIv3rvmxJhq5yc8rhd9V59pwybZt29Lw4cMzVtLixl+Ap7huueUW9c9jF22lJxZvVX9veHhVeumW70VQ+NNU7BJ79uyhTZs2EYvFqlWrVmwYAm8PvgJQPSbBF3xlCcRr\/amnnqJJkyYdZBTixpHz6tWrafDgwbRw4UI68sgjVULvoYceSgMHDqSf\/vSndPXVV+dl\/xu9Dw\/n3PAzeaMx+lW1uOFcnUaNGpUR4LK6\/LVj36dFn+5Un3Ey8f\/8Apv3mTYVzsvauHGj+tUAcWNKzbwc+JqzcikJvi7UzOuArzkrk5IcufFGbxYtWkQjRoxA5MYEnr\/M+++\/r5aDV65cmZo2bUpffPGFEhI1atRQ\/75ixQq1wV+25dgu9zWtoxONu3fvHrhiymROEsnEprQPLoewszs7k5rga0LJvQz4urMzqQm+JpTcy5iMb+7W81tTdFpq37596ugFntfjpeBLly5VERwdJdm+fbva\/4ajOJz3woIn11dUcYOdiaN5DJ1XNH5htcE3jFC0z8E3Gr+w2uAbRija5xA3jvw4l+LGG2+k66+\/XkVFONfFK27Y7Lhx4+jpp58WP1uK792vXz8139iiRYuyN9L5N5mSisOcj52JHRvHv6uh84rGL6w2+IYRivY5+EbjF1YbfMMIRfs8bHyLZj2\/tUUjN3opeLdu3ahjx46B4mby5MkqqpMp5yUuPDq\/hu3pPW101KakpMR5nxtv5IZXSC0b9KO4HrlC2EHnJetm8AVfWQKy1tF+ZflC3Djy1YKifv36akUSb+gXNC1VrVo1Gjt2LB122GGOdzKv5j9+IdvuxGw1zPk4dsGcfVBJdF7R+IXVBt8wQtE+B99o\/MJqg28YoWifh41v0aznt7Zo5IZf7dVXX1XHG3Dk5thjj6XRo0er\/BrOw3nsscdUQvGQIUPU1FUSrzDne5OJceyCvQfRedkzs6kBvja07MuCrz0zmxrga0PLvmzY+GZvMTk1xMXN\/v37VZ4LL6XetWtXuTevUqWKSjjmaSv+exKvMOe3HLyQeGqKr8dvKqGbzv5+c78kvk\/Sngmdl6xHwBd8ZQnIWkf7leUbNr7J3l3Wuri40Y\/PK6MWL16s\/tu7dy+deeaZdP7551PdunVl3zCi9WzOx0qpiHCJCJ1XdIbZLIAv+MoSkLWO9ivLF+JGlm+irduIG04mxrELdu5E52XHy7Y0+NoSsysPvna8bEuDry0xu\/IQN3a8DirNU1Mff\/wxPffcc7Rjxw71OScZX3fdddSsWbOI1mWrZ3M+komjs0fnFZ0hIjeyDMEXfPNHQPbOEDcR+PXX5G8AAByASURBVPJ01K9\/\/WuVWMwix3tVqlSJ2rdvT\/fddx9Vr149wl3kqpqKGyQTu\/kA4saNm2kt8DUl5VYOfN24mdYCX1NSbuUgbty4KTHDicQTJkygXr16qYMmDz\/8cGWNRQ9v4Md73Nx1110qqTiJVzbnY6VUdI+h84rOEJEFWYbgC775IyB7Z4gbR76bN2+mTp060dlnn03333\/\/QYdjsvjhqA2fP8UCKInJxdmc710pNeDypjTg8mRPsTm6UbQaxI0oXiRsy+IFX\/AVJiBrHuLGka\/eoZijNnwMQ9D10ksvqeiO9A7Fjq+QcRM\/rJRyJVq+HsRNPBwzWQFf8JUlIGsd7VeWL8SNI989e\/ZQ7969VT4N71Dsz6vhJeH33HOPmqIaOXJkXg7ODHu1TM7HmVJh5Mw+R+dlxsm1FPi6kjOrB75mnFxLga8rObN6EDdmnAJL8U7EnE\/TpEkT6t+\/PzVt2pQqV65MHNXhHYr54MoHH3yQTj311LL6vKFfgwYNItw1vqom4gZnSrnzRuflzs6kJviaUHIvA77u7Exqgq8JJfcyEDeO7PS0FAscm6tx48Y0d+5cmypiZTM5H8vA40GOzisejpmsgC\/4yhKQtY72K8sX4saR7+7du2n+\/PnE01M2V40aNejSSy+1qSJWNpPzfz5qGc1btUXdF8vA3fGj83JnZ1ITfE0ouZcBX3d2JjXB14SSexmIG3d2BV8zk\/O9y8CxUsrdzei83NmZ1ARfE0ruZcDXnZ1JTfA1oeReBuLGnV1ZzS1bttDbb79N7777rkosbtWqFf3whz9M5PJv7+tmcn69Pm+WFcOBme4NBJ2XOzuTmuBrQsm9DPi6szOpCb4mlNzLQNy4s6Nvv\/1WbdTHq6X4796LE4d5mXhpaWlB7VCMZeARGoSvKjqv+FgGWQJf8JUlIGsd7VeWL8RNBL4zZsygAQMGUOvWrdVOxLxaiq+1a9fSww8\/TAsWLFD73FxzzTUR7iJXNcj5\/mXgODDTnT86L3d2JjXB14SSexnwdWdnUhN8TSi5l4G4cWS3c+dO6tq1KxUVFal9bGrWrFnOEicc8z44HNEZM2ZMwexzM+WdDdRrygr1LlgG7tg4\/l0NnVc0fmG1wTeMULTPwTcav7Da4BtGKNrnEDeO\/PRScN7npmPHjoFWJk+erKatCmmH4mcWf0G9p66EuHFsF95q6LxigJjFBPiCrywBWetov7J8IW4c+W7atEkdu3DttdeqCE3QxRGdF154gaZNm5aYjfu8zxnkfByY6dggAqqh84qPZZAl8AVfWQKy1tF+ZflC3Djy3bdvH\/Xp04dWrFihTgBv1qz8wZJr1qyh22+\/nUpKSlTCcbVq1RzvJFctTNxgGXg09ui8ovELqw2+YYSifQ6+0fiF1QbfMELRPoe4icCPT\/zu0qWL2sjvsssuo\/POO09Z4839Xn\/9dZVnw9NSvDQ8iVeQ87EMPD5PofOKjyUiN7IswRd8c09A9o4QNxH5fvTRR+qAzGXLltH+\/fuVtUqVKlHLli3pgQceoObNm0e8g1x1v\/OxDDxe1hA38fL0WwNf8JUlIGsd7VeWL8RNTHw5esOb+fFVt27dRK6O8r+q3\/lYBh5TY\/i3GXRe8fKEuJHlCb7gm1sCsneDuJHlm2jrYZGbr4dflOjnT\/rDQdzIegh8wVeWgKx1tF9ZvhA3snwTbd3vfJwGHq+70HnFyxORBVme4Au+uSUgezeIG1m+ibbudz5v3seb+PGF08Cjuw7iJjrDbBbAF3xlCchaR\/uV5QtxI8s30db9zvfucXPT2cXEh2bicieAzsudnUlN8DWh5F4GfN3ZmdQEXxNK7mUgbtzZFXxNv\/NbDl5IvGKKr1mlrej84+sU\/Dvm8wXQecnSB1\/wlSUgax3tV5YvxI0s30Rb9zsfe9zE6y50XvHy9FsDX\/CVJSBrHe1Xli\/EjSzfRFv3Ov+7Q+sTR270hchNdNeh84rOMJsF8AVfWQKy1tF+ZflC3MjyTbT1bOJm2aAfqVPBcbkTQOflzs6kJviaUHIvA77u7Exqgq8JJfcyEDfu7Aq+ptf5876oSrxaii8WNSxucEUjgM4rGr+w2uAbRija5+AbjV9YbfANIxTtc4ibaPwKurbX+c98sI+GzF4LcROjR9F5xQgzwBT4gq8sAVnraL+yfCFuZPkm2rrX+Q\/O24E9bmL2FjqvmIH6zIEv+MoSkLWO9ivLF+JGlm+irXud3\/PFTTR\/9Vb1vLf9uCE9fH1yD\/xMNFTPw6HzkvUU+IKvLAFZ62i\/snwhbmT5Jtp6JnEz4PKmNODyZol+9kJ4OHResl4CX\/CVJSBrHe1Xli\/EjSzfRFv3Ov\/04X8ve1bemZh3KMYVjQA6r2j8wmqDbxihaJ+DbzR+YbXBN4xQtM8hbqLxK+ja2vmTZ86hqyb+o+xdsMdNPG5F5xUPx0xWwBd8ZQnIWkf7leULcSPLN9HWtfMffHIWcc6NvrDHTTxuQ+cVD0eIG1mO4Au++SEge1eIG1m+ibaeSdx8PfyiRD93oTwcxI2sp8AXfGUJyFpH+5XlC3EjyzfR1rXzr\/z1JOJ9bvjCBn7xuQydV3wsgyyBL\/jKEpC1jvYryxfiRpZvoq1D3Mi6B50X+MoSkLWO9gu+sgRkrUPcyPJNtHXt\/NZ3TaBZq\/erZz3vuDr0Uq9WiX7uQnk4DA6yngJf8JUlIGsd7VeWL8SNLN9EW9fOr9N+BK39Z5F6Vl4CzkvBcUUngM4rOsNsFsAXfGUJyFpH+5XlC3EjyzfR1rXzt7cZQt8dWl89Kzbwi89l6LziYxlkCXzBV5aArHW0X1m+EDeyfBNtXTt\/a9v\/LntObOAXn8vQecXHEuJGliX4gm\/uCcjeEeJGlm+irbPz23f9JXHkRl\/Y4yY+l0HcxMcSg68sS\/AF39wTkL0jxI0s30RbDxI32J04PpdB3MTHEoOvLEvwBd\/cE5C9I8SNLN9EWw+alkLkJj6XQdzExxKDryxL8AXf3BOQvSPEjSzfRFv3ixts4BevuyBu4uXptwa+4CtLQNY62q8sX4gbWb6Jtg5xI+sedF7gK0tA1jraL\/jKEpC1DnEjyzfR1v3iBhv4xesuDA7x8kTkRpYn+IJvbgnI3g3iRpavtfWdO3dS165dafHixWV1+\/btS6WlpVltLV++nDp37kw7duwoV+6cc86h8ePHU1HRgU36vBfEjbV7rCpA3Fjhsi4MvtbIrCqArxUu68Lga43MqgLEjRUu2cIbNmygdu3aUcOGDcsEiRYtl1xyCQ0fPjzjA8yZM4f69etHkyZNohYtWhg9qF\/cYAM\/I2zGhdB5GaNyKgi+TtiMK4GvMSqnguDrhM24EsSNMSr5gpkEyqhRo2jq1Kk0ffp0Ki4uDnwQLjN37tyMUZqgShA3sj5F5wW+sgRkraP9gq8sAVnrEDeyfGOxzsJl7NixWaMyffr0UffKFt3xP4xf3GB34ljcVWYEg0O8PP3WwBd8ZQnIWkf7leULcSPLNxbrLFyWLFmSMXKjp7M4r2blypVl92zbtm1WseMXN6N\/3kCdCp4pOhTLy1QgI+i8ZJ0NvuArS0DWOtpvvHx5HPRe69atow4dOqgZjcaNG8d7szxbq7R\/\/\/79eX6GyLfnqaoePXpQtqTioLycoPydsMhN0dsPUdXNH6nE5FtuuSXys1d0A3v27KFNmzYpsVi1atWKjiP29wff2JGWMwi+4CtLIF7rTz31lJrd8F8QN\/FyjsWaFi0lJSVWuTT65loYjRkzhtq0aXPQM\/kjN6+2q6nK8GCM6E10F+7evZs2btyofjVA3ETn6bcAvvEz9VoEX\/CVJRCvdf5B743eLFq0iEaMGIHITbyYo1uLKmz4CbSN7t27By4l94ob7E4c3Wd+C2vXrqWJEyeqpf1pC4vGT8veIvjaM7OpAb42tOzLgq89M5sayLmxoZWjsjrikm2PGpNHgbgxoSRXJs1fLjlq5pbB15yVS0nwdaFmXgd8zVm5lEwz34LMudHCJiwZ2OvsTNNPYXvfeCM32J3Y5euTvU6av1zx07K3CL72zGxqgK8NLfuy4GvPzKZGmvkWnLgx3bDP72C9q\/H69evLVlSZ2PKKm6aH7KRZpa1s2g7KhhDQ2frPPvsspqUEWgv4CkD1mARf8JUlIGsdq6Vk+VpZ5yXfM2fOzFhHJwZn2vfGXz\/s2AZ2Pu9qPPvoblT9s\/l06P9OsHpeFAYBEAABEACBpBI499xzacqUKUl9POfnKrjIjfObRqjIAof\/wwUCIAACIAACaSLACznSuJgD4iZNrRTvAgIgAAIgAAIgQBA3aAQgAAIgAAIgAAKpIgBxkyp34mVAAARAAARAAAQgbtAGQAAEQAAEQAAEUkUA4iZV7sTLgAAIgAAIgAAIQNygDYAACIAACIAACKSKAMRNqtyJlwEBEAABEAABEIC4QRsAARAAARAAARBIFQGIm1S5Ey8DAiAAAiAAAiAAcZOlDejzqBYvXqxK8S6O06dPp+LiYrQcCwLeIy9q1apFkyZNohYtWmS1oA861YXAPjMuF75eaxs2bKB27dpR+\/btqbS01MKz6S\/qwtbfbzAlfSxM+onZvaELX91e9a7x6BvsmPtLsw\/4Gj58eDRDCasNcZPBIbqDatiwYZnTuREsWbIEAseiETMzPqx0\/PjxVFRURJnO\/PKa5DLDhg0rNyCwnT\/\/+c9Gwsji8Qq+qAvfoM6Nz2sLO2et4GFZvoALWz3wcr\/hbfP+9mz5KKksHoXvWWedhX45hlah+9q2bdtC3MTAsyBMBA3C+IVr5zodffH+ag0SjV6rmT4H+4PZu\/D1W\/FGyCBuvqfjypb7jalTp5b7ARTW5u2+VekoHYXv2LFjy\/3IWb58OXXu3Jm6d++OyKNh8\/BHFyFuDMGloZj\/V4V+p0z\/noZ3jvsdgjp6vkemf892fy1uvL\/Y4n7eQrMXla9m2rVrVxVlwLTU9y3Aha0eMFq3bo1BNuTL5MJX9x0QN9F6Kt1OOaI+ceJEuueee8g7QxHNenJqY1oqwBfZfmlhasq88WYSgiZTU\/674NfZwdyj8PW28f79+yPnxofXha0Wi4MGDaJVq1apqVW+TPPMzL9ZhV\/ShS+\/ddCPHExZu7eHNEcVIW4sxY1L1MG96RV2zUwdGIek+\/XrZ5w\/4\/2lgYTu79tEFL7edswWkVBc\/rvmwlYL8B07dpA3zB+UQ1bY3+zoT+\/C13tXbyLyOeecU5bfFP3JKpYFiJuK5W\/K5nCIG\/PGELUD03fSHRlWnEQfgNmCHoSHDh1Kbdq0Kfs1jGmpaMJRcy0pKSk32Or+hK3rJGPzb1E6S7r2DTpXx5sfBvHo3kYgbtzZFWRNTEvF4zbX0HPQLzQIm4N94sI3qG0jWTsetlrcXHLJJQetPMGPIjNhnm3KOptIRC6kW58NcePGraBrIaE4uvtckwb5zt5sfgibYF+48PVOnQRZxZ4hB6i4sM2W9A5xU761ufBFRD16n+y3AHETP9PEWwz68uEXrp3bgnJrTL5MusyKFSuM83LsniwdpV35+t8e7frg9uDKNmjBgUmbT0eLNH8LF76I3JjzNS2Z5raJhOIMrSBoMy6slDL9yhwoF5QIbLJSCqsfzDi78oW4CefryjZoasqkzYc\/UbpKuPJFzk287QDiJl6eBWPNv9ERQvZurgvbYt0rGjdu3Kg25OIVJ0EXVkYcTMWGb9DRIYjcZG7XLmz9xwNgKXi8fP1Tq+Dr1i97f4Binxt3hqgJAiAAAiAAAiAAAjkhgGmpnGDGTUAABEAABEAABHJFAOImV6RxHxAAARAAARAAgZwQgLjJCWbcBARAAARAAARAIFcEIG5yRRr3AQEQAAEQAAEQyAkBiJucYMZNQAAEQAAEQAAEckUA4iZXpHEfEAABEAABEACBnBCAuMkJZtwEBEAABEAABEAgVwQgbnJFGvcBARAAARAAARDICQGIm5xgxk1AgOjbb7+lF198kXbv3k0333yzNZLPP\/+cxo8fT6WlpdSgQQPr+lEqvPHGG\/Tb3\/6W1q9fT40aNaKpU6eqP22vbIdL2tpKQnn\/bsR9+\/ZV\/snH5d+5FwfO5sMLuGdSCEDcJMUTeI7UE4h6zEG+Tpbevn073X777UrY9OjRg4466ig677zzqGbNmtY+S6u4Ofroo9WxISeccAKdeOKJ1lziqLBlyxZatGgR\/e\/\/\/q8SwRA3cVCFjUIlAHFTqJ7DcxccgUIVN3EKkjhtJaEBJPF99OGSEDdJaCF4hnwRgLjJF3ncN1UE9u\/fTy+99BI9\/PDDtG7dOqpWrRq1bNmS7r\/\/fmrevDllO+yPD2h97LHHaObMmfTll19SpUqViA9pveuuu+jqq69W\/+89wJHBtW3bloYPH64YfvTRR\/Sb3\/xG\/WLn6\/TTT6f+\/fvTueeeG8qYp7pGjhxJL7\/8Mu3atYv4AL1f\/OIXatqsevXqpAdKr6FsUy\/M4bXXXqM\/\/vGP9MknnygOP\/rRj2jQoEF03HHHkRYDrVq1Ij4E9dFHH6VNmzap+\/Iz6\/fl++3du5eeeeYZmjhxomLKto888kjq0qUL3Xrrrer5+NIRrdtuu03d97vvvqPf\/e53dN1119Hq1avV3xcsWEBVqlShm266iU477TTll0mTJlGLFi2UDZ4q5PtMmDCBvvrqKzriiCPoP\/7jP+iOO+6goqKijByDxI33pOULL7yQ\/vCHP9DmzZtVO7jnnntU1It9munKFKEzjdxB3IQ2exSoAAQgbiqAk\/GK8gReeeUVJUZ+8pOf0OWXX05bt26l\/\/7v\/1aD2LPPPkt8cjEPOsOGDaMf\/\/jH9LOf\/UyJj8MOO0wJl7feeouuv\/56Ovvss2nNmjU0ZcoUNciOHTtW2Xz33XfVYMyD9J133kknnXQSnXnmmcom12\/atCl17NhRvejkyZNp1apVSvxcccUVGV+eBQOLBBYX\/Oexxx6rBNb8+fOpXbt2SgDwoPzmm2\/SI488Qj\/4wQ+yTr2w+OBoAb9jSUmJeh4e\/Pl56tWrR08++aQSGGyb\/52ncm655RaqUaOGEhX8HMyMGbCtoUOH0rhx45Tgad26NW3btk3ZYtF03333qWfR4oZFZd26dalbt27qHhdffLESVvxe7At+lsMPP1wJGBYyLJy0uGExwiJmyZIlZT545513aMaMGUqgsqjgukFXNnGzYsUK9Qw33HCDYsv342dnQdemTRuIG\/mvJe5QgQlA3FRg5+PV4yPAAoOjIDwY64Hw7bffVsJj8ODBajALmpbiqAsnoHKUwJuIyoMrR1A4x0X\/u\/+XO+fC8OBdXFysIkY6B4YHbr4v58jwYB40MLN4YIHACc4sLFgo8cVJzyxOOGKihYbp1Mtnn31GHTp0UIKAhZWOrPztb3+j3r170913362EHYsbjq48\/fTT1KxZM3Vf\/b6c28NlWVR1795d5a9w5IVFAl\/6Hhz10ZEr5sLP\/OCDD9KNN96oyun34yjSE088QRwp4ssr6LS4YUHHkS+2c\/7555c1Cn5uFkt8f46U2YqbZcuWlROYnBPDUScWvJn8osUaJ2xPnz5d+VZfiNzE932FpfQTgLhJv4\/xhjkgwOKChQ0PzhxRCFrNZJNzo8teeeWVNGDAAPUG\/sFt6dKlKvJx1VVXqakO78URHY4GeadevJ+zeOjUqZMSFyNGjCgTD1yGIw78DhxJ4nubihteUcVCjKfYMkUmvNNSHA3S0zOm99BTPhwJ0s\/NXDjC5X3XbO\/H03CccMvleQqPI2EcGePoDUfY9MURH37GCy64oExI+ZtStsgNl+X7eKe1+Fm5nbCwO\/XUUwNbJqalcvCFxS1STwDiJvUuxgvmggBHbfQgyffj3BCeemKRoKMT2cTNnj171JQFC4vFixcTR3048uLNrfEPekH5MN53ZeHAg\/6ll156EIIg8aQL+QdsU+HBgzYLm0yCiu1nspXp3zkKxWw\/\/PBDJUDmzZunOPHUlRYOQeKG63AUh\/\/jSJBf+PXr1089J+cBde3aVTHPdHG0iTnyFKKNuOE8Ih1d0vVM8mEgbnLxjcU90k4A4ibtHsb75YwAT4XwwDtt2jSaPXt2WWKxzn0JEjc8ePOUCuej8JQQTyHx\/jGc6Pr666+XixpkEjcuq2IkxE2QyDARA0Gih1my+OC8G050PvTQQ1WOzllnnaWECEfGsombtWvXqqTo9u3bG4mboCiLScMJSygOEjc9e\/bMKDr5nhA3JuRRBgSyE4C4QQsBASECH3\/8scqZ4WRSFiCcc8H5Jjzg6jyav\/zlL8SDHedi9OrVq2wKQw\/OnHTszS3x5mLwIM85Nzw1paeuTF\/FZFqKo04c9TCN3GSaluJEYZ4Cu+yyy5TgYAYsUrwDv\/8eemqM3\/\/3v\/+9ShbmS+etcBJyNnGT7f28Iuzkk0+mX\/3qV2o1G+cZcWK2zZVN3HAOFPudn1VffG8WsnyvTPvhZCrDq8k4D8ifi+N\/XpPokM07oiwIFCIBiJtC9BqeOVEEduzYQf\/5n\/+pIi88MOnE3q+\/\/loJj\/r162cUN1yeB8CnnnqqLOmVoxY8gPGy4Z\/\/\/OcZxY1OKObcEK7fpEkTxYWjQVyXl4ZzUrCeFvNCC0so5gGYE415SsZU3GRKKJ41a5Za5s2rhDjHxUTc6AGa82M470hfPF3HwpBFSTZxkymhWIsjjrB5E4p5pRtPK3Lejc4D4mkwTmrm5eP8edAVtlrKm6zNy\/y5PfCUJfs80yaIvCKMBd3o0aPVqi++\/vGPf6i6\/\/rXvyBuEvXtx8MklQDETVI9g+cqKAK8Ioenl3jVEQsSvl544QUVEdBLf3U0gT\/jaA0nqr733ntqAOWpKI7e8IDHK5g4KsMrinhg1xEOPejxIMfLzfler776qloZxUmrbIcTYrk+D8wsuDhylGlPFZOl4LziyVTceJeCc8SFE5L\/\/ve\/q+TZM844Qwk\/Tgg2ETccueH3ZAb8XjwNNXfuXLX0fd++fUoIZhM3zJijXxwx4qXfvBScbbDY4Pc+5JBDysQNi0QWTAsXLlRigplzjs\/zzz9PderUUSubeN8hW3HDPuTEZ++9+V7e1VtaxHn3DtLvzivEeMUcXyxe+VlYnOnIjfYL5\/Z4E5cRuSmorgMPK0QA4kYILMxWLAIcteEpIx64ePDky79pGw\/+PNCzCPrmm29U3sUll1yiNv976KGHVAIx55bwvjYsfngFFu\/topcNc5IsCxnej4U3xtNJrrzlPtdnocRigAdinmphkcV7vmS7gjbx46moa6+9tmwpt6m44fv4N\/FjscZiRgswm4Ri3tOHl9Hzcnm9KSInAnMk6M9\/\/rPaP+iYY45Rosm\/Wkq\/M2\/ip6NYehM\/fiaeFvImPuuNFJ977jm1vxD74aKLLqKBAwdmPUMrW+SGWbAvOQrG0TUWeLysnNuFvoLEDX\/mfXcWSLwKj\/36X\/\/1XxA3Fatrwds6EoC4cQSHaiAAAoVJgKe6OOmb\/3M5\/NP71rZLwXNBDJGbXFDGPZJOAOIm6R7C84EACFgT4KX1nD9TuXJlFQHTe83oQ0B5qs6\/B431TTIsbdd78bC9OO5h+1wQN7bEUD6NBCBu0uhVvBMIgICaduJprR\/+8IdqvyBOtDY9msIUn47ceE8F5xwY3jsn1+IGp4Kbeg3lKgIBiJuK4GW8IwhUQAKcB8XJ1byTMedBcRSHV2tx3g4nPGc7vNIUlxY3Os+KE4N5CX0+xI3\/cFaX\/Y9M3xvlQCDpBP4fX4YnDCNC0u0AAAAASUVORK5CYII=","height":273,"width":454}}
%---
%[output:178ec249]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"     1.320000000000000e+01"}}
%---
%[output:7b55c0ed]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"     7.500000000000000e-03"}}
%---
%[output:50dd316e]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"     7.500000000000000e-03"}}
%---
