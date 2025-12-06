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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANMAAAB\/CAYAAAB12fdMAAAAAXNSR0IArs4c6QAAGL5JREFUeF7tXQ2QVcWVbpQNkA1IACN\/gTEoklRtJP6BRFdSWaRSFhgpS2B2IwuIwAq4BpBfYSWC8re7gEaJkimsXUalRIVsagmShRBZWQGxXIsCpAKEvxEHCRjELSqz9TU5z\/N6uu\/t++7tN+\/NnFtlybzb3bf79Pn6\/PTp081qa2vrRo8erd577z0V91x\/\/fVq1apVql27dnFF5b1QoMlRoBnA9Pjjj6u5c+dGguT06dPKp1yTo6AMWCjwZwo0q6urqytnarz99tuqsrJSrVmzRvXt2zeToXz22Wdq+vTpuq2nnnpKtWrVSj399NNq4MCB6tprr83kGwsXLlToO0l6\/Pvw4cNq6NChmbRvNnLgwAH16KOPqkWLFkWOAWNfvny5GjNmTKwGknWf0ceRI0eq48ePq0GDBuVoH0UQ21wFIaCjUdAAc4l51JIJah4eUeHsFHv55ZfVihUrVFVVVRAw1dbWaiaaOHFiMDBhwsGktDi4GM4EuascMX6WfQadZ8yYkWhhbGgwQWMDfoYNG6a0ZKIObdiwQdOuc+fOVsahimRfPfnkk3ryqf7Zs2d1\/a1bt+a1YdYjKUJS5bHHHtNAxmRTm+Ykmn0cO3asmjZtml7dSTK1b99eM2X\/\/v3Vli1b1I033qiZByvtypUrdZO04h09ejSPgXk7sA1JMg0ZMkSXo4dLQNtEmsyIv81vQ9JRucWLF6upU6fmbFaMC1KB27E0VvSBGA7\/xu9om\/rkorON8W39Wr9+vWZmzgO7d++u9xvobPZv0qRJmmbEQ655RNu2b4OnMI\/02OrTHPH+de3aVX+X8x637bm0Qz1ql9oiGpp0v+OOO3RX2rRpo3kIj2t8NJdWNY9PGHVgwIABmoBQpUwmJubbtWuXBiERu1u3bmrOnDlq3rx5ujPoFCaMVnmsyCAgMUvUqshXViI8mAiPCSYCEZgWY3nppZc0WLkEuOGGG7zAZPbZVPN4++gLpxEfT9Q7UzKhXkVFhV6o+GpNiwUWBMwBMSXowBcAG52nTJmSWyBtCxBJGN5n9GvBggVq6dKlms4YG+YU7ZuLEa+3f\/9+p+odRZMoyUSgWLJkSd5Yib+I99BPkvKDBw\/WABg+fLjmW3wbYAePmrxHIKf31A5fkEktN8eH30HfWJuJHA933XWXGjduXG4V5KuyCRhiYoAGQMJ7mixOFAICrRYudYqLUtOmiGIM6iMkLZiPP+Zq7ZJMcWCyjQfM3bNnzzxgkVShhWTdunU5m8mm5tmkDGwqrgZFjZ33C\/W4mspXeNPWtC1ofHG1SXZzoXXNF\/1OC7JJE5KCNvuXL1rcm0xzfOTIEb1gmosW\/ubSibQuApMp1WmxIEmE\/8PWnDBhQk6QmOMjetYDkzmJ9HEMlE+IL5geeeQRrcaYDwDUvXv3vBXMBaYo\/TyKoWyTR\/3ICkwcsGibVi8bQPj4XGCi1RQrpCl5XaDg5Wx0PnToUG5FhmQ1VWbUIaaySRgACN5eeHNJw+CSiaQ81HT+cPWUMzW3szhNosDk0lpMVRt\/k3ZA6vKpU6e0NOI87AITAZ23+8ADD6jx48drM8Q2vhyYzH0ml71kes18wWRKJt4Zs804ycRXNGrHRzIRA0BiZg0mvrpeeeWV9VYv1yrsAlOU+plEMnE6RzlQSOqQnfH888\/nFgT+b9COezg5mEzJVG\/l\/PMPaSWTzQkUBSbYPdzTy+mQRjKZ48uB6fz589oBEbcRaxLCphbRakoqjq\/NRKti1KSbujzpxVy6kU3hWvnIlgOD06pFzO5rf9hc41yN4CpKITYTZ0xu1KNdbjOZ7+JsJhdTkQFPCw6cNSRdN23aVM++dTlwimUzmbYdtizg7qf+c8lEziPUIbqSlDLBhPo2m4rGy+nitJmSbNrOnDlTHTt2TH3wwQe688Q4fHVo3bq1\/p17VOK8eT5gSuLNM921Lo+ayzNmMiatwi5vo6m384WJf5urPZz5aNXHYoTJu+mmm7RahQd9AQPQmEJ487g2wu2p5557Tj377LPa04gykLx4YJuYfSa7Oak3z+apdO0ZRnnzbGDiThq8v\/3229W2bdvqqc+0P8n5tCBvXhbhRA3t63epFY3td9POS7L\/BfDiMR0xjY1GacYTpUm42iXe79evX7w3z6dzAiYfKmVThks6tBi1n8O\/6BsBkU0vy7MVc0\/KJxY1LwKi3MOJynPapNeNkQKx+0whBw1bBP\/JIxRoCArA+YL\/snoaDEwAEfafduzYkdVYpB2hQCIK9OnTRyGcKytANRiYyDODwXTp0iUREaSw0ovQsmXLNDMI\/ZJxxLYTzdWT286ptq+PThRUG\/eVPDBx9zNctAijmD17toJLPKujB9ShEEcn4gbbmN4jqmH16tVqxIgROo5PHn8KVL9zUj1UvTccmHhYDDa7MFGzZs3Sganbt2+PDd2PGwrcuLwdAVMcxaLfX7hwQZ04cUJ16tRJtWzZMl1jTax2cDDxk7TYHSYwAWRpT9iSy5FHcwuY0nGwgKlw+i3c+Du1cOOhcJIJXaNjDqNGjVJr167VwX08WraQ7gOM8+fPVz169FB79uzJSTgC08MPP6zuvvtu1bFjx0Kab7J1AKaTJ0+qq666SkcjyONPgaWbf6+Wbj4aFkzoDg\/ZwN++m4KuoUC9w4MYuurq6npgwjvo\/ffff78\/NaSk+vzzz3WYEUJ8WrRoIRRJQIEH151Uu45dCA+mBH2KLQr1jtRFxHfZwARvFNQ\/kUyx5MwrAIlfU1Oj6SY2UzLa3bPyfbXj8KfhwGQGo5rdcx3NiBqGeWIXZeElhEtXbKZkDGCWFpupcPq1+\/F\/6cpBXeNgfrhceTAk\/YYoWi5Zkg4F4LFJpiyzCiXtUzmXFzAVNnu7j5xVf\/Ovu8KCyZUXj37H+Rmc6YjLr+caooCpsMl31RIwJafnkdMXVO8n\/ltXvOz8x6rNr6aF2bSlfSZKTIFNWu7ShsftjTfeSL3fRCQQNS85M\/AaAqZk9Pvth2fU4J++m6v0y\/taZZ5vsV44kenNo+QgkydPzjQSQsCUjBnEZiqcXhOq96o175zMNfD6+N7qS7X7woOp8C4nqylgSkYvAVMyekGlW\/yrQ+rf\/+dErmK3di3V+n\/4jsL\/Q\/Bfgwe6igMiGZNQaVHz6tMNAPrF+6fU7Dc+rPeSAwkvg4PJ5srGh31OHCZliRCDSdqHci4vYLo0ewAQglbfOnjGOp0miELa7DnJBK8d2UVIQwVXOGXBpOyiWTKfgCkdNZsimACcLftPq7W7apzgAVUBoNfG9VZXd3CHWYXgvzwwUUArUjzRflOoq2RCDCYde5ZX7cYMJoAGz7JfH1H7a\/4YCRwCz9e\/2lI9M\/ybGkg+Twj+y4GJZ1lBIkS6fgRZMClXd1xuPZ9BhBSzSb5f7mUbA5gINP+y+bD68KPzsaChOQNgbr\/mq2rqnRXe4DHnOyiY8DEuhSCd6EaEEE6CEIMpd4Ak6X+5gIkA8\/SWI2rviXgpY9IAwPluj7Zq2sCr9StfyRNHyxD8J968OKqX6PtSARPAAgZ\/eedJ9W87LrmhXc6AKFKiDahqM39wtbr1G22DUz0omOLCiQoNI3JRJcRggs9ACX2gWGACWHC15Cs7T6rfHPhE\/f6TC9qDVshDgOn7jSvUj\/p0zlTSJO1PCP7L3RwYdUG075WISQYUYjBJvl\/uZbMAE0mV5b8+ojbtrS1YqnBbBv+GhIFaRipZVqpZlnMWgv+s3rwsHQ0imbJkgS\/aigITgWRfzR\/Vc785qo37NBLFBpaJ3+um7vxWey2lShEscVQPCqa4j2f9PsRgsu5jKbYH5v1TXZ164bdH1Y4PP1bNmzfXB93SPgQISBUY\/JW3dGpQNSzteOLqh+A\/LzVPIiDipib9e4Dk1Ln\/U6++W6PeP\/ZpJpLElCgATP\/r2qkff7972UqU9JS+1EIQMDVUrvEQg8mK0Fm2A5C89u5HavO+WlVXpzIFSec2zbVkgjT5Zqe\/VA\/c1lW1bH5Zo5YoWc1NCP5zJqGkTodwPoRaGbIitKsdsg\/w\/7cOfqLeOvgHdbj2s0wBgm9zlevbXVurv7+1s2phgCQLB0RoepVy+0HB5LpMmd9ybqaU4mefXKDjV6DYLtUKsSGcdBIJJK\/v+Ui9ubdWHT59IShAAJZ+PdrqXXwOniT9FjAloVb9skHBlHSfiQfG0lWOuPCJ34ZOAKWr4\/mQQgzGRl4A5eyFi6pq+3Ed55WFV4t\/xzTch97UUV1+WbOCQeLLIgImX0rZy4Xgvzw1j9\/pyY+tQ+pE3TjnAg0HnJmrPMRgDp46r\/7xlX0F7cC7AHJzxRXqb2\/pqP7i8tKyRQRMJQ4mdM880xSXhJJAYVPzzCPw\/P5Sepc2o+uU1w6pNe98cZoyisQw2Du1bq5tktHf7aI6fOVLwSVIuil315aMroVTFplwkesEVxplaWZkFptnJuY3h8qj0qEKcqAlzeh6\/OxF9U9vfqyzcpoPAea+b7dW3\/paC4W\/G+MjGV0Ln9UXX3xRJ0fFU5JgAjigJuImblcEBb+kmMCUJKMr7J9f\/u8pNe8\/828bhKR5ZZT\/WZbCp6F0akpG18LnApIJmbaQDDUImCija7du3bzSeZkOC9tt3gDM1q1btb1F7ePfOMGb1GYCkJCqiQdZjvvrrmrBD68tnKplXFNspnSTl5T\/fL4Wu8\/E7RyzQZtr3LyChrvGbTaTz8pgAsl1rt9nwI2ljIAp3UwGB5Otez7qWyHDSjKYQc+8m\/PQCZAuUVvAVAjXfVEnCf\/5filWMmVtpFHHfAfDM3ECSHtm3+o7tkZdTsCUbnp9+S\/JV\/KOYIwePVrf+VNVVZX5HbYuFTFOzaMbC0Qi5VNQwJSEzeuXDQqmdF1LXttnMFy9QybO264Jf5w5+UgapoaAKR3dffgv6Rcy22dK+uG4wfAbC3C+ZsND30n6iUZdXsCUbnrj+K+Q1ksWTFwqwU4qx9OchUyIbx0Bky+l7OWaDJhEKsUzioApnkZRJYKCKWnUeLqhRJ905B48kUp2SguY0nFgEDDV1tbWwYtXStmJSMUTV7ibYQRMJQgmOrYeKqe4a8iulYGreA\/1\/7r6yeBr0lGtkdYWMKWb2GCSCQn7cWftlClTrBKqmAlVRMXzYxIBkx+dki7maVotOW+eqHh+0ylg8qNTkwWTePH8GUTA5E8rW8kgah63mVyOiGKpeRxMuGtn+M0d01GsEdcWMKWb3KBgcnUNRyjoFsF03c+vbRsMLrd6\/BcHdUFxiUdTW8CUjhsbBEyhvHy2wYi95M8gAiZ\/WhVdzYsy1OKOoxcyLBNMXMVDQCsCW+VxU0DAlI47gkomOlZu27yNOyYRNSx+0pZnOjIHw13iYi\/FM4qAKZ5GUSWCgild1+y10eHq6mqdU+Lo0aO5e3KRQ88cTPU7J\/UV9GIv+c2EgMmPTlEaV2VlZZiEKvjogQMH8hge6buyuhwabS9YsEAtXbpUZy8ywcSjxE\/\/8\/fSUaoJ1BYwpZvkoJLJlZU1Lh+ez5BI1bOpeZSEcuJ\/IBH+GZ3nbuf0m32abdJlJAll4dMfPAll6KjxqCSUA+6pVGvrvq+pc2OXlupnQ2R\/KY5VJAllHIXc74uShBJSaMWKFbkcEOSUQJ67qFzjvsNyJaG80LaHgmTCs+7Bv1J9K77i22STLSdJKAuf+uBJKKlrsG1Gjhypjh8\/rn+KyzUe5zHxSUJ5sUMvnWBSnA\/+DCI2kz+tbCWD2kzpuuau7eMa\/93lFeLJSzgBAqaEBDOKBwVTqEgH15D5YObvaqGdD3IY0J9BBEz+tGoQyQQpUlFRkXdhWbouu2tzMI17sy53YbEkmfSjuIDJj04+izl8Alk89ZJQ2iIgQkaNL3zm52rspjo9Fknp5T+lAiZ\/WjWIZErXvWS1STJxMC27r5f6Ud9OyRpqoqUFTOkmPojNhIQqDXlsnYNJYvL8GUTA5E+rJiOZHvrJTxUcEHgkBbI\/gwiY\/GnVIGAKGZtnDojE7PUjFqitf7hKv5YDgf4MImDyp1XRwRQyNi9qMJ\/e9qi62OE6XUQCXP0ZRMDkT6uigyl0bJ5LMhGYZI8pGXMImJLRy8V\/ac7qmW3mpfoKHZvHP05q3tk7F6o\/fbmDuMUT8oaAKSHBjOJBvHmUnYi+lWVsXtRwaTBnfrhKF5M9pmTMIWBKRq+iS6Z03UtW2wTTtIEVatrAq5M10oRLC5jSTX5RJFO6LvrXNsEkbnF\/2qGkgCkZvZqUZBIwJWMOAVMyegmY0tGrUdcWMKWb3kat5skeUzLmEDAlo1fRJROh1fxwyKhx8uYJmJIxh4ApGb2KCibK94BcD1md7\/B1jcuGbXLGEDAlpxmvEVTNK+SkLTZ5Z8yYofvokl782Hrnzp1zyVq4N0\/2mJIzhoApOc2KBiZ8COA4dOiQVyYiM6kkQIMkLMje2qpVK91vV7wf3gmY0jGDgCkd\/YJLpjT3M\/FUyAQmSLvJkyermTNnKqREtq0MsJn+7uavqUX39EhHnSZWW5JQFj7hwZNQFt61SzVt+SNMh8bYsWNzUo9LpgdvaavG9mmbtgtNqr4koSx8uouShJLUsg0bNqhBgwbpvOOzZ8+2ShY+FB\/10JXRFZJp6HXN1NBLpzDk8aTAxYsX1ebNm1WvXr1U9+7dPWtJMVCgpqZG7dy5U7366qthEvcTs8NJMGTIELV69Wo1a9YstX79erV9+\/Y8W4hPSZKMRjyjK27FmDp1qtp0xRD15d0\/V80\/3iczLRQoKgX69OmjFi9erLp27ZrJd\/OyEyEXxNy5c1VtbW0OTAAZ\/Y7bK0wgRV3RCVXOldEV7QBQ+E8eoUBDUAAgygpI6H\/eeSbyyI0aNUqtXbtWjR8\/Xk2YMEHvO5m5xm0bvKQaLlq0SIMS4OOucW4zNQTx5JtCgZAUyAMTPmSCJE2u8ZAdl7aFAqVGgXpgKrUOSn+EAuVCAQFTucyU9LPkKZAHJu4ap57DDuJRDVmNiKuTokq6qcrnxBWyZaYaENs0nkujonPia9tL5MDEXePkbKDfUDVLQPHICLTN77otdCCNtR7fw3NtQ9iiTxorPbIYFwV1I69+kOxExUz1hckHY6xatUrH8U2fPl0NHz68KNHqWUxGsdqwbXTT7fUUsoW++GyaF6vPpf4d0HT58uXq3nvvVVOmTNFe6qxOSeSpebYVLsmmrC8h+XdQB2Dq169fUa6y8e1jKZQzVRG+CNGen6ma88j8UhhDqfYhxJEjrytliCBZHRIUMPmxmA+YzJZsgPP7WtMqFRRMxSSlqHl+1PZV83hrZr54vy81vVLBwVQsb544IPyZN84BgTmbP3++GjFihD7mgvJRsZT+X27cJYOCyebNAzlth\/6yIDN3jWfpUcmib6XUhhnJT15VgAbP0KFDFXeNi83kN3tBwVRMb57fcKWUUKC8KFAv0BVnmaqqqrTKQCseNm7NQNfyGqb0VigQngL1wol4khR8vrFHJ\/ju0WS5RcBVN59ohRC79UlZi\/c5a54glT9UtE3SsRZavsnH5jUEmJJmgiolMIXaXIcWRAdS+YZ0oYzdEPWaDJhsRjoOQVZWVmq6k4TgkpmM+d27d+dSmtGqzM9puVZq0zsKR0vPnj0VJa5xOQt426iD\/T1sbPfu3VtHjSALFP+mrc849IY6Z86cUdu2bdNhM3j4eHk2qbjx2Pa8EI3RunVr3XZUqreKigrtKIlaFARMDQH\/Ar5pYwQ6Acwlk7nhyY\/ZczWPu59xUnjkyJFqyZIl9cJSuCcUcWAIX4E92r59e2fWJt4fSqf2xBNPKBy4xANvHm8LCwKFZtFhTJSbNGmSBhMAC3uXe68InNQeT03gGo+NhgAmX1zQnmlbc7oJmApg3lKrEjWJUWqebY9n8ODB9cKfbPaU+U2+ATtgwAArmFz9NDdvo1KoUZ8JTBSmZYaK0d9z5sxR8+bNywvn8hmPufCYeRSJBwRMpYaGDPrDI4W5SsIBY9u0JvWPmILABK8nf0xHgm0fg9pwgcm192GCjIOJ1DneH\/SFwEQ2jrmZa4IpbjwuNY\/2vQRMRg6IDHi2LJrgqzRUHMpiazJclGSKM8RDSCb6JgcT7Dke8WBKJqoTJ5mSjseUTK6YQC6ZojZKxWYqC+gobS\/wzLI8axIHDAcTwABHASWTcdlMVG7YsGH1ot4LsZm4nUZxdlDDXnjhhdwxFReYeJ9NyeRrM7nG47KZKHqF95uzhUkD2Fm2iBcBU5mACd3k4UtczaPfoRqNGTMm52lDGfy9ceNGbfRDguGSAps3z7VXZPPmAZxRNo+tDjkMbJIJzgzyDvI+ky3EJQ6NFU4J1NmzZ0\/u0GdcFikbmFauXKk5AM4cvkdkk0ZwmgBI586d04sCvJp8gRMwlRGYpKv5FHBJEhed4mymtPQVMKWloNQvGgXMPBFJow3MCAikZLad+i1kQBIBUQjVpI5QoBFT4P8BVYnXFEAZNL4AAAAASUVORK5CYII=","height":0,"width":0}}
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
