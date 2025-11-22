% Script done by GEMINI on 2025/11/16 at 8.19 PM

%==========================================================================
% HIGH-FREQUENCY TRANSFORMER DIMENSIONING (9.6 kHz)
% Core Material: Nanocrystalline Alloy (Finemet-type)
% REVISED SCRIPT
%==========================================================================

clc;
clear;
close all;

%% 1. Input Data and Design Parameters
%--------------------------------------------------------------------------
V1 = 1250;           % Primary RMS Voltage [V]
I1 = 400;            % Primary RMS Current [A]
f = 9600;            % Operating Frequency [Hz]
ratio_V = 1;         % Transformation Ratio V1/V2 (1:1)
Sn = V1 * I1;        % Apparent Power [VA]

% Design Parameters (Optimized for Nanocrystalline at 9.6 kHz)
Bmax = 0.8;          % Max Magnetic Flux Density [T]
J = 2.5;             % Current Density [A/mm^2] 
rho_Cu = 1.72e-8;    % Copper Resistivity [Ohm*m]
mu0 = 4*pi*1e-7;     % Permeability of Free Space [H/m]
Cos_phi = 0.95;      % Power Factor (Assumed)

% CORE LOSS PARAMETERS (Nanocrystalline Estimate)
P_spec = 12;         % Specific Core Loss [W/kg] (Estimate for 9.6 kHz and 0.8 T)
rho_Fe = 7800;       % Nanocrystalline Alloy Density [kg/m^3]

disp('--- INITIAL ELECTRICAL PARAMETERS ---');
fprintf('Nominal Power (Sn): %.2f kVA\n', Sn/1000);

%% 2. Core Area (S_Fe) and Turns (N) Calculation
% We constrain N1 to 5 turns to set the magnetic flux density Bmax=0.8T
N1 = 5; 
N2 = 5; % N2 = N1 / ratio_V

% Calculate the required core area based on Faraday's Law
S_Fe = V1 / (4.44 * f * Bmax * N1); % Core Area [m^2]

% Calculate Currents and Copper Cross-Sections
I2 = I1 / ratio_V;
% Correction: J is in A/mm^2, so we divide by J and multiply by 1e-6 to get m^2
A_Cu1 = I1 / (J * 1e6); % Copper Area Primary [m^2]
A_Cu2 = I2 / (J * 1e6); % Copper Area Secondary [m^2]

disp('----------------------------------------------------');
fprintf('Core Section Area (S_Fe): %.4f m^2 (%.2f cm^2)\n', S_Fe, S_Fe*1e4);
fprintf('Primary Turns (N1): %d\n', N1);
fprintf('Primary Copper Area (A_Cu1): %.2f mm^2\n', A_Cu1*1e6);
disp('----------------------------------------------------');

%% 3. Core Loss (P_Fe) Estimation
% Estimation of Core Volume and Mass
L_Fe = 1.6; % Mean magnetic path length [m] (Estimate for a 500 kVA core)
Vol_Fe = S_Fe * L_Fe; % Core Volume [m^3]
M_Fe = Vol_Fe * rho_Fe; % Core Mass [kg]

P_Fe = P_spec * M_Fe; % Core Loss [W]

%% 4. Copper Loss (P_Cu) Estimation
L_turn = 0.8; % Mean length per turn [m] (Estimate)
L1 = N1 * L_turn; % Total Primary Conductor Length [m]
L2 = N2 * L_turn; % Total Secondary Conductor Length [m]

% Resistance and Loss Calculation (Assuming Litz wire to mitigate skin/proximity effect)
R1 = rho_Cu * L1 / A_Cu1; % Primary Resistance [Ohm]
R2 = rho_Cu * L2 / A_Cu2; % Secondary Resistance [Ohm]

P_Cu = (I1^2 * R1) + (I2^2 * R2); % Total Copper Loss [W]

disp('--- LOSS ESTIMATION ---');
fprintf('Core Mass (M_Fe): %.2f kg\n', M_Fe);
fprintf('Core Loss (P_Fe): %.2f kW\n', P_Fe/1000);
fprintf('Copper Loss (P_Cu): %.3f kW\n', P_Cu/1000); % Showing more precision here
fprintf('Total Losses (P_tot): %.3f kW\n', (P_Fe + P_Cu)/1000);
disp('----------------------------------------------------');

%% 5. Efficiency (Eta) Calculation
P_out = Sn * Cos_phi; % Active Output Power [W]
P_tot = P_Fe + P_Cu; % Total Losses [W]

Eta = (P_out / (P_out + P_tot)) * 100; % Efficiency [%]

fprintf('Estimated Efficiency (Eta, cos(phi)=%.2f): %.2f %%\n', Cos_phi, Eta);
disp('----------------------------------------------------');

%% 6. Leakage Inductance (Ld) Estimation
% Estimated Geometric Parameters (Highly dependent on physical core selection)
L_avv = 0.35;         % Winding Length along the core leg [m]
h1 = 0.01;            % Radial Thickness of Primary Winding [m]
h2 = 0.01;            % Radial Thickness of Secondary Winding [m]
d = 0.005;            % Radial distance between P and S (Insulation) [m]

% Ld calculation (referred to Primary) using simplified concentric model
L_d_calc = (mu0 * N1^2 / L_avv) * ( (h1 + h2)/3 + d );

% Correction Factor for Skin/Proximity Effect (Fp)
Fp = 1.3; 
L_d_eff = L_d_calc * Fp; % Effective Leakage Inductance [H]

% Calculate Leakage Reactance (Xd)
Xd = 2 * pi * f * L_d_eff;

% Estimate Short Circuit Voltage (Vcc%)
Vcc_perc = (Xd * I1 / V1) * 100;

% Output Results Leakage Inductance (Corrected Display)
disp('--- LEAKAGE INDUCTANCE AND REACTANCE ESTIMATION ---');
fprintf('Calculated Leakage Inductance (Ld_calc): %.6f H (%.2f uH)\n', L_d_calc, L_d_calc * 1e6);
fprintf('Effective Leakage Inductance (Ld_eff): %.6f H (%.2f uH)\n', L_d_eff, L_d_eff * 1e6);
fprintf('Leakage Reactance (Xd): %.3f Ohm\n', Xd);
fprintf('Estimated Short Circuit Voltage (Vcc): %.2f %%\n', Vcc_perc);
disp('----------------------------------------------------');