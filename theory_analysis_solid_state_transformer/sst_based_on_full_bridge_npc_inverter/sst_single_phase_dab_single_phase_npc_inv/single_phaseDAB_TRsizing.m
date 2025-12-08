
%% 1. Input Data and Design Parameters
%--------------------------------------------------------------------------
V1 = 800;            % Primary RMS Voltage [V]
I1 = 375;            % Primary RMS Current [A]
I2 = 375;            % Primary RMS Current [A]
f = 12e3;             % Operating Frequency [Hz]
n12 = I2/I1;             % Transformation Ratio V1/V2 (1:1)
Sn = V1 * I1;        % Apparent Power [VA]

% Design Parameters (Optimized for Nanocrystalline at 12 kHz)
Bmax = 0.5;          % Max Magnetic Flux Density [T]
J = 3.0;             % Current Density [A/mm^2] 
rho_Cu = 1.72e-8;    % Copper Resistivity [Ohm*m]
mu0 = 4*pi*1e-7;     % Permeability of Free Space [H/m]
Cos_phi = 0.95;      % Power Factor (Assumed)

% CORE LOSS PARAMETERS (Nanocrystalline Estimate)
fbase = 20e3; % [Hz]
Bbase = 0.3;  % [Wb/m^2]
P_spec = 16 * f/fbase * Bmax/Bbase;        % Specific Core Loss [W/kg]
rho_Fe = 7800;       % Nanocrystalline Alloy Density [kg/m^3]

disp('--- INITIAL ELECTRICAL PARAMETERS ---');
fprintf('Nominal Power (Sn): %.2f kVA\n', Sn/1000);
fprintf('Nominal Primary Voltage (Vn): %.2f V\n', V1);
fprintf('Nominal Primary Current (I1n): %.2f V\n', I1);
fprintf('Nominal Secondary Current (I2n): %.2f V\n', I2);
fprintf('Nominal Frequency: %.2f kHz\n', f/1e3);

%% 2. Core Area (S_Fe) and Turns (n) Calculation
% We constrain N1 to 5 turns to set the magnetic flux density Bmax=0.8T
n1 = 6; 
n2 = n1*n12; % N2 = N1 / ratio_V

% Calculate the required core area based on Faraday's Law
S_Fe = V1 * 1e4 / (4.44 * f * Bmax * n1); % Core Area [cm^2]

% Correction: J is in A/mm^2, so we divide by J and multiply by 1e-6 to get m^2
A_Cu1 = I1 / J; % Copper Area Primary [mm^2]
A_Cu2 = I2 / J; % Copper Area Secondary [mm^2]

% Copper band length supposing a thikness of 0.5mm
Lband1 = A_Cu1 / 0.5; % Copper band length [mm]
Lband2 = A_Cu2 / 0.5; % Copper band length [mm]

% Core Depth/Width using AM-NC-412 AMMET Nanocrystalline cut cores
L_core_height = 29; % Core width in [cm]
L_core_width = 6; % Core width in [cm]
L_core_length = 95; % Core depth in [cm]
L_core_depth = S_Fe/L_core_width; % Core depth in [cm]
L_core_depth_sel = 10; % Core depth in [cm]

disp('----------------------------------------------------');
fprintf('Core Section Area (S_Fe): %.4f cm^2\n', S_Fe);
fprintf('Primary Turns (n1): %d\n', n1);
fprintf('Secondary Turns (n2): %d\n', n2);
fprintf('Primary Copper Area (A_Cu1): %.2f mm^2\n', A_Cu1);
fprintf('Primary Copper Band Length: %.2f cm\n', Lband1/10);
fprintf('Secondary Copper Band Length (L_b2): %.2f cm\n', Lband2/10);
fprintf('Core Height (AM-NC-412 AMMET): %.2f cm\n', L_core_height);
fprintf('Core Width (AM-NC-412 AMMET): %.2f cm\n', L_core_width);
fprintf('Core Length (AM-NC-412 AMMET): %.2f cm\n', L_core_length);
fprintf('Core Dept (AM-NC-412 AMMET): %.2f cm\n', L_core_depth);
fprintf('Core Dept selected (AM-NC-412 AMMET): %.2f cm\n', L_core_depth_sel);
fprintf('Specific Core Loss (AM-NC-412 AMMET): %.2f W/kg\n', P_spec);
disp('----------------------------------------------------');

L_core_depth = L_core_depth_sel;

%% 3. Core Loss (P_Fe) Estimation (AM-NC-412 AMMET Nanocrystalline cut cores)
% Estimation of Core Volume and Mass
L_Fe = L_core_length;                % Mean magnetic path length [m]
Vol_Fe = S_Fe * L_Fe * 1e-6;       % Core Volume [m^3]
M_Fe = Vol_Fe * rho_Fe;     % Core Mass [kg]

P_Fe = P_spec * M_Fe;       % Core Loss [W]

%% 4. Copper Loss (P_Cu) Estimation
L_turn = (L_core_depth*2 + L_core_width*2) * 1e-2; % Mean length per turn [m] (Estimate)
L1 = n1 * L_turn; % Total Primary Conductor Length [m]
L2 = n2 * L_turn; % Total Secondary Conductor Length [m]

% Resistance and Loss Calculation (Assuming Litz wire to mitigate skin/proximity effect)
R1 = rho_Cu * L1 / A_Cu1 * 1e6; % Primary Resistance [Ohm]
R2 = rho_Cu * L2 / A_Cu2 * 1e6; % Secondary Resistance [Ohm]

P_Cu = (I1^2 * R1) + (I2^2 * R2); % Total Copper Loss [W]

disp('--- LOSS ESTIMATION ---');
fprintf('Core Mass (M_Fe): %.2f kg\n', M_Fe);
fprintf('Core Loss (P_Fe): %.2f W\n', P_Fe);
fprintf('Copper Loss (P_Cu): %.2f W\n', P_Cu); % Showing more precision here
fprintf('Total Losses per Phase (P_tot): %.2f W\n', (P_Fe + P_Cu));
disp('----------------------------------------------------');

%% 5. Efficiency (Eta) Calculation
P_out = Sn * Cos_phi; % Active Output Power [W]
P_tot = P_Fe + P_Cu; % Total Losses [W]

Eta = (P_out / (P_out + P_tot)) * 100; % Efficiency [%]

fprintf('Estimated Efficiency (Eta, cos(phi)=%.2f): %.2f %%\n', Cos_phi, Eta);
disp('----------------------------------------------------');

%% 6. Leakage Inductance (Ld) Estimation
% Estimated Geometric Parameters (Highly dependent on physical core selection)
L_avv = Lband1 * 1e-3;      % Winding Length along the core leg [m]
h1 = 50.0 * 1e-3;           % Radial Thickness of Primary Winding [m]
h2 = 50.0 * 1e-3;           % Radial Thickness of Secondary Winding [m]
d = h1 + h2;                % Radial distance between P and S (Insulation) [m]

% Ld calculation (referred to Primary) using simplified concentric model
L_d_calc = (mu0 * n1^2 / L_avv) * ( (h1 + h2)/3 + d );

% Correction Factor for Skin/Proximity Effect (Fp)
Fp = 1.3; 
L_d_eff = L_d_calc * Fp; % Effective Leakage Inductance [H]

% Calculate Leakage Reactance (Xd)
Xd = 2 * pi * f * L_d_eff;

% Estimate Short Circuit Voltage (Vcc%)
Vcc_perc = (Xd * I1 / V1) * 100;

K = 0.5;
Lband1_m = Lband1/1000; % [m]
L_core_depth_m = L_core_depth/100; % [m]
L_core_width_m = L_core_width/100; % [m]
Lsigma = mu0 * n1^2 * K * Lband1_m * L_core_depth_m / L_core_width_m;

% Output Results Leakage Inductance (Corrected Display)
disp('--- LEAKAGE INDUCTANCE AND REACTANCE ESTIMATION ---');
fprintf('Calculated Leakage Inductance (Ld_calc): %.6f H (%.2f uH)\n', L_d_calc, L_d_calc * 1e6);
fprintf('Effective Leakage Inductance (Ld_eff): %.6f H (%.2f uH)\n', L_d_eff, L_d_eff * 1e6);
fprintf('Leakage Reactance (Xd): %.3f Ohm\n', Xd);
fprintf('Estimated Short Circuit Voltage (Vcc): %.2f %%\n', Vcc_perc);
disp('----------------------------------------------------');