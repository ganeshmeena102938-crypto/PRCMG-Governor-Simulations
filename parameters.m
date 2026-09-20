% =========================================================================
% Model Parameters for PRCMG Governor Simulation (ME319)
% =========================================================================

clear; clc;

% System & Engine Parameters
J = 1.0;       % Normalized Engine Rotational Inertia
B = 0.1;       % Internal Viscous Friction

% Governor Linkage & Dashpot Parameters
M_eq = 0.5;    % Equivalent Linkage Mass
c_d = 0.8;     % Viscous Dashpot Damping Coefficient
K_s = 2.0;     % Reference Spring Stiffness

% Control & Sensing Gains
K_w = 1.5;     % Linearized Centrifugal Gain
K_t = 1.0;     % Throttle Valve Gain Constant

disp('Model parameters loaded successfully.');
