% ========================================================================
% Model Parameters for HMPSG Governor Simulation (ME319)
% ========================================================================

clear; clc;

% System & Engine Parameters
J = 1.0;       % Normalized Engine Rotational Inertia
B = 0.1;       % Internal Viscous Friction

% Governor Linkage & Dashpot Parameters
M_eq = 0.5;    % Equivalent Linkage Mass
c_d = 0.8;     % Viscous Dashpot Damping Coefficient
K_s = 2.0;     % Reference Spring Stiffness

% Control & Sensing Gain
K_p = 1.5;     % Fluidic Pressure-Sensing Control Gain

disp('Model parameters loaded successfully.');
