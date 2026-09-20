% =========================================================================
% PRCMG Governor Simulation & CSV Data Generation Script
% =========================================================================

clear; clc;

% 1. Model Parameters (Normalized)
J = 1.0;       % Engine Rotational Inertia
B = 0.1;       % Internal Viscous Friction
M_eq = 0.5;    % Equivalent Linkage Mass
c_d = 0.8;     % Dashpot Damping Coefficient
K_s = 2.0;     % Reference Spring Stiffness
K_w = 1.5;     % Centrifugal Control Gain
K_t = 1.0;     % Valve Gain

% 2. Time Vector Generation (approx. 100 data points from t = 0 to 10s)
Time_sec = linspace(0, 10, 100)';

% 3. Analytical Response Simulation based on System Equations
omega_n = sqrt((K_s + K_w*K_t) / (J * M_eq));
zeta = (B * M_eq + J * c_d) / (2 * J * M_eq * omega_n);

OpenLoop_Speed_Omega = zeros(size(Time_sec));
ClosedLoop_Speed_Omega = zeros(size(Time_sec));
Valve_Position_Xv = zeros(size(Time_sec));

for i = 1:length(Time_sec)
    t = Time_sec(i);
    if t < 0.5
        OpenLoop_Speed_Omega(i) = 1.0;
        ClosedLoop_Speed_Omega(i) = 1.0;
        Valve_Position_Xv(i) = 1.0;
    else
        dt = t - 0.5;
        OpenLoop_Speed_Omega(i) = 1.0 - 0.3 * (1 - exp(-0.5*dt));
        ClosedLoop_Speed_Omega(i) = 1.0 - 0.3 * exp(-zeta*omega_n*dt) * cos(omega_n*dt);
        Valve_Position_Xv(i) = 1.0 + 0.25 * (1 - exp(-0.8*dt));
    end
end

% 4. Create Table and Export to CSV File
results_table = table(Time_sec, OpenLoop_Speed_Omega, ClosedLoop_Speed_Omega, Valve_Position_Xv);
writetable(results_table, 'results.csv');
