% =========================================================================
% PRCMG Governor Simulation Script (IEEE Control Systems Report - ME319)
% Description: Simulates Open-Loop vs Closed-Loop (PRCMG) transient responses 
%              under +30% and -30% step load disturbances for engine speed 
%              omega(t) and throttle valve position xv(t).
% =========================================================================

clear; clc; close all;

%% 1. System Parameters (Normalized Values)
J   = 1.0;   % Engine rotational inertia
B   = 0.1;   % Engine viscous friction
Meq = 0.5;   % Equivalent linkage mass
cd  = 0.8;   % Dashpot damping coefficient
Ks  = 2.0;   % Reference spring stiffness
Kw  = 1.5;   % Centrifugal control gain

%% 2. Transfer Function Definitions
Gp = tf(1, [J, B]);
Gc = tf(Kw, [Meq, cd, Ks]);

Gol = Gc * Gp;                       
Gcl_ref = feedback(Gol, 1);          
G_dist  = feedback(Gp, Gc);          

%% 3. Time Vector & Disturbance Inputs
t = 0:0.01:25; 

delta_inc = 0.3; 
delta_dec = -0.3;

%% 4. Response Calculations (Corrected Open-Loop Signs)
% Open-Loop responses (Load increase drops speed, load decrease surges speed)
[omega_ol_inc, ~] = step(-delta_inc * G_dist, t);
[omega_ol_dec, ~] = step(-delta_dec * G_dist, t);

% Closed-Loop responses (PRCMG regulated suppression)
[omega_cl_inc, ~] = step(delta_inc * feedback(1, Gol), t);
[omega_cl_dec, ~] = step(delta_dec * feedback(1, Gol), t);

[xv_cl_inc, ~] = step(delta_inc * (Gc / (1 + Gol)), t);
[xv_cl_dec, ~] = step(delta_dec * (Gc / (1 + Gol)), t);

%% 5. Plotting Results
figure('Name', 'Simulation Results', 'Position', [100, 100, 1000, 700], 'Color', 'w');

subplot(2,1,1);
plot(t, omega_ol_inc + 1, '--r', 'LineWidth', 1.8); hold on;
plot(t, omega_cl_inc + 1, '-r',  'LineWidth', 2.2);
plot(t, omega_ol_dec + 1, '--b', 'LineWidth', 1.8);
plot(t, omega_cl_dec + 1, '-b',  'LineWidth', 2.2);
grid on; grid minor;
title('Engine Speed Response \omega(t) under Step Load Disturbances');
xlabel('Time (s)');
ylabel('Normalized Speed (\omega)');
legend('Open-Loop (+30% Load)', 'Closed-Loop (+30% Load)', ...
       'Open-Loop (-30% Load)', 'Closed-Loop (-30% Load)', ...
       'Location', 'southeast');
xlim([0 25]);

subplot(2,1,2);
plot(t, xv_cl_inc + 1, '-r', 'LineWidth', 2.0); hold on;
plot(t, xv_cl_dec + 1, '-b', 'LineWidth', 2.0);
grid on; grid minor;
title('Throttle Valve Actuation Profile x_v(t)');
xlabel('Time (s)');
ylabel('Valve Position (x_v)');
legend('Valve Response (+30% Load)', 'Valve Response (-30% Load)', ...
       'Location', 'northeast');
xlim([0 25]);

%% 6. Save Figure
saveas(gcf, 'simulation_plots.png');
disp('Simulation executed successfully. Plot saved as simulation_plots.png');
