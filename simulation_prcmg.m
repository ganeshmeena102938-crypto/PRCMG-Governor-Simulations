% =========================================================================
% HMPSG Governor Simulation Script (IEEE Control Systems Report - ME319)
% Description: Comprehensive simulation including Open/Closed-Loop responses,
%              Throttle Valve Position, Tracking Error, Frequency Response (Bode),
%              and Control Effort (Valve Velocity).
% =========================================================================

clear; clc; close all;

%% 1. System Parameters (Normalized Values)
J   = 1.0;   % Engine rotational inertia
B   = 0.1;   % Engine viscous friction
Meq = 0.5;   % Equivalent linkage mass
cd  = 0.8;   % Dashpot damping coefficient
Ks  = 2.0;   % Reference spring stiffness
Kp  = 1.5;   % Fluidic Pressure-Sensing Control Gain

%% 2. Transfer Function Definitions
Gp = tf(1, [J, B]);
Gc = tf(Kp, [Meq, cd, Ks]);

Gol = Gc * Gp;                     
Gcl_ref = feedback(Gol, 1);         
G_dist  = feedback(Gp, Gc);         

%% 3. Time Vector & Disturbance Inputs
t = 0:0.01:25; 

delta_inc = 0.3;  % +30% Load disturbance
delta_dec = -0.3; % -30% Load disturbance

%% 4. Response Calculations 
% Open-Loop responses (Load increase drops speed, load decrease surges speed)
[omega_ol_inc, ~] = step(-delta_inc * G_dist, t);
[omega_ol_dec, ~] = step(-delta_dec * G_dist, t);

% Closed-Loop responses (HMPSG regulated suppression)
[omega_cl_inc, ~] = step(delta_inc * feedback(1, Gol), t);
[omega_cl_dec, ~] = step(delta_dec * feedback(1, Gol), t);

[xv_cl_inc, ~] = step(delta_inc * (Gc / (1 + Gol)), t);
[xv_cl_dec, ~] = step(delta_dec * (Gc / (1 + Gol)), t);

% Additional Extra Calculations:
% 1. Tracking Error (Deviation from setpoint 1.0 for closed-loop)
error_inc = omega_cl_inc;
error_dec = omega_cl_dec;

% 3. Control Effort / Valve Velocity (Numerical derivative of valve position)
dt = t(2) - t(1);
vel_inc = [0, diff(xv_cl_inc) / dt]';
vel_dec = [0, diff(xv_cl_dec) / dt]';

%% 5. Plotting Results (Figure 1: Time-Domain Transient Responses & Extra Metrics)
figure('Name', 'HMPSG Comprehensive Analysis', 'Position', [50, 50, 1000, 1000], 'Color', 'w');

% --- Subplot 1: Engine Speed Response ---
subplot(4,1,1);
plot(t, omega_ol_inc + 1, '--r', 'LineWidth', 1.8); hold on;
plot(t, omega_cl_inc + 1, '-r',  'LineWidth', 2.2);
plot(t, omega_ol_dec + 1, '--b', 'LineWidth', 1.8);
plot(t, omega_cl_dec + 1, '-b',  'LineWidth', 2.2);
grid on; grid minor;
title('Engine Speed Response \omega(t) under Step Load Disturbances');
ylabel('Speed (\omega)');
legend('Open-Loop (+30%)', 'Closed-Loop (+30%)', ...
       'Open-Loop (-30%)', 'Closed-Loop (-30%)', ...
       'Location', 'southeast');
xlim([0 25]);

% --- Subplot 2: Throttle Valve Actuation Profile ---
subplot(4,1,2);
plot(t, xv_cl_inc + 1, '-r', 'LineWidth', 2.0); hold on;
plot(t, xv_cl_dec + 1, '-b', 'LineWidth', 2.0);
grid on; grid minor;
title('Throttle Valve Actuation Profile x_v(t)');
ylabel('Valve Pos (x_v)');
legend('Valve (+30%)', 'Valve (-30%)', 'Location', 'northeast');
xlim([0 25]);

% --- Subplot 3: Tracking Error Response ---
subplot(4,1,3);
plot(t, error_inc, '-r', 'LineWidth', 1.8); hold on;
plot(t, error_dec, '-b', 'LineWidth', 1.8);
grid on; grid minor;
title('Closed-Loop Speed Tracking Error e(t)');
ylabel('Error');
legend('Error (+30%)', 'Error (-30%)', 'Location', 'northeast');
xlim([0 25]);

% --- Subplot 4: Control Effort / Valve Velocity ---
subplot(4,1,4);
plot(t, vel_inc, '-r', 'LineWidth', 1.8); hold on;
plot(t, vel_dec, '-b', 'LineWidth', 1.8);
grid on; grid minor;
title('Control Effort - Throttle Valve Velocity (dx_v/dt)');
xlabel('Time (s)');
ylabel('Velocity');
legend('Velocity (+30%)', 'Velocity (-30%)', 'Location', 'northeast');
xlim([0 25]);

%% 6. Save Main Figure
saveas(gcf, 'simulation_plots.png');
disp('Main time-domain simulation plots saved as simulation_plots.png');

%% 7. Frequency Response (Bode Plot Figure)
figure('Name', 'HMPSG Frequency Response - Bode Plot', 'Position', [1100, 50, 700, 500], 'Color', 'w');
bode(Gol);
grid on; grid minor;
title('Bode Diagram of Open-Loop System Gol(s)');
saveas(gcf, 'bode_plot.png');
disp('Bode frequency response plot saved as bode_plot.png');
