# PRCMG Governor Simulation (ME319 Report)
**Project Title:** Inertial–Hydraulic Dashpot Governor with Mechanical Rate Compensation (PRCMG)

## Overview
This repository contains the MATLAB/Octave simulation code corresponding to **Section 4.5 (Simulation & Results)** of the IEEE-style control systems report. 

## File Description
- `simulation_prcmg.m`: Executes the LTI model simulation comparing Open-Loop versus Closed-Loop (PRCMG) behavior under step load disturbances.

## Simulation Scenarios Covered
1. **Case 1 (+30% Load Increase):** $T_L: 1 \rightarrow 1.3$ (Open-loop drift vs. Closed-loop regulation for speed $\omega(t)$ and valve position $x_v(t)$).
2. **Case 2 (-30% Load Decrease):** $T_L: 1 \rightarrow 0.7$ (Open-loop surge vs. Closed-loop damped suppression).

## How to Run
1. Open MATLAB or GNU Octave.
2. Run the script `simulation_prcmg.m`.
3. The generated plots will display the engine speed response $\omega(t)$ and throttle valve actuation $x_v(t)$ for all open-loop and closed-loop cases.
