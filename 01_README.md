# Hydro-Mechanical Pressure-Sensing Governor (HMPSG) Simulation

This repository contains the official MATLAB/Octave simulation code, mathematical models, and transient response results for the IEEE-style control systems report.

---

## Project & Student Details
* **Project Title:** Hydro-Mechanical Pressure-Sensing Governor (HMPSG) Simulation
* **Course:** ME319 (Control Systems) — Assignment 1
* **Prepared By:** Ganesh Kumar Meena
* **Roll Number:** 24B2295
* **Submission Type:** Individual Assignment

---

## Project Overview
The **Hydro-Mechanical Pressure-Sensing Governor (HMPSG)** integrates a gear-pump-driven fluidic pressure sensing piston with a dashpot-based rate compensation linkage to automatically regulate engine speed against variable external step load disturbances. This project analyzes the linear time-invariant (LTI) model, closed-loop stability via the Routh-Hurwitz criterion, and transient performance under $\pm 30\%$ load changes.

---

## Software & Toolchain Requirements
To successfully run and reproduce the simulations, ensure the following software is installed:
* **MATLAB** (Recommended: R2024a or newer) or **GNU Octave** (v8.0 or higher)
* **Control System Toolbox** (Required for transfer function definitions and step response functions)

---

## Model Parameters
The simulation uses the following normalized physical parameters:
* **Engine Inertia ($J$):** $1.0$
* **Engine Friction ($B$):** $0.1$
* **Equivalent Linkage Mass ($M_{eq}$):** $0.5$
* **Dashpot Damping ($c_d$):** $0.8$
* **Spring Stiffness ($K_s$):** $2.0$
* **Combined Control Gain ($K_p K_t$):** $1.5$ ($K_p = 1.5, K_t = 1.0$)
---

## How to Run and Reproduce Results

Follow these steps to execute the simulation and generate the plots locally:

1. **Clone or Download** this repository to your local machine:
   ```bash
  git clone https://github.com/ganeshmeena102938-crypto/PRCMG-Governor-Simulations.git
