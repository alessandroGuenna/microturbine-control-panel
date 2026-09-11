# Microturbine Control Panel

Interactive MATLAB application that simulates the **control panel of a ~80 kW
cogeneration (CHP) gas microturbine**. Behind the interface (MATLAB App Designer)
runs a dynamic Simulink model of the single-shaft recuperated Brayton cycle, with
heat recovery for domestic hot water.

> **Note.** The model does not reproduce a specific real machine: it is a
> learning/practice project. With accurate manufacturer data it could be
> calibrated towards a *digital twin* of an existing microturbine.

🔗 **Full description and model deep-dive:**
https://alessandroguenna.github.io/projects/microturbine/

## Contents

| File / folder | Description |
|---|---|
| `ControlPanel.mlapp` | The app interface (App Designer) |
| `microturbine_model_v2.slx` | Dynamic Simulink model of the cycle |
| `data_v2.m` | Initialisation: parameters, design point, speed schedule |
| `MATLAB functions/` | Support functions (compressor maps, design point, recuperator, control…) |
| `Allegati/` | Compressor-map data and model resources |

## How to run it

1. Open the project in **MATLAB** (with **Simulink** and **Simscape**).
2. Open `ControlPanel.mlapp` and press **Run**.
3. In the app: set the ambient temperature → **Initialize** → **START**,
   then vary the load with the *Set load* slider.

## Model

Single-shaft recuperated Brayton cycle: compressor (map) → recuperator →
combustion chamber → turbine (choked) → recuperator → heat-recovery boiler (HRB).
The controller tracks the requested power with an **optimal-speed schedule**
(maximum efficiency) and protections (surge margin, maximum turbine temperature,
proportional load shedding).
