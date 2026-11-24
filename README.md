# 3D Wrist Joint Movement Modeling & PID Control Simulation

Biomechanical 3D wrist joint simulation for **EB234506 Biomodeling** (Biomedical Engineering Department, Institut Teknologi Sepuluh Nopember), provided in both **Lazarus / Free Pascal** (Linux/Windows/macOS) and **Delphi VCL** (Windows, Delphi 11/12/13).

## Model Overview

- **Anthropometry (Dempster Regression):** Default `BW = 60 kg`, `BH = 160 cm` (`m = 0.006 * BW`, `l = 0.0575 * BH`), uniform rod inertia $I_g = \frac{m l^2}{12}$ and $I_o = I_g + m\left(\frac{l}{2}\right)^2$.
- **Lagrangian Dynamics (Single Pendulum per Plane):**
  $$I_o \ddot{\theta} + m g \frac{l}{2} \sin(\theta) = \tau_{\text{active}} + \tau_{\text{passive}}$$
  Integrated via 4th-order Runge-Kutta (RK4, $\Delta t = 0.001\text{ s}$).
- **Linear Passive Joint Torque:**
  $$\tau_{\text{passive}} = -C\dot{\theta} - k(\theta - \theta_0)$$
  Includes dedicated **Passive Movement Test** modes for both Frontal (Dorsal/Palmar) and Sagittal (Radial/Ulnar) axes.
- **Active PID Control:** Sinusoidal reference ($\pm 30^\circ$, $90^\circ$ phase offset for transversal circumduction) with live `tau1`/`tau2`, `Target` vs `Realize`, `error 1`/`error 2`, `RMSE`, and `Transversal Phase Angle` plots.

## Project Structure

```
3D-Wrist-Joint/
├── lazarus/                 # Cross-platform Lazarus 4.x / FPC 3.2+ (OpenGLContext + TAChart)
│   ├── WristSim.lpi
│   ├── WristSim.lpr
│   ├── uMainForm.pas        # Single-file physics, 3D OpenGL renderer, and UI
│   ├── uMainForm.lfm
│   └── LAZARUS_TUTORIAL.md  # Step-by-step beginner guide
└── delphi/                  # Windows Embarcadero Delphi 11/12/13 VCL (OpenGL + TeeChart)
    ├── WristSim.dproj
    ├── WristSim.dpr
    ├── uMainForm.pas        # Single-file physics, 3D OpenGL renderer, and UI
    └── uMainForm.dfm
```

## Quick Start

### Lazarus (Linux / Windows / macOS)
1. Open `lazarus/WristSim.lpi` in Lazarus IDE (`startlazarus`) and press **`F9`**.
2. Or build from CLI:
   ```bash
   lazbuild --ws=qt6 lazarus/WristSim.lpi
   ./lazarus/WristSim
   ```
3. New to Lazarus? Follow [`lazarus/LAZARUS_TUTORIAL.md`](lazarus/LAZARUS_TUTORIAL.md).

### Delphi (Windows, RAD Studio 11 / 12 / 13)
1. Open `delphi/WristSim.dproj` (or `delphi/WristSim.dpr`) in RAD Studio.
2. Select target **Win32** or **Win64** and press **`F9`**.
   *(Uses standard `OpenGL` and bundled `VCLTee` TeeChart units — no third-party packages required.)*
