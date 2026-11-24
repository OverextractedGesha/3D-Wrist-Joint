# Beginner's Guide: Building & Running the 3D Wrist Simulation in Lazarus

This guide walks you through navigating **Lazarus IDE** from scratch and copy-pasting the 3 Object Pascal units (`uWristPhysics.pas`, `uWristRender.pas`, `uMainForm.pas`).

> **Shortcut:** All ready-to-compile project files are already saved in this folder (`WristSim.lpi`). You can either open `WristSim.lpi` directly (**Project → Open Project...**) or follow the steps below to build it yourself from scratch and learn Lazarus!

---

## Step 1: Open Lazarus & Create a New Application

1. Launch Lazarus (`startlazarus` from terminal or your app launcher).
2. In the top menu bar, click **Project → New Project...**
3. Select **Application** (GUI desktop app) and click **OK**.
4. Save immediately: **Project → Save Project As...**
   - Create/choose your project folder.
   - Save the project file as `WristSim.lpi`.
   - Save the default main unit (`Unit1.pas`) as `uMainForm.pas`.

---

## Step 2: Add Required Packages (`LazOpenGLContext` & `TAChartLazarusPkg`)

Lazarus uses **Project Inspector** to manage files and packages.

1. Open **Project → Project Inspector** (or press `Ctrl+Shift+F11`).
2. Click the **Add** button (green `+`) → **New Requirement**.
3. Type `LazOpenGLContext` in the search box → click **OK**.
4. Click **Add** → **New Requirement** again.
5. Type `TAChartLazarusPkg` → click **OK**.

You should now see three items under **Required Packages**: `LCL`, `LazOpenGLContext`, and `TAChartLazarusPkg`.

---

## Step 3: Create the Two Logic Units (`uWristPhysics.pas` & `uWristRender.pas`)

1. Click **File → New Unit** (adds a new empty Pascal unit to your project).
2. Click **File → Save As...** and name it `uWristPhysics.pas`.
3. Select all (`Ctrl+A`) and paste the full code of **`uWristPhysics.pas`**.
4. Repeat **File → New Unit** → **Save As...** → `uWristRender.pas`.
5. Select all (`Ctrl+A`) and paste the full code of **`uWristRender.pas`**.

---

## Step 4: Set Up `uMainForm.pas` & Form Events

Because all buttons, charts, and the 3D OpenGL viewport are built automatically in code inside `FormCreate`, you **do not** need to drag 30+ components onto the Form Designer!

1. Switch to the `uMainForm.pas` tab in the **Source Editor**.
2. Press `F12` to toggle between the **Form Designer** (visual window) and **Source Editor** (code).
3. In the **Object Inspector** (left dock), select `Form1` and change:
   - `Name` → `FormMain`
4. Click the **Events** tab in the Object Inspector and double-click:
   - `OnCreate`
   - `OnDestroy`
   - `OnResize`
   *(This links the `.lfm` form file to `FormCreate`, `FormDestroy`, and `FormResize`.)*
5. Back in the **Source Editor** (`F12`), select all (`Ctrl+A`) in `uMainForm.pas` and paste the full code of **`uMainForm.pas`**.

---

## Step 5: Verify `WristSim.lpr` (Main Program File)

1. Click **Project → View Project Source** to open `WristSim.lpr`.
2. Ensure `uMainForm`, `uWristPhysics`, and `uWristRender` are listed in `uses`, and `Application.CreateForm(TFormMain, FormMain);` is present.

---

## Step 6: Compile & Run!

- Press **`F9`** (or click the green **Run ▶** button on the toolbar).
- If Lazarus asks to save modified files, click **Yes**.
- The window **Wrist Joint Movement Model and Control Simulation, BME Dept. ITS** will open!

### Useful Lazarus Shortcuts to Remember
| Shortcut | Action |
| :--- | :--- |
| `F9` | Compile & Run |
| `Ctrl+F9` | Compile only (check for errors) |
| `F12` | Toggle between Form Designer and Code Editor |
| `Ctrl+Shift+F11` | Open Project Inspector |
| `Ctrl+Space` | Code completion (autocomplete) |
| `Ctrl+Click` | Jump to symbol/procedure definition |

---

## Step 7: Running the Project Experiments (Per `BioMod Project-2627.pdf`)

1. **Anthropometry (Default 60 kg BW, 160 cm BH):**
   - Check the bottom-right readout: `Hand: m = 0.360 kg, l = 9.20 cm | Io = 0.001016 kg*m^2`.
   - Edit `BW (kg)` or `BH (cm)` and press `Tab` or click `Reset` to update for any subject.
2. **Passive Movement Test (Requirement 2):**
   - Select **Passive Test: Dorsal** or **Passive Test: Radial** in the `Mode` dropdown.
   - Click **Passive D** / **Passive R** — the wrist releases from `30°` with zero active torque ($\tau_{\text{active}} = 0$).
   - Iteratively adjust `C damp` and `k stiff` and click **Reset** → **Start** to observe underdamped vs. critically damped passive decay.
3. **Active 3D Transversal PID Control (Requirement 3):**
   - Select **Transversal (3D)** and click **Transversal**.
   - Observe the 5 live charts matching PDF Figure 2:
     - Top-left: Active joint torques `tau1`, `tau2` (N·m)
     - Top-right: `Radial/Ulnar Flexion` (`Target` vs `Realize`)
     - Middle-left: `RMSE` and tracking errors `error 1`, `error 2`
     - Middle-right: `Dorsal/Palmar` (`Target` vs `Realize`)
     - Bottom-left: `Transversal Phase Angle` circular trajectory
   - Left-click and drag on the 3D OpenGL viewport to orbit the camera; click **Reset** to restore the default view.
