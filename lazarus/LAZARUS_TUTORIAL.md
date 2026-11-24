# Beginner's Guide: Single-File 3D Wrist Simulation in Lazarus

> **Shortcut:** Ready-to-open project is already saved in `lazarus/WristSim.lpi`. Open it (**Project → Open Project...**) and press **`F9`**. Or follow the 5 steps below to build from scratch by copy-pasting `uMainForm.pas`.

---

## Step 1: Create & Save a Blank Project
1. Open Lazarus (`startlazarus`).
2. **Project → New Project... → Application → OK**.
3. **Project → Save Project As...** → pick/create an empty folder:
   - Save project (`.lpi`) as **`WristSim`**
   - Save unit (`unit1.pas`) as **`uMainForm`**

## Step 2: Add 2 Packages
1. Press **`Ctrl+Shift+F11`** (**Project Inspector**).
2. Click **Add (`+`) → New Requirement** → type **`LazOpenGLContext`** → **OK**.
3. Click **Add (`+`) → New Requirement** → type **`TAChartLazarusPkg`** → **OK**.
4. Close Project Inspector.

## Step 3: Rename Form & Wire 3 Events
1. In **Object Inspector** (left dock), select **`Form1: TForm1`**.
2. **Properties** tab: set `Name` → **`FormMain`** → press `Enter`.
3. **Events** tab: double-click **`OnCreate`**, **`OnDestroy`**, and **`OnResize`**.

## Step 4: Paste `uMainForm.pas`
1. In **Source Editor**, select the **`uMainForm`** tab (**`Code`** sub-tab).
2. Press **`Ctrl+A`** → **`Ctrl+V`** (paste `uMainForm.pas`) → **`Ctrl+S`**.

## Step 5: Run
- Press **`F9`** (or click green **▶** play icon).
- Click **`Transversal`** to run the 3D wrist simulation. Left-drag the 3D view to orbit camera; use bottom-right **`Mode`** dropdown for single-axis or **Passive Test** modes.
