unit uMainForm;

// =============================================================================
// 3D WRIST JOINT MOVEMENT MODELING & PID CONTROL SIMULATION (LAZARUS / FPC)
// Course : EB234506 Biomodeling - Biomedical Engineering Dept., ITS
// Layout : 3 sections inside one file for easy navigation:
//   PART 1 : TWristSimulator  -> Biomechanics, Lagrangian ODE, RK4, Passive, PID
//   PART 2 : TWristRenderer   -> 3D OpenGL Arm, Wrist, Palm & 5 Fingers
//   PART 3 : TFormMain        -> Figure 2 UI, 5 TAChart Plots, Timer & Events
// =============================================================================

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Math, Forms, Controls, Graphics, Dialogs, StdCtrls, ExtCtrls,
  GL, GLU, OpenGLContext, TAGraph, TASeries, TATypes, TAChartUtils;

type
  // Simulation operating modes selectable from the bottom-right Mode dropdown
  TSimMode = (
    smTransversal,    // 3D circular motion: both axes active (90 deg phase shift)
    smDorsalPalmar,   // Frontal plane only: active PID tracking (dorsi/palmar flexion)
    smRadialUlnar,    // Sagittal plane only: active PID tracking (radial/ulnar flexion)
    smPassiveDorsal,  // Passive test (Frontal): tau_active = 0, released from 30 deg
    smPassiveRadial   // Passive test (Sagittal): tau_active = 0, released from 30 deg
  );

  // [REQ 2] Dempster anthropometric regression data (PDF page 2) + rod inertia
  TAnthropometry = record
    BodyWeightKg: Double;     // Subject Body Weight BW (kg), default 60.0
    BodyHeightCm: Double;     // Subject Body Height BH (cm), default 160.0
    HandMassKg: Double;       // m_hand    = 0.6%  * BW (Dempster)
    HandLengthM: Double;      // l_hand    = 5.75% * BH
    ForearmMassKg: Double;    // m_forearm = 1.6%  * BW (Dempster)
    ForearmLengthM: Double;   // l_forearm = 15.70% * BH
    UpperArmMassKg: Double;   // m_upper   = 2.8%  * BW (Dempster)
    UpperArmLengthM: Double;  // l_upper   = 17.20% * BH
    Ig: Double;               // Uniform rod inertia about CoG:   Ig = m*l^2 / 12
    Io: Double;               // Parallel-axis inertia at wrist:  Io = Ig + m*(l/2)^2
  end;

  // [REQ 3] PID controller gains for active joint torque
  TPIDParams = record
    Kp: Double;               // Proportional gain (N*m / rad)
    Ki: Double;               // Integral gain     (N*m / (rad*s))
    Kd: Double;               // Derivative gain   (N*m*s / rad)
  end;

  // [REQ 2] Linear passive viscoelastic joint parameters: tau_p = -C*theta_dot - k*(theta - theta_0)
  TPassiveParams = record
    DampingC: Double;         // Viscous damping coefficient C (N*m*s / rad)
    StiffnessK: Double;       // Elastic spring stiffness k    (N*m / rad)
    RestAngleDeg: Double;     // Equilibrium/resting angle theta_0 (degrees)
  end;

  // Per-axis kinematic, dynamic, and PID state variables
  TAxisState = record
    ThetaRad: Double;         // Physics state: current joint angle (rad)
    ThetaDot: Double;         // Physics state: current angular velocity (rad/s)
    TargetDeg: Double;        // Reference sinusoidal angle for chart (deg)
    RealizeDeg: Double;       // Simulated realized angle for chart & 3D (deg)
    ErrorDeg: Double;         // Absolute tracking error |Target - Realize| (deg)
    ActiveTorque: Double;     // Active muscular/PID torque tau_active (N*m)
    PassiveTorque: Double;    // Passive ligament/tissue torque tau_passive (N*m)
    IntegralErr: Double;      // Accumulated integral of error (rad*s) for PID I-term
    PrevErrRad: Double;       // Previous step error (rad) for PID D-term
  end;

  // ===========================================================================
  // PART 1: BIOMECHANICAL PHYSICS ENGINE & PID CONTROLLER (Pure Math, No GUI)
  // ===========================================================================
  TWristSimulator = class
  private
    FAnthro: TAnthropometry;
    FPIDDorsal: TPIDParams;         // PID gains for Frontal plane (Dorsal/Palmar)
    FPIDRadial: TPIDParams;         // PID gains for Sagittal plane (Radial/Ulnar)
    FPassiveDorsal: TPassiveParams; // Passive C, k for flexor/extensor
    FPassiveRadial: TPassiveParams; // Passive C, k for abductor/adductor
    FDorsal: TAxisState;            // Axis 1 state: Dorsal/Palmar flexion
    FRadial: TAxisState;            // Axis 2 state: Radial/Ulnar flexion
    FMode: TSimMode;
    FSimTime: Double;               // Elapsed simulation time t (seconds)
    FDeltaTime: Double;             // Fixed RK4 step size dt = 0.001 s (1000 Hz)
    FFreqHz: Double;                // Sinusoidal target frequency f (Hz)
    FAmplitudeDeg: Double;          // Sinusoidal target amplitude A (30 deg)
    FGravity: Double;               // Gravitational acceleration g = 9.81 m/s^2
    FStepCount: Integer;            // Number of completed RK4 steps N
    FSquaredErrSum: Double;         // Running sum of squared errors for RMSE
    FRMSE: Double;                  // Live Root Mean Square Error (deg)

    function ComputePassiveTorque(AngleRad, VelRad: Double; const P: TPassiveParams): Double;
    function ComputeAccel(AngleRad, VelRad, ActiveTau: Double; const P: TPassiveParams): Double;
    procedure StepRK4(var AngleRad, VelRad: Double; ActiveTau: Double; const P: TPassiveParams);
    procedure StepAxisActive(var Axis: TAxisState; TargetDegVal: Double;
      const PID: TPIDParams; const PassP: TPassiveParams);
    procedure StepAxisPassive(var Axis: TAxisState; const PassP: TPassiveParams);
  public
    constructor Create;
    procedure UpdateAnthropometry(BWKg, BHCm: Double);
    procedure SetMode(AMode: TSimMode);
    procedure Reset;
    procedure Step;

    property Anthro: TAnthropometry read FAnthro;
    property PIDDorsal: TPIDParams read FPIDDorsal write FPIDDorsal;
    property PIDRadial: TPIDParams read FPIDRadial write FPIDRadial;
    property PassiveDorsal: TPassiveParams read FPassiveDorsal write FPassiveDorsal;
    property PassiveRadial: TPassiveParams read FPassiveRadial write FPassiveRadial;
    property Dorsal: TAxisState read FDorsal;
    property Radial: TAxisState read FRadial;
    property Mode: TSimMode read FMode;
    property SimTime: Double read FSimTime;
    property StepCount: Integer read FStepCount;
    property RMSE: Double read FRMSE;
  end;

  // ===========================================================================
  // PART 2: 3D OPENGL RENDERER (Upper Arm, Forearm, Wrist, Palm & 5 Fingers)
  // ===========================================================================
  TWristRenderer = class
  private
    FQuadSphere: PGLUquadric;     // GLU sphere helper for joints & fingertips
    FQuadCylinder: PGLUquadric;   // GLU cylinder helper for limb segments & phalanges
    FInitialized: Boolean;
    FCamYaw: Single;              // Camera horizontal orbit angle (deg)
    FCamPitch: Single;            // Camera vertical orbit angle (deg)
    FCamDist: Single;             // Camera zoom distance from origin

    procedure DrawUpperArmAndForearm(VisualScale: Single);
    procedure DrawHand(VisualScale: Single);
    procedure DrawFingers(VisualScale: Single; SimTime: Double);
  public
    constructor Create;
    destructor Destroy; override;

    procedure InitGL;
    procedure FreeGL;
    procedure Resize(W, H: Integer);
    procedure Render(DorsalDeg, RadialDeg, VisualScale: Double; SimTime: Double);
    procedure ResetCamera;

    property CamYaw: Single read FCamYaw write FCamYaw;
    property CamPitch: Single read FCamPitch write FCamPitch;
    property CamDist: Single read FCamDist write FCamDist;
  end;

  // ===========================================================================
  // PART 3: MAIN APPLICATION FORM (Strictly Matches PDF Figure 2 Layout)
  // ===========================================================================
  TFormMain = class(TForm)
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure FormResize(Sender: TObject);
  private
    FSim: TWristSimulator;
    FRenderer: TWristRenderer;
    FTimer: TTimer;               // 16 ms (~60 FPS) simulation & render loop
    FRunning: Boolean;

    // Left Pane (42% width): 4-button toolbar + 3D OpenGL canvas
    FPnlLeft: TPanel;
    FPnlToolbar: TPanel;
    FBtnStart: TButton;
    FBtnStop: TButton;
    FBtnReset: TButton;
    FBtnClose: TButton;
    FGLBox: TOpenGLControl;

    // Right Pane (58% width): 3x2 grid of 5 charts + bottom-right parameter panel
    FPnlRight: TPanel;
    FChartTorque: TChart;         // Row 1, Col 1: tau1 & tau2 (Active Torques)
    FChartRadial: TChart;         // Row 1, Col 2: Radial/Ulnar Target vs Realize
    FChartError: TChart;          // Row 2, Col 1: RMSE title + error 1 & error 2
    FChartDorsal: TChart;         // Row 2, Col 2: Dorsal/Palmar Target vs Realize
    FChartPhase: TChart;          // Row 3, Col 1: Transversal Phase Angle circle
    FPnlParams: TPanel;           // Row 3, Col 2: Mode, BW/BH, PID, Passive C/k

    // TAChart Data Series
    FSerTau1, FSerTau2: TLineSeries;
    FSerRadTarget, FSerRadRealize: TLineSeries;
    FSerErr1, FSerErr2: TLineSeries;
    FSerDorTarget, FSerDorRealize: TLineSeries;
    FSerPhase: TLineSeries;

    // Parameter Inputs
    FCmbMode: TComboBox;
    FEdtBW, FEdtBH: TEdit;
    FEdtKp, FEdtKi, FEdtKd: TEdit;
    FEdtC, FEdtK: TEdit;
    FLblAnthroInfo: TLabel;

    // Left-click mouse drag orbit state
    FDragging: Boolean;
    FLastMouseX, FLastMouseY: Integer;

    procedure BuildUI;
    function CreateStyledChart(AParent: TWinControl; const ATitle: string;
      ShowLegend: Boolean; UseFixedY: Boolean = False;
      YMinVal: Double = -30.0; YMaxVal: Double = 30.0): TChart;
    function AddSeriesToChart(AChart: TChart; const ATitle: string;
      AColor: TColor; AWidth: Integer = 1): TLineSeries;
    procedure LayoutRightGrid;
    procedure ApplyInputsToSim;
    procedure ClearAllCharts;
    procedure UpdateAnthroLabel;

    // UI Button & Timer Handlers
    procedure OnStartClick(Sender: TObject);
    procedure OnStopClick(Sender: TObject);
    procedure OnResetClick(Sender: TObject);
    procedure OnCloseClick(Sender: TObject);
    procedure OnModeChange(Sender: TObject);
    procedure OnParamExit(Sender: TObject);
    procedure OnSimTimer(Sender: TObject);

    // OpenGL Viewport Handlers
    procedure OnGLPaint(Sender: TObject);
    procedure OnGLResize(Sender: TObject);
    procedure OnGLMouseDown(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure OnGLMouseMove(Sender: TObject; Shift: TShiftState; X, Y: Integer);
    procedure OnGLMouseUp(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
  public
  end;

var
  FormMain: TFormMain;

implementation

{$R *.lfm}

const
  DEG_TO_RAD        = Pi / 180.0;
  RAD_TO_DEG        = 180.0 / Pi;
  MAX_INTEGRAL      = 2.0;   // PID anti-windup clamp (rad*s) prevents overshoot blowup
  MAX_TORQUE        = 50.0;  // Maximum physical wrist torque clamp (N*m)
  SUBSTEPS_PER_TICK = 10;    // 10 RK4 steps * 0.001 s = 10 ms physics per UI frame

  // OpenGL Phong material & directional light settings (warm ivory shaded limb)
  MAT_SPECULAR: array[0..3] of GLfloat = (0.75, 0.75, 0.65, 1.0);
  MAT_DIFFUSE:  array[0..3] of GLfloat = (0.92, 0.90, 0.80, 1.0);
  MAT_AMBIENT:  array[0..3] of GLfloat = (0.32, 0.30, 0.26, 1.0);
  MAT_SHININESS: GLfloat = 36.0;
  LIGHT_POS:    array[0..3] of GLfloat = (10.0, 12.0, 14.0, 1.0);
  LIGHT_DIFF:   array[0..3] of GLfloat = (1.0, 0.98, 0.90, 1.0);
  LIGHT_AMB:    array[0..3] of GLfloat = (0.30, 0.30, 0.32, 1.0);

{ ========================= PART 1: TWristSimulator ========================= }

constructor TWristSimulator.Create;
begin
  inherited Create;
  FGravity := 9.81;        // Standard gravity (m/s^2)
  FDeltaTime := 0.001;     // 1 ms fixed timestep (1000 Hz) for RK4 stability
  FFreqHz := 0.8;          // 0.8 Hz sinusoidal wrist movement speed
  FAmplitudeDeg := 30.0;   // +/- 30 deg range of motion (matches PDF Figure 2)
  FMode := smTransversal;

  // [REQ 2] Default subject from PDF: 60 kg BW, 160 cm BH
  UpdateAnthropometry(60.0, 160.0);

  // [REQ 3] Tuned PID gains: critically damped tracking with low RMSE (~1-2 deg)
  FPIDDorsal.Kp := 15.0;
  FPIDDorsal.Ki := 40.0;
  FPIDDorsal.Kd := 0.25;

  FPIDRadial.Kp := 15.0;
  FPIDRadial.Ki := 40.0;
  FPIDRadial.Kd := 0.25;

  // [REQ 2] Default passive spring-damper values (underdamped free decay ~1-2 s)
  FPassiveDorsal.DampingC := 0.010;
  FPassiveDorsal.StiffnessK := 0.20;
  FPassiveDorsal.RestAngleDeg := 0.0;

  FPassiveRadial.DampingC := 0.012;
  FPassiveRadial.StiffnessK := 0.24;
  FPassiveRadial.RestAngleDeg := 0.0;

  Reset;
end;

// [REQ 2] Computes segment masses, lengths (Dempster table, PDF p.2), and
// uniform rod moments of inertia about center of gravity (Ig) and wrist axis (Io).
procedure TWristSimulator.UpdateAnthropometry(BWKg, BHCm: Double);
var
  BHM: Double;
begin
  if BWKg < 10.0 then BWKg := 10.0;
  if BHCm < 50.0 then BHCm := 50.0;

  BHM := BHCm / 100.0; // Convert height from cm to meters
  FAnthro.BodyWeightKg := BWKg;
  FAnthro.BodyHeightCm := BHCm;

  // Dempster body segment percentages (PDF page 2)
  // For 60 kg, 160 cm -> Hand: m = 0.360 kg, l = 0.092 m
  FAnthro.HandMassKg := 0.006 * BWKg;
  FAnthro.HandLengthM := 0.0575 * BHM;
  FAnthro.ForearmMassKg := 0.016 * BWKg;
  FAnthro.ForearmLengthM := 0.1570 * BHM;
  FAnthro.UpperArmMassKg := 0.028 * BWKg;
  FAnthro.UpperArmLengthM := 0.1720 * BHM;

  // Uniform rod moment of inertia (PDF Req 2):
  //   Ig = m * l^2 / 12               (about center of mass)
  //   Io = Ig + m * (l/2)^2 = m*l^2/3 (about proximal wrist pivot via Parallel-Axis Theorem)
  FAnthro.Ig := (FAnthro.HandMassKg * Sqr(FAnthro.HandLengthM)) / 12.0;
  FAnthro.Io := FAnthro.Ig + FAnthro.HandMassKg * Sqr(FAnthro.HandLengthM / 2.0);
end;

procedure TWristSimulator.SetMode(AMode: TSimMode);
begin
  FMode := AMode;
  Reset;
end;

// Resets clock, RMSE accumulator, and axis states.
// In Passive Test modes, displaces the wrist to 30 deg (velocity = 0) so the user
// can observe free damped oscillation under gravity + passive spring-damper torque.
procedure TWristSimulator.Reset;
begin
  FSimTime := 0.0;
  FStepCount := 0;
  FSquaredErrSum := 0.0;
  FRMSE := 0.0;

  FillChar(FDorsal, SizeOf(FDorsal), 0);
  FillChar(FRadial, SizeOf(FRadial), 0);

  if FMode = smPassiveDorsal then
  begin
    FDorsal.ThetaRad := 30.0 * DEG_TO_RAD;
    FDorsal.RealizeDeg := 30.0;
  end
  else if FMode = smPassiveRadial then
  begin
    FRadial.ThetaRad := 30.0 * DEG_TO_RAD;
    FRadial.RealizeDeg := 30.0;
  end;
end;

// [REQ 2] Linear Passive Joint Torque model:
//   tau_passive = -C * theta_dot - k * (theta - theta_0)
//   -C * theta_dot      : viscous damping opposing angular velocity
//   -k * (theta-theta0) : elastic restoring torque pulling back toward theta_0
function TWristSimulator.ComputePassiveTorque(AngleRad, VelRad: Double;
  const P: TPassiveParams): Double;
var
  Theta0Rad: Double;
begin
  Theta0Rad := P.RestAngleDeg * DEG_TO_RAD;
  Result := -P.DampingC * VelRad - P.StiffnessK * (AngleRad - Theta0Rad);
end;

// [REQ 1] Single-Pendulum Lagrangian Equation of Motion for one plane:
//   Kinetic Energy   : Ek = (1/2) * Io * theta_dot^2
//   Potential Energy : Ep = m * g * (l/2) * (1 - cos(theta))
//   Lagrangian       : L  = Ek - Ep
//   Euler-Lagrange   : d/dt(dL/dtheta_dot) - dL/dtheta = tau_active + tau_passive
//   Resulting ODE    : Io * theta_ddot + m*g*(l/2)*sin(theta) = tau_active + tau_passive
//   Solving for accel: theta_ddot = (tau_active + tau_passive - m*g*(l/2)*sin(theta)) / Io
function TWristSimulator.ComputeAccel(AngleRad, VelRad, ActiveTau: Double;
  const P: TPassiveParams): Double;
var
  GravTau, PassiveTau, NetTau: Double;
begin
  GravTau := -FAnthro.HandMassKg * FGravity * (FAnthro.HandLengthM * 0.5) * Sin(AngleRad);
  PassiveTau := ComputePassiveTorque(AngleRad, VelRad, P);
  NetTau := ActiveTau + PassiveTau + GravTau;
  Result := NetTau / FAnthro.Io;
end;

// 4th-Order Runge-Kutta (RK4) integrator for the 2nd-order Lagrangian ODE.
// Splits theta_ddot = f(theta, theta_dot) into two 1st-order ODEs:
//   d(Angle)/dt = Vel
//   d(Vel)/dt   = ComputeAccel(Angle, Vel, ActiveTau)
procedure TWristSimulator.StepRK4(var AngleRad, VelRad: Double; ActiveTau: Double;
  const P: TPassiveParams);
var
  k1x, k1v, k2x, k2v, k3x, k3v, k4x, k4v, dt: Double;
begin
  dt := FDeltaTime;

  // Slope at start of interval (t)
  k1x := VelRad;
  k1v := ComputeAccel(AngleRad, VelRad, ActiveTau, P);

  // Slope at midpoint (t + dt/2) using k1
  k2x := VelRad + 0.5 * dt * k1v;
  k2v := ComputeAccel(AngleRad + 0.5 * dt * k1x, VelRad + 0.5 * dt * k1v, ActiveTau, P);

  // Refined slope at midpoint (t + dt/2) using k2
  k3x := VelRad + 0.5 * dt * k2v;
  k3v := ComputeAccel(AngleRad + 0.5 * dt * k2x, VelRad + 0.5 * dt * k2v, ActiveTau, P);

  // Slope at end of interval (t + dt) using k3
  k4x := VelRad + dt * k3v;
  k4v := ComputeAccel(AngleRad + dt * k3x, VelRad + dt * k3v, ActiveTau, P);

  // Weighted RK4 update: (k1 + 2*k2 + 2*k3 + k4) / 6
  AngleRad := AngleRad + (dt / 6.0) * (k1x + 2.0 * k2x + 2.0 * k3x + k4x);
  VelRad   := VelRad   + (dt / 6.0) * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
end;

// [REQ 3] Active Movement Step with PID Control:
// Computes active torque tau_active = Kp*e + Ki*integral(e) + Kd*(de/dt),
// then feeds tau_active directly into the Lagrangian RK4 dynamics.
procedure TWristSimulator.StepAxisActive(var Axis: TAxisState; TargetDegVal: Double;
  const PID: TPIDParams; const PassP: TPassiveParams);
var
  TargetRad, ErrRad, DerivRad, Tau: Double;
begin
  Axis.TargetDeg := TargetDegVal;
  TargetRad := TargetDegVal * DEG_TO_RAD;

  // Error e(t) in radians (consistent with physical N*m/rad PID gains)
  ErrRad := TargetRad - Axis.ThetaRad;

  // Integral term with anti-windup clamp
  Axis.IntegralErr := EnsureRange(Axis.IntegralErr + ErrRad * FDeltaTime,
    -MAX_INTEGRAL, MAX_INTEGRAL);

  // Derivative term (zeroed on step 0 to avoid an artificial initial spike)
  if FStepCount = 0 then
    DerivRad := 0.0
  else
    DerivRad := (ErrRad - Axis.PrevErrRad) / FDeltaTime;

  // PID active joint torque (clamped to physical actuator limit)
  Tau := (PID.Kp * ErrRad) + (PID.Ki * Axis.IntegralErr) + (PID.Kd * DerivRad);
  Tau := EnsureRange(Tau, -MAX_TORQUE, MAX_TORQUE);

  Axis.ActiveTorque := Tau;
  Axis.PassiveTorque := ComputePassiveTorque(Axis.ThetaRad, Axis.ThetaDot, PassP);
  Axis.PrevErrRad := ErrRad;

  // Advance physical angle & velocity via RK4
  StepRK4(Axis.ThetaRad, Axis.ThetaDot, Tau, PassP);

  Axis.RealizeDeg := Axis.ThetaRad * RAD_TO_DEG;
  Axis.ErrorDeg := Abs(Axis.TargetDeg - Axis.RealizeDeg);
end;

// [REQ 2] Passive Movement Test Step:
// Sets active torque = 0 so only gravity and passive spring-damper (C, k) act on the joint.
procedure TWristSimulator.StepAxisPassive(var Axis: TAxisState;
  const PassP: TPassiveParams);
begin
  Axis.TargetDeg := PassP.RestAngleDeg;
  Axis.ActiveTorque := 0.0;
  Axis.PassiveTorque := ComputePassiveTorque(Axis.ThetaRad, Axis.ThetaDot, PassP);

  StepRK4(Axis.ThetaRad, Axis.ThetaDot, 0.0, PassP);

  Axis.RealizeDeg := Axis.ThetaRad * RAD_TO_DEG;
  Axis.ErrorDeg := Abs(Axis.TargetDeg - Axis.RealizeDeg);
end;

// Advances the whole simulation by one fixed step (dt = 0.001 s) and updates RMSE.
procedure TWristSimulator.Step;
var
  RefDorsal, RefRadial, ErrSq: Double;
begin
  FSimTime := FSimTime + FDeltaTime;

  // [REQ 3] Sinusoidal references (90-degree phase shift creates a circle in Transversal mode):
  //   Frontal  (Dorsal/Palmar): A * sin(2*pi*f*t)
  //   Sagittal (Radial/Ulnar) : A * cos(2*pi*f*t)
  RefDorsal := FAmplitudeDeg * Sin(2.0 * Pi * FFreqHz * FSimTime);
  RefRadial := FAmplitudeDeg * Cos(2.0 * Pi * FFreqHz * FSimTime);

  case FMode of
    smTransversal:
      begin
        StepAxisActive(FDorsal, RefDorsal, FPIDDorsal, FPassiveDorsal);
        StepAxisActive(FRadial, RefRadial, FPIDRadial, FPassiveRadial);
        ErrSq := Sqr(FDorsal.ErrorDeg) + Sqr(FRadial.ErrorDeg);
      end;

    smDorsalPalmar:
      begin
        StepAxisActive(FDorsal, RefDorsal, FPIDDorsal, FPassiveDorsal);
        FRadial.TargetDeg := 0.0;
        FRadial.RealizeDeg := 0.0;
        FRadial.ErrorDeg := 0.0;
        FRadial.ActiveTorque := 0.0;
        ErrSq := Sqr(FDorsal.ErrorDeg);
      end;

    smRadialUlnar:
      begin
        StepAxisActive(FRadial, RefRadial, FPIDRadial, FPassiveRadial);
        FDorsal.TargetDeg := 0.0;
        FDorsal.RealizeDeg := 0.0;
        FDorsal.ErrorDeg := 0.0;
        FDorsal.ActiveTorque := 0.0;
        ErrSq := Sqr(FRadial.ErrorDeg);
      end;

    smPassiveDorsal:
      begin
        StepAxisPassive(FDorsal, FPassiveDorsal);
        FRadial.TargetDeg := 0.0;
        FRadial.RealizeDeg := 0.0;
        FRadial.ErrorDeg := 0.0;
        FRadial.ActiveTorque := 0.0;
        ErrSq := Sqr(FDorsal.ErrorDeg);
      end;

    smPassiveRadial:
      begin
        StepAxisPassive(FRadial, FPassiveRadial);
        FDorsal.TargetDeg := 0.0;
        FDorsal.RealizeDeg := 0.0;
        FDorsal.ErrorDeg := 0.0;
        FDorsal.ActiveTorque := 0.0;
        ErrSq := Sqr(FRadial.ErrorDeg);
      end;
  end;

  // Cumulative Root Mean Square Error: RMSE = sqrt( (1/N) * sum(e^2) )
  Inc(FStepCount);
  FSquaredErrSum := FSquaredErrSum + ErrSq;
  FRMSE := Sqrt(FSquaredErrSum / FStepCount);
end;

{ ========================= PART 2: TWristRenderer ========================= }

constructor TWristRenderer.Create;
begin
  inherited Create;
  FQuadSphere := nil;
  FQuadCylinder := nil;
  FInitialized := False;
  ResetCamera;
end;

destructor TWristRenderer.Destroy;
begin
  FreeGL;
  inherited Destroy;
end;

procedure TWristRenderer.ResetCamera;
begin
  FCamYaw := -18.0;
  FCamPitch := 8.0;
  FCamDist := 13.5;
end;

// Configures OpenGL depth testing, smooth lighting, material color, and GLU quadrics
procedure TWristRenderer.InitGL;
begin
  if FInitialized then Exit;

  glClearColor(0.30, 0.25, 0.52, 1.0); // Indigo background matching PDF Figure 2
  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_NORMALIZE);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_POSITION, @LIGHT_POS[0]);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  @LIGHT_DIFF[0]);
  glLightfv(GL_LIGHT0, GL_AMBIENT,  @LIGHT_AMB[0]);

  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   @MAT_AMBIENT[0]);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   @MAT_DIFFUSE[0]);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  @MAT_SPECULAR[0]);
  glMaterialf(GL_FRONT_AND_BACK,  GL_SHININESS, MAT_SHININESS);

  FQuadSphere := gluNewQuadric();
  FQuadCylinder := gluNewQuadric();
  gluQuadricNormals(FQuadSphere, GLU_SMOOTH);
  gluQuadricNormals(FQuadCylinder, GLU_SMOOTH);

  FInitialized := True;
end;

// Frees GLU quadric objects to prevent native memory leaks on exit
procedure TWristRenderer.FreeGL;
begin
  if FQuadSphere <> nil then
  begin
    gluDeleteQuadric(FQuadSphere);
    FQuadSphere := nil;
  end;
  if FQuadCylinder <> nil then
  begin
    gluDeleteQuadric(FQuadCylinder);
    FQuadCylinder := nil;
  end;
  FInitialized := False;
end;

procedure TWristRenderer.Resize(W, H: Integer);
var
  Aspect: Double;
begin
  if H <= 0 then H := 1;
  if W <= 0 then W := 1;
  Aspect := W / H;

  glViewport(0, 0, W, H);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(40.0, Aspect, 0.5, 100.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
end;

// Draws kinematic chain: Shoulder -> Upper Arm -> Elbow -> Forearm -> Wrist Joint.
// Leaves OpenGL local origin directly at the wrist center (+Z pointing down the hand).
procedure TWristRenderer.DrawUpperArmAndForearm(VisualScale: Single);
var
  UpperLen, ForearmLen: Single;
begin
  UpperLen := 3.4 * VisualScale;
  ForearmLen := 3.1 * VisualScale;

  // Shoulder sphere
  gluSphere(FQuadSphere, 0.72 * VisualScale, 28, 28);

  // Upper arm tapered cylinder
  glRotatef(18.0, 0.0, 0.0, -1.0);
  glRotatef(90.0, 1.0, 0.0, 0.0);
  gluCylinder(FQuadCylinder, 0.65 * VisualScale, 0.48 * VisualScale, UpperLen, 28, 8);

  // Elbow sphere + slight natural elbow flexion (matches PDF Figure 2 silhouette)
  glTranslatef(0.0, 0.0, UpperLen);
  gluSphere(FQuadSphere, 0.50 * VisualScale, 28, 28);
  glRotatef(22.0, 0.0, 1.0, 0.0);

  // Forearm tapered cylinder ending at wrist joint sphere
  gluCylinder(FQuadCylinder, 0.48 * VisualScale, 0.34 * VisualScale, ForearmLen, 28, 8);
  glTranslatef(0.0, 0.0, ForearmLen);
  gluSphere(FQuadSphere, 0.36 * VisualScale, 28, 28);
end;

// Draws the palm as a 6-sided rectangular prism with outward normals for smooth shading
procedure TWristRenderer.DrawHand(VisualScale: Single);
var
  W, T, L: Single;
begin
  W := 0.44 * VisualScale; // Half-width (radial-ulnar)
  T := 0.16 * VisualScale; // Half-thickness (dorsal-palmar)
  L := 1.05 * VisualScale; // Palm length (wrist to knuckles)

  glBegin(GL_QUADS);
    // Distal cap (knuckles)
    glNormal3f(0.0, 0.0, 1.0);
    glVertex3f(-W,  T, L); glVertex3f( W,  T, L);
    glVertex3f( W, -T, L); glVertex3f(-W, -T, L);

    // Proximal cap (wrist)
    glNormal3f(0.0, 0.0, -1.0);
    glVertex3f(-W,  T, 0.0); glVertex3f(-W, -T, 0.0);
    glVertex3f( W, -T, 0.0); glVertex3f( W,  T, 0.0);

    // Palmar surface
    glNormal3f(0.0, 1.0, 0.0);
    glVertex3f(-W,  T, 0.0); glVertex3f( W,  T, 0.0);
    glVertex3f( W,  T, L);   glVertex3f(-W,  T, L);

    // Dorsal surface
    glNormal3f(0.0, -1.0, 0.0);
    glVertex3f(-W, -T, 0.0); glVertex3f(-W, -T, L);
    glVertex3f( W, -T, L);   glVertex3f( W, -T, 0.0);

    // Radial side
    glNormal3f(1.0, 0.0, 0.0);
    glVertex3f( W,  T, 0.0); glVertex3f( W, -T, 0.0);
    glVertex3f( W, -T, L);   glVertex3f( W,  T, L);

    // Ulnar side
    glNormal3f(-1.0, 0.0, 0.0);
    glVertex3f(-W,  T, 0.0); glVertex3f(-W,  T, L);
    glVertex3f(-W, -T, L);   glVertex3f(-W, -T, 0.0);
  glEnd();
end;

// Draws 4 fingers + thumb (3 phalanges each). glPushMatrix/glPopMatrix isolates each finger.
procedure TWristRenderer.DrawFingers(VisualScale: Single; SimTime: Double);
var
  L1, L2, L3, Space, Curl: Single;

  procedure DrawOneFinger(XOff, BaseR, ScaleLen, ExtraYaw: Single);
  begin
    glPushMatrix();
      glTranslatef(XOff, 0.0, 0.0);
      glRotatef(ExtraYaw, 0.0, 1.0, 0.0);

      // Proximal phalanx
      glRotatef(Curl * 0.45, 1.0, 0.0, 0.0);
      gluSphere(FQuadSphere, BaseR * 1.05, 14, 14);
      gluCylinder(FQuadCylinder, BaseR, BaseR * 0.92, L1 * ScaleLen, 14, 4);

      // Middle phalanx
      glTranslatef(0.0, 0.0, L1 * ScaleLen);
      glRotatef(Curl * 0.55, 1.0, 0.0, 0.0);
      gluSphere(FQuadSphere, BaseR * 0.94, 14, 14);
      gluCylinder(FQuadCylinder, BaseR * 0.92, BaseR * 0.82, L2 * ScaleLen, 14, 4);

      // Distal phalanx + fingertip
      glTranslatef(0.0, 0.0, L2 * ScaleLen);
      glRotatef(Curl * 0.45, 1.0, 0.0, 0.0);
      gluSphere(FQuadSphere, BaseR * 0.84, 14, 14);
      gluCylinder(FQuadCylinder, BaseR * 0.82, BaseR * 0.65, L3 * ScaleLen, 14, 4);

      glTranslatef(0.0, 0.0, L3 * ScaleLen);
      gluSphere(FQuadSphere, BaseR * 0.65, 14, 14);
    glPopMatrix();
  end;

begin
  L1 := 0.28 * VisualScale;
  L2 := 0.24 * VisualScale;
  L3 := 0.20 * VisualScale;
  Space := 0.18 * VisualScale;
  Curl := 10.0 + 6.0 * Abs(Sin(SimTime * 1.2)); // Subtle natural finger relaxation

  // Four fingers mounted at distal palm edge
  glPushMatrix();
    glTranslatef(0.0, 0.0, 1.05 * VisualScale);
    DrawOneFinger(-Space * 1.65, 0.068 * VisualScale, 0.82, -4.0); // Pinky
    DrawOneFinger(-Space * 0.55, 0.074 * VisualScale, 0.96, -1.0); // Ring
    DrawOneFinger( Space * 0.55, 0.078 * VisualScale, 1.05,  1.0); // Middle
    DrawOneFinger( Space * 1.65, 0.075 * VisualScale, 0.95,  4.0); // Index
  glPopMatrix();

  // Thumb mounted on radial side of palm
  glPushMatrix();
    glTranslatef(0.42 * VisualScale, 0.06 * VisualScale, 0.48 * VisualScale);
    DrawOneFinger(0.0, 0.088 * VisualScale, 0.72, 34.0);
  glPopMatrix();
end;

// Main 3D render pass: applies camera view, draws arm, rotates wrist by 2-DOF
// simulated angles (RadialDeg, DorsalDeg), then draws hand + fingers.
procedure TWristRenderer.Render(DorsalDeg, RadialDeg, VisualScale: Double; SimTime: Double);
var
  ScaleF: Single;
begin
  if not FInitialized then
    InitGL;

  ScaleF := Single(VisualScale);
  if ScaleF < 0.5 then ScaleF := 0.5;

  glClear(GL_COLOR_BUFFER_BIT or GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  // Camera position & mouse-drag orbit rotation
  glTranslatef(0.3, 3.1, -FCamDist);
  glRotatef(FCamPitch, 1.0, 0.0, 0.0);
  glRotatef(FCamYaw,   0.0, 1.0, 0.0);

  glLightfv(GL_LIGHT0, GL_POSITION, @LIGHT_POS[0]);

  glPushMatrix();
    DrawUpperArmAndForearm(ScaleF);

    // Apply 2-DOF wrist rotations right at the wrist joint center:
    glRotatef(Single(RadialDeg), 0.0, 1.0, 0.0); // Sagittal plane (Radial/Ulnar)
    glRotatef(Single(DorsalDeg), 1.0, 0.0, 0.0); // Frontal plane  (Dorsal/Palmar)

    DrawHand(ScaleF);
    DrawFingers(ScaleF, SimTime);
  glPopMatrix();
end;

{ ========================= PART 3: TFormMain (GUI) ========================= }

procedure TFormMain.FormCreate(Sender: TObject);
begin
  Caption := 'Wrist Joint Movement Model and Control Simulation, BME Dept. ITS';
  Width := 1240;
  Height := 720;
  Position := poScreenCenter;
  Color := RGBToColor(76, 64, 132);

  FSim := TWristSimulator.Create;
  FRenderer := TWristRenderer.Create;
  FRunning := False;
  FDragging := False;

  BuildUI;
  UpdateAnthroLabel;
  LayoutRightGrid;

  // 16 ms timer (~60 FPS) drives both RK4 substeps and chart/3D updates
  FTimer := TTimer.Create(Self);
  FTimer.Interval := 16;
  FTimer.Enabled := False;
  FTimer.OnTimer := @OnSimTimer;
end;

// Clean teardown: stop timer, free OpenGL quadrics, and free physics/renderer objects
procedure TFormMain.FormDestroy(Sender: TObject);
begin
  if Assigned(FTimer) then
    FTimer.Enabled := False;

  if Assigned(FGLBox) and FGLBox.HandleAllocated then
  begin
    FGLBox.MakeCurrent;
    FRenderer.FreeGL;
  end;

  FreeAndNil(FRenderer);
  FreeAndNil(FSim);
end;

procedure TFormMain.FormResize(Sender: TObject);
begin
  if Assigned(FPnlLeft) and Assigned(FPnlRight) then
  begin
    FPnlLeft.Width := Round(ClientWidth * 0.42);
    LayoutRightGrid;
  end;
end;

// Helper to create a clean white TAChart matching PDF Figure 2 styling
function TFormMain.CreateStyledChart(AParent: TWinControl; const ATitle: string;
  ShowLegend: Boolean; UseFixedY: Boolean; YMinVal, YMaxVal: Double): TChart;
begin
  Result := TChart.Create(Self);
  Result.Parent := AParent;
  Result.Color := clWhite;
  Result.BackColor := clWhite;
  Result.Frame.Color := clGray;
  Result.Title.Text.Text := ATitle;
  Result.Title.Visible := (ATitle <> '');
  Result.Title.Font.Size := 8;
  Result.Title.Font.Color := clNavy;
  Result.Legend.Visible := ShowLegend;
  Result.Legend.BackgroundBrush.Color := clWhite;
  Result.Legend.Frame.Color := clSilver;
  Result.Legend.Font.Size := 7;
  Result.LeftAxis.Marks.LabelFont.Size := 7;
  Result.BottomAxis.Marks.LabelFont.Size := 7;
  Result.LeftAxis.Grid.Color := clSilver;
  Result.BottomAxis.Grid.Color := clSilver;
  Result.Margins.Left := 4;
  Result.Margins.Top := 2;
  Result.Margins.Right := 4;
  Result.Margins.Bottom := 2;

  if UseFixedY then
  begin
    Result.Extent.UseYMin := True;
    Result.Extent.UseYMax := True;
    Result.Extent.YMin := YMinVal;
    Result.Extent.YMax := YMaxVal;
  end;
end;

function TFormMain.AddSeriesToChart(AChart: TChart; const ATitle: string;
  AColor: TColor; AWidth: Integer): TLineSeries;
begin
  Result := TLineSeries.Create(AChart);
  Result.Title := ATitle;
  Result.SeriesColor := AColor;
  Result.LinePen.Width := AWidth;
  AChart.AddSeries(Result);
end;

// Builds the entire Figure 2 UI programmatically (zero manual Form Designer dragging)
procedure TFormMain.BuildUI;
  function MakeBtn(const ACaption: string; ALeft: Integer; AClick: TNotifyEvent): TButton;
  begin
    Result := TButton.Create(Self);
    Result.Parent := FPnlToolbar;
    Result.Caption := ACaption;
    Result.Left := ALeft;
    Result.Top := 3;
    Result.Width := 92;
    Result.Height := 26;
    Result.OnClick := AClick;
  end;

  function MakeField(const ALbl: string; const ADefault: string;
    AX, AY, AEdtW: Integer): TEdit;
  var
    L: TLabel;
  begin
    L := TLabel.Create(Self);
    L.Parent := FPnlParams;
    L.Caption := ALbl;
    L.Left := AX;
    L.Top := AY + 3;
    L.Font.Size := 8;
    L.Font.Color := clWhite;

    Result := TEdit.Create(Self);
    Result.Parent := FPnlParams;
    Result.Text := ADefault;
    Result.Left := AX + 56;
    Result.Top := AY;
    Result.Width := AEdtW;
    Result.Height := 23;
    Result.Font.Size := 8;
    Result.OnExit := @OnParamExit;
  end;

var
  LblMode: TLabel;
begin
  // --- LEFT PANE: Top 4-button strip + 3D OpenGL Viewport ---
  FPnlLeft := TPanel.Create(Self);
  FPnlLeft.Parent := Self;
  FPnlLeft.Align := alLeft;
  FPnlLeft.Width := Round(ClientWidth * 0.42);
  FPnlLeft.BevelOuter := bvNone;
  FPnlLeft.Color := Color;

  FPnlToolbar := TPanel.Create(Self);
  FPnlToolbar.Parent := FPnlLeft;
  FPnlToolbar.Align := alTop;
  FPnlToolbar.Height := 32;
  FPnlToolbar.BevelOuter := bvNone;
  FPnlToolbar.Color := clBtnFace;

  FBtnStart := MakeBtn('Transversal', 4,   @OnStartClick);
  FBtnStop  := MakeBtn('Stop',        100, @OnStopClick);
  FBtnReset := MakeBtn('Reset',       196, @OnResetClick);
  FBtnClose := MakeBtn('Close',       292, @OnCloseClick);

  FGLBox := TOpenGLControl.Create(Self);
  FGLBox.Parent := FPnlLeft;
  FGLBox.Align := alClient;
  FGLBox.AutoResizeViewport := False;
  FGLBox.OnPaint := @OnGLPaint;
  FGLBox.OnResize := @OnGLResize;
  FGLBox.OnMouseDown := @OnGLMouseDown;
  FGLBox.OnMouseMove := @OnGLMouseMove;
  FGLBox.OnMouseUp := @OnGLMouseUp;

  // --- RIGHT PANE: Exact 3x2 Figure 2 Chart Grid ---
  FPnlRight := TPanel.Create(Self);
  FPnlRight.Parent := Self;
  FPnlRight.Align := alClient;
  FPnlRight.BevelOuter := bvNone;
  FPnlRight.Color := Color;

  // Chart 1 (Row 1, Col 1): Active Joint Torques tau1 & tau2 (N*m)
  FChartTorque := CreateStyledChart(FPnlRight, '', True, False);
  FSerTau1 := AddSeriesToChart(FChartTorque, 'tau1', clRed, 1);
  FSerTau2 := AddSeriesToChart(FChartTorque, 'tau2', clGreen, 1);

  // Chart 2 (Row 1, Col 2): Radial/Ulnar Flexion Target vs Realize (-30..30 deg)
  FChartRadial := CreateStyledChart(FPnlRight, 'Radial/Ulnar Flexion', True, True, -30.0, 30.0);
  FSerRadTarget  := AddSeriesToChart(FChartRadial, 'Target', clRed, 1);
  FSerRadRealize := AddSeriesToChart(FChartRadial, 'Realize', clGreen, 1);

  // Chart 3 (Row 2, Col 1): Live RMSE title + error 1 & error 2 (0..30 deg)
  FChartError := CreateStyledChart(FPnlRight, 'RMSE: 0.0000', True, True, 0.0, 30.0);
  FSerErr1 := AddSeriesToChart(FChartError, 'error 1', clRed, 1);
  FSerErr2 := AddSeriesToChart(FChartError, 'error 2', clGreen, 1);

  // Chart 4 (Row 2, Col 2): Dorsal/Palmar Flexion Target vs Realize (-30..30 deg)
  FChartDorsal := CreateStyledChart(FPnlRight, 'Dorsal/Palmar', True, True, -30.0, 30.0);
  FSerDorTarget  := AddSeriesToChart(FChartDorsal, 'Target', clRed, 1);
  FSerDorRealize := AddSeriesToChart(FChartDorsal, 'Realize', clGreen, 1);

  // Chart 5 (Row 3, Col 1): Transversal Phase Angle (Dorsal vs Radial circle)
  FChartPhase := CreateStyledChart(FPnlRight, 'Transversal Phase Angle', False, True, -30.0, 30.0);
  FChartPhase.Extent.UseXMin := True;
  FChartPhase.Extent.UseXMax := True;
  FChartPhase.Extent.XMin := -30.0;
  FChartPhase.Extent.XMax := 30.0;
  FSerPhase := AddSeriesToChart(FChartPhase, 'Phase', clBlack, 1);
  FSerPhase.ShowLines := False;
  FSerPhase.ShowPoints := True;
  FSerPhase.Pointer.Style := psCircle;
  FSerPhase.Pointer.HorizSize := 2;
  FSerPhase.Pointer.VertSize := 2;
  FSerPhase.Pointer.Brush.Color := clBlack;
  FSerPhase.Pointer.Pen.Color := clBlack;

  // Bottom-Right Cell (Row 3, Col 2): Minimal Parameter Controls
  FPnlParams := TPanel.Create(Self);
  FPnlParams.Parent := FPnlRight;
  FPnlParams.BevelOuter := bvNone;
  FPnlParams.Color := RGBToColor(62, 52, 112);

  LblMode := TLabel.Create(Self);
  LblMode.Parent := FPnlParams;
  LblMode.Caption := 'Mode:';
  LblMode.Left := 12;
  LblMode.Top := 14;
  LblMode.Font.Size := 8;
  LblMode.Font.Color := clWhite;

  FCmbMode := TComboBox.Create(Self);
  FCmbMode.Parent := FPnlParams;
  FCmbMode.Style := csDropDownList;
  FCmbMode.Left := 68;
  FCmbMode.Top := 10;
  FCmbMode.Width := 180;
  FCmbMode.Items.Add('Transversal (3D)');
  FCmbMode.Items.Add('Dorsal/Palmar (Frontal)');
  FCmbMode.Items.Add('Radial/Ulnar (Sagittal)');
  FCmbMode.Items.Add('Passive Test: Dorsal');
  FCmbMode.Items.Add('Passive Test: Radial');
  FCmbMode.ItemIndex := 0;
  FCmbMode.OnChange := @OnModeChange;

  FEdtBW := MakeField('BW (kg):', '60',    12,  44, 62);
  FEdtBH := MakeField('BH (cm):', '160',  142,  44, 62);

  FEdtKp := MakeField('Kp:',      '15.0',  12,  76, 62);
  FEdtKi := MakeField('Ki:',      '40.0', 142,  76, 62);
  FEdtKd := MakeField('Kd:',      '0.25',  12, 108, 62);

  FEdtC  := MakeField('C damp:',  '0.010', 142, 108, 62);
  FEdtK  := MakeField('k stiff:', '0.20',   12, 140, 62);

  FLblAnthroInfo := TLabel.Create(Self);
  FLblAnthroInfo.Parent := FPnlParams;
  FLblAnthroInfo.Left := 12;
  FLblAnthroInfo.Top := 174;
  FLblAnthroInfo.Font.Size := 8;
  FLblAnthroInfo.Font.Color := RGBToColor(220, 220, 245);
end;

// Dynamically positions the 5 charts + parameter panel into an even 3x2 grid on resize
procedure TFormMain.LayoutRightGrid;
const
  GAP = 6;
var
  CellW, CellH: Integer;
begin
  if not Assigned(FChartTorque) then Exit;

  CellW := (FPnlRight.ClientWidth - 3 * GAP) div 2;
  CellH := (FPnlRight.ClientHeight - 4 * GAP) div 3;
  if CellW < 80 then CellW := 80;
  if CellH < 80 then CellH := 80;

  FChartTorque.SetBounds(GAP,             GAP,                  CellW, CellH);
  FChartRadial.SetBounds(2 * GAP + CellW, GAP,                  CellW, CellH);

  FChartError.SetBounds(GAP,              2 * GAP + CellH,      CellW, CellH);
  FChartDorsal.SetBounds(2 * GAP + CellW, 2 * GAP + CellH,      CellW, CellH);

  FChartPhase.SetBounds(GAP,              3 * GAP + 2 * CellH,  CellW, CellH);
  FPnlParams.SetBounds(2 * GAP + CellW,   3 * GAP + 2 * CellH,  CellW, CellH);
end;

// Reads BW, BH, PID gains (Kp, Ki, Kd), and Passive (C, k) from text boxes into FSim
procedure TFormMain.ApplyInputsToSim;
var
  BW, BH, Kp, Ki, Kd, CVal, KVal: Double;
  PID: TPIDParams;
  PassD, PassR: TPassiveParams;
begin
  BW := StrToFloatDef(FEdtBW.Text, 60.0);
  BH := StrToFloatDef(FEdtBH.Text, 160.0);
  FSim.UpdateAnthropometry(BW, BH);

  Kp := StrToFloatDef(FEdtKp.Text, 15.0);
  Ki := StrToFloatDef(FEdtKi.Text, 40.0);
  Kd := StrToFloatDef(FEdtKd.Text, 0.25);

  PID.Kp := Kp;
  PID.Ki := Ki;
  PID.Kd := Kd;
  FSim.PIDDorsal := PID;
  FSim.PIDRadial := PID;

  CVal := StrToFloatDef(FEdtC.Text, 0.010);
  KVal := StrToFloatDef(FEdtK.Text, 0.20);

  PassD := FSim.PassiveDorsal;
  PassD.DampingC := CVal;
  PassD.StiffnessK := KVal;
  FSim.PassiveDorsal := PassD;

  PassR := FSim.PassiveRadial;
  PassR.DampingC := CVal;
  PassR.StiffnessK := KVal;
  FSim.PassiveRadial := PassR;

  UpdateAnthroLabel;
end;

procedure TFormMain.UpdateAnthroLabel;
begin
  FLblAnthroInfo.Caption := Format(
    'Hand: m = %.3f kg, l = %.2f cm | Io = %.6f kg*m^2',
    [FSim.Anthro.HandMassKg, FSim.Anthro.HandLengthM * 100.0, FSim.Anthro.Io]
  );
end;

procedure TFormMain.ClearAllCharts;
begin
  FSerTau1.Clear;
  FSerTau2.Clear;
  FSerRadTarget.Clear;
  FSerRadRealize.Clear;
  FSerErr1.Clear;
  FSerErr2.Clear;
  FSerDorTarget.Clear;
  FSerDorRealize.Clear;
  FSerPhase.Clear;
  FChartError.Title.Text.Text := 'RMSE: 0.0000';
end;

procedure TFormMain.OnStartClick(Sender: TObject);
begin
  ApplyInputsToSim;
  if FSim.StepCount = 0 then
    ClearAllCharts;
  FRunning := True;
  FTimer.Enabled := True;
end;

procedure TFormMain.OnStopClick(Sender: TObject);
begin
  FRunning := False;
  FTimer.Enabled := False;
end;

procedure TFormMain.OnResetClick(Sender: TObject);
begin
  FRunning := False;
  FTimer.Enabled := False;
  ApplyInputsToSim;
  FSim.Reset;
  FRenderer.ResetCamera;
  ClearAllCharts;
  FGLBox.Invalidate;
end;

procedure TFormMain.OnCloseClick(Sender: TObject);
begin
  Close;
end;

procedure TFormMain.OnModeChange(Sender: TObject);
begin
  FRunning := False;
  FTimer.Enabled := False;
  ApplyInputsToSim;

  case FCmbMode.ItemIndex of
    0: begin FSim.SetMode(smTransversal);   FBtnStart.Caption := 'Transversal'; end;
    1: begin FSim.SetMode(smDorsalPalmar);  FBtnStart.Caption := 'Dorsal';      end;
    2: begin FSim.SetMode(smRadialUlnar);   FBtnStart.Caption := 'Radial';      end;
    3: begin FSim.SetMode(smPassiveDorsal); FBtnStart.Caption := 'Passive D';   end;
    4: begin FSim.SetMode(smPassiveRadial); FBtnStart.Caption := 'Passive R';   end;
  end;

  ClearAllCharts;
  FGLBox.Invalidate;
end;

procedure TFormMain.OnParamExit(Sender: TObject);
begin
  ApplyInputsToSim;
  FGLBox.Invalidate;
end;

// Main simulation tick (fires every 16 ms):
// Runs 10 physics substeps (10 * 0.001 s = 10 ms simulated time), appends points
// to all 5 charts, updates RMSE, and requests an OpenGL 3D redraw.
procedure TFormMain.OnSimTimer(Sender: TObject);
var
  i: Integer;
  t: Double;
begin
  if not FRunning then Exit;

  for i := 1 to SUBSTEPS_PER_TICK do
    FSim.Step;

  t := FSim.SimTime;

  FSerTau1.AddXY(t, FSim.Dorsal.ActiveTorque);
  FSerTau2.AddXY(t, FSim.Radial.ActiveTorque);

  FSerRadTarget.AddXY(t, FSim.Radial.TargetDeg);
  FSerRadRealize.AddXY(t, FSim.Radial.RealizeDeg);

  FSerErr1.AddXY(t, FSim.Dorsal.ErrorDeg);
  FSerErr2.AddXY(t, FSim.Radial.ErrorDeg);
  FChartError.Title.Text.Text := Format('RMSE: %.6f', [FSim.RMSE]);

  FSerDorTarget.AddXY(t, FSim.Dorsal.TargetDeg);
  FSerDorRealize.AddXY(t, FSim.Dorsal.RealizeDeg);

  FSerPhase.AddXY(FSim.Dorsal.RealizeDeg, FSim.Radial.RealizeDeg);

  FGLBox.Invalidate;
end;

// OpenGL paint callback: draws current 3D wrist pose and swaps double buffer
procedure TFormMain.OnGLPaint(Sender: TObject);
var
  VisScale: Double;
begin
  if not FGLBox.HandleAllocated then Exit;
  FGLBox.MakeCurrent;
  FRenderer.Resize(FGLBox.ClientWidth, FGLBox.ClientHeight);
  VisScale := FSim.Anthro.BodyHeightCm / 160.0; // Scales 3D limb with subject BH
  FRenderer.Render(FSim.Dorsal.RealizeDeg, FSim.Radial.RealizeDeg, VisScale, FSim.SimTime);
  FGLBox.SwapBuffers;
end;

procedure TFormMain.OnGLResize(Sender: TObject);
begin
  if FGLBox.HandleAllocated then
  begin
    FGLBox.MakeCurrent;
    FRenderer.Resize(FGLBox.ClientWidth, FGLBox.ClientHeight);
    FGLBox.Invalidate;
  end;
end;

// Left-click and drag on the 3D viewport to orbit camera yaw/pitch
procedure TFormMain.OnGLMouseDown(Sender: TObject; Button: TMouseButton;
  Shift: TShiftState; X, Y: Integer);
begin
  if Button = mbLeft then
  begin
    FDragging := True;
    FLastMouseX := X;
    FLastMouseY := Y;
  end;
end;

procedure TFormMain.OnGLMouseMove(Sender: TObject; Shift: TShiftState; X, Y: Integer);
begin
  if FDragging then
  begin
    FRenderer.CamYaw := FRenderer.CamYaw + (X - FLastMouseX) * 0.4;
    FRenderer.CamPitch := FRenderer.CamPitch + (Y - FLastMouseY) * 0.4;
    FLastMouseX := X;
    FLastMouseY := Y;
    FGLBox.Invalidate;
  end;
end;

procedure TFormMain.OnGLMouseUp(Sender: TObject; Button: TMouseButton;
  Shift: TShiftState; X, Y: Integer);
begin
  if Button = mbLeft then
    FDragging := False;
end;

end.
