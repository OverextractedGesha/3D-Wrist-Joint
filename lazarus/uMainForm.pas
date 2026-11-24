unit uMainForm;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Math, Forms, Controls, Graphics, Dialogs, StdCtrls, ExtCtrls,
  GL, GLU, OpenGLContext, TAGraph, TASeries, TATypes, TAChartUtils;

type
  TSimMode = (
    smTransversal,
    smDorsalPalmar,
    smRadialUlnar,
    smPassiveDorsal,
    smPassiveRadial
  );

  TAnthropometry = record
    BodyWeightKg: Double;     // BW (kg), default 60.0
    BodyHeightCm: Double;     // BH (cm), default 160.0
    HandMassKg: Double;       // 0.6% BW (Dempster)
    HandLengthM: Double;      // 5.75% BH
    ForearmMassKg: Double;    // 1.6% BW (Dempster)
    ForearmLengthM: Double;   // 15.70% BH
    UpperArmMassKg: Double;   // 2.8% BW (Dempster)
    UpperArmLengthM: Double;  // 17.20% BH
    Ig: Double;               // m * l^2 / 12
    Io: Double;               // Ig + m * (l/2)^2 = m * l^2 / 3
  end;

  TPIDParams = record
    Kp: Double;               // N*m / rad
    Ki: Double;               // N*m / (rad*s)
    Kd: Double;               // N*m*s / rad
  end;

  TPassiveParams = record
    DampingC: Double;         // C (N*m*s / rad)
    StiffnessK: Double;       // k (N*m / rad)
    RestAngleDeg: Double;     // theta_0 (deg)
  end;

  TAxisState = record
    ThetaRad: Double;
    ThetaDot: Double;
    TargetDeg: Double;
    RealizeDeg: Double;
    ErrorDeg: Double;
    ActiveTorque: Double;
    PassiveTorque: Double;
    IntegralErr: Double;
    PrevErrRad: Double;
  end;

  TWristSimulator = class
  private
    FAnthro: TAnthropometry;
    FPIDDorsal: TPIDParams;
    FPIDRadial: TPIDParams;
    FPassiveDorsal: TPassiveParams;
    FPassiveRadial: TPassiveParams;
    FDorsal: TAxisState;
    FRadial: TAxisState;
    FMode: TSimMode;
    FSimTime: Double;
    FDeltaTime: Double;
    FFreqHz: Double;
    FAmplitudeDeg: Double;
    FGravity: Double;
    FStepCount: Integer;
    FSquaredErrSum: Double;
    FRMSE: Double;

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

  TWristRenderer = class
  private
    FQuadSphere: PGLUquadric;
    FQuadCylinder: PGLUquadric;
    FInitialized: Boolean;
    FCamYaw: Single;
    FCamPitch: Single;
    FCamDist: Single;

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

  TFormMain = class(TForm)
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure FormResize(Sender: TObject);
  private
    FSim: TWristSimulator;
    FRenderer: TWristRenderer;
    FTimer: TTimer;
    FRunning: Boolean;

    FPnlLeft: TPanel;
    FPnlToolbar: TPanel;
    FBtnStart: TButton;
    FBtnStop: TButton;
    FBtnReset: TButton;
    FBtnClose: TButton;
    FGLBox: TOpenGLControl;

    FPnlRight: TPanel;
    FChartTorque: TChart;
    FChartRadial: TChart;
    FChartError: TChart;
    FChartDorsal: TChart;
    FChartPhase: TChart;
    FPnlParams: TPanel;

    FSerTau1, FSerTau2: TLineSeries;
    FSerRadTarget, FSerRadRealize: TLineSeries;
    FSerErr1, FSerErr2: TLineSeries;
    FSerDorTarget, FSerDorRealize: TLineSeries;
    FSerPhase: TLineSeries;

    FCmbMode: TComboBox;
    FEdtBW, FEdtBH: TEdit;
    FEdtKp, FEdtKi, FEdtKd: TEdit;
    FEdtC, FEdtK: TEdit;
    FLblAnthroInfo: TLabel;

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

    procedure OnStartClick(Sender: TObject);
    procedure OnStopClick(Sender: TObject);
    procedure OnResetClick(Sender: TObject);
    procedure OnCloseClick(Sender: TObject);
    procedure OnModeChange(Sender: TObject);
    procedure OnParamExit(Sender: TObject);
    procedure OnSimTimer(Sender: TObject);

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
  MAX_INTEGRAL      = 2.0;
  MAX_TORQUE        = 50.0;
  SUBSTEPS_PER_TICK = 10;

  MAT_SPECULAR: array[0..3] of GLfloat = (0.75, 0.75, 0.65, 1.0);
  MAT_DIFFUSE:  array[0..3] of GLfloat = (0.92, 0.90, 0.80, 1.0);
  MAT_AMBIENT:  array[0..3] of GLfloat = (0.32, 0.30, 0.26, 1.0);
  MAT_SHININESS: GLfloat = 36.0;
  LIGHT_POS:    array[0..3] of GLfloat = (10.0, 12.0, 14.0, 1.0);
  LIGHT_DIFF:   array[0..3] of GLfloat = (1.0, 0.98, 0.90, 1.0);
  LIGHT_AMB:    array[0..3] of GLfloat = (0.30, 0.30, 0.32, 1.0);

{ ========================= TWristSimulator ========================= }

constructor TWristSimulator.Create;
begin
  inherited Create;
  FGravity := 9.81;
  FDeltaTime := 0.001;
  FFreqHz := 0.8;
  FAmplitudeDeg := 30.0;
  FMode := smTransversal;

  UpdateAnthropometry(60.0, 160.0);

  FPIDDorsal.Kp := 15.0;
  FPIDDorsal.Ki := 40.0;
  FPIDDorsal.Kd := 0.25;

  FPIDRadial.Kp := 15.0;
  FPIDRadial.Ki := 40.0;
  FPIDRadial.Kd := 0.25;

  FPassiveDorsal.DampingC := 0.010;
  FPassiveDorsal.StiffnessK := 0.20;
  FPassiveDorsal.RestAngleDeg := 0.0;

  FPassiveRadial.DampingC := 0.012;
  FPassiveRadial.StiffnessK := 0.24;
  FPassiveRadial.RestAngleDeg := 0.0;

  Reset;
end;

procedure TWristSimulator.UpdateAnthropometry(BWKg, BHCm: Double);
var
  BHM: Double;
begin
  if BWKg < 10.0 then BWKg := 10.0;
  if BHCm < 50.0 then BHCm := 50.0;

  BHM := BHCm / 100.0;
  FAnthro.BodyWeightKg := BWKg;
  FAnthro.BodyHeightCm := BHCm;

  FAnthro.HandMassKg := 0.006 * BWKg;
  FAnthro.HandLengthM := 0.0575 * BHM;
  FAnthro.ForearmMassKg := 0.016 * BWKg;
  FAnthro.ForearmLengthM := 0.1570 * BHM;
  FAnthro.UpperArmMassKg := 0.028 * BWKg;
  FAnthro.UpperArmLengthM := 0.1720 * BHM;

  FAnthro.Ig := (FAnthro.HandMassKg * Sqr(FAnthro.HandLengthM)) / 12.0;
  FAnthro.Io := FAnthro.Ig + FAnthro.HandMassKg * Sqr(FAnthro.HandLengthM / 2.0);
end;

procedure TWristSimulator.SetMode(AMode: TSimMode);
begin
  FMode := AMode;
  Reset;
end;

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

function TWristSimulator.ComputePassiveTorque(AngleRad, VelRad: Double;
  const P: TPassiveParams): Double;
var
  Theta0Rad: Double;
begin
  Theta0Rad := P.RestAngleDeg * DEG_TO_RAD;
  Result := -P.DampingC * VelRad - P.StiffnessK * (AngleRad - Theta0Rad);
end;

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

procedure TWristSimulator.StepRK4(var AngleRad, VelRad: Double; ActiveTau: Double;
  const P: TPassiveParams);
var
  k1x, k1v, k2x, k2v, k3x, k3v, k4x, k4v, dt: Double;
begin
  dt := FDeltaTime;

  k1x := VelRad;
  k1v := ComputeAccel(AngleRad, VelRad, ActiveTau, P);

  k2x := VelRad + 0.5 * dt * k1v;
  k2v := ComputeAccel(AngleRad + 0.5 * dt * k1x, VelRad + 0.5 * dt * k1v, ActiveTau, P);

  k3x := VelRad + 0.5 * dt * k2v;
  k3v := ComputeAccel(AngleRad + 0.5 * dt * k2x, VelRad + 0.5 * dt * k2v, ActiveTau, P);

  k4x := VelRad + dt * k3v;
  k4v := ComputeAccel(AngleRad + dt * k3x, VelRad + dt * k3v, ActiveTau, P);

  AngleRad := AngleRad + (dt / 6.0) * (k1x + 2.0 * k2x + 2.0 * k3x + k4x);
  VelRad   := VelRad   + (dt / 6.0) * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
end;

procedure TWristSimulator.StepAxisActive(var Axis: TAxisState; TargetDegVal: Double;
  const PID: TPIDParams; const PassP: TPassiveParams);
var
  TargetRad, ErrRad, DerivRad, Tau: Double;
begin
  Axis.TargetDeg := TargetDegVal;
  TargetRad := TargetDegVal * DEG_TO_RAD;

  ErrRad := TargetRad - Axis.ThetaRad;
  Axis.IntegralErr := EnsureRange(Axis.IntegralErr + ErrRad * FDeltaTime,
    -MAX_INTEGRAL, MAX_INTEGRAL);

  if FStepCount = 0 then
    DerivRad := 0.0
  else
    DerivRad := (ErrRad - Axis.PrevErrRad) / FDeltaTime;

  Tau := (PID.Kp * ErrRad) + (PID.Ki * Axis.IntegralErr) + (PID.Kd * DerivRad);
  Tau := EnsureRange(Tau, -MAX_TORQUE, MAX_TORQUE);

  Axis.ActiveTorque := Tau;
  Axis.PassiveTorque := ComputePassiveTorque(Axis.ThetaRad, Axis.ThetaDot, PassP);
  Axis.PrevErrRad := ErrRad;

  StepRK4(Axis.ThetaRad, Axis.ThetaDot, Tau, PassP);

  Axis.RealizeDeg := Axis.ThetaRad * RAD_TO_DEG;
  Axis.ErrorDeg := Abs(Axis.TargetDeg - Axis.RealizeDeg);
end;

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

procedure TWristSimulator.Step;
var
  RefDorsal, RefRadial, ErrSq: Double;
begin
  FSimTime := FSimTime + FDeltaTime;
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

  Inc(FStepCount);
  FSquaredErrSum := FSquaredErrSum + ErrSq;
  FRMSE := Sqrt(FSquaredErrSum / FStepCount);
end;

{ ========================= TWristRenderer ========================= }

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

procedure TWristRenderer.InitGL;
begin
  if FInitialized then Exit;

  glClearColor(0.30, 0.25, 0.52, 1.0);
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

procedure TWristRenderer.DrawUpperArmAndForearm(VisualScale: Single);
var
  UpperLen, ForearmLen: Single;
begin
  UpperLen := 3.4 * VisualScale;
  ForearmLen := 3.1 * VisualScale;

  gluSphere(FQuadSphere, 0.72 * VisualScale, 28, 28);

  glRotatef(18.0, 0.0, 0.0, -1.0);
  glRotatef(90.0, 1.0, 0.0, 0.0);
  gluCylinder(FQuadCylinder, 0.65 * VisualScale, 0.48 * VisualScale, UpperLen, 28, 8);

  glTranslatef(0.0, 0.0, UpperLen);
  gluSphere(FQuadSphere, 0.50 * VisualScale, 28, 28);

  glRotatef(22.0, 0.0, 1.0, 0.0);
  gluCylinder(FQuadCylinder, 0.48 * VisualScale, 0.34 * VisualScale, ForearmLen, 28, 8);

  glTranslatef(0.0, 0.0, ForearmLen);
  gluSphere(FQuadSphere, 0.36 * VisualScale, 28, 28);
end;

procedure TWristRenderer.DrawHand(VisualScale: Single);
var
  W, T, L: Single;
begin
  W := 0.44 * VisualScale;
  T := 0.16 * VisualScale;
  L := 1.05 * VisualScale;

  glBegin(GL_QUADS);
    glNormal3f(0.0, 0.0, 1.0);
    glVertex3f(-W,  T, L); glVertex3f( W,  T, L);
    glVertex3f( W, -T, L); glVertex3f(-W, -T, L);

    glNormal3f(0.0, 0.0, -1.0);
    glVertex3f(-W,  T, 0.0); glVertex3f(-W, -T, 0.0);
    glVertex3f( W, -T, 0.0); glVertex3f( W,  T, 0.0);

    glNormal3f(0.0, 1.0, 0.0);
    glVertex3f(-W,  T, 0.0); glVertex3f( W,  T, 0.0);
    glVertex3f( W,  T, L);   glVertex3f(-W,  T, L);

    glNormal3f(0.0, -1.0, 0.0);
    glVertex3f(-W, -T, 0.0); glVertex3f(-W, -T, L);
    glVertex3f( W, -T, L);   glVertex3f( W, -T, 0.0);

    glNormal3f(1.0, 0.0, 0.0);
    glVertex3f( W,  T, 0.0); glVertex3f( W, -T, 0.0);
    glVertex3f( W, -T, L);   glVertex3f( W,  T, L);

    glNormal3f(-1.0, 0.0, 0.0);
    glVertex3f(-W,  T, 0.0); glVertex3f(-W,  T, L);
    glVertex3f(-W, -T, L);   glVertex3f(-W, -T, 0.0);
  glEnd();
end;

procedure TWristRenderer.DrawFingers(VisualScale: Single; SimTime: Double);
var
  L1, L2, L3, Space, Curl: Single;

  procedure DrawOneFinger(XOff, BaseR, ScaleLen, ExtraYaw: Single);
  begin
    glPushMatrix();
      glTranslatef(XOff, 0.0, 0.0);
      glRotatef(ExtraYaw, 0.0, 1.0, 0.0);

      glRotatef(Curl * 0.45, 1.0, 0.0, 0.0);
      gluSphere(FQuadSphere, BaseR * 1.05, 14, 14);
      gluCylinder(FQuadCylinder, BaseR, BaseR * 0.92, L1 * ScaleLen, 14, 4);

      glTranslatef(0.0, 0.0, L1 * ScaleLen);
      glRotatef(Curl * 0.55, 1.0, 0.0, 0.0);
      gluSphere(FQuadSphere, BaseR * 0.94, 14, 14);
      gluCylinder(FQuadCylinder, BaseR * 0.92, BaseR * 0.82, L2 * ScaleLen, 14, 4);

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
  Curl := 10.0 + 6.0 * Abs(Sin(SimTime * 1.2));

  glPushMatrix();
    glTranslatef(0.0, 0.0, 1.05 * VisualScale);
    DrawOneFinger(-Space * 1.65, 0.068 * VisualScale, 0.82, -4.0);
    DrawOneFinger(-Space * 0.55, 0.074 * VisualScale, 0.96, -1.0);
    DrawOneFinger( Space * 0.55, 0.078 * VisualScale, 1.05,  1.0);
    DrawOneFinger( Space * 1.65, 0.075 * VisualScale, 0.95,  4.0);
  glPopMatrix();

  glPushMatrix();
    glTranslatef(0.42 * VisualScale, 0.06 * VisualScale, 0.48 * VisualScale);
    DrawOneFinger(0.0, 0.088 * VisualScale, 0.72, 34.0);
  glPopMatrix();
end;

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

  glTranslatef(0.3, 3.1, -FCamDist);
  glRotatef(FCamPitch, 1.0, 0.0, 0.0);
  glRotatef(FCamYaw,   0.0, 1.0, 0.0);

  glLightfv(GL_LIGHT0, GL_POSITION, @LIGHT_POS[0]);

  glPushMatrix();
    DrawUpperArmAndForearm(ScaleF);
    glRotatef(Single(RadialDeg), 0.0, 1.0, 0.0);
    glRotatef(Single(DorsalDeg), 1.0, 0.0, 0.0);
    DrawHand(ScaleF);
    DrawFingers(ScaleF, SimTime);
  glPopMatrix();
end;

{ ========================= TFormMain ========================= }

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

  FTimer := TTimer.Create(Self);
  FTimer.Interval := 16;
  FTimer.Enabled := False;
  FTimer.OnTimer := @OnSimTimer;
end;

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

  FPnlRight := TPanel.Create(Self);
  FPnlRight.Parent := Self;
  FPnlRight.Align := alClient;
  FPnlRight.BevelOuter := bvNone;
  FPnlRight.Color := Color;

  FChartTorque := CreateStyledChart(FPnlRight, '', True, False);
  FSerTau1 := AddSeriesToChart(FChartTorque, 'tau1', clRed, 1);
  FSerTau2 := AddSeriesToChart(FChartTorque, 'tau2', clGreen, 1);

  FChartRadial := CreateStyledChart(FPnlRight, 'Radial/Ulnar Flexion', True, True, -30.0, 30.0);
  FSerRadTarget  := AddSeriesToChart(FChartRadial, 'Target', clRed, 1);
  FSerRadRealize := AddSeriesToChart(FChartRadial, 'Realize', clGreen, 1);

  FChartError := CreateStyledChart(FPnlRight, 'RMSE: 0.0000', True, True, 0.0, 30.0);
  FSerErr1 := AddSeriesToChart(FChartError, 'error 1', clRed, 1);
  FSerErr2 := AddSeriesToChart(FChartError, 'error 2', clGreen, 1);

  FChartDorsal := CreateStyledChart(FPnlRight, 'Dorsal/Palmar', True, True, -30.0, 30.0);
  FSerDorTarget  := AddSeriesToChart(FChartDorsal, 'Target', clRed, 1);
  FSerDorRealize := AddSeriesToChart(FChartDorsal, 'Realize', clGreen, 1);

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

procedure TFormMain.OnGLPaint(Sender: TObject);
var
  VisScale: Double;
begin
  if not FGLBox.HandleAllocated then Exit;
  FGLBox.MakeCurrent;
  FRenderer.Resize(FGLBox.ClientWidth, FGLBox.ClientHeight);
  VisScale := FSim.Anthro.BodyHeightCm / 160.0;
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
