unit uWristPhysics;

{$mode objfpc}{$H+}

interface

uses
  SysUtils, Math;

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
    Ig: Double;               // m * l^2 / 12 (uniform rod about CoG)
    Io: Double;               // Ig + m * (l/2)^2 = m * l^2 / 3 (about wrist joint)
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
    ThetaRad: Double;         // Current joint angle (rad)
    ThetaDot: Double;         // Current angular velocity (rad/s)
    TargetDeg: Double;        // Reference angle (deg)
    RealizeDeg: Double;       // Realized angle (deg)
    ErrorDeg: Double;         // TargetDeg - RealizeDeg (deg)
    ActiveTorque: Double;     // tau_active (N*m)
    PassiveTorque: Double;    // tau_passive (N*m)
    IntegralErr: Double;      // Integral of error (rad*s)
    PrevErrRad: Double;       // Previous error (rad)
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
    property DeltaTime: Double read FDeltaTime write FDeltaTime;
    property FreqHz: Double read FFreqHz write FFreqHz;
    property AmplitudeDeg: Double read FAmplitudeDeg write FAmplitudeDeg;
    property StepCount: Integer read FStepCount;
    property RMSE: Double read FRMSE;
  end;

implementation

const
  DEG_TO_RAD = Pi / 180.0;
  RAD_TO_DEG = 180.0 / Pi;
  MAX_INTEGRAL = 2.0;   // Anti-windup clamp (rad*s)
  MAX_TORQUE   = 50.0;  // Physical safety saturation (N*m)

constructor TWristSimulator.Create;
begin
  inherited Create;
  FGravity := 9.81;
  FDeltaTime := 0.001;
  FFreqHz := 0.8;
  FAmplitudeDeg := 30.0;
  FMode := smTransversal;

  // Default subject from BioMod Project-2627.pdf: 60 kg BW, 160 cm BH
  UpdateAnthropometry(60.0, 160.0);

  // Tuned PID gains for low-RMSE tracking with physical Lagrangian dynamics
  FPIDDorsal.Kp := 15.0;
  FPIDDorsal.Ki := 40.0;
  FPIDDorsal.Kd := 0.25;

  FPIDRadial.Kp := 15.0;
  FPIDRadial.Ki := 40.0;
  FPIDRadial.Kd := 0.25;

  // Linear passive joint torque parameters: tau_p = -C * theta_dot - k * (theta - theta_0)
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

  // Dempster segment weights & lengths (BioMod Project-2627.pdf, page 2)
  FAnthro.HandMassKg := 0.006 * BWKg;
  FAnthro.HandLengthM := 0.0575 * BHM;
  FAnthro.ForearmMassKg := 0.016 * BWKg;
  FAnthro.ForearmLengthM := 0.1570 * BHM;
  FAnthro.UpperArmMassKg := 0.028 * BWKg;
  FAnthro.UpperArmLengthM := 0.1720 * BHM;

  // Uniform rod model: Ig = m*l^2/12, Io = Ig + m*(l/2)^2
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

  // Passive movement tests release the wrist from a displaced initial angle
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
  // Linear passive joint torque: -C * theta_dot - k * (theta - theta_0)
  Result := -P.DampingC * VelRad - P.StiffnessK * (AngleRad - Theta0Rad);
end;

function TWristSimulator.ComputeAccel(AngleRad, VelRad, ActiveTau: Double;
  const P: TPassiveParams): Double;
var
  GravTau, PassiveTau, NetTau: Double;
begin
  // Single pendulum Lagrangian EOM:
  // Io * theta_ddot + m * g * (l/2) * sin(theta) = tau_active + tau_passive
  GravTau := -FAnthro.HandMassKg * FGravity * (FAnthro.HandLengthM * 0.5) * Sin(AngleRad);
  PassiveTau := ComputePassiveTorque(AngleRad, VelRad, P);
  NetTau := ActiveTau + PassiveTau + GravTau;
  Result := NetTau / FAnthro.Io;
end;

procedure TWristSimulator.StepRK4(var AngleRad, VelRad: Double; ActiveTau: Double;
  const P: TPassiveParams);
var
  k1x, k1v, k2x, k2v, k3x, k3v, k4x, k4v: Double;
  dt: Double;
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
  RefDorsal, RefRadial: Double;
  ErrSq: Double;
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

end.
