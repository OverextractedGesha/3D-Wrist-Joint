unit uMainForm;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, StdCtrls, ExtCtrls,
  OpenGLContext, TAGraph, TASeries, TATypes, TAChartUtils,
  uWristPhysics, uWristRender;

type
  TFormMain = class(TForm)
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure FormResize(Sender: TObject);
  private
    FSim: TWristSimulator;
    FRenderer: TWristRenderer;
    FTimer: TTimer;
    FRunning: Boolean;

    // Left Pane (Toolbar + 3D OpenGL Viewport)
    FPnlLeft: TPanel;
    FPnlToolbar: TPanel;
    FBtnStart: TButton;
    FBtnStop: TButton;
    FBtnReset: TButton;
    FBtnClose: TButton;
    FGLBox: TOpenGLControl;

    // Right Pane (Exact 3x2 Grid matching Figure 2)
    FPnlRight: TPanel;
    FChartTorque: TChart;
    FChartRadial: TChart;
    FChartError: TChart;
    FChartDorsal: TChart;
    FChartPhase: TChart;
    FPnlParams: TPanel;

    // Chart Series
    FSerTau1, FSerTau2: TLineSeries;
    FSerRadTarget, FSerRadRealize: TLineSeries;
    FSerErr1, FSerErr2: TLineSeries;
    FSerDorTarget, FSerDorRealize: TLineSeries;
    FSerPhase: TLineSeries;

    // Compact Parameter Controls (bottom-right cell)
    FCmbMode: TComboBox;
    FEdtBW, FEdtBH: TEdit;
    FEdtKp, FEdtKi, FEdtKd: TEdit;
    FEdtC, FEdtK: TEdit;
    FLblAnthroInfo: TLabel;

    // Mouse Orbit State
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
  SUBSTEPS_PER_TICK = 10; // 10 * 0.001s = 10ms physics per 15ms timer tick

procedure TFormMain.FormCreate(Sender: TObject);
begin
  Caption := 'Wrist Joint Movement Model and Control Simulation, BME Dept. ITS';
  Width := 1240;
  Height := 720;
  Position := poScreenCenter;
  Color := RGBToColor(76, 64, 132); // Indigo accent matching Figure 2

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
  // === LEFT PANE (Toolbar + 3D OpenGL Viewport) ===
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

  // === RIGHT PANE (3x2 Grid strictly matching Figure 2) ===
  FPnlRight := TPanel.Create(Self);
  FPnlRight.Parent := Self;
  FPnlRight.Align := alClient;
  FPnlRight.BevelOuter := bvNone;
  FPnlRight.Color := Color;

  // Row 1, Col 1: Active Torques (tau1, tau2)
  FChartTorque := CreateStyledChart(FPnlRight, '', True, False);
  FSerTau1 := AddSeriesToChart(FChartTorque, 'tau1', clRed, 1);
  FSerTau2 := AddSeriesToChart(FChartTorque, 'tau2', clGreen, 1);

  // Row 1, Col 2: Radial/Ulnar Flexion (Target vs Realize)
  FChartRadial := CreateStyledChart(FPnlRight, 'Radial/Ulnar Flexion', True, True, -30.0, 30.0);
  FSerRadTarget  := AddSeriesToChart(FChartRadial, 'Target', clRed, 1);
  FSerRadRealize := AddSeriesToChart(FChartRadial, 'Realize', clGreen, 1);

  // Row 2, Col 1: RMSE & Errors (error 1, error 2)
  FChartError := CreateStyledChart(FPnlRight, 'RMSE: 0.0000', True, True, 0.0, 30.0);
  FSerErr1 := AddSeriesToChart(FChartError, 'error 1', clRed, 1);
  FSerErr2 := AddSeriesToChart(FChartError, 'error 2', clGreen, 1);

  // Row 2, Col 2: Dorsal/Palmar (Target vs Realize)
  FChartDorsal := CreateStyledChart(FPnlRight, 'Dorsal/Palmar', True, True, -30.0, 30.0);
  FSerDorTarget  := AddSeriesToChart(FChartDorsal, 'Target', clRed, 1);
  FSerDorRealize := AddSeriesToChart(FChartDorsal, 'Realize', clGreen, 1);

  // Row 3, Col 1: Transversal Phase Angle (-30..30 on both axes)
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

  // Row 3, Col 2: Minimal Parameter Panel (fits cleanly in Figure 2's bottom-right space)
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
