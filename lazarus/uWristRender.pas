unit uWristRender;

{$mode objfpc}{$H+}

interface

uses
  SysUtils, GL, GLU;

type
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

implementation

const
  MAT_SPECULAR: array[0..3] of GLfloat = (0.75, 0.75, 0.65, 1.0);
  MAT_DIFFUSE:  array[0..3] of GLfloat = (0.92, 0.90, 0.80, 1.0);
  MAT_AMBIENT:  array[0..3] of GLfloat = (0.32, 0.30, 0.26, 1.0);
  MAT_SHININESS: GLfloat = 36.0;
  LIGHT_POS:    array[0..3] of GLfloat = (10.0, 12.0, 14.0, 1.0);
  LIGHT_DIFF:   array[0..3] of GLfloat = (1.0, 0.98, 0.90, 1.0);
  LIGHT_AMB:    array[0..3] of GLfloat = (0.30, 0.30, 0.32, 1.0);

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

  // Indigo/slate-purple background matching BioMod Project-2627.pdf Figure 2
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

  // Shoulder joint sphere
  gluSphere(FQuadSphere, 0.72 * VisualScale, 28, 28);

  // Upper arm angled slightly forward/left like Figure 2
  glRotatef(18.0, 0.0, 0.0, -1.0);
  glRotatef(90.0, 1.0, 0.0, 0.0);
  gluCylinder(FQuadCylinder, 0.65 * VisualScale, 0.48 * VisualScale, UpperLen, 28, 8);

  // Move to elbow joint
  glTranslatef(0.0, 0.0, UpperLen);
  gluSphere(FQuadSphere, 0.50 * VisualScale, 28, 28);

  // Slight natural elbow bend (matching Figure 2 silhouette)
  glRotatef(22.0, 0.0, 1.0, 0.0);

  // Forearm tapered cylinder to wrist
  gluCylinder(FQuadCylinder, 0.48 * VisualScale, 0.34 * VisualScale, ForearmLen, 28, 8);

  // Move to wrist joint center
  glTranslatef(0.0, 0.0, ForearmLen);
  gluSphere(FQuadSphere, 0.36 * VisualScale, 28, 28);
end;

procedure TWristRenderer.DrawHand(VisualScale: Single);
var
  W, T, L: Single;
begin
  // Palm dimensions (width, thickness, length)
  W := 0.44 * VisualScale;
  T := 0.16 * VisualScale;
  L := 1.05 * VisualScale;

  glBegin(GL_QUADS);
    // Distal cap (knuckles)
    glNormal3f(0.0, 0.0, 1.0);
    glVertex3f(-W,  T, L); glVertex3f( W,  T, L);
    glVertex3f( W, -T, L); glVertex3f(-W, -T, L);

    // Proximal cap (wrist)
    glNormal3f(0.0, 0.0, -1.0);
    glVertex3f(-W,  T, 0.0); glVertex3f(-W, -T, 0.0);
    glVertex3f( W, -T, 0.0); glVertex3f( W,  T, 0.0);

    // Palmar face
    glNormal3f(0.0, 1.0, 0.0);
    glVertex3f(-W,  T, 0.0); glVertex3f( W,  T, 0.0);
    glVertex3f( W,  T, L);   glVertex3f(-W,  T, L);

    // Dorsal face
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

  // Four fingers at distal edge of palm
  glPushMatrix();
    glTranslatef(0.0, 0.0, 1.05 * VisualScale);
    DrawOneFinger(-Space * 1.65, 0.068 * VisualScale, 0.82, -4.0); // Pinky
    DrawOneFinger(-Space * 0.55, 0.074 * VisualScale, 0.96, -1.0); // Ring
    DrawOneFinger( Space * 0.55, 0.078 * VisualScale, 1.05,  1.0); // Middle
    DrawOneFinger( Space * 1.65, 0.075 * VisualScale, 0.95,  4.0); // Index
  glPopMatrix();

  // Thumb on radial edge
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

  // Position camera so full arm + wrist + hand are centered in left pane
  glTranslatef(0.3, 3.1, -FCamDist);
  glRotatef(FCamPitch, 1.0, 0.0, 0.0);
  glRotatef(FCamYaw,   0.0, 1.0, 0.0);

  glLightfv(GL_LIGHT0, GL_POSITION, @LIGHT_POS[0]);

  glPushMatrix();
    // Arm ends with local origin at wrist joint center, +Z pointing distally
    DrawUpperArmAndForearm(ScaleF);

    // Apply 2-DOF wrist joint rotations:
    // Radial/Ulnar flexion (Sagittal plane) + Dorsi/Palmar flexion (Frontal plane)
    glRotatef(Single(RadialDeg), 0.0, 1.0, 0.0);
    glRotatef(Single(DorsalDeg), 1.0, 0.0, 0.0);

    DrawHand(ScaleF);
    DrawFingers(ScaleF, SimTime);
  glPopMatrix();
end;

end.
