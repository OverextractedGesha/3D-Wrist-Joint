program WristSim;

uses
  Vcl.Forms,
  uMainForm in 'uMainForm.pas' {FormMain},
  uWristPhysics in 'uWristPhysics.pas',
  uWristRender in 'uWristRender.pas';

{$R *.res}

begin
  Application.Initialize;
  Application.MainFormOnTaskbar := True;
  Application.CreateForm(TFormMain, FormMain);
  Application.Run;
end.
