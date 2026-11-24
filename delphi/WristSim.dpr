program WristSim;

uses
  Vcl.Forms,
  uMainForm in 'uMainForm.pas' {FormMain};

{$R *.res}

begin
  Application.Initialize;
  Application.MainFormOnTaskbar := True;
  Application.CreateForm(TFormMain, FormMain);
  Application.Run;
end.
