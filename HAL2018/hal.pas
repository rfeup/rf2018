unit hal;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, LResources, Forms, Controls, Graphics, Dialogs,
  StdCtrls, ExtCtrls, IniPropStorage, structs;

type

  { TFHal }

  TFHal = class(TForm)
    BSetMission: TButton;
    EWGoToWarehouse: TEdit;
    EApproachSpeed2: TEdit;
    EApproachSpeed3: TEdit;
    EDefaultSpeed: TEdit;
    EApproachSpeed: TEdit;
    EWFollowArc: TEdit;
    EWRotateAndGo: TEdit;
    EDefaultSpeed3: TEdit;
    EWGoToXY: TEdit;
    EPathRadius2: TEdit;
    EPathRadius3: TEdit;
    EPeakTresh: TEdit;
    EMission1: TEdit;
    EPathRadius: TEdit;
    GBVariables: TGroupBox;
    IniPropStorage: TIniPropStorage;
    LGoToWarehouse: TLabel;
    LApproachSpeed2: TLabel;
    LApproachSpeed3: TLabel;
    LDefaultSpeed: TLabel;
    LApproachSpeed: TLabel;
    LWFollowArc: TLabel;
    LWRotateAndGo: TLabel;
    LDefaultSpeed4: TLabel;
    LPathRadius1: TLabel;
    LPathRadius2: TLabel;
    LPathRadius3: TLabel;
    LPeakTresh: TLabel;
    LPathRadius: TLabel;
    RGBoxColorRead: TRadioGroup;
    RGOperationMode: TRadioGroup;
    BStart: TButton;
    CBDebug: TCheckBox;
    EditMacFrom: TEdit;
    EditMacTo: TEdit;
    EActionDebug: TLabeledEdit;
    EditYDebug: TLabeledEdit;
    EditThDebug: TLabeledEdit;
    debugBoxColor: TLabeledEdit;
    GBWareMac: TGroupBox;
    From: TLabel;
    EditXDebug: TLabeledEdit;
    DebugMenu: TMemo;
    TimerToCheckColor: TTimer;
    TWaitAtStart: TTimer;
    Tolabel: TLabel;
    GBParts: TGroupBox;
    Part1: TEdit;
    Part2: TEdit;
    Part3: TEdit;
    Part4: TEdit;
    Part5: TEdit;
    RGMission: TRadioGroup;
    procedure BSetMissionClick(Sender: TObject);
    procedure BStartClick(Sender: TObject);

    procedure ChangePartColor(color_part:  array of TPart);

    procedure FormShow(Sender: TObject);


    function setcolor(PartType : TPartType; var partNumber: integer): TColor;
    procedure TimerToCheckColorTimer(Sender: TObject);
    procedure TWaitAtStartTimer(Sender: TObject);
  private
    { private declarations }
  public
    { public declarations }
  end;


var
  FHal: TFHal;


implementation

uses state, laserloc;

{------------------------------------------------------------------------------
       TFHal.SetColor
------------------------------------------------------------------------------}

function TFHal.setcolor(PartType : TPartType; var partNumber: integer): TColor;
var partcolor:TColor;
begin

    if PartType = R then begin
      partcolor:= clRed;

            DebugMenu.append('red');
            PartsScript[partNumber].PartType:= R;
            PartsScript[partNumber].ID:= partNumber;
            PartsScript[partNumber].Node:= BoxPlaces[partNumber];


      //end;

   end else if PartType = G then begin
      partcolor:= clGreen;
      //if ve = 1 then begin
      //      DebugMenu.append('green');
      //      PartsScript[totalParts].PartType:= G;
      //      PartsScript[totalParts].ID:= totalParts;
      //      PartsScript[totalParts].Node:= BoxPlaces[totalParts];
      //      ve := 25;
      //end
      //else if  ((ve = 2) and (partNumber = 5)) then begin
            DebugMenu.append('green');
            PartsScript[partNumber].PartType:= G;
            PartsScript[partNumber].ID:= partNumber;
            PartsScript[partNumber].Node:= BoxPlaces[partNumber];
      //      ve := 25;
      //end;
   end else if PartType = B then begin
      partcolor:= clBlue;
      //if ve = 1 then begin
            DebugMenu.append('blue');
            PartsScript[partNumber].PartType:= B;
            PartsScript[partNumber].ID:= partNumber;
            PartsScript[partNumber].Node:= BoxPlaces[partNumber];
            ve := 25;
      //end
      //else if  ((ve = 2) and (partNumber = 5)) then begin
      //      DebugMenu.append('blue');
      //      PartsScript[totalParts].PartType:= B;
      //      PartsScript[totalParts].ID:= totalParts;
      //      PartsScript[totalParts].Node:= BoxPlaces[totalParts];
      //      ve := 25;
      //end;
   end else begin
      partcolor:= clBlack;
   end;
   Result:=partcolor;
end;

procedure TFHal.TimerToCheckColorTimer(Sender: TObject);
begin
   Inc(ActionScriptCount);
    with ActionScript[ActionScriptCount] do begin
         atype := atWait;
     end;
    FHal.TimerToCheckColor.Enabled:=false;
end;



{------------------------------------------------------------------------------
       TFHal.ChangePartColor
------------------------------------------------------------------------------}

procedure TFHal.ChangePartColor(color_part: array of TPart);
var partN: integer;
begin
    partN:=1;
    ////color_part[1].PartType:=B;
    Part1.Color := setcolor(color_part[1].PartType, partN);
    partN:=2;
    ////color_part[2].PartType:=R;
    Part2.Color := setcolor(color_part[2].PartType, partN);
    partN:=3;
    ////color_part[3].PartType:=G;
    Part3.Color := setcolor(color_part[3].PartType, partN);
    partN:=4;
    ////color_part[4].PartType:=G;
    Part4.Color := setcolor(color_part[4].PartType, partN);
    partN:=5;
    ////color_part[5].PartType:=B;
    Part5.Color := setcolor(color_part[5].PartType, partN);
end;



{------------------------------------------------------------------------------
       TFHal.FormShow
------------------------------------------------------------------------------}

procedure TFHal.FormShow(Sender: TObject);
begin
   Part1.Color := clBlack;
   Part2.Color := clBlack;
   Part3.Color := clBlack;
   Part4.Color := clBlack;
   Part5.Color := clBlack;
end;











{------------------------------------------------------------------------------
       TFHal.BSetMissionClick
------------------------------------------------------------------------------}

procedure TFHal.BSetMissionClick(Sender: TObject);
var missao: integer;
begin

  Pose_loc.x := BoxPlaces[18].x;
  Pose_loc.y := BoxPlaces[18].y;
  Pose_loc.theta := BoxPlaces[18].teta;
  RobotState.servo1 := 0;
  RobotState.servo2 := 0;

  DebugMenu.Clear;
  ve := 0;
  rebuild := 0;

  case RGOperationMode.ItemIndex of
       0 : BuildStateMachine(false, false);
       1 : BuildStateMachine(true, false);
       2 : BuildStateMachine(false, true);
  end;

  ResetStateMachine();

   (*DebugMenu.Clear;
   DebugMenu.Append('red');
   DebugMenu.Append('blue');
   DebugMenu.Append('green');
   DebugMenu.Append('green');
   DebugMenu.Append('blue');*)



end;

{------------------------------------------------------------------------------
       TFHal.StartButtonClick
------------------------------------------------------------------------------}

procedure TFHal.BStartClick(Sender: TObject);
begin

  //while FLaserLoc.StartButton=false do begin
    // FLaserLoc.CBSendLock.Checked :=false;
  //end;

  //if FLaserLoc.StartButton=true then begin
  //   if RGMission.ItemIndex=0 then begin
  //      case RGOperationMode.ItemIndex of
  //           0 : BuildStateMachine(false, false);
  //           1 : BuildStateMachine(true, false);
  //           2 : BuildStateMachine(false, true);
  //      end;
  //
  //      ResetStateMachine();
  //      FLaserLoc.CBSendLock.Checked :=true;
  //   end else begin
  //       TWaitAtStart.Enabled:=true;
  //   end
  //end
end;



procedure TFHal.TWaitAtStartTimer(Sender: TObject);
begin
  //quando acaba timer faz reset à máquina de estados e envia comandos para motores
   case RGOperationMode.ItemIndex of
       0 : BuildStateMachine(false, false);
       1 : BuildStateMachine(true, false);
       2 : BuildStateMachine(false, true);
   end;

   ResetStateMachine();
   FLaserLoc.CBSendLock.Checked :=true;
   FHal.TWaitAtStart.Enabled:=false;


end;

  //end;



initialization
  {$I hal.lrs}

end.

