unit main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, TAGraph, TASeries, lNetComponents,
  SdpoSerial, SdpoJoystick, Forms, Controls, Graphics, Dialogs,
  StdCtrls, IniPropStorage, ExtCtrls, ComCtrls, Math,
  motor, lNet, RLan, lclintf;

type

  { TFMain }

  TFMain = class(TForm)
    BPIDSet: TButton;
    BSetServos: TButton;
    BWxSet: TButton;
    BVelsSet: TButton;
    BOpenComPort: TButton;
    BCloseComPort: TButton;
    BSerialSendRaw: TButton;
    BReset: TButton;
    BMxSet: TButton;
    BStop: TButton;
    BSetConfig: TButton;
    CBChartCurrentActive: TCheckBox;
    BMemoClear: TButton;
    CBRawDebug: TCheckBox;
    CBPID: TCheckBox;
    CBJoystick: TCheckBox;
    CBRemoteSpeeds: TCheckBox;
    CBStartButton: TCheckBox;
    CBMotorButton: TCheckBox;
    EditJoyX: TEdit;
    EditJoyDevice: TEdit;
    EditJoyB: TEdit;
    EditKf: TEdit;
    EditKp: TEdit;
    EditKi: TEdit;
    EditKd: TEdit;
    EditLidarMotorRPM: TEdit;
    EditControlDelta: TEdit;
    EditServoRightLock: TEdit;
    EditServoLeftLow: TEdit;
    EditServoRightLow: TEdit;
    EditServoLeftUp: TEdit;
    EditServoLeftLock: TEdit;
    EditServoRightUp: TEdit;
    EditW1: TEdit;
    EditW2: TEdit;
    EditW3: TEdit;
    EditTicksToRads: TEdit;
    EditWheelDiameter: TEdit;
    EditV: TEdit;
    EditVn: TEdit;
    EditW: TEdit;
    EditWheelDist: TEdit;
    Label10: TLabel;
    Label11: TLabel;
    Label12: TLabel;
    Label13: TLabel;
    Label14: TLabel;
    Label15: TLabel;
    Label16: TLabel;
    Label17: TLabel;
    Label18: TLabel;
    Label19: TLabel;
    Label20: TLabel;
    Label21: TLabel;
    Label22: TLabel;
    Label23: TLabel;
    Label24: TLabel;
    Label25: TLabel;
    Label26: TLabel;
    Label27: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    Label8: TLabel;
    Label9: TLabel;
    RGControlType: TRadioGroup;
    TabNet: TTabSheet;
    UDPVel: TLUDPComponent;
    RGServosPosition: TRadioGroup;
    SBW1: TScrollBar;
    SBW2: TScrollBar;
    SBW3: TScrollBar;
    SBV: TScrollBar;
    SBVn: TScrollBar;
    SBW: TScrollBar;
    Joystick: TSdpoJoystick;
    SeriesSpeedM2: TLineSeries;
    SeriesSpeedM3: TLineSeries;
    SeriesCurrentM2: TLineSeries;
    SeriesCurrentM3: TLineSeries;
    ChartSpeeds: TChart;
    CBXY: TCheckBox;
    ChartCurrents: TChart;
    CBChartSpeedActive: TCheckBox;
    EditM1: TEdit;
    EditDebug: TEdit;
    EditLidarMotorVoltage: TEdit;
    EditM2: TEdit;
    EditM3: TEdit;
    Label2: TLabel;
    Label3: TLabel;
    Label4: TLabel;
    PageControl: TPageControl;
    SBM1: TScrollBar;
    SBM2: TScrollBar;
    SBM3: TScrollBar;
    SeriesCurrentM1: TLineSeries;
    SeriesSpeedM1: TLineSeries;
    EditRawData: TEdit;
    MemoDebug: TMemo;
    Serial: TSdpoSerial;
    EditComPort: TEdit;
    IniPropStorage: TIniPropStorage;
    Label1: TLabel;
    TabCurrent: TTabSheet;
    TabDebug: TTabSheet;
    TabConfig: TTabSheet;
    TabServos: TTabSheet;
    TabJoystick: TTabSheet;
    TabSpeed: TTabSheet;
    UDPOdo: TLUDPComponent;
    procedure BCloseComPortClick(Sender: TObject);
    procedure BMemoClearClick(Sender: TObject);
    procedure BMxSetClick(Sender: TObject);
    procedure BOpenComPortClick(Sender: TObject);
    procedure BPIDSetClick(Sender: TObject);
    procedure BResetClick(Sender: TObject);
    procedure BSerialSendRawClick(Sender: TObject);
    procedure BSetConfigClick(Sender: TObject);
    procedure BSetServosClick(Sender: TObject);
    procedure BStopClick(Sender: TObject);
    procedure BVelsSetClick(Sender: TObject);
    procedure BWxSetClick(Sender: TObject);
    procedure CBJoystickClick(Sender: TObject);
    procedure CBXYChange(Sender: TObject);
    procedure EditControlDeltaChange(Sender: TObject);
    procedure EditMxKeyDown(Sender: TObject; var Key: Word; Shift: TShiftState);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure UDPVelReceive(aSocket: TLSocket);
    procedure RGServosPositionClick(Sender: TObject);
    procedure SBMxChange(Sender: TObject);
    procedure SBVVnWChange(Sender: TObject);
    procedure SBWxChange(Sender: TObject);
    procedure SerialRxData(Sender: TObject);
    procedure EditVelsKeyDown(Sender: TObject; var Key: Word; Shift: TShiftState
      );
    procedure EditWxKeyDown(Sender: TObject; var Key: Word; Shift: TShiftState);
  private
    procedure processFrame(channel: char; value: integer);
    procedure ReceiveData(s: string);
    procedure SendChannel(channel: char; value: integer);
    procedure SendChannel(channel: char; value: single);
    procedure SendMotorPWM(MotorIdx: byte; PWM: integer);
    procedure SendMotorWRef(MotorIdx: byte; WRef: integer);
    procedure SendOdo;
    procedure SetComState(newState: boolean);
    procedure SetServosPosition;
    procedure VVnWToWheelSpeeds;
    { private declarations }
  public
    serialData: string;

    channel: char;
    nfile,frame: integer;
    frameData: string;

    dt: double;
    TicsToRads, WheelDiameter, WheelDist: double;

    Set_PWM_M: array [1..3] of integer;
    PWM_M: array [1..3] of integer;
    Current:  array [1..3] of integer;
    Odo: array [1..3] of integer;
    Wr: array [1..3] of double;
    WrRef: array [1..3] of double;

    actOdoM: array [1..3] of integer;
    PID: array [1..3] of TPID;
    V, Vn, W: double;
    maxWr: double;
    Vbat: double;

    ServoLeftLow, ServoLeftUp , ServoLeftLock: integer;
    ServoRightLow, ServoRightUp , ServoRightLock: integer;
    ServoLeftPosition, ServoRightPosition: integer;

    RemoteV, RemoteVn, RemoteW: double;
    RemoteTime: qword;

    sample: integer;
    start_robot, motor_button: boolean;
    LIDARMotorVoltageAD: integer;
    LIDARMotorRPM: integer;

    sline: string;
    procedure Debug(s: string);
  end;

var
  FMain: TFMain;
  NetInBuf: TUDPBuffer;
  NetOutBuf: TUDPBuffer;
  Servo_pos:Integer;


procedure SendMessage(c: char; val: integer);
procedure SendRaw(s: string);


implementation


{$R *.lfm}

procedure SendMessage(c: char; val: integer);
begin
  FMain.Serial.WriteData(c + IntToHex(word(Val) and $FFFF, 4));
end;

procedure SendRaw(s: string);
begin
  FMain.Serial.WriteData(s);
end;



{ TFMain }

procedure TFMain.SetComState(newState: boolean);
var comColor: TColor;
begin
  try
  if newState then begin
    Serial.Device := EditComPort.Text;
    Serial.Open;
  end else begin
    Serial.Close;
  end;
  finally
    if Serial.Active then comColor := clGreen
    else comColor := clRed;
    EditComPort.Color := comColor;
  end;
end;



procedure TFMain.Debug(s: string);
begin
  MemoDebug.Lines.Add(s);
  while MemoDebug.Lines.Count > 1000 do begin
    MemoDebug.Lines.Delete(0);
  end;
end;


procedure TFMain.SendChannel(channel: char; value: integer);
begin
  if Serial.Active then begin
    Serial.WriteData(channel + IntToHex(value, 4));
  end;
end;


procedure TFMain.SendChannel(channel: char; value: single);
begin
  if Serial.Active then begin
    Serial.WriteData(channel + IntToHex(PInteger(@value)^, 4));
  end;
end;



procedure TFMain.BOpenComPortClick(Sender: TObject);
begin
  SetComState(true);
end;

procedure TFMain.BPIDSetClick(Sender: TObject);
var Ki, Kd, Kp, Kf: single;
begin
  Kf := strtofloat(EditKf.Text);
  Kp := strtofloat(EditKp.Text);
  Ki := strtofloat(EditKi.Text);
  Kd := strtofloat(EditKd.Text);

  if CBPID.Checked then begin
    SendChannel('h', Kp);
    SendChannel('i', Ki);
    SendChannel('j', Kd);
    SendChannel('k', Kf);
  end else begin
    Serial.WriteData('   ');

    SendChannel('l', Kp);
    SendChannel('m', Ki);
    SendChannel('n', Kd);
    SendChannel('o', Kf);
  end;
end;


procedure TFMain.BResetClick(Sender: TObject);
begin
  if Serial.Active then begin
    Serial.SetDTR(false);
    Serial.SetDTR(true);
  end;
end;

procedure TFMain.BSerialSendRawClick(Sender: TObject);
begin
  Serial.WriteData(EditRawData.Text);
end;

procedure TFMain.BMemoClearClick(Sender: TObject);
begin
  MemoDebug.Clear;
  sample := 0;
end;

procedure TFMain.BSetConfigClick(Sender: TObject);
begin
  TicsToRads := strtofloat(EditTicksToRads.Text);
  WheelDiameter := strtofloat(EditWheelDiameter.Text);
  WheelDist := strtofloat(EditWheelDist.Text);
end;

procedure TFMain.SetServosPosition;
begin
  //case RGServosPosition.ItemIndex of
  case Servo_pos of
    0: begin
      ServoLeftPosition := ServoLeftLow;
      ServoRightPosition:= ServoRightLow;
    end;
    1: begin
      ServoLeftPosition := ServoLeftUp;
      ServoRightPosition:= ServoRightUP;
    end;
    2: begin
      ServoLeftPosition := ServoLeftLock;
      ServoRightPosition:= ServoRightLock;
    end;
  end;

end;


procedure TFMain.BSetServosClick(Sender: TObject);
begin
  ServoLeftLow := StrToInt(EditServoLeftLow.Text);
  ServoLeftUp := StrToInt(EditServoLeftUp.Text);
  ServoLeftLock := StrToInt(EditServoLeftLock.Text);

  ServoRightLow := StrToInt(EditServoRightLow.Text);
  ServoRightUp := StrToInt(EditServoRightUp.Text);
  ServoRightLock := StrToInt(EditServoRightLock.Text);
  Servo_pos:=RGServosPosition.ItemIndex;
  SetServosPosition();
end;

procedure TFMain.BStopClick(Sender: TObject);
begin
  SBM1.Position := 0;
  SBM2.Position := 0;
  SBM3.Position := 0;

  SBW1.Position := 0;
  SBW2.Position := 0;
  SBW3.Position := 0;

  SBV.Position := 0;
  SBVn.Position := 0;
  SBW.Position := 0;
end;

procedure TFMain.BMxSetClick(Sender: TObject);
begin
  SBM1.Position := StrToIntDef(EditM1.Text, 0);
  SBM2.Position := StrToIntDef(EditM2.Text, 0);
  SBM3.Position := StrToIntDef(EditM3.Text, 0);
end;


procedure TFMain.BVelsSetClick(Sender: TObject);
begin
  SBV.Position := round(1000 * StrToFloatDef(EditV.Text, 0));
  SBVn.Position := round(1000 * StrToFloatDef(EditVn.Text, 0));
  SBW.Position := round(1000 * StrToFloatDef(EditW.Text, 0));
end;

procedure TFMain.BWxSetClick(Sender: TObject);
begin
  SBW1.Position := round(1000 * StrToFloatDef(EditW1.Text, 0) / maxWr);
  SBW2.Position := round(1000 * StrToFloatDef(EditW2.Text, 0) / maxWr);
  SBW3.Position := round(1000 * StrToFloatDef(EditW3.Text, 0) / maxWr);
end;

procedure TFMain.CBJoystickClick(Sender: TObject);
begin
  if CBJoystick.Checked then begin
    Joystick.DeviceLin := EditJoyDevice.Text;
    Joystick.Active := true;
  end else begin
    Joystick.Active := false;
  end;
end;

procedure TFMain.CBXYChange(Sender: TObject);
begin
  SeriesCurrentM1.Clear;
  SeriesSpeedM1.Clear
end;

procedure TFMain.EditControlDeltaChange(Sender: TObject);
begin

end;



procedure TFMain.EditMxKeyDown(Sender: TObject; var Key: Word;
  Shift: TShiftState);
begin
  if key = 13 then BMxSet.Click;
end;

procedure TFMain.EditVelsKeyDown(Sender: TObject; var Key: Word; Shift: TShiftState
  );
begin
  if key = 13 then BVelsSet.Click;
end;

procedure TFMain.EditWxKeyDown(Sender: TObject; var Key: Word;
  Shift: TShiftState);
begin
  if key = 13 then BWxSet.Click;
end;

procedure TFMain.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  IniPropStorage.WriteBoolean('COM_port_state', Serial.Active);
  Serial.Active := false;
end;

procedure TFMain.FormCreate(Sender: TObject);
begin
  sample := 0;
  dt := 0.04;
  maxWr := 40;
  Vbat := 12;
  UDPVel.Connect('127.0.0.1',9006);
  UDPVel.Listen(9006);
  UDPOdo.Connect('127.0.0.1',9001);
  //EditM1.tag := ptrint(SBM1);
end;

procedure TFMain.FormShow(Sender: TObject);
var openSerial: boolean;
begin
  BSetConfig.Click;
  BPIDSet.Click;
  BSetServos.Click;
  openSerial := IniPropStorage.ReadBoolean('COM_port_state', true);
  SetComState(openSerial);
  BMemoClear.Click;
end;

procedure TFMain.UDPVelReceive(aSocket: TLSocket);
var ii,NumberBytes: integer;
    dd: word;
    Messa: String;
begin
    UDPVel.GetMessage(Messa);
    if messa = '' then begin
      RemoteV := 0;
      RemoteVN:= 0;
      RemoteW := 0;
      Servo_pos:= 0;
      exit;
    end;
    RemoteTime := GetTickCount64();

    ClearUDPBuffer(NetInBuf);
    NumberBytes := length(Messa);
    if NumberBytes >= UDPBufSize then
      exit;
    NetInBuf.MessSize := NumberBytes;
    NetInBuf.ReadDisp := 0;
    move(Messa[1], NetInBuf.data[0], NumberBytes);
    if chr(NetGetByte(NetInBuf)) <> 'V' then
       exit;
    if chr(NetGetByte(NetInBuf)) <> 'N' then
       exit;
    if chr(NetGetByte(NetInBuf)) <> 'W' then
       exit;
    if chr(NetGetByte(NetInBuf)) <> 'S' then
       exit;

    RemoteV :=netgetint(NetInBuf)/1000;
    RemoteVN:=netgetint(NetInBuf)/1000;
    RemoteW :=netgetint(NetInBuf)/1000;
    MemoDebug.lines.add(floattostr(RemoteV)) ;
     MemoDebug.lines.add(floattostr(RemoteVn)) ;
      MemoDebug.lines.add(floattostr(RemoteW)) ;
    Servo_pos := round(netgetint(NetInBuf)/1000);
    SetServosPosition;
   // UDPVel.SendMessage('V:'+#10+FloatToStr(RemoteV)+#10+'Vn:'+#10+FloatToStr(RemoteVn)+#10+'W:'+#10+FloatToStr(RemoteW)+#10,'127.0.0.1:9808');
end;



procedure TFMain.RGServosPositionClick(Sender: TObject);
begin
  SetServosPosition();
end;



procedure TFMain.SBMxChange(Sender: TObject);
var SBMx: TScrollBar;
    EditMx: TEdit;
begin
  SBMx := Sender as TScrollBar;
  Set_PWM_M[1 + SBMx.tag] := SBMx.Position;

  SendMotorPWM(1 + SBMx.tag, Set_PWM_M[1 + SBMx.tag]);
  EditMx := TEdit(FindChildControl('EditM' + inttostr(SBMx.tag + 1)));
  if Assigned(EditMx) then
    EditMx.Text := IntToStr(Set_PWM_M[1 + SBMx.tag]);
end;


procedure TFMain.VVnWToWheelSpeeds();
begin
  WrRef[1] := (-sqrt(3)/2 * V + 0.5 * Vn + WheelDist * W) / (0.5 * WheelDiameter);
  WrRef[2] := (                      -Vn + WheelDist * W) / (0.5 * WheelDiameter);
  WrRef[3] := ( sqrt(3)/2 * V + 0.5 * Vn + WheelDist * W) / (0.5 * WheelDiameter);
end;


procedure TFMain.SBVVnWChange(Sender: TObject);
var i: integer;
begin
  V := SBV.Position / 1000;
  EditV.Text := format('%g', [V]);
  Vn := SBVn.Position / 1000;
  EditVn.Text := format('%g', [Vn]);
  W := SBW.Position / 1000;
  Editw.Text := format('%g', [W]);

  VVnWToWheelSpeeds();

  SBW1.Position := round(WrRef[1] * 1000 / maxWr);
  SBW2.Position := round(WrRef[2] * 1000 / maxWr);
  SBW3.Position := round(WrRef[3] * 1000 / maxWr);
end;



procedure TFMain.SBWxChange(Sender: TObject);
var SBWx: TScrollBar;
    EditWx: TEdit;
begin
  SBWx := Sender as TScrollBar;
  WrRef[1 + SBWx.tag] := maxWr * SBWx.Position / 1000;

  EditWx := TEdit(FindChildControl('EditW' + inttostr(SBWx.tag + 1)));
  if Assigned(EditWx) then
    EditWx.Text := format('%.3g',[WrREf[1 + SBWx.tag]]) ;
end;


// PWM between -255 and 255
procedure TFMain.SendMotorPWM(MotorIdx: byte; PWM: integer);
var data: Longword;
begin
  data := MotorIdx;
  data := data shl 24;

  data := data or (PWM and $FFFF);
  Serial.WriteData('M' + IntToHex(data, 8));
end;


procedure TFMain.SendMotorWRef(MotorIdx: byte; WRef: integer);
var data: Longword;
begin
  data := MotorIdx;
  data := data shl 24;

  data := data or (WRef and $FFFF);
  Serial.WriteData('r' + IntToHex(data, 8));
end;


function isHexDigit(c: char): boolean;
begin
  result := c in ['0'..'9', 'A'..'F'];
end;

procedure TFMain.SendOdo();
var ld:word;
begin

    ClearUDPBuffer(NetOutBuf);
     //
    NetPutByte(NetOutBuf, ord('V'));
    NetPutByte(NetOutBuf, ord('W'));
    NetPutByte(NetOutBuf, ord('3'));

    ld := round(Wr[3]*1000);
    NetPutShort(NetOutBuf, ld);

    ld := round(Wr[1]*1000);
    NetPutShort(NetOutBuf, ld);

    ld := round(Wr[2]*1000);
    NetPutShort(NetOutBuf, ld);

    NetPutWord(NetOutBuf, ord(start_robot));

    UDPOdo.Send(NetOutBuf.data, NetOutBuf.MessSize, '127.0.0.1' + ':9001');
end;

procedure TFMain.processFrame(channel: char; value: integer);
var s: string;
    i: integer;
    SeriesCurrentMx, SeriesSpeedMx: TLineSeries;
    actTime: QWord;
begin
  //MemoDebug.Text := MemoDebug.Text + channel;
  if channel = 'p' then begin // Metapacket delimiter
    if value = 0 then begin   // End of metapacket

      if CBJoystick.Checked then begin
        EditJoyX.text := format('%d, %d, %d', [Joystick.Axis[0], Joystick.Axis[1], Joystick.Axis[2]]);
        V := -0.5 * (Joystick.Axis[5] / 32768); //1
        Vn := -0.5 * (Joystick.Axis[2] / 32768);   //0
        W := -5 * (Joystick.Axis[0] / 32768);    //2

        SBV.Position := round(1000 * V);
        SBVn.Position := round(1000 * Vn);
        SBW.Position := round(1000 * W);

        EditJoyB.text := format('%d, %d, %d', [Joystick.Buttons[0], Joystick.Buttons[1], Joystick.Buttons[2]]);
        if Joystick.Buttons[0] > 0 then
          RGServosPosition.ItemIndex := 0;
        if Joystick.Buttons[1] > 0 then
          RGServosPosition.ItemIndex := 1;
      end;

      if CBRemoteSpeeds.Checked then begin
        actTime := GetTickCount64();

        if actTime - RemoteTime > 300 then begin
          RemoteV := 0;
          RemoteVN:= 0;
          RemoteW := 0;
          Servo_pos:= 0;
        end;

        SBV.Position := round(1000 * RemoteV);
        SBVn.Position := round(1000 * RemoteVn);
        SBW.Position := round(1000 * RemoteW);

        RGServosPosition.ItemIndex := Servo_pos;

      end;

      if RGControlType.ItemIndex = 0 then begin
        Set_PWM_M[1] := SBM1.Position;
        Set_PWM_M[2] := SBM2.Position;
        Set_PWM_M[3] := SBM3.Position;

        for i := 1 to 3 do begin
          SendMotorPWM(i, Set_PWM_M[i]);
        end;

      end else if RGControlType.ItemIndex = 1 then begin
        for i := 1 to 3 do begin
          SendMotorWRef(i, round(dt * WrRef[i] / TicsToRads));
        end;
        //EditDebug.Text:= IntToStr(round(dt * WrRef[1] / TicsToRads));

        //Serial.WriteData('G' + IntToHex(ServoLeftPosition, 4));
        //Serial.WriteData('H' + IntToHex(ServoRightPosition, 4));
        //Serial.WriteData('S' + IntToHex((ServoLeftPosition shl 16) or ServoRightPosition, 8));
      end;
      Serial.WriteData('S' + IntToHex((ServoLeftPosition shl 16) or ServoRightPosition, 8));

      sline := '';
      for i := 1 to 3 do begin
        // Show Speed
        if CBChartSpeedActive.Checked then begin
          SeriesSpeedMx := TLineSeries(ChartSpeeds.Series[i - 1]);
          if CBXY.Checked then begin
            SeriesSpeedMx.AddXY(PWM_M[i], Wr[i]);
          end else begin
            SeriesSpeedMx.Add(Wr[i]);
          end;
          while SeriesSpeedMx.Count > 10 * 50 do begin
            SeriesSpeedMx.Delete(0);
          end;
        end;
      end;

      CBStartButton.Checked := start_robot;
      CBMotorButton.Checked := motor_button;
      inc(sample);

    end else begin
      start_robot := ((value shr 8) and $02) <> 0;
      motor_button:= ((value shr 8) and $01) <> 0;
    end;
  end else if channel = 'k' then begin
    EditControlDelta.Text := IntToStr(value);
  end else if channel in ['m', 'n', 'o'] then begin
    i := 1 + ord(channel) - ord('m');
    PWM_M[i] := value;
  end else if channel in ['v'] then begin
    LIDARMotorVoltageAD := value and $FFFF;
    LIDARMotorRPM := (value shr 16) and $FFFF;
    //EditLidarMotorVoltage.Text := IntToStr(LIDARMotorVoltageAD / 4096 * 3300);  // 2.88 -> 3600
    EditLidarMotorVoltage.Text := format('%.2f', [2 * LIDARMotorVoltageAD / 4096 * 3.3]);
    EditLidarMotorRPM.Text := format('%d', [LIDARMotorRPM]);

  end else if channel in ['r', 's', 't'] then begin
    i := 1 + ord(channel) - ord('r');
    if sample <= 10 then actOdoM[i] := value; // First sample must be discarded

    EditDebug.Text := IntToStr(sample);

    Odo[i] := value - actOdoM[i];
    Wr[i] := Odo[i] * TicsToRads / dt;
    actOdoM[i] := value;
    if channel = 't' then SendOdo();

  end;
end;


// Sample Pwm I speed

procedure TFMain.ReceiveData(s: string);
var //b: byte;
    c: char;
    value: integer;
begin
  if s = '' then exit;
  serialData := serialData + s;

  if CBRawDebug.Checked then begin
    Debug(s);
  end;

  while Length(serialData) > 0 do begin
    c := serialData[1];
    serialData := copy(serialData, 2, maxint);
    if frame = -1 then begin
      if (c in ['G'..']']) or (c in ['g'..'}']) or (c = ';') then begin
        frame := 0;
        channel := c;
        frameData := '';
      end;
    end else begin
      if isHexDigit(c) then begin
        frameData := frameData + c;
        inc(frame);
        if frame = 8 then begin
          value := StrToIntDef('$' + frameData, -1);
          processFrame(channel, value);
          frame := -1;
        end;
      end else begin
        frame := -1;
      end;
    end;
  end;
end;


procedure TFMain.SerialRxData(Sender: TObject);
var s: string;
begin
  s := Serial.ReadData;
  ReceiveData(s);
end;

procedure TFMain.BCloseComPortClick(Sender: TObject);
begin
  SetComState(false);
end;


end.

