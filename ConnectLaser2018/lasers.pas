unit lasers;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, LResources, Forms, Controls, Graphics, Dialogs,
  StdCtrls, ExtCtrls, ComCtrls, IniPropStorage, Buttons, SdpoSerial, TAGraph,
  TASeries, SynEdit, lNetComponents, math, Rlan, lclintf, FitData, dynmatrix;

const
  MaxHokuyoScans = 1024;

  MaxNeatoScans = 360;
  MaxNeatoPackets = 90;

type
  TLaserPoint = record
    d, ang: single;
    x, y: single;
    seg, bad: integer;
  end;

 TLaserPoints = array of TLaserPoint;

 TSinCos = record
   sin_ang, cos_ang: single;
 end;

 TSinCosTable = array of TSinCos;


 TLocConfig = record
   Xmin, Xmax, Ymin, Ymax: double;
   ExpectedRadius, RadiusTolerance: double;
   AngularResolution: double;
 end;

  { TLaserScan }

  TLaserScan = class
  private
    SinCosTable: TSinCosTable;
    SinCosTableDone: boolean;
  public
    status: integer;
    timestamp: dword;
    start, stop, cluster: integer;
    Rays: TLaserPoints;
    count: integer;

    constructor Create(numRays: integer);
    destructor Destroy; override;
    procedure CalcSinCosTable;
  end;

  TNeatoRay = record
    index, speed: integer;
    dist, strength : integer;
    invalid_data, strength_warning: boolean;
  end;

 TNeatoRays = array[0..MaxNeatoScans-1] of TNeatoRay;


  { TFLasers }

  TFLasers = class(TForm)
    BCloseComNeato: TButton;
    BLaserOn: TButton;
    BOpenComNeato: TButton;
    BNeatoSetPars: TButton;
    BScan: TButton;
    BStartLocNet: TButton;
    BStopLocNet: TButton;
    BTestFitData: TButton;
    BSetFitData: TButton;
    CBDrawRadial: TCheckBox;
    CBShowNeatoSpeed: TCheckBox;
    Chart: TChart;
    CBTimer: TCheckBox;
    CBLocActive: TCheckBox;
    CBDrawLines: TCheckBox;
    EditNeatoA0: TEdit;
    EditExpectedRadius: TEdit;
    EditAngularResolution: TEdit;
    EditNeatoA1: TEdit;
    EditNeatoA2: TEdit;
    EditNeatoRayIndex: TEdit;
    EditNeatoRayValue: TEdit;
    EditRadiusTolerance: TEdit;
    EditLocToIP: TEdit;
    EditYmax: TEdit;
    EditXmin: TEdit;
    EditSpeed: TEdit;
    EditComPortNeato: TEdit;
    EditXmax: TEdit;
    EditYmin: TEdit;
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
    UDPLoc: TLUDPComponent;
    Memo: TMemo;
    MemoFitData: TMemo;
    PageControl: TPageControl;
    PaintBox: TPaintBox;
    Serial: TSdpoSerial;
    SerialNeato: TSdpoSerial;
    TabChart: TTabSheet;
    TabRadial: TTabSheet;
    TabFitData: TTabSheet;
    TabSheetNeato: TTabSheet;
    Timer: TTimer;
    BVersion: TButton;
    BReset: TButton;
    CB3ByteData: TCheckBox;
    BLaserOff: TButton;
    BContionousScan: TButton;
    CBShowRawData: TCheckBox;
    CBShowProcData: TCheckBox;
    BTime: TButton;
    BAdjustTimeOff: TButton;
    BAdjustTimeOn: TButton;
    Label1: TLabel;
    EditComPort: TEdit;
    BOpenCom: TButton;
    BCloseCom: TButton;
    IniPropStorage: TIniPropStorage;
    BHihSensitivityOn: TButton;
    BHihSensitivityOff: TButton;
    Label2: TLabel;
    BSpecs: TButton;
    BStatus: TButton;
    ComboBoxSpeed: TComboBox;
    Label3: TLabel;
    BSetSpeed: TButton;
    CBAutoStart: TCheckBox;
    PageControlConfig: TPageControl;
    TabSheetHokuyo: TTabSheet;
    TabSheetDebug: TTabSheet;
    TabSheetSegments: TTabSheet;
    Label4: TLabel;
    EditSegMaxPointDist: TEdit;
    BSegSetValues: TButton;
    EditSegSize: TEdit;
    Label5: TLabel;
    EditSegSizeTol: TEdit;
    Label6: TLabel;
    CBSegFilter: TCheckBox;
    CSLaserData: TLineSeries;
    EditException: TEdit;
    CBShowSize: TCheckBox;
    Label7: TLabel;
    EditTotalSize: TEdit;
    BRecord: TButton;
    BStop: TButton;
    BPlay: TButton;
    BPause: TButton;
    EditSourceFileName: TEdit;
    Label8: TLabel;
    Label9: TLabel;
    EditDestFileName: TEdit;
    TimerFileData: TTimer;
    TabFiles: TTabSheet;
    ImageRec: TImage;
    ApplicationProperties: TApplicationProperties;
    ImageRecDisable: TImage;
    ImagePlay: TImage;
    ImagePlayDisable: TImage;
    ImagePause: TImage;
    UDP: TLUDPComponent;
    TabNet: TTabSheet;
    Label10: TLabel;
    EditSendToIP: TEdit;
    BStartNet: TButton;
    BStopNet: TButton;
    procedure BCloseComNeatoClick(Sender: TObject);
    procedure BLaserOnClick(Sender: TObject);
    procedure BNeatoSetParsClick(Sender: TObject);
    procedure BOpenComNeatoClick(Sender: TObject);
    procedure BScanClick(Sender: TObject);
    procedure BSetFitDataClick(Sender: TObject);
    procedure BStartLocNetClick(Sender: TObject);
    procedure BStopLocNetClick(Sender: TObject);
    procedure BTestFitDataClick(Sender: TObject);
    procedure CBTimerChange(Sender: TObject);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure SerialNeatoRxData(Sender: TObject);
    procedure SerialRxData(Sender: TObject);
    procedure TimerTimer(Sender: TObject);
    procedure BVersionClick(Sender: TObject);
    procedure BResetClick(Sender: TObject);
    procedure BLaserOffClick(Sender: TObject);
    procedure BContionousScanClick(Sender: TObject);
    procedure BTimeClick(Sender: TObject);
    procedure BAdjustTimeOnClick(Sender: TObject);
    procedure BAdjustTimeOffClick(Sender: TObject);
    procedure BOpenComClick(Sender: TObject);
    procedure BCloseComClick(Sender: TObject);
    procedure BHihSensitivityOnClick(Sender: TObject);
    procedure BHihSensitivityOffClick(Sender: TObject);
    procedure BSpecsClick(Sender: TObject);
    procedure BStatusClick(Sender: TObject);
    procedure BSetSpeedClick(Sender: TObject);
    procedure PaintBoxMouseDown(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Integer);
    procedure PaintBoxMouseUp(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Integer);
    procedure PaintBoxMouseMove(Sender: TObject; Shift: TShiftState; X, Y: Integer);
    procedure PaintBoxMouseWheel(Sender: TObject; Shift: TShiftState; WheelDelta: Integer; MousePos: TPoint;
      var Handled: Boolean);
    procedure BSegSetValuesClick(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure BRecordClick(Sender: TObject);
    procedure BStopClick(Sender: TObject);
    procedure BPlayClick(Sender: TObject);
    procedure BPauseClick(Sender: TObject);
    procedure TimerFileDataTimer(Sender: TObject);
    procedure ApplicationPropertiesIdle(Sender: TObject; var Done: Boolean);
    procedure ImageRecDisableClick(Sender: TObject);
    procedure ImageRecClick(Sender: TObject);
    procedure ImagePlayDisableClick(Sender: TObject);
    procedure ImagePlayClick(Sender: TObject);
    procedure ImagePauseClick(Sender: TObject);
    procedure BStartNetClick(Sender: TObject);
    procedure BStopNetClick(Sender: TObject);
  private
    procedure ActivateComPortNeato(open: boolean);
    procedure LocateCircle(var LaserScan: TLaserScan);
    procedure processLaserMessage(str: string);
    procedure processLaserScan(slist: TStringList; var LaserScan: TLaserScan);
    procedure DrawLaserScan(LaserScan: TLaserScan);
    procedure ProcessTimeRequest(slist: TStringList);
    procedure ActivateComPort(open: boolean);
    procedure SendLaserMessage(mtype: integer);
    procedure SendLaserNeatoMessage(mtype: integer);
    procedure SendLocMessage;
    { private declarations }
  public
    qpfc: int64;
    //CSLaserData: TLineSeries;
    SerialData, SerialDataNeato: string;
    NeatoRays: TNeatoRays;
    xc, yc: integer;
    mc: double;
    mouse_down_x, mouse_down_y: integer;
    mouse_down: boolean;

    SegMaxPointDist2, SegSize, SegSizeTol: double;
    TotalSize, TotalSize2: double;

    XYReList: TDMatrix;
    XYReListCount, bestXYReListIdx: integer;

    SourceStream, DestStream: TMemoryStream;
    SourceFrame, DestFrame: integer;
    LastFrame: string;
    paused: boolean;

    NetOutBuf: TUDPBuffer;
    NeatoScan, lastNeatoScan: DWORD;
    NeatoPacketCount, SerialPacketCount: integer;
    a2,a1,a0 : double;
  end;

var
  FLasers: TFLasers;
  LaserScanG, LaserScanNeato: TLaserScan;
  //SinCosTable: TSinCosTable;  // Precalc all sin and cos
  LocConfig: TLocConfig;


implementation

//uses windows;

{ TLaserScan }

constructor TLaserScan.Create(numRays: integer);
begin
  count := numRays;
  start := 0;
  stop := count - 1;
  SetLength(Rays, count);
  SetLength(SinCosTable, count);
end;

destructor TLaserScan.Destroy;
begin
  inherited Destroy;
end;

{ TFLasers }

procedure TFLasers.ActivateComPort(open: boolean);
begin
  Serial.Device := EditComPort.Text;
  if open then begin
    SerialData := '';
    Serial.Open;
  end else begin
    Serial.close;
  end;

  if Serial.Active then begin
    EditComPort.Color := clgreen;
  end else begin
    EditComPort.Color := clSkyBlue;
  end;

end;

procedure TFLasers.ActivateComPortNeato(open: boolean);
begin
  SerialNeato.Device := EditComPortNeato.Text;
  if open then begin
    SerialDataNeato := '';
    SerialNeato.Open;
  end else begin
    SerialNeato.close;
  end;

  if SerialNeato.Active then begin
    EditComPortNeato.Color := clgreen;
  end else begin
    EditComPortNeato.Color := clSkyBlue;
  end;

end;



procedure TFLasers.FormCreate(Sender: TObject);
begin
  LaserScanG := TLaserScan.Create(MaxHokuyoScans);
  LaserScanG.start := 44;
  LaserScanG.stop := 725;
  LaserScanNeato:= TLaserScan.Create(MaxNeatoScans);
  NeatoPacketCount := 0;
  SerialPacketCount := 0;

  XYReList := Mzeros(256, 5);
  XYReListCount := 0;


  xc := PaintBox.Width div 2;
  yc := PaintBox.Height div 2;
  mc := 50;
  Caption := caption + ' - ' + datetimetostr(filedatetodatetime(fileage(application.ExeName)));
  SerialData := '';
  SerialDataNeato := '';

  SourceStream := TMemoryStream.Create;
  DestStream := TMemoryStream.Create;

  //memo.Lines.add(inttostr(sizeof(TLaserScan)));
  //CSLaserData := TLineSeries.Create(Chart);
  //Chart.AddSeries(CSLaserData);
end;

procedure TFLasers.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  IniPropStorage.WriteBoolean('LocNetActive', UDPLoc.Connected);
  IniPropStorage.WriteBoolean('NetActive', UDP.Connected);
  IniPropStorage.WriteBoolean('ComPortOpen', Serial.Active);
  IniPropStorage.WriteBoolean('ComPortNeatoOpen', SerialNeato.Active);
  Serial.WriteData('QT' + #$0A);
  Serial.Close;
  SerialNeato.Close;
  IniPropStorage.WriteInteger('Xc', xc);
  IniPropStorage.WriteInteger('Yc', yc);
  IniPropStorage.WriteInteger('Mag', round(mc * 1000));
end;

procedure TFLasers.CBTimerChange(Sender: TObject);
begin
  Timer.Enabled := CBTimer.Checked;
end;

procedure TFLasers.FormShow(Sender: TObject);
begin
  IniPropStorage.Restore;
  xc := IniPropStorage.ReadInteger('Xc', xc);
  yc := IniPropStorage.ReadInteger('Yc', yc);
  mc := IniPropStorage.ReadInteger('Mag', round(mc * 1000)) / 1000;
  if mc = 0 then mc := 1;

  ActivateComPort(IniPropStorage.ReadBoolean('ComPortOpen', false));
  ActivateComPortNeato(IniPropStorage.ReadBoolean('ComPortNeatoOpen', false));
  //Serial.Open;
  if Serial.Active then begin
    Serial.WriteData('SCIP2.0' + #$0A); // Change Protocol
    Serial.WriteData('RS' + #$0A); // Reset
    Serial.WriteData('VV' + #$0A); // version
    if CBAutoStart.Checked then
      BContionousScanClick(Sender);
  end;

  BSegSetValuesClick(Sender);
  BSetFitData.Click();
  BNeatoSetPars.Click();

  //QueryPerformanceFrequency(qpfc);
  //EditDebug.Text := inttostr(qpfc div 1000);

  if IniPropStorage.ReadBoolean('NetActive', false) then Udp.Listen(9875);
  if IniPropStorage.ReadBoolean('LocNetActive', false) then UDPLoc.Listen(9878);
end;

procedure TFLasers.SerialNeatoRxData(Sender: TObject);
var s, stmp: string;
    i, k, idx, packet_index, tmp_data, view_ray_index: integer;
    serialsize: integer;
begin
{  if SourceFrame > 0 then begin // Play Mode
    if not paused then begin
      try
        s := SourceStream.ReadAnsiString;
        inc(SourceFrame);
        LastFrame := s;
      except
        on E: exception do begin // Reached the end and loop to the begining
          SourceStream.Seek(0, soFromBeginning);
          SourceFrame := 1;
        end;
      end;
    end else begin // Paused
      s := lastFrame;
    end;
  end else begin // Normal operation from the serial port
    s := serial.ReadData;
  end;

  if DestFrame > 0 then begin  // Record Mode
    DestStream.WriteAnsiString(s);
    inc(DestFrame);
  end;}
  inc(SerialPacketCount);

  if not SerialNeato.Active then exit;
  //while memo.Lines.Count < 90 do memo.Lines.add('');

  s := SerialNeato.ReadData;
  if s = '' then exit;

  serialsize := Length(s);

  SerialDataNeato := SerialDataNeato + s;
  try

    //if CBShowRawData.Checked then Memo.Lines.text := Memo.Lines.text + s;
    while true do begin
      idx := pos(chr($FA), SerialDataNeato);
      if idx <= 0 then break; // No FA in stream

      if idx > 1 then begin  // Remove Chars before FA
        SerialDataNeato := copy(SerialDataNeato, idx, maxint);
      end;

      if Length(SerialDataNeato) > 22 then begin
        // Process packet
        // <start byte> <index>  <speed> <Data 0> <Data 1> <Data 2> <Data 3> <checksum>
        //  1            2        3-4     5        9        13       17       21-22
        // Data
        // byte 0: <distance 7:0>
        // byte 1: <"invalid data" flag> <"strength warning" flag> <distance 13:8>
        // byte 2: <signal strength 7:0>
        // byte 3: <signal strength 15:8>

        // TNeatoPacket = record
        //   index, speed: integer;
        //   dist, strength : integer;
        //   invalid_data, strength_warning: boolean;
        // end;

        packet_index := ord(SerialDataNeato[2]) - $A0;

        if (packet_index < 0) or (packet_index >= MaxNeatoPackets) then begin
          SerialDataNeato := copy(SerialDataNeato, 22 + 1, maxint);
          //raise Exception.Create('Bad Neato Packet: index = '+ inttostr(packet_index));
          memo.Lines.Add('Bad Neato Packet: index = '+ inttostr(packet_index));
          break;
        end;

        inc(NeatoPacketCount);

        if packet_index = 0 then begin
          for i := 0 to LaserScanNeato.count - 1 do begin
            LaserScanNeato.Rays[i].d := 0;
            LaserScanNeato.Rays[i].ang := i;
          end;
        end;

        for k := 0 to 3 do begin
          i := 4 * packet_index + k;
          NeatoRays[i].index := packet_index;
          NeatoRays[i].speed :=  ord(SerialDataNeato[3]) +  256 * ord(SerialDataNeato[4]);
          tmp_data := ord(SerialDataNeato[5 + k * 4]) +  256 * ord(SerialDataNeato[6 + k * 4]);
          NeatoRays[i].dist := tmp_data and $3FFF;
          NeatoRays[i].invalid_data := (tmp_data and $8000) <> 0;
          NeatoRays[i].strength_warning := (tmp_data and $4000) <> 0;
          NeatoRays[i].strength := ord(SerialDataNeato[7 + k * 4]) +  256 * ord(SerialDataNeato[8 + k * 4]);
        end;


        if packet_index = 89 then begin
          view_ray_index := StrToIntDef(EditNeatoRayIndex.Text, -1);
          for i := 0 to LaserScanNeato.count - 1 do begin
            if not NeatoRays[i].invalid_data then begin
              LaserScanNeato.Rays[i].d := 1e-3 * NeatoRays[i].dist;
              if LaserScanNeato.Rays[i].d > 0 then begin
                //a2:= -0.024181451147513;//-0.0242593522508926;
                //a1:= 1.17; //2.03429716705016;//1.037039514388;
                //a0:= -0.00664616011389633;//-0.00868952234902554;
                LaserScanNeato.Rays[i].d := a2*(LaserScanNeato.Rays[i].d*LaserScanNeato.Rays[i].d) + a1*(LaserScanNeato.Rays[i].d) + a0;
              end;
            end else begin
              LaserScanNeato.Rays[i].d := 0;//LaserScanNeato.Rays[max(0, i-1)].d;
            end;
            if view_ray_index = i then begin
              EditNeatoRayValue.Text := format('%.6f', [LaserScanNeato.Rays[i].d]);
            end;
            LaserScanNeato.Rays[i].ang := i;
            LaserScanNeato.Rays[i].x := LaserScanNeato.Rays[i].d * cos(degtorad(i));
            LaserScanNeato.Rays[i].y := LaserScanNeato.Rays[i].d * sin(degtorad(i));
          end;

          if UDP.Connected then SendLaserNeatoMessage(1);
          if CBDrawRadial.Checked then DrawLaserScan(LaserScanNeato);

          NeatoScan := GetTickCount();
          if CBShowNeatoSpeed.Checked then begin
            while memo.Lines.Count > 50 do memo.Lines.Delete(0);
            memo.Lines.Add(format('Neato: scan time = %d, Count = %d , serial = %d, size = %d', [NeatoScan - lastNeatoScan, NeatoPacketCount, SerialPacketCount, serialsize]));
            //Memo.VertScrollBar.Position := 1000;
            //Memo.ScrollBy(0, 100);
            //Memo.SelStart:=;
          end;
          //EditSpeed.Text := format('Neato: scan time = %d, Count = %d', [NeatoScan - lastNeatoScan, NeatoPacketCount]);
          lastNeatoScan := NeatoScan;
          NeatoPacketCount := 0;

          //SerialDataNeato := copy(SerialDataNeato, 22 + 1, maxint);
          //break;
        end;

        stmp := format('Neato: Index = %d, Speed = %d', [NeatoRays[packet_index].index, NeatoRays[packet_index].speed]);
        //EditSpeed.Text := stmp;

        //memo.Lines.Add(stmp);
        //while memo.Lines.Count > 20 do memo.Lines.Delete(0);
        //memo.Lines[packet_index] := stmp;


        if CBShowRawData.Checked then begin
          stmp := '';
          for i := 1  to 22 do begin
            stmp := stmp + inttohex(ord(SerialDataNeato[i]), 2) + ' ';
          end;
          Memo.Lines.Add(stmp);
          //while Memo.Lines.Count >= 20 do Memo.Lines.Delete(0);
        end;

        SerialDataNeato := copy(SerialDataNeato, 22 + 1, maxint);
      end else begin
        break
      end;
    end;
  except on E: exception do begin
    Memo.Lines.add(E.message);
  end;
  end;
end;

procedure TFLasers.SerialRxData(Sender: TObject);
var s: string;
    i, idx: integer;
begin
  if SourceFrame > 0 then begin // Play Mode
    if not paused then begin
      try
        s := SourceStream.ReadAnsiString;
        inc(SourceFrame);
        LastFrame := s;
      except
        on E: exception do begin // Reached the end and loop to the begining
          SourceStream.Seek(0, soFromBeginning);
          SourceFrame := 1;
        end;
      end;
    end else begin // Paused
      s := lastFrame;
    end;
  end else begin // Normal operation from the serial port
    s := serial.ReadData;
  end;

  if DestFrame > 0 then begin  // Record Mode
    DestStream.WriteAnsiString(s);
    inc(DestFrame);
  end;

  SerialData := SerialData + s;
  try
    //Memo.Lines.Delimiter := '[';// #$0A;
    if CBShowRawData.Checked then Memo.Lines.DelimitedText := s;
    //if CBShowRawData.Checked then Memo.Lines.text := s;

    while true do begin
      idx := 0;  // Check for the terminator: LF + LF
      for i := 1  to Length(SerialData) - 1 do begin
        if (SerialData[i] = chr($0A)) and (SerialData[i+1] = chr($0A)) then begin
          idx := i;
          break;
        end;
      end;

      if idx > 0 then begin // If terminator found then extract and processs message
        s := Copy(SerialData, 1, idx);
        SerialData := Copy(SerialData, idx + 2,  maxint);
        processLaserMessage(s);
      end else begin
        break; // if terminator not found then exit and wait for more bytes
      end;
    end;
  except on E: exception do begin
    Memo.Lines.add(E.message);
  end;
  end;
end;


{procedure TFLasers.ProcessSerialData(s: string);
var i, idx: integer;
begin

  if DestFrame > 0 then begin
    DestStream.WriteAnsiString(s);
    inc(DestFrame);
  end;

  SerialData := SerialData + s;
  try
    //Memo.Lines.Delimiter := '[';// #$0A;
    if CBShowRawData.Checked then Memo.Lines.DelimitedText := s;
    //if CBShowRawData.Checked then Memo.Lines.text := s;

    while true do begin
      idx := 0;  // Check for the terminator: LF + LF
      for i := 1  to Length(SerialData) - 1 do begin
        if (SerialData[i] = chr($0A)) and (SerialData[i+1] = chr($0A)) then begin
          idx := i;
          break;
        end;
      end;

      if idx > 0 then begin // If terminator found then extract and processs message
        s := Copy(SerialData, 1, idx);
        SerialData := Copy(SerialData, idx + 2,  maxint);
        processLaserMessage(s);
      end else begin
        break; // if terminator not found then exit and wait for more bytes
      end;
    end;
  except on E: exception do begin
    Memo.Lines.add(E.message);
  end;
  end;
end;
}

procedure TFLasers.TimerTimer(Sender: TObject);
begin
  BScan.Click;
end;

procedure TFLasers.BLaserOnClick(Sender: TObject);
begin
  Serial.WriteData('BM' + #$0A);
end;

procedure TFLasers.BNeatoSetParsClick(Sender: TObject);
begin
  a0 := StrToFloat(EditNeatoA0.Text);
  a1 := StrToFloat(EditNeatoA1.Text);
  a2 := StrToFloat(EditNeatoA2.Text);
end;

procedure TFLasers.BCloseComNeatoClick(Sender: TObject);
begin
  ActivateComPortNeato(false);
end;

procedure TFLasers.BOpenComNeatoClick(Sender: TObject);
begin
  SerialNeato.Device := EditComPortNeato.Text;
  ActivateComPortNeato(true);
end;

procedure TFLasers.BScanClick(Sender: TObject);
begin
  if not CB3ByteData.Checked then begin
    Serial.WriteData('GS0000076801' + #$0A);
  end else begin
    Serial.WriteData('GD0000076801' + #$0A);
  end;
end;

procedure TFLasers.BSetFitDataClick(Sender: TObject);
begin
  with LocConfig do begin
    Xmin := StrToFloat(EditXmin.Text);
    Xmax := StrToFloat(EditXmax.Text);
    Ymin := StrToFloat(EditYmin.Text);
    Ymax := StrToFloat(EditYmax.Text);
    ExpectedRadius := StrToFloat(EditExpectedRadius.Text);
    RadiusTolerance := StrToFloat(EditRadiusTolerance.Text);
    AngularResolution := StrToFloat(EditAngularResolution.Text);
  end;
end;

procedure TFLasers.BStartLocNetClick(Sender: TObject);
begin
  UDPLoc.Listen(9878);
end;

procedure TFLasers.BStopLocNetClick(Sender: TObject);
begin
  UDPLoc.Disconnect;
  //EditSendToIP.Color := clred;
end;


procedure TFLasers.BTestFitDataClick(Sender: TObject);
var C: TDMatrix;
begin
  C := TestFitCircle();
  MemoFitData.Lines.Add(format('(%g, %g) %g', [C[0, 0], C[0, 1], C[0, 2]]));
end;


procedure TFLasers.BVersionClick(Sender: TObject);
begin
  Serial.WriteData('VV' + #$0A);
end;

procedure TFLasers.BResetClick(Sender: TObject);
begin
  Serial.WriteData('RS' + #$0A);
end;

procedure TFLasers.BLaserOffClick(Sender: TObject);
begin
  Serial.WriteData('QT' + #$0A);
end;

procedure TFLasers.BContionousScanClick(Sender: TObject);
begin
  if not CB3ByteData.Checked then begin
    Serial.WriteData('MS0000076801000' + #$0A);
  end else begin
    Serial.WriteData('MD0000076801000' + #$0A);
  end;
end;

procedure TFLasers.BAdjustTimeOnClick(Sender: TObject);
begin
  Serial.WriteData('TM0' + #$0A);
end;

procedure TFLasers.BTimeClick(Sender: TObject);
begin
  Serial.WriteData('TM1' + #$0A);
end;

procedure TFLasers.BAdjustTimeOffClick(Sender: TObject);
begin
  Serial.WriteData('TM2' + #$0A);
end;

procedure TFLasers.BOpenComClick(Sender: TObject);
begin
  Serial.Device := EditComPort.Text;
  ActivateComPort(true);
end;

procedure TFLasers.BCloseComClick(Sender: TObject);
begin
  ActivateComPort(false);
end;

procedure TFLasers.BHihSensitivityOnClick(Sender: TObject);
begin
  Serial.WriteData('HS1' + #$0A);
end;

procedure TFLasers.BHihSensitivityOffClick(Sender: TObject);
begin
  Serial.WriteData('HS0' + #$0A);
end;

procedure TFLasers.BSpecsClick(Sender: TObject);
begin
  Serial.WriteData('PP' + #$0A);
end;

procedure TFLasers.BStatusClick(Sender: TObject);
begin
  Serial.WriteData('II' + #$0A);
end;

procedure TFLasers.BSetSpeedClick(Sender: TObject);
begin
  if ComboBoxSpeed.ItemIndex < 0 then exit;
  Serial.WriteData('CR'+ ComboBoxSpeed.Items[ComboBoxSpeed.ItemIndex] + #$0A);
end;

procedure TFLasers.PaintBoxMouseDown(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Integer);
begin
  mouse_down_x := x;
  mouse_down_y := y;
  mouse_down := true;
end;

procedure TFLasers.PaintBoxMouseUp(Sender: TObject; Button: TMouseButton; Shift: TShiftState; X, Y: Integer);
begin
  xc := xc + (x - mouse_down_x);
  yc := yc + (y - mouse_down_y);
  mouse_down := false;
end;

procedure TFLasers.PaintBoxMouseMove(Sender: TObject; Shift: TShiftState; X, Y: Integer);
begin
  if mouse_down then begin
    xc := xc + (x - mouse_down_x);
    yc := yc + (y - mouse_down_y);
    mouse_down_x := x;
    mouse_down_y := y;
  end;
end;

procedure TFLasers.PaintBoxMouseWheel(Sender: TObject; Shift: TShiftState; WheelDelta: Integer; MousePos: TPoint;
  var Handled: Boolean);
begin
  mc := mc * Power(1.1, WheelDelta / 120);
end;

procedure TFLasers.BSegSetValuesClick(Sender: TObject);
begin
  SegMaxPointDist2 := sqr(strTofloat(EditSegMaxPointDist.text));
  SegSize := strTofloat(EditSegSize.text);
  SegSizeTol := strTofloat(EditSegSizeTol.text)/100;
  TotalSize := strTofloat(EditTotalSize.text);
  TotalSize2 := sqr(TotalSize);
end;

procedure TFLasers.FormDestroy(Sender: TObject);
begin
  SourceStream.Free;
  DestStream.Free;

  LaserScanG.Free;
  LaserScanNeato.Free;
end;

procedure TFLasers.BRecordClick(Sender: TObject);
begin
  if FileExists(EditDestFileName.Text) then begin
    if MessageDlg(EditDestFileName.Text + 'already exists!'+ #10+ #13 +
                  'Is it OK to overwrite it?',
                   mtConfirmation , [mbOk,mbCancel], 0) = mrCancel then exit;
  end;
  DestStream.Clear;
  DestFrame := 1;
end;

procedure TFLasers.BStopClick(Sender: TObject);
begin
  if DestFrame > 0 then begin
    DestFrame := 0;
    DestStream.SaveToFile(EditDestFileName.Text);
  end;

  if SourceFrame > 0 then begin
    TimerFileData.Enabled := false;
    SourceFrame := 0;
  end;
end;

procedure TFLasers.BPlayClick(Sender: TObject);
begin
  if Serial.Active then exit;
  SourceStream.Clear;
  SourceStream.LoadFromFile(EditSourceFileName.Text);
  SourceFrame := 1;
  TimerFileData.Enabled := true;
end;

procedure TFLasers.BPauseClick(Sender: TObject);
begin
  paused := not paused;
end;

procedure TFLasers.TimerFileDataTimer(Sender: TObject);
begin
  if SourceFrame > 0 then SerialRxData(Sender);
end;

procedure TFLasers.ApplicationPropertiesIdle(Sender: TObject; var Done: Boolean);
begin
  ImageRec.Visible := DestFrame > 0; // Rec Mode
  ImageRecDisable.Visible := not ImageRec.Visible;

  ImagePlay.Visible := SourceFrame > 0; // Play Mode
  ImagePlayDisable.Visible := not ImagePlay.Visible;

  ImagePause.Visible := paused;


  if UDP.Connected  then begin
    EditSendToIP.Color := clgreen;
  end else begin
    EditSendToIP.Color := clSkyBlue;
  end;

  if UDPLoc.Connected  then begin
    EditLocToIP.Color := clgreen;
  end else begin
    EditLocToIP.Color := clSkyBlue;
  end;

  done := true;
end;


procedure TFLasers.ImageRecClick(Sender: TObject);
begin
  BStopClick(Sender);
end;

procedure TFLasers.ImageRecDisableClick(Sender: TObject);
begin
  BRecordClick(Sender);
end;

procedure TFLasers.ImagePlayClick(Sender: TObject);
begin
  BStopClick(Sender);
end;

procedure TFLasers.ImagePlayDisableClick(Sender: TObject);
begin
  BPlayClick(Sender);
end;

procedure TFLasers.ImagePauseClick(Sender: TObject);
begin
  paused := not paused;
end;

procedure TFLasers.BStartNetClick(Sender: TObject);
begin
  UDP.Listen(9875);
end;

procedure TFLasers.BStopNetClick(Sender: TObject);
begin
  UDP.Disconnect;
  EditSendToIP.Color := clred;
end;

procedure TLaserScan.CalcSinCosTable;
var i: integer;
    ang: double;
begin
  for i := 0 to MaxHokuyoScans - 1 do begin
    ang := degtorad(270 * i / (3*256) - 135);
    SinCosTable[i].cos_ang := cos(ang);
    SinCosTable[i].sin_ang := sin(ang);
  end;
  SinCosTableDone := true;
end;

procedure TFLasers.processLaserScan(slist: TStringList; var LaserScan: TLaserScan);
var s, command: string;
    i, j, x, firstdataline: integer;
begin
  with LaserScan do begin
    command := copy(slist[0], 1, 2);

    start := strtointdef(copy(slist[0], 3, 4), -1);
    stop := strtointdef(copy(slist[0], 7, 4), -1);

    if (start = -1) or (stop = -1) then exit;
    cluster := strtointdef(copy(slist[0], 11, 2), 0);

    status := strtointdef(copy(slist[1], 1, 2), 0);
    if not(status in [0,99]) then exit;

    if CBShowProcData.Checked then begin
      memo.Clear;
      //memo.Lines.Add('dummy');
      //i := slist.Count;
      memo.Lines.Add(format('Lines: %d',[slist.Count]));
      memo.Lines.Add(format('%s(%d): %d->%d',[command, status, start, stop]));
    end;

    if slist.Count < 3 then exit;
    s := copy(slist[2], 1, 4);
    if length(s) < 4 then exit;
    timestamp := (((ord(s[1]) - $30) * 64 + (ord(s[2]) - $30)) * 64 + (ord(s[3]) - $30)) * 64 + (ord(s[4]) - $30);
    //memo.Lines.Add(format('%s',[copy(slist[2], 1, 4)]));

    if CBShowProcData.Checked then begin
      memo.Lines.Add(format('%d',[timestamp]));
    end;

    if command[1] = 'G' then begin
      firstdataline := 3;
    end else if command[1] = 'M' then begin
      firstdataline := 3;
    end;

    x := 0;
    if command[2] = 'S' then begin  // 2 bytes encoding
      for i := firstdataline to slist.Count - 1 do begin
        s := slist[i];
        for j := 1 to Length(s) div 2 do begin
          Rays[x].d := ((ord(s[2*j-1]) - $30) * 64 + (ord(s[2*j]) - $30)) / 1000;
          Rays[x].ang := degtorad(270 * x / (3*256) - 135);
          inc(x);
        end;
      end;
    end else if command[2] = 'D' then begin // 3 bytes encoding
      s := '';
      for i := firstdataline to slist.Count - 1 do begin
        s := s + copy(slist[i],1, min(64, length(slist[i]) - 1));
      end;
      for j := 1 to Length(s) div 3 do begin
        Rays[x].d := (((ord(s[3*j-2]) - $30) * 64 + (ord(s[3*j-1]) - $30)) * 64 + (ord(s[3*j]) - $30)) / 1000;
        Rays[x].ang := degtorad(270 * x / (3*256) - 135);
        inc(x);
      end;
    end;

    if not SinCosTableDone then CalcSinCosTable;
    for i := start to stop do begin
      Rays[i].x :=  SinCosTable[i].cos_ang * Rays[i].d;
      Rays[i].y :=  SinCosTable[i].sin_ang * Rays[i].d;
      Rays[i].bad :=  ord(Rays[i].d <= (23/1000 + 1e-6));
    end;
  end;
end;


procedure TFLasers.DrawLaserScan(LaserScan: TLaserScan);
var i: integer;
    xd, yd: integer;
    rl: integer;
begin
  with LaserScan do begin
    if PageControl.ActivePage = TabChart then begin
      CSLaserData.Clear;
      for i := start to stop do begin
        CSLaserData.AddXY(Rays[i].ang, Rays[i].d);
      end;
    end else if PageControl.ActivePage = TabRadial then begin
      PaintBox.Canvas.Brush.Style := bsSolid;
      PaintBox.Canvas.Clear;
      if CBDrawLines.Checked then begin
        PaintBox.Canvas.moveTo(round(xc + Rays[start].x * mc), round(yc - Rays[start].y * mc));
        for i := start+1 to stop do begin
          if (Rays[i].bad = 0) and (Rays[i-1].bad = 0) then begin
            if (Rays[i].seg and 1) = 0 then begin
              PaintBox.Canvas.Pen.Color := clred;
            end else begin
              PaintBox.Canvas.Pen.Color := clblack;
            end;
          end else begin
            //PaintBox.Canvas.Pen.Color := PaintBox.Color;
            PaintBox.Canvas.Pen.Color := clwhite;
          end;
          PaintBox.Canvas.LineTo(round(xc + Rays[i].x * mc), round(yc - Rays[i].y * mc));
        end;
      end else begin
        rl := 2;
        for i := start to stop do begin
          if (Rays[i].bad = 0) and (Rays[i-1].bad = 0) then begin
            if (Rays[i].seg and 1) = 0 then begin
              PaintBox.Canvas.Pen.Color := clred;
            end else begin
              PaintBox.Canvas.Pen.Color := clblack;
            end;
          end else begin
            PaintBox.Canvas.Pen.Color := clwhite;
          end;
          xd := round(xc + Rays[i].x * mc);
          yd := round(yc - Rays[i].y * mc);
          PaintBox.Canvas.Rectangle(xd - rl, yd - rl, xd + rl, yd + rl);
          //PaintBox.Canvas.TextOut(xd + 10, yd, IntToStr(Rays[i].seg));
        end;
      end;

      PaintBox.Canvas.Brush.Style := bsClear;
      PaintBox.Canvas.Pen.Color := clGreen;
      for i := 0 to XYReListCount - 1 do begin
        //PaintBox.Canvas.MoveTo(round(xc + XYReList[i, 0] * mc), round(yc - XYReList[i, 1] * mc));
        //PaintBox.Canvas.AngleArc(round(xc + XYReList[i, 0] * mc), round(yc - XYReList[i, 1] * mc), round(XYReList[i, 2] * mc), 0, 360);
        if i = bestXYReListIdx then
          PaintBox.Canvas.EllipseC(round(xc + XYReList[i, 0] * mc), round(yc - XYReList[i, 1] * mc), round(XYReList[i, 2] * mc), round(XYReList[i, 2] * mc));
      end;

    end;
  end;
end;

procedure TFLasers.ProcessTimeRequest(slist: TStringList);
var s, command: string;
    status, ControlCode, timestamp: integer;
begin
    command := copy(slist[0], 1, 2);

    ControlCode := strtointdef(copy(slist[0], 3, 1), -1);
    if ControlCode = - 1 then exit;

    status := strtointdef(copy(slist[1], 1, 2), 0);
    if not(status in [0,99]) then exit;

    if (ControlCode = 1) then begin
      if slist.Count < 3 then exit;
      s := copy(slist[2], 1, 4);
      if length(s) < 4 then exit;
      timestamp := (((ord(s[1]) - $30) * 64 + (ord(s[2]) - $30)) * 64 + (ord(s[3]) - $30)) * 64 + (ord(s[4]) - $30);
    end;

    if CBShowProcData.Checked then begin
      memo.Clear;
      memo.Lines.Add(format('%d',[slist.Count]));
      memo.Lines.Add(format('%s(%d): %d',[command, status, timestamp]));
    end;
end;

procedure TFLasers.processLaserMessage(str: string);
var slist: TStringList;
    command: string;
    t_start, t_stop: int64;
begin
  slist := TStringList.Create;
  try
  try
    Slist.Text := str;
    //memo.Clear;
    //memo.Lines.AddStrings(slist);

    if slist.Count = 0 then exit;
    command := copy(slist[0], 1, 2);
    if (command = 'GS') or (command = 'MS') or (command = 'GD') or (command = 'MD') then begin
      ProcessLaserScan(slist, LaserScanG);
      //QueryPerformanceCounter(t_start);
      if CBLocActive.Checked then begin
        LocateCircle(LaserScanG);
        if UDPLoc.Connected then SendLocMessage();
      end;
      if UDP.Connected then SendLaserMessage(1);

      DrawLaserScan(LaserScanG);
      //QueryPerformanceCounter(t_stop);
      //EditDebug.Text := format('%.4g',[1e6 * (t_stop - t_start) / qpfc]);
    end else if (command = 'TM') then begin
      ProcessTimeRequest(slist);
    end;

  except on E: Exception do
    EditException.Text := E.message;
  end;

  finally
    Slist.Free;
  end;
end;


procedure TFLasers.SendLocMessage();
var i: integer;
    x, y, fit: single;
begin
  if bestXYReListIdx < 0 then exit;
  x := XYReList[bestXYReListIdx, 0];
  y := XYReList[bestXYReListIdx, 1];
  fit := XYReList[bestXYReListIdx, 3];

  ClearUDPBuffer(NetOutBuf);
  // tag this packet as Hokuyo Localization Version 1
  NetPutByte(NetOutBuf, ord('H'));
  NetPutByte(NetOutBuf, ord('L'));
  NetPutByte(NetOutBuf, ord('1'));
  NetPutFloat(NetOutBuf, x);
  NetPutFloat(NetOutBuf, y);
  NetPutFloat(NetOutBuf, fit);
  UDPLoc.Send(NetOutBuf.data, NetOutBuf.MessSize, EditLocToIP.Text + ':9879');
end;


procedure TFLasers.SendLaserMessage(mtype: integer);
var i: integer;
    ld: word;
begin
  ClearUDPBuffer(NetOutBuf);
  // tag this packet as Hokuyo -> Anyone
  NetPutByte(NetOutBuf, ord('H'));
  NetPutByte(NetOutBuf, ord('A'));
  NetPutByte(NetOutBuf, ord('1'));
  //NetPutByte(NetOutBuf, ord('0') + mtype); // Type = 1,2,3
  for i := 44 to 725 do begin
    ld := round(LaserScanG.Rays[i].d * 1000);
    NetPutWord(NetOutBuf, ld);
  end;
  UDP.Send(NetOutBuf.data, NetOutBuf.MessSize, EditSendToIP.Text + ':9876');
end;

procedure TFLasers.SendLaserNeatoMessage(mtype: integer);
var i: integer;
    ld: word;
begin
  ClearUDPBuffer(NetOutBuf);
  // tag this packet as Neato -> Anyone
  NetPutByte(NetOutBuf, ord('N'));
  NetPutByte(NetOutBuf, ord('A'));
  NetPutByte(NetOutBuf, ord('1'));
  //NetPutByte(NetOutBuf, ord('0') + mtype); // Type = 1,2,3
  for i := 0 to 359 do begin
    ld := round(LaserScanNeato.Rays[i].d * 1000);
    NetPutWord(NetOutBuf, ld);
  end;
  UDP.Send(NetOutBuf.data, NetOutBuf.MessSize, EditSendToIP.Text + ':9876');
end;


procedure TFLasers.LocateCircle(var LaserScan: TLaserScan);
var i, j, c, act_seg, act_seg_size{, last_good_ray_i}: integer;
    xs, ys: double;
    Pall, Pseg, XYRe: TDMatrix;
    Np: integer;
    dc, expectedPoints, meanPointError: double;
    NumGoodPoints: integer;
    xce, yce, fit, NumPointsFit: double;
    max_fit: double;
begin
  with LaserScan do begin
    act_seg := 0;
    act_seg_size := 0;
    for i := start to stop do begin
      Rays[i].seg := 0;
      if Rays[i].bad <> 0 then continue;
      if (Rays[i].x > LocConfig.Xmax) or
         (Rays[i].x < LocConfig.Xmin) or
         (Rays[i].y > LocConfig.Ymax) or
         (Rays[i].y < LocConfig.Ymin) then begin
        Rays[i].bad := 2;
        continue;
      end;
      // If there is no open segment
      if act_seg_size = 0 then begin
        act_seg_size := 1;
        inc(act_seg);
        Rays[i].seg := act_seg;
        xs := Rays[i].x;
        ys := Rays[i].y;
      end else begin
        // There is an open segment and the point is compatible
        //if sqr(Rays[last_good_ray_i].x - Rays[i].x) + sqr(Rays[last_good_ray_i].y - Rays[i].y) < sqr(1.3 * LocConfig.ExpectedRadius) then begin
        if sqr(xs / act_seg_size  - Rays[i].x) + sqr(ys / act_seg_size - Rays[i].y) < sqr(1.3 * LocConfig.ExpectedRadius) then begin
          inc(act_seg_size);
          Rays[i].seg := act_seg;
          xs += Rays[i].x;
          ys += Rays[i].y;
        // There is an open segment and the point is incompatible
        // So create a new segment
        end else begin
          act_seg_size := 1;
          inc(act_seg);
          Rays[i].seg := act_seg;
          xs := Rays[i].x;
          ys := Rays[i].y;
        end;
      end;
    end;

    // Last ray has a new dummy segment
    Rays[stop].seg := act_seg + 1;
    Rays[stop].bad := 0;


    MemoFitData.Clear;
    XYReListCount := 0;
    Pall := Mzeros(stop, 2);
    act_seg := -1;
    act_seg_size := 0;
    for i := start to stop do begin
      if Rays[i].bad <> 0 then continue;

      if (Rays[i].seg <> act_seg) then begin
        if act_seg_size > 6 then begin
          Pseg := MCrop(Pall, 0, 0, act_seg_size - 1, 1);
          //Pseg := MCrop(Pall, 1, 0, act_seg_size - 2, 1);
          //XYRe := FitCircle(Pseg);
          XYRe := FitCircleLMmin(Pseg, LocConfig.ExpectedRadius, Memo.Lines);
          //if (XYRe[0, 2] > 0) and (abs(XYRe[0, 2] -  LocConfig.ExpectedRadius) < 0.02) then begin

            Xce := XYRe[0, 0];
            Yce := XYRe[0, 1];

            Np := round(XYRe[0, 4]);
            NumGoodPoints := 0;
            for j := 0 to Pseg.NumRows - 1 do begin
              if sqr(Xce) + sqr(Yce) > sqr(Pseg[j, 0]) + sqr(Pseg[j, 1]) then begin
                inc(NumGoodPoints);
              end else begin
                dec(NumGoodPoints);
              end;
            end;

            dc := sqrt(sqr(XYRe[0, 0])  + sqr(XYRe[0, 1]));
            if dc > 0 then begin
              expectedPoints := 2 * arctan(0.8 * LocConfig.ExpectedRadius / dc) / degtorad(LocConfig.AngularResolution);
            end else begin
              expectedPoints := 1024;
            end;
            meanPointError := sqrt(XYRe[0, 3])/ XYRe[0, 4];

            NumPointsFit := abs(1 - max(0, NumGoodPoints) / expectedPoints);
            fit := meanPointError + meanPointError * sqrt(NumPointsFit);

            if NumPointsFit > 0.3 then fit := fit * 100;

            //MStampRow(XYReList, XYRe, XYReListCount);
            for c := 0 to XYRe.NumCols - 1 do begin
              XYReList[XYReListCount, c] := XYRe[0, c];
            end;
            XYReList[XYReListCount, 3] := fit;

            inc(XYReListCount);
            if XYReListCount>= 255 then XYReListCount := 255;

            MemoFitData.Lines.Add(format('(%.3f, %.3f) %.3f [%.4f] (%.3f) %.6f',
                                          [xce, yce, XYRe[0, 2], meanPointError, max(0, NumGoodPoints) / round(expectedPoints), fit]));
          //end;
        end;

        act_seg_size := 1;
        act_seg := Rays[i].seg;
        Pall[0, 0] := Rays[i].x;
        Pall[0, 1] := Rays[i].y;
      end else begin

        Pall[act_seg_size, 0] := Rays[i].x;
        Pall[act_seg_size, 1] := Rays[i].y;
        inc(act_seg_size);
      end;
    end;

    bestXYReListIdx := -1;
    max_fit := MaxDouble;
    for i := 0 to XYReListCount - 1 do begin
      if XYReList[i, 3] < max_fit then begin
        max_fit := XYReList[i, 3];
        bestXYReListIdx := i;
      end;
    end;

  end;
end;

initialization
  {$I lasers.lrs}

end.

