function [answer] = dropdown()
global btn_close UavXposition Uav2Xposition Uav3Xposition Uav4Xposition Uav5Xposition fig4;
global TruckHDOP TruckVDOP Perror Terror Truckposition
global ARPS_HDOP ARPS_positionerror ARPS_timingerror ARPS_VDOP RevTri_HDOP RevTri_VDOP RevTri_timingerror RevTri_positionerror Tri_HDOP Tri_VDOP Tri_timingerror Tri_positionerror;
global COPPanel COP_PAX
global vector_position0 vector_position vector_position3 vector_position4 vector_position5
global cop_flag
global uav1x uav2x uav3x uav4x uav1y uav2y uav3y uav4y uav1z uav2z uav3z uav4z 
global xyzpositionLabel IDlabel uavflag
global TruckPDOP TruckHDOP
global start_flag
global ARPSlabel EARPSlabel
global f

%기본 리턴 값 (4, 1, 5, 50)
btn_close = 0;
start_flag=1;
if btn_close == 0
    retval1 = 2;
    retval2 = 2;
    retval3 = 20;
    retval4 = 50;
    retval5 = 20;
    retval6 = 10;
    retval7 = 0;
    retval8 = 0;
    pp= 0;
    %기본 박스 만들기
    f = uifigure;
    f.Position= [0 40 1970 330];
    f.Name = 'FNT-14 Simulator Control Box';
    f.Color = [0 0 0];
    tp_temp = 0;
    
    algo_temp = 0;
%     FNT14Label = uilabel(f);
%     FNT14Label.FontSize = 36;
%     FNT14Label.Position = [100 423 462 45];
%     FNT14Label.Text = 'FNT-14 Simulator Controller';
    
%     OPERATIONMODEDropDownLabel = uilabel(f);
%     OPERATIONMODEDropDownLabel.BackgroundColor = [0.8 0.8 0.8];
%     OPERATIONMODEDropDownLabel.HorizontalAlignment = 'right';
%     OPERATIONMODEDropDownLabel.FontSize = 20;
%     OPERATIONMODEDropDownLabel.FontWeight = 'bold';
%     OPERATIONMODEDropDownLabel.Position = [151 382 188 24];
%     OPERATIONMODEDropDownLabel.Text = 'OPERATION MODE';

        
        


    
    %  COP 패널 디자인
    
    COPPanel = uipanel(f);
    COPPanel.BackgroundColor = [0 0 0];
    COPPanel.ForegroundColor = [1 1 1];
    COPPanel.Title = 'Common Operation Picture';
    COPPanel.FontSize = 16;

    COPPanel.Position = [1480 3 430 310];
    COP_PAX = axes(COPPanel);
    COP_PAX.Color = [0 0 0];
    COP_PAX.XColor = [1 1 1];
    COP_PAX.YColor = [1 1 1];
    
    %     X = imread('https://upload.wikimedia.org/wikipedia/commons/thumb/a/a5/South_Korea-Gyeonggi.svg/341px-South_Korea-Gyeonggi.svg.png');
    %     imagesc(COP_PAX, X)
    
    % Create UAV position !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    UAVPositionPanel = uipanel(f);
    UAVPositionPanel.Title = 'UAV/GS Position Information';
    UAVPositionPanel.BackgroundColor = [0 0 0];
    UAVPositionPanel.ForegroundColor = [1 1 1];
    UAVPositionPanel.FontSize = 16;
    UAVPositionPanel.Position = [15 3 400 310];
    
    
    UAV1ParameterPanel = uipanel(UAVPositionPanel);
    UAV1ParameterPanel.BackgroundColor = [0 0 0];
    UAV1ParameterPanel.ForegroundColor = [1 1 1];
    UAV1ParameterPanel.Title = 'UAV #1 Position';
    UAV1ParameterPanel.FontSize = 14;
    UAV1ParameterPanel.Position = [9 230 190 50];
    
    UavXposition = uilabel(UAV1ParameterPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    UavXposition.FontSize = 13;
    UavXposition.FontColor = [1 1 1];
    UavXposition.Position = [5 1 350 30 ];
    UavXposition.Text = 'Loading';
   
    
    UAV2ParameterPanel = uipanel(UAVPositionPanel);
    UAV2ParameterPanel.BackgroundColor = [0 0 0];
    UAV2ParameterPanel.ForegroundColor = [1 1 1];
    UAV2ParameterPanel.Title = 'UAV #2 Position';
    UAV2ParameterPanel.FontSize = 14;
    UAV2ParameterPanel.Position = [205 230 190 50];
    
        Uav2Xposition = uilabel(UAV2ParameterPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    Uav2Xposition.FontSize = 13;
    Uav2Xposition.FontColor = [1 1 1];
    Uav2Xposition.Position = [5 1 350 30 ];
    Uav2Xposition.Text = 'Loading';
    
    UAV3ParameterPanel = uipanel(UAVPositionPanel);
    UAV3ParameterPanel.BackgroundColor = [0 0 0];
    UAV3ParameterPanel.ForegroundColor = [1 1 1];
    UAV3ParameterPanel.Title = 'UAV #3 Position';
    UAV3ParameterPanel.FontSize = 14;
    UAV3ParameterPanel.Position = [9 175 190 50];
    
        Uav3Xposition = uilabel(UAV3ParameterPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    Uav3Xposition.FontSize = 13;
    Uav3Xposition.FontColor = [1 1 1];
    Uav3Xposition.Position = [5 1 350 30 ];
    Uav3Xposition.Text = 'Loading';
    
    UAV4ParameterPanel = uipanel(UAVPositionPanel);
    UAV4ParameterPanel.BackgroundColor = [0 0 0];
    UAV4ParameterPanel.ForegroundColor = [1 1 1];
    UAV4ParameterPanel.Title = 'UAV #4 Position';
    UAV4ParameterPanel.FontSize = 14;
    UAV4ParameterPanel.Position = [205 175 190 50];

        Uav4Xposition = uilabel(UAV4ParameterPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    Uav4Xposition.FontSize = 13;
    Uav4Xposition.FontColor = [1 1 1];
    Uav4Xposition.Position = [5 1 350 30 ];
    Uav4Xposition.Text = 'Loading';

    UAV5ParameterPanel = uipanel(UAVPositionPanel);
    UAV5ParameterPanel.BackgroundColor = [0 0 0];
    UAV5ParameterPanel.ForegroundColor = [1 1 1];
    UAV5ParameterPanel.Title = 'UAV #5 Position';
    UAV5ParameterPanel.FontSize = 14;
    UAV5ParameterPanel.Position = [9 120 190 50];

        Uav5Xposition = uilabel(UAV5ParameterPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    Uav5Xposition.FontSize = 13;
    Uav5Xposition.FontColor = [1 1 1];
    Uav5Xposition.Position = [5 1 350 30 ];
    Uav5Xposition.Text = 'Loading';
    
      
    %Create GS information

    
    GCS1ParameterPanel = uipanel(UAVPositionPanel);
    GCS1ParameterPanel.BackgroundColor = [0 0 0];
    GCS1ParameterPanel.ForegroundColor = [1 1 1];
    GCS1ParameterPanel.Title = 'GS #1 Position';
    GCS1ParameterPanel.FontSize = 14;
    GCS1ParameterPanel.Position = [9 60 190 50];
    
    GCSXposition = uilabel(GCS1ParameterPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    GCSXposition.FontSize = 13;
    GCSXposition.FontColor = [1 1 1];
    GCSXposition.Position = [5 1 350 30 ];
    GCSXposition.Text = 'x : 40.00, y : -40.00, z : 00.00';
   
    
    GCS2ParameterPanel = uipanel(UAVPositionPanel);
    GCS2ParameterPanel.BackgroundColor = [0 0 0];
    GCS2ParameterPanel.ForegroundColor = [1 1 1];
    GCS2ParameterPanel.Title = 'GS #2 Position';
    GCS2ParameterPanel.FontSize = 14;
    GCS2ParameterPanel.Position = [205 60 190 50];
    
        GCS2Xposition = uilabel(GCS2ParameterPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    GCS2Xposition.FontSize = 13;
    GCS2Xposition.FontColor = [1 1 1];
    GCS2Xposition.Position = [5 1 350 30 ];
    GCS2Xposition.Text = 'x : -40.00, y : -40.00, z : 00.00';
    
    GCS3ParameterPanel = uipanel(UAVPositionPanel);
    GCS3ParameterPanel.BackgroundColor = [0 0 0];
    GCS3ParameterPanel.ForegroundColor = [1 1 1];
    GCS3ParameterPanel.Title = 'GS #3 Position';
    GCS3ParameterPanel.FontSize = 14;
    GCS3ParameterPanel.Position = [9 5 190 50];
    
        GCS3Xposition = uilabel(GCS3ParameterPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    GCS3Xposition.FontSize = 13;
    GCS3Xposition.FontColor = [1 1 1];
    GCS3Xposition.Position = [5 1 350 30 ];
    GCS3Xposition.Text = 'x : -40.00, y : 40.00, z : 00.00';
    
    GCS4ParameterPanel = uipanel(UAVPositionPanel);
    GCS4ParameterPanel.BackgroundColor = [0 0 0];
    GCS4ParameterPanel.ForegroundColor = [1 1 1];
    GCS4ParameterPanel.Title = 'GS #4 Position';
    GCS4ParameterPanel.FontSize = 14;
    GCS4ParameterPanel.Position = [205 5 190 50];
    
    GCS4Xposition = uilabel(GCS4ParameterPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    GCS4Xposition.FontSize = 13;
    GCS4Xposition.FontColor = [1 1 1];
    GCS4Xposition.Position = [5 1 350 30 ];
    GCS4Xposition.Text = 'x : 40.00, y : 40.00, z : 00.00';
    
    
    
    % Create Truck information !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%     ReceiverPositionPanel = uipanel(f);
%     ReceiverPositionPanel.Title = 'Receiver Information';
%     ReceiverPositionPanel.FontSize = 16;
%     ReceiverPositionPanel.Position = [1070 9 400 361];
    
    
    
    % Create SystemParameterPanel !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    SystemParameterPanel = uipanel(f);
    SystemParameterPanel.Title = 'System Parameters';
    SystemParameterPanel.BackgroundColor = [0 0 0];
    SystemParameterPanel.ForegroundColor = [1 1 1];
    SystemParameterPanel.FontSize = 16;
%     SystemParameterPanel.Position = [15 9 620 361];
    SystemParameterPanel.Position = [440 9 300 305];
    
    OPERATIONMODEDropDown= uidropdown(SystemParameterPanel,'ValueChangedFcn',@(OPERATIONMODEDropDown,event) selectionmode(OPERATIONMODEDropDown,retval1));
    OPERATIONMODEDropDown.Items = {'E-ARPS', 'ARPS'};
    OPERATIONMODEDropDown.FontSize = 14;
    OPERATIONMODEDropDown.FontColor = [1 1 1];
    OPERATIONMODEDropDown.FontWeight = 'bold';
    OPERATIONMODEDropDown.BackgroundColor = [0.2 0.2 0.2];
    OPERATIONMODEDropDown.Position = [80 258 120 20];
    OPERATIONMODEDropDown.Value = 'E-ARPS';
    
    % Create UAVParameterPanel
    UAVParameterPanel = uipanel(SystemParameterPanel);
    UAVParameterPanel.Title = 'UAVs Parameters';
    UAVParameterPanel.FontSize = 14;
    UAVParameterPanel.BackgroundColor = [0 0 0];
    UAVParameterPanel.ForegroundColor = [1 1 1];
    UAVParameterPanel.Position = [9 35 280 220];
    
    NumberofUAVDropDownLabel = uilabel(UAVParameterPanel);
    NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    NumberofUAVDropDownLabel.FontSize = 14;
    NumberofUAVDropDownLabel.FontColor = [1 1 1];
    NumberofUAVDropDownLabel.Position = [13 160 107 22];
    NumberofUAVDropDownLabel.Text = 'Number of UAVs';
    
    % Create NumberofUAVDropDown
    NumberofUAVDropDown = uidropdown(UAVParameterPanel,'ValueChangedFcn',@(NumberofUAVDropDown,event) selectionnuav(NumberofUAVDropDown,retval2));
    NumberofUAVDropDown.Items = {'5', '4'};
    NumberofUAVDropDown.FontSize = 14;
    NumberofUAVDropDown.FontColor = [1 1 1];
    NumberofUAVDropDown.Position = [135 160 100 22];
    NumberofUAVDropDown.BackgroundColor = [0.2 0.2 0.2];
    NumberofUAVDropDown.Value = '5';
    
    % Create AltitudeofUAVDropDownLabel
    AltitudeofUAVDropDownLabel = uilabel(UAVParameterPanel);
    AltitudeofUAVDropDownLabel.HorizontalAlignment = 'right';
    AltitudeofUAVDropDownLabel.FontSize = 14;
    AltitudeofUAVDropDownLabel.FontColor = [1 1 1];
    AltitudeofUAVDropDownLabel.Position = [16 130 104 22];
    AltitudeofUAVDropDownLabel.Text = 'Altitude of UAVs';
    
    % Create AltitudeofUAVDropDown
    AltitudeofUAVDropDown = uidropdown(UAVParameterPanel,'ValueChangedFcn',@(AltitudeofUAVDropDown,event) sel_UAValti(AltitudeofUAVDropDown,retval3));
    AltitudeofUAVDropDown.Items = {'20km','5km'}
    AltitudeofUAVDropDown.FontSize = 14;
    AltitudeofUAVDropDown.FontColor = [1 1 1];
    AltitudeofUAVDropDown.Position = [135 130 100 22];
    AltitudeofUAVDropDown.BackgroundColor = [0.2 0.2 0.2];
    AltitudeofUAVDropDown.Value = '20km';
    
    % Create RadiusofUAVDropDownLabel
    RadiusofUAVDropDownLabel = uilabel(UAVParameterPanel);
    RadiusofUAVDropDownLabel.HorizontalAlignment = 'right';
    RadiusofUAVDropDownLabel.FontSize = 14;
    RadiusofUAVDropDownLabel.FontColor = [1 1 1];
    RadiusofUAVDropDownLabel.Position = [5 100 110 22];
    RadiusofUAVDropDownLabel.Text = 'Radius of UAVs';
    
    % Create RadiusofUAVDropDown
    RadiusofUAVDropDown = uidropdown(UAVParameterPanel,'ValueChangedFcn',@(RadiusofUAVDropDown,event) sel_UAVradi(RadiusofUAVDropDown,retval4));
    RadiusofUAVDropDown.Items = {'30km','50km','70km'};
    RadiusofUAVDropDown.Position = [135 100 100 22];
    RadiusofUAVDropDown.FontSize = 14;
    RadiusofUAVDropDown.FontColor = [1 1 1];
    RadiusofUAVDropDown.BackgroundColor = [0.2 0.2 0.2];
    RadiusofUAVDropDown.Value = '50km';
    
    % Create StartButton
    StartButton = uibutton(SystemParameterPanel, 'push',....
        'ButtonPushedFcn', @cb_start);
    StartButton.Text = 'Start';
    StartButton.Position = [15 7 80 22];
    StartButton.FontColor = [1 1 1];
    StartButton.BackgroundColor = [0.2 0.2 0.2];

    % Create CloseButton
    CloseButton = uibutton(SystemParameterPanel, 'push',....
        'ButtonPushedFcn', @cb_close);
    CloseButton.Position = [100 7 80 22];
    CloseButton.Text = 'Close';
    CloseButton.FontColor = [1 1 1];
    CloseButton.BackgroundColor = [0.2 0.2 0.2];
    algoButton = uibutton(SystemParameterPanel, 'push',....
        'ButtonPushedFcn', @algo);
    algoButton.Position = [185 7 80 22];
    algoButton.FontColor = [1 1 1];
    algoButton.BackgroundColor = [0.2 0.2 0.2];
    algoButton.Text = 'Algorithm';
    

    
    
    % Create CenterUAVParameterPanel
    CenterUAVParameterPanel = uipanel(SystemParameterPanel);
    CenterUAVParameterPanel.Title = 'Center UAV Parameter';
    CenterUAVParameterPanel.Position = [16 40 239 85];
    CenterUAVParameterPanel.BackgroundColor = [0 0 0];
    CenterUAVParameterPanel.ForegroundColor = [1 1 1];
    
    % Create AltitudeofUAVsDropDownLabel
    AltitudeofUAVsDropDownLabel = uilabel(CenterUAVParameterPanel);
    AltitudeofUAVsDropDownLabel.HorizontalAlignment = 'right';
    AltitudeofUAVsDropDownLabel.FontSize = 13;
    AltitudeofUAVsDropDownLabel.Position = [16 37 96 22];
    AltitudeofUAVsDropDownLabel.FontColor = [1 1 1];
    AltitudeofUAVsDropDownLabel.Text = 'Altitude of UAV';
    
    % Create AltitudeofUAVsDropDown
    AltitudeofUAVsDropDown = uidropdown(CenterUAVParameterPanel,'ValueChangedFcn',@(AltitudeofUAVsDropDown,event) sel_pUAValti(AltitudeofUAVsDropDown,retval5));
    AltitudeofUAVsDropDown.FontSize = 14;
    AltitudeofUAVsDropDown.FontColor = [1 1 1];
    AltitudeofUAVsDropDown.BackgroundColor = [0.2 0.2 0.2];
    %AltitudeofUAVsDropDown.ForegroundColor = [1 1 1];
    AltitudeofUAVsDropDown.Items = {'20km','5km'}
    AltitudeofUAVsDropDown.Position = [130 37 100 22];
    
    % Create RadiusofUAVsDropDownLabel
    RadiusofUAVsDropDownLabel = uilabel(CenterUAVParameterPanel);
    RadiusofUAVsDropDownLabel.HorizontalAlignment = 'right';
    RadiusofUAVsDropDownLabel.FontSize = 14;
    RadiusofUAVsDropDownLabel.FontColor = [1 1 1];
    RadiusofUAVsDropDownLabel.BackgroundColor = [0 0 0];
    RadiusofUAVsDropDownLabel.Position = [8 8 103 22];
    RadiusofUAVsDropDownLabel.Text = 'Radius of UAV';
    
    % Create RadiusofUAVsDropDown
    RadiusofUAVsDropDown = uidropdown(CenterUAVParameterPanel,'ValueChangedFcn',@(RadiusofUAVsDropDown,event) sel_pUAVradi(RadiusofUAVsDropDown,retval6));
    RadiusofUAVsDropDown.Position = [130 8 100 22];
    RadiusofUAVsDropDown.Items = {'10km'}
    RadiusofUAVsDropDown.FontSize = 14;
    RadiusofUAVsDropDown.FontColor = [1 1 1];
    RadiusofUAVsDropDown.BackgroundColor = [0.2 0.2 0.2]
    
     % Create CarParameterPanel
            CarParameterPanel = uipanel(f);
            CarParameterPanel.Title = 'Receiver Parameter';
            CarParameterPanel.BackgroundColor = [0 0 0];
            CarParameterPanel.ForegroundColor = [1 1 1];
            CarParameterPanel.Position = [765 9 330 305];
            CarParameterPanel.FontSize = 16;
            
                TruckLabel = uilabel(CarParameterPanel);
                TruckLabel.FontSize = 16;
                TruckLabel.FontColor = [1 1 1];
                TruckLabel.Position = [9 240 300 50];
                TruckLabel.Text = 'Receiver Performance Evaluation';
            
                TruckHDOPPanel = uipanel(CarParameterPanel);
                TruckHDOPPanel.Title = 'HDOP';
                TruckHDOPPanel.FontSize = 14;
                TruckHDOPPanel.BackgroundColor = [0 0 0];
                TruckHDOPPanel.ForegroundColor = [1 1 1];
                TruckHDOPPanel.Position = [9 200 150 50];
            
                TruckHDOP = uilabel(TruckHDOPPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    TruckHDOP.FontSize = 13;
    TruckHDOP.Position = [5 1 350 30 ];
    TruckHDOP.Text = 'Loading';
    TruckHDOP.FontColor = [1 1 1];
   
    
    TruckPDOPPanel = uipanel(CarParameterPanel);
    TruckPDOPPanel.Title = 'PDOP';
    TruckPDOPPanel.FontSize = 14;
    TruckPDOPPanel.BackgroundColor = [0 0 0];
    TruckPDOPPanel.ForegroundColor = [1 1 1];
    TruckPDOPPanel.Position = [165 200 150 50];
    
        TruckPDOP = uilabel(TruckPDOPPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    TruckPDOP.FontSize = 13;
    TruckPDOP.FontColor = [1 1 1];
    TruckPDOP.Position = [5 1 350 30 ];
    TruckPDOP.Text = 'Loading';
    
    TruckPerrorPanel = uipanel(CarParameterPanel);
    TruckPerrorPanel.Title = 'Position error(m)';
    TruckPerrorPanel.FontSize = 14;
    TruckPerrorPanel.BackgroundColor = [0 0 0];
    TruckPerrorPanel.ForegroundColor = [1 1 1];
    TruckPerrorPanel.Position = [9 145 150 50];
    

        Perror = uilabel(TruckPerrorPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    Perror.FontSize = 13;
    Perror.FontColor = [1 1 1];
    Perror.Position = [5 1 350 30 ];
    Perror.Text = 'Loading';
    
    
     TruckpositionLabel = uilabel(CarParameterPanel);
    TruckpositionLabel.FontSize = 16;
    TruckpositionLabel.FontColor = [1 1 1];
    TruckpositionLabel.Position = [9 100 250 50];
    TruckpositionLabel.Text = 'Receiver Position Information';
    
    
    TruckpositionLabelPanel = uipanel(CarParameterPanel);
    TruckpositionLabelPanel.Title = 'Receiver Position';
    TruckpositionLabelPanel.FontSize = 14;
    TruckpositionLabelPanel.BackgroundColor = [0 0 0];
    TruckpositionLabelPanel.ForegroundColor = [1 1 1];
    TruckpositionLabelPanel.Position = [9 60 306 50];
    
    Truckposition = uilabel(TruckpositionLabelPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    Truckposition.FontSize = 13;
    Truckposition.FontColor = [1 1 1];
    Truckposition.Position = [5 1 350 30 ];
    Truckposition.Text = 'Loading';


    % Create InformationPanel !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    InformationPanel = uipanel(f);
    InformationPanel.Title = 'Estimated Position';
    InformationPanel.FontSize = 16;
    InformationPanel.BackgroundColor = [0 0 0];
    InformationPanel.ForegroundColor = [1 1 1];
%     SystemParameterPanel.Position = [15 9 620 361];
    InformationPanel.Position = [1120 9 330 305];
    
    InformationDropDown= uidropdown(InformationPanel,'ValueChangedFcn',@(InformationDropDown,event) uavselection(InformationDropDown));
    InformationDropDown.Items = {'UAV1', 'UAV2','UAV3','UAV4','UAV5', 'GFAC'};
    InformationDropDown.FontSize = 14;
    InformationDropDown.FontColor = [1 1 1];
    InformationDropDown.FontWeight = 'bold';
    InformationDropDown.BackgroundColor = [0.2 0.2 0.2];
    InformationDropDown.Position = [80 258 120 20];
    InformationDropDown.Value = 'UAV1';

    PlatformPanel = uipanel(InformationPanel);
    PlatformPanel.Title = 'Platform';
    PlatformPanel.FontSize = 14;
    PlatformPanel.BackgroundColor = [0 0 0];
    PlatformPanel.ForegroundColor = [1 1 1];
    PlatformPanel.Position = [9 200 150 50];
    
        Platformlabel = uilabel(PlatformPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    Platformlabel.FontSize = 13;
    Platformlabel.FontColor = [1 1 1];
    Platformlabel.Position = [5 1 350 30 ];
    Platformlabel.Text = 'UAV';
    
    IDPanel = uipanel(InformationPanel);
    IDPanel.Title = 'ID';
    IDPanel.FontSize = 14;
    IDPanel.BackgroundColor = [0 0 0];
    IDPanel.ForegroundColor = [1 1 1];
    IDPanel.Position = [165 200 150 50];
    


         ARPSPanel = uipanel(InformationPanel);
    ARPSPanel.Title = 'ARPS';
    ARPSPanel.FontSize = 14;
    ARPSPanel.BackgroundColor = [0 0 0];
    ARPSPanel.ForegroundColor = [1 1 1];
    ARPSPanel.Position = [9 20 150 120];
    
        ARPSlabel = uilabel(ARPSPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    ARPSlabel.FontSize = 13;
    ARPSlabel.FontColor = [1 1 1];
    ARPSlabel.Position = [10 30 300 50 ];
    ARPSlabel.Text = 'loading';

     EARPSPanel = uipanel(InformationPanel);
    EARPSPanel.Title = 'E-ARPS';
    EARPSPanel.FontSize = 14;
    EARPSPanel.BackgroundColor = [0 0 0];
    EARPSPanel.ForegroundColor = [1 1 1];
    EARPSPanel.Position = [165 20 150 120];
    
        EARPSlabel = uilabel(EARPSPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    EARPSlabel.FontSize = 13;
    EARPSlabel.FontColor = [1 1 1];
    EARPSlabel.Position = [10 30 300 50 ];
    EARPSlabel.Text = 'loading';
 

        IDlabel = uilabel(IDPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    IDlabel.FontSize = 13;
    IDlabel.FontColor = [1 1 1];
    IDlabel.Position = [5 1 350 30 ];
    IDlabel.Text = '1';
    
%{

    LatitudePanel = uipanel(InformationPanel);
    LatitudePanel.Title = 'altitude(m)';
    LatitudePanel.FontSize = 14;
    LatitudePanel.BackgroundColor = [0 0 0];
    LatitudePanel.ForegroundColor = [1 1 1];
    LatitudePanel.Position = [9 145 150 50];
    
        Latitudelabel = uilabel(LatitudePanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    Latitudelabel.FontSize = 13;
    Latitudelabel.FontColor = [1 1 1];
    Latitudelabel.Position = [5 1 350 30 ];
    Latitudelabel.Text = 'NaN';


%{
    platform_temp = get(InformationDropDown,'value');
        if strcmp(platform_temp,'UAV1')
            IDlabel.Text = uav1z;
        elseif strcmp(platform_temp,'UAV2')
            IDlabel.Text = uav2z;
        elseif strcmp(platform_temp,'UAV3')
            IDlabel.Text = uav3z;
        elseif strcmp(platform_temp,'UAV4')
            IDlabel.Text = uav4z;
            elseif strcmp(platform_temp,'Truck')
            IDlabel.Text = '0';
        
        end
%}



    LongitudePanel = uipanel(InformationPanel);
    LongitudePanel.Title = 'Longitude';
    LongitudePanel.FontSize = 14;
    LongitudePanel.BackgroundColor = [0 0 0];
    LongitudePanel.ForegroundColor = [1 1 1];
    LongitudePanel.Position = [165 145 150 50];
    
        Longitudelabel = uilabel(LongitudePanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    Longitudelabel.FontSize = 13;
    Longitudelabel.FontColor = [1 1 1];
    Longitudelabel.Position = [5 1 350 30 ];
    Longitudelabel.Text = 'NaN';


%}
    
    xyzpositionPanel = uipanel(InformationPanel);
    xyzpositionPanel.Title = 'Node Position';
    xyzpositionPanel.FontSize = 14;
    xyzpositionPanel.Position = [9 145 306 50];
    xyzpositionPanel.BackgroundColor = [0 0 0];
    xyzpositionPanel.ForegroundColor = [1 1 1];
    
    % 재혁 
    %{
    xyzpositionPanel = uipanel(InformationPanel);
    xyzpositionPanel.Title = 'x, y, z position';
    xyzpositionPanel.FontSize = 14;
    xyzpositionPanel.BackgroundColor = [0 0 0];
    xyzpositionPanel.ForegroundColor = [1 1 1];
    xyzpositionPanel.Position = [9 60 306 50];
    %}
    xyzpositionLabel = uilabel(xyzpositionPanel);
%     NumberofUAVDropDownLabel.HorizontalAlignment = 'right';
    xyzpositionLabel.FontSize = 13;
    xyzpositionLabel.FontColor = [1 1 1];
    xyzpositionLabel.Position = [5 1 350 30 ];
    xyzpositionLabel.Text = 'NaN';



     
    

%             % Create PositionofXaxisEditFieldLabel
%             PositionofXaxisEditFieldLabel = uilabel(CarParameterPanel);
%             PositionofXaxisEditFieldLabel.HorizontalAlignment = 'right';
%             PositionofXaxisEditFieldLabel.Position = [17 212 98 22];
%             PositionofXaxisEditFieldLabel.Text = 'Position of X axis';
% 
%             % Create PositionofXaxisEditField
%             PositionofXaxisEditField = uieditfield(CarParameterPanel, 'ValueChangedFcn',@(PositionofXaxisEditField,event) sel_xcar(PositionofXaxisEditField,retval7));
%             PositionofXaxisEditField.Position = [130 212 100 22];

%             % Create PositionofYaxisEditFieldLabel
%             PositionofYaxisEditFieldLabel = uilabel(CarParameterPanel);
%             PositionofYaxisEditFieldLabel.HorizontalAlignment = 'right';
%             PositionofYaxisEditFieldLabel.Position = [17 172 98 22];
%             PositionofYaxisEditFieldLabel.Text = 'Position of Y axis';
% 
%             % Create PositionofYaxisEditField
%             PositionofYaxisEditField = uieditfield(CarParameterPanel, 'ValueChangedFcn',@(PositionofYaxisEditField,event) sel_ycar(PositionofYaxisEditField,retval8));
%             PositionofYaxisEditField.Position = [130 172 100 22];
            
            
            
            % Create ResultPanel
            ResultPanel = uipanel(CarParameterPanel);
            ResultPanel.Title = 'Result';
            ResultPanel.Position = [1 10 320 140];
            
            UITable = uitable(ResultPanel);
            UITable.ColumnName = {'ARPS';'OP model'};
            UITable.RowName = {'HDOP';'VDOP';'P_error'};
            UITable.FontSize = 10;
            
            UITable.Position = [2 5 300 100];

%             % Create PositionErrorLabel
%             PositionErrorLabel = uilabel(ResultPanel);
%             PositionErrorLabel.Position = [13 38 85 22];
%             PositionErrorLabel.Text = 'Position Error :';
% 
%             % Create TimingErrorLabel
%              TimingErrorLabel = uilabel( ResultPanel);
%              TimingErrorLabel.Position = [20 13 78 22];
%              TimingErrorLabel.Text = 'Timing Error :';
% 
%             % Create XXDOPLabel
%              XXDOPLabel = uilabel( ResultPanel);
%              XXDOPLabel.Position = [40 89 58 22];
%              XXDOPLabel.Text = 'XY DOP :';
% 
%             % Create YYDOPLabel
%              YYDOPLabel = uilabel( ResultPanel);
%              YYDOPLabel.Position = [40 64 58 22];
%              YYDOPLabel.Text = 'Z DOP :';
% 
%             % Create PositionErrorANSLabel
%              PositionErrorANSLabel = uilabel( ResultPanel);
%              PositionErrorANSLabel.Position = [108 38 106 22];
%              PositionErrorANSLabel.Text = 'Position Error ANS';
% 
%             % Create TimingErrorANSLabel
%              TimingErrorANSLabel = uilabel( ResultPanel);
%              TimingErrorANSLabel.Position = [108 13 99 22];
%              TimingErrorANSLabel.Text = 'Timing Error ANS';
% 
%             % Create XXDOPANSLabel
%              XXDOPANSLabel = uilabel( ResultPanel);
%              XXDOPANSLabel.Position = [108 89 79 22];
%              XXDOPANSLabel.Text = 'XY DOP ANS';
% 
%             % Create YYDOPANSLabel
%              YYDOPANSLabel = uilabel( ResultPanel);
%              YYDOPANSLabel.Position = [108 64 79 22];
%              YYDOPANSLabel.Text = 'Z DOP ANS';
    TPButton = uibutton(CarParameterPanel, 'push',....
        'ButtonPushedFcn', @tp);
    TPButton.Text = 'Position coverage';
    TPButton.FontColor = [1 1 1];
    TPButton.BackgroundColor = [0.2 0.2 0.2];
    TPButton.Position = [100 20 120 22];
    
    if OPERATIONMODEDropDown.Value == 'E-ARPS'
        set(CenterUAVParameterPanel, 'Visible', 'On');
    end
    set(ResultPanel,'Visible','Off');
    





end

    function cb_start(source, event)
        if retval1 == 1
             [answer] = [retval1 retval2 retval3 retval4]
        elseif retval1 ==2
             [answer] = [retval1 retval2 retval3 retval4 retval5 retval6 retval7 retval8]
        end
        if retval7 || retval8
            %값 주는거 추가하기
            set(ResultPanel,'Visible','on');
        end
        if retval7 ==0 && retval8 ==0
                 uiresume(f);
        elseif (retval1==2 && retval3==15) && (retval7 ~=0 || retval8 ~=0)
            d = {round(ARPS_HDOP(retval7+1,retval8+1),2),round(RevTri_HDOP(retval7+1,retval8+1),2);round(ARPS_VDOP(retval7+1,retval8+1),2),round(RevTri_VDOP(retval7+1,retval8+1),2);round(ARPS_positionerror(retval7+1,retval8+1),2),RevTri_timingerror(retval7+1,retval8+1);ARPS_timingerror(retval7+1,retval8+1),RevTri_positionerror(retval7+1,retval8+1);};
            UITable.Data = d;
        elseif (retval1 ==2 && retval3==20) && (retval7 ~=0 || retval8 ~=0)
             d = {round(ARPS_HDOP(retval7+1,retval8+1),2),round(Tri_HDOP(retval7+1,retval8+1),2);round(ARPS_VDOP(retval7+1,retval8+1),2),round(Tri_VDOP(retval7+1,retval8+1),2);round(ARPS_positionerror(retval7+1,retval8+1),2),round(Tri_positionerror(retval7+1,retval8+1),2);ARPS_timingerror(retval7+1,retval8+1),Tri_timingerror(retval7+1,retval8+1);};
            UITable.Data = d;
        end
        %test
         
            
         uiresume(f);
         start_flag =1;
    end

    function cb_close(source, event)
        close all;
        btn_close =1;
        close(f);
    end

    function algo(source, event)
        fig3 = figure(3);
        if algo_temp ==0
            set(fig3, 'OuterPosition', [700 400 800 600]);
            images = imread('algo.jpg');
            imshow(images);
            algo_temp = algo_temp +1;
        else
            close(fig3);
            algo_temp = 0;
        end
    end

function tp(source, event)
    
        if tp_temp == 0
            TPButton.Text = 'Time coverage';
            figure(4);
            tpimage = imread('u4t.png');
            
            imshow(tpimage);
            fig4= figure(4);
        set(fig4, 'OuterPosition', [1064 35 440 370]);
            tp_temp = tp_temp +1;
        else
            TPButton.Text = 'Position coverage';
            figure(4);
            tpimage = imread('u4p.png');
            imshow(tpimage);
            fig4= figure(4);
        set(fig4, 'OuterPosition', [1064 35 440 370]);
            tp_temp = 0;

        end
        
    end

    function selectionmode(dd, value)
        retval = get(dd, 'value');
        if strcmp(retval,'ARPS')
            retval1 = 1;
            set(CenterUAVParameterPanel, 'Visible', 'Off');
        elseif strcmp(retval,'E-ARPS')
            retval1 = 2;
            set(CenterUAVParameterPanel, 'Visible', 'On');
        end
        fprintf("return value 1 = %d\n",retval1);
    end

    function selectionnuav(dd, value)
        retval = get(dd,'value');
        if retval == '4'
            retval2 = 1;
        elseif retval == '5'
            retval2 = 2;
        end
        fprintf("return value 2 = %d\n",retval2);
    end

    function sel_UAValti(dd, value)
        retval = get(dd,'value');
        if strcmp(retval,'5km')
            retval3 = 15;
        elseif strcmp(retval,'10km')
            retval3 = 10;
        elseif strcmp(retval,'15km')
            retval3 = 15;
        elseif strcmp(retval,'20km')
            retval3 = 20;
        end
        fprintf("return value 2 = %d\n",retval2);
    end
function sel_UAVradi(dd, value)
        retval = get(dd,'value');
        if strcmp(retval,'30km')
            retval4 = 30;
        elseif strcmp(retval,'50km')
            retval4 = 50;
        elseif strcmp(retval,'70km')
            retval4 = 70;
        end
        fprintf("return value 2 = %d\n",retval2);
end
    function sel_pUAValti(dd, value)
        retval = get(dd,'value');
        if strcmp(retval,'5km')
            retval5 = 5;
        elseif strcmp(retval,'10km')
            retval5 = 10;
        elseif strcmp(retval,'15km')
            retval5 = 15;
        elseif strcmp(retval,'20km')
            retval5 = 20;
        end
        fprintf("return value 2 = %d\n",retval2);
    end
function sel_pUAVradi(dd, value)
        retval = get(dd,'value');
        if strcmp(retval,'10km')
            retval6 = 10;
        elseif strcmp(retval,'50km')
            retval6 = 50;
        elseif strcmp(retval,'60km')
            retval6 = 60;
        end
        fprintf("return value 2 = %d\n",retval2);
end
function sel_xcar(dd, value)
        retval = get(dd,'value');
        retval7 = str2num(retval);
end

function sel_ycar(dd, value)
        retval = get(dd,'value');
        retval8 = str2num(retval);
end

uavflag = 9999;
 function uavselection(dd)
    platform_temp = get(dd,'value');
        if strcmp(platform_temp,'UAV1')
            Platformlabel.Text = 'UAV';
            IDlabel.Text = '1';
            uavflag = 1 ;

        elseif strcmp(platform_temp,'UAV2')
            Platformlabel.Text = 'UAV';
            IDlabel.Text = '2';
            uavflag = 2 ;

        elseif strcmp(platform_temp,'UAV3')
            Platformlabel.Text = 'UAV';
            IDlabel.Text = '3';
            uavflag = 3;

        elseif strcmp(platform_temp,'UAV4')
            Platformlabel.Text = 'UAV';
            IDlabel.Text = '4';
            uavflag = 4;

             elseif strcmp(platform_temp,'UAV5')
            Platformlabel.Text = 'UAV';
            IDlabel.Text = '5';
            uavflag = 5;

            elseif strcmp(platform_temp,'GFAC')
            Platformlabel.Text = 'GFAC';
            IDlabel.Text = '1';
            uavflag = 0;
          
        
        end
 end

uiwait(f);

%
%
%     box = dialog('Name', 'Config Simulator');
%
%
%
%     FNT14Label = uilabel('Parent', box, 'Style', 'label',...
%         'Position', [30 365 200 20], 'FontSize', 16,...
%         'String', 'FNT-14 Simulator');
%     % Add a text1
%     txt1 = uicontrol('Parent', box, 'Style','text',...
%         'Position',[20 365 200 20],...
%         'String','The Number of UAVs:');
%
%     % Add a Dropdown Menu1
%     popup1 = uicontrol('Parent', box, 'Style', 'popup',...
%            'String', {'4 Nodes','5 Nodes'},...
%            'Position', [250 365 150 20],...
%            'Callback', @run1);
%
%     % Add a text2
%     txt2 = uicontrol('Parent', box, 'Style','text',...
%         'Position',[0 275 200 20],...
%         'String','Operation Mode:');
%
%     % Add a Dropdown Menu2
%     popup2 = uicontrol('Parent', box, 'Style', 'popup',...
%            'String', {'ARPS','입체작전'},...
%            'Position', [250 275 150 20],...
%            'Callback', @run2);
%
%     % Add a text 3
%     txt3 = uicontrol('Parent', box, 'Style','text',...
%         'Position',[0 185 200 20],...
%         'String','Radius:');
%
%     % Add a Dropdown Menu3
%     popup3 = uicontrol('Parent', box, 'Style', 'popup',...
%            'String', {'5km','10km','15km', '20km'},...
%            'Position', [250 185 150 20],...
%            'Callback', @run3);
%
%     % Add a text 4
%     txt4 = uicontrol('Parent', box, 'Style','text',...
%         'Position',[0 85 200 20],...
%         'String','Altitude:');
%
%     % Add a Dropdown Menu4
%     popup4 = uicontrol('Parent', box, 'Style', 'popup',...
%            'String', {'50km'},...
%            'Position', [250 85 150 20],...
%            'Callback', @run4);
%
%
%     % Create push button
%     btn = uicontrol('Parent', box, 'Style', 'pushbutton', 'String', 'Run',...
%         'Position', [20 20 50 30],...
%         'Callback', @run);
%
%     btn2 = uicontrol('Parent', box, 'Style', 'pushbutton', 'String', 'Close',...
%         'Position', [100 20 50 30],...
%         'Callback', @run5);
%
%     set(txt1, 'FontSize', 15);
%     set(popup1, 'FontSize', 13);
%     set(txt2, 'FontSize', 15);
%     set(popup2, 'FontSize', 13);
%
%     set(txt3, 'FontSize', 15);
%     set(popup3, 'FontSize', 13);
%     set(txt4, 'FontSize', 15);
%     set(popup4, 'FontSize', 13);
%     set(btn, 'FontSize', 13);
%     set(btn2, 'FontSize', 13);
%
%     btn_close = 0;
%
% % Make figure visble after adding all components
% f.Visible = 'on';
% % This code uses dot notation to set properties.
%
%
%
%     function run1(source, event)
%         retval = get(source, 'Value');
%
%         if retval == 1
%             retval1 = 1;
%         elseif retval == 2
%             retval1 = 2;
%         elseif retval == 3
%             retval1 = 3;
%         end
%         btn.Value = 1;
%     end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%
%     function run2(source, event)
%         retval = get(source, 'value');
%
%         if retval == 1
%             retval2 = retval;
%         elseif retval == 2
%             retval2 = retval;
%         elseif retval == 3
%             retval2 = retval;
%         end
%         btn.Value = 1;
%     end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%
%     function run3(source, event)
%         retval = get(source, 'value');
%
%         if retval == 1
%             retval3 = 5*3;
%         elseif retval == 2
%             retval3 = 10*3;
%         elseif retval == 3
%             retval3 = 15*3;
%         elseif retval == 4
%             retval3 = 20*3;
%         end
%         btn.Value = 1;
%     end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%
%     function run4(source, event)
%         retval = get(source, 'value');
%
%         if retval == 1
%             retval4 = 20*1.5;
%         end
%     end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%
%     function run5(source, event)
%         btn_close =1;
%         close(box);
%     end
% uiwait(box);
%

end