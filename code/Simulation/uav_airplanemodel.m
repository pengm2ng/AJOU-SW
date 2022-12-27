% Clean the variables
% close all;
% clear all;
% clc;

clc;
clear all;
% close all;
x_left1=0;

slot_temp = 0;
button_temp = 0;
trans_y=0;
graph_temp =0;
uav5_mode = 0;
cnt = 0;
global ARPS_HDOP ARPS_positionerror ARPS_timingerror ARPS_PDOP RevTri_HDOP RevTri_PDOP RevTri_timingerror RevTri_positionerror Tri_HDOP Tri_PDOP Tri_timingerror Tri_positionerror;
global btn_close fig4;
global BL_a BL_b BL_c;
global TruckHDOP TruckPDOP Perror Terror Truckposition
global UavXposition Uav2Xposition Uav3Xposition Uav4Xposition Uav5Xposition;
global xLim yLim;
global vector_position0 vector_position vector_position2 vector_position3 vector_position4 vector_position5 cop_flag
global uav1x uav2x uav3x uav4x uav5x uav1y uav2y uav3y uav4y uav5y uav1z uav2z uav3z uav4z uav5z
global xyzpositionLabel IDlabel uavflag
global start_flag
global ARPSlabel EARPSlabel
global f


cop_flag = 0;
start_flag=1;
%% MAT 파일 불러오기
flat_dop30 = load('flat_dop30.mat');
fdop30 = flat_dop30.DOP_usr;
flat_dop50 = load('flat_dop50.mat');
fdop50 = flat_dop50.DOP_usr;
flat_dop70 = load('flat_dop70.mat');
fdop70 = flat_dop70.DOP_usr;

threed_dop30 = load('threed_dop30.mat');
tdop30 = threed_dop30.DOP_usr;
threed_dop50 = load('threed_dop50.mat');
tdop50 = threed_dop50.DOP_usr;
threed_dop70 = load('threed_dop70.mat');
tdop70 = threed_dop70.DOP_usr;

fpositionerr30 = load('flat_positionerr30.mat');
fpose30 = fpositionerr30.Total_PositionErr;
fpositionerr50 = load('flat_positionerr50.mat');
fpose50 = fpositionerr50.Total_PositionErr;
fpositionerr70 = load('flat_positionerr70.mat');
fpose70 = fpositionerr70.Total_PositionErr;


tpositionerr30 = load('threed_positionerr30.mat');
tpose30 = tpositionerr30.Total_PositionErr;
tpositionerr50 = load('threed_positionerr50.mat');
tpose50 = tpositionerr50.Total_PositionErr;
tpositionerr70 = load('threed_positionerr70.mat');
tpose70 = tpositionerr70.Total_PositionErr;

fuavposition30 = load('flat_uav_position30.mat');
fuavpos30 = fuavposition30.relxyz_temp;
fuavposition50 = load('flat_uav_position50.mat');
fuavpos50 = fuavposition50.relxyz_temp;
fuavposition70 = load('flat_uav_position70.mat');
fuavpos70 = fuavposition70.relxyz_temp;

tuavposition30 = load('threed_uav_position30.mat');
tuavpos30 = tuavposition30.relxyz_temp;
tuavposition50 = load('threed_uav_position50.mat');
tuavpos50 = tuavposition50.relxyz_temp;
tuavposition70 = load('threed_uav_position70.mat');
tuavpos70 = tuavposition70.relxyz_temp;

truckposition = load('truck_position.mat');
truckPos = truckposition.usrxyz_temp;


[ARPS_HDOP ARPS_positionerror ARPS_timingerror ARPS_PDOP RevTri_HDOP RevTri_PDOP RevTri_timingerror RevTri_positionerror Tri_HDOP Tri_PDOP Tri_timingerror Tri_positionerror] = addresults();
fig1 = figure(1) ;                   %figure handle 생성
set(fig1, 'OuterPosition', [1492 392 470 688]);  %figure위치 및 크기 지정
world=vrworld('addDMZ.wrl', 'new');
open(world);
fig=vrfigure(world,[0 0 1500 630]);
set(fig, 'Viewpoint', 'Top View');
fig2= figure(1);

  

while 1

    [answer] = dropdownex();
   
    if start_flag == 1
        cnt = 0;
        graph_temp = 0;
       clf('reset')
% retval3 = answer(3) = 고도
% answer(4) = 반지름


frame_t = 10;
% Define the parameter t
n=30;
distance = 1: 155;
t=0:0.05:100;
temp =0;
did =20;
if answer(1) ==2
    frame_t = 10000000;
end
% UI 생성
xLim = 300;
yLim = 100;
% answer = [1 2 40 30]
% answer = cell2mat(answer);
%number of node
flag1 = answer(1);
%mode
flag2 = answer(2);
%radius
a= answer(4);
%altitude
b= answer(3);
% if(cancel)
%  return;
% end
center_uav_height = 0;
% Define a and b
% a=40;    % 회전 반경(3~5 대의 UAV)
if flag1 ==2
    ans6=answer(6);   % 회전 반경(1대의 UAV)

    center_uav_height = answer(5);

end
frame = 15;

% b=30;    %고도
% a3 = 4;
% b3 = 1;
% a4 = 5;
% b4 = 1;


%[1492 0 470 1080]
% fig2 = figure(2);
% set(fig2, 'OuterPosition', [400 100 400 300]);
% set(fig2, 'name', ['   ' 'Y축' ]);
% fig3 = figure(3);
% set(fig3, 'OuterPosition', [800 100 400 300]);
% set(fig3, 'name', ['   ' 'Z축' ]);

% fig2 = figure(2);
% set(fig2, 'OuterPosition', [1492 32 470 368]);

% Access the 3D World from MATLAB

% airpln=vrnode(world, 'Plane');
% airpln2=vrnode(world, 'Plane2');
% airpln3=vrnode(world, 'Plane3');
% airpln4=vrnode(world, 'Plane4');

%트럭 생성
truck=vrnode(world, 'Truck');

%GCS 생성 및 위치 지정
GCS_1 = vrnode(world, 'GCS_1');
GCS_2 = vrnode(world, 'GCS_2');
GCS_3 = vrnode(world, 'GCS_3');
GCS_4 = vrnode(world, 'GCS_4');
GCS_1.translation = [-40 0 -40];
GCS_2.translation = [-40 0 40];
GCS_3.translation = [40 0 -40];
GCS_4.translation = [40 0 40];

%radarhead 생성 및 위치 지정
radar_head = vrnode(world,'radar_head');
radar_head.translation = [100 9 -20];
radar_head.rotation = [0 0 1 0];

%redline 생성 및 위치 지정
redline = vrnode(world,'redline');
redline.translation = [70 0 0];

vrdrawnow;
%텍스트 생성
Text1 = vrnode(world, 'Text1');
picture.translation = [500 35 -50];
Text6 = vrnode(world, 'Text6');
shape_text2 = vrnode(Text6,'children','S','Shape');
Appear_text2 = vrnode(shape_text2,'appearance','appear','Appearance');
material_text2 = vrnode(Appear_text2,'material','mater','Material');
re_text2 = vrnode(shape_text2,'geometry','RT0','Text');
Text6.scale = [2.5 2.5 2.5];
Text7 = vrnode(world, 'Text7');
shape_text3 = vrnode(Text7,'children','S','Shape');
Appear_text3 = vrnode(shape_text3,'appearance','appear','Appearance');
material_text3 = vrnode(Appear_text3,'material','mater','Material');
re_text3 = vrnode(shape_text3,'geometry','RT0','Text');
Text7.scale = [2.5 2.5 2.5];

Text8 = vrnode(world, 'Text8');
shape_text4 = vrnode(Text8,'children','S','Shape');
Appear_text4 = vrnode(shape_text4,'appearance','appear','Appearance');
material_text4= vrnode(Appear_text4,'material','mater','Material');
re_text4 = vrnode(shape_text4,'geometry','RT0','Text');
Text8.scale = [2.5 2.5 2.5];

Text9 = vrnode(world, 'Text9');
shape_text5 = vrnode(Text9,'children','S','Shape');
Appear_text5 = vrnode(shape_text5,'appearance','appear','Appearance');
material_text5 = vrnode(Appear_text5,'material','mater','Material');
re_text5 = vrnode(shape_text5,'geometry','RT0','Text');
Text9.scale = [2.5 2.5 2.5];

Text10= vrnode(world, 'Text10');
shape_text6 = vrnode(Text10,'children','S','Shape');
Appear_text6 = vrnode(shape_text6,'appearance','appear','Appearance');
material_text6 = vrnode(Appear_text6,'material','mater','Material');
re_text6 = vrnode(shape_text6,'geometry','RT0','Text');
Text10.scale = [2.5 2.5 2.5];

% ilscoord2 = vrnode(ils2, 'coord', 'ILSCoordinate', 'Coordinate');


material_text2.diffuseColor = [0 0 0];
material_text3.diffuseColor = [0 0 0];
material_text4.diffuseColor = [0 0 0];
material_text5.diffuseColor = [0 0 0];
material_text6.diffuseColor = [0 0 0];
% s_text6 = vrnode(Text6,'children','appearance','Appearance');
% a_text6 = vrnode(s_text6, 'appearance','Mat','Material');
% a_text6.diffuseColor = [0.5 0.5 0.5];
text2 = vrnode(world, 'Text2');
text3 = vrnode(world, 'Text3');

text5 = vrnode(world, 'Text5');
% world.Text.string = {'Time = 0'};

% re_text2.string = 'please';
% 붉은 원 생성
circle = vrnode(world, 'Circle');




vector_z=[0 0 -1];
vector_z2 = [1 0 0];

%line 생성
s1 = vrnode(world, 'S', 'Shape');
ils1 = vrnode(s1, 'geometry', 'ILS', 'IndexedLineSet');
ilscoord1 = vrnode(ils1, 'coord', 'ILSCoordinate', 'Coordinate');
ilscolor1 = vrnode(ils1, 'color', 'ColorCoordinate', 'Color');
ils1.colorPerVertex = 'FALSE';



s2 = vrnode(world, 'S', 'Shape');
ils2 = vrnode(s2, 'geometry', 'ILS', 'IndexedLineSet');
ilscoord2 = vrnode(ils2, 'coord', 'ILSCoordinate', 'Coordinate');
ilscolor2 = vrnode(ils2, 'color', 'ColorCoordinate', 'Color');
ils2.colorPerVertex = 'FALSE';



s3 = vrnode(world, 'S', 'Shape');
ils3 = vrnode(s3, 'geometry', 'ILS', 'IndexedLineSet');
ilscoord3 = vrnode(ils3, 'coord', 'ILSCoordinate', 'Coordinate');
ilscolor3 = vrnode(ils3, 'color', 'ColorCoordinate', 'Color');
ils3.colorPerVertex = 'FALSE';



s4 = vrnode(world, 'S', 'Shape');
ils4 = vrnode(s4, 'geometry', 'ILS', 'IndexedLineSet');
ilscoord4 = vrnode(ils4, 'coord', 'ILSCoordinate', 'Coordinate');
ilscolor4 = vrnode(ils4, 'color', 'ColorCoordinate', 'Color');
ils4.colorPerVertex = 'FALSE';

a1 = vrnode(world, 'S', 'Shape');
tils1 = vrnode(a1, 'geometry', 'ILS', 'IndexedLineSet');
ilscoord1 = vrnode(tils1, 'coord', 'ILSCoordinate', 'Coordinate');
ilscolor1 = vrnode(tils1, 'color', 'ColorCoordinate', 'Color');
tils1.colorPerVertex = 'FALSE';





%UAV Position 사용할 TEXT 선언
UAV_Position1 = vrnode(world, 'UAV_position1');
shape_UP1 = vrnode(UAV_Position1,'children','S','Shape');
appearance_UP1 = vrnode(shape_UP1,'appearance','appear','Appearance');
material_UP1 = vrnode(appearance_UP1,'material','mater','Material');
text_UP1 = vrnode(shape_UP1,'geometry','RT1','Text');
material_UP1.diffuseColor = [0 0 0];

UAV_Position2 = vrnode(world, 'UAV_position2');
shape_UP2 = vrnode(UAV_Position2,'children','S','Shape');
appearance_UP2 = vrnode(shape_UP2,'appearance','appear','Appearance');
material_UP2 = vrnode(appearance_UP2,'material','mater','Material');
text_UP2 = vrnode(shape_UP2,'geometry','RT2','Text');
material_UP2.diffuseColor = [0 0 0];

UAV_Position3 = vrnode(world, 'UAV_position3');
shape_UP3 = vrnode(UAV_Position3,'children','S','Shape');
appearance_UP3 = vrnode(shape_UP3,'appearance','appear','Appearance');
material_UP3 = vrnode(appearance_UP3,'material','mater','Material');
text_UP3 = vrnode(shape_UP3,'geometry','RT3','Text');
material_UP3.diffuseColor = [0 0 0];

UAV_Position4 = vrnode(world, 'UAV_position4');
shape_UP4 = vrnode(UAV_Position4,'children','S','Shape');
appearance_UP4 = vrnode(shape_UP4,'appearance','appear','Appearance');
material_UP4 = vrnode(appearance_UP4,'material','mater','Material');
text_UP4 = vrnode(shape_UP4,'geometry','RT4','Text');
material_UP4.diffuseColor = [0 0 0];

if flag2 == 2
    UAV_Position5 = vrnode(world, 'UAV_position5');
    shape_UP5 = vrnode(UAV_Position5,'children','S','Shape');
    appearance_UP5 = vrnode(shape_UP5,'appearance','appear','Appearance');
    material_UP5 = vrnode(appearance_UP5,'material','mater','Material');
    text_UP5 = vrnode(shape_UP5,'geometry','RT2','Text');
    material_UP5.diffuseColor = [0 0 0];
end



switch(flag2)
    case 1
        % UAV 4대 생성
        airpln=vrnode(world, 'Plane');
        airpln2=vrnode(world, 'Plane2');
        airpln3=vrnode(world, 'Plane3');
        airpln4=vrnode(world, 'Plane4');
        airplnArry = [airpln airpln2 airpln3 airpln4];


    case 2
        % UAV 5대 생성
        airpln=vrnode(world, 'Plane');
        airpln2=vrnode(world, 'Plane2');
        airpln3=vrnode(world, 'Plane3');
        airpln4=vrnode(world, 'Plane4');
        airpln5=vrnode(world, 'Plane5');
        airplnArry = [airpln airpln2 airpln3 airpln4 airpln5];

    case 3
        % UAV 6대 생성
        airpln=vrnode(world, 'Plane');
        airpln2=vrnode(world, 'Plane2');
        airpln3=vrnode(world, 'Plane3');
        airpln4=vrnode(world, 'Plane4');
        airpln5=vrnode(world, 'Plane5');
        airpln6=vrnode(world, 'Plane6');
        airplnArry = [airpln airpln2 airpln3 airpln4 airpln5 airpln6];
end


position_x = 0;
position_y = 0;

if length(answer) >=7
    position_x = answer(7);
    position_y = answer(8);
end

Text6.translation = [35 70 80];
Text7.translation = [29 67 80];
Text8.translation = [29 64 80];
Text9.translation = [29 61 80];
Text10.translation = [29 58 80];

pos_cnt = 1;
k=2;

    for i=1:length(t)
        if btn_close ==1
            close(world);
            close all;
            btn_close = 1;
            button_temp = 0;
            uav_airplanemodel;
        end

        if button_temp == 0
            button_temp = 1;
        end


        temp = temp+1;

        %     vector_position0 = [t(i) -3 -30];                      %직선 트럭의 위치, 속도, 몸체 방향
        vector_position0 = [t(i)+position_x 3 0+position_y];
        truck.translation = vector_position0;
        Text1.translation = vector_position0 + [0 4 0];

        vector_velocity0 = [1 0 0];
        vector0 = cross(vector_velocity0, vector_z2);
        vector0 = vector0/norm(vector0);
        theta0 = acos(dot(vector_velocity0, vector_z2)/(norm(vector_velocity0)*norm(vector_z2)));
        truck.rotation = [vector0 -theta0];


        if (flag2 == 2)
            switch(flag1)
                case 1 % 5대 UAV 모두 수평 원
                    vector_position = [a*cos(t(i)/frame)  b a*sin(t(i)/frame)];  %선회 비행
                    vector_position2 = [a*cos(t(i)/frame+pi/5*2)  b a*sin(t(i)/frame+pi/5*2)];
                    vector_position3 = [a*cos(t(i)/frame+pi/5*4)  b a*sin(t(i)/frame+pi/5*4)];
                    vector_position4 = [a*cos(t(i)/frame+pi/5*6)  b a*sin(t(i)/frame+pi/5*6)];
                    vector_position5 = [a*cos(t(i)/frame+pi/5*8)  b a*sin(t(i)/frame+pi/5*8)];
                    
                    radar_head.rotation = [0 1 0 (pi/20)*t(i)];

                    redline.translation = [70+t(i) 0-t(i)/20 0];

                    if rem(i,k) == 0

                        if answer(4) == 30
                            uav1x = fuavpos30(pos_cnt,1);
                            uav1y = fuavpos30(pos_cnt,2);
                            uav1z = fuavpos30(pos_cnt,3);

                            uav2x = fuavpos30(pos_cnt,4);
                            uav2y = fuavpos30(pos_cnt,5);
                            uav2z = fuavpos30(pos_cnt,6);

                            uav3x = fuavpos30(pos_cnt,7);
                            uav3y = fuavpos30(pos_cnt,8);
                            uav3z = fuavpos30(pos_cnt,9);

                            uav4x = fuavpos30(pos_cnt,10);
                            uav4y = fuavpos30(pos_cnt,11);
                            uav4z = fuavpos30(pos_cnt,12);

                            uav5x = fuavpos30(pos_cnt,13);
                            uav5y = fuavpos30(pos_cnt,14);
                            uav5z = fuavpos30(pos_cnt,15);

                        elseif answer(4) == 50

                            uav1x = fuavpos50(pos_cnt,1);
                            uav1y = fuavpos50(pos_cnt,2);
                            uav1z = fuavpos50(pos_cnt,3);

                            uav2x = fuavpos50(pos_cnt,4);
                            uav2y = fuavpos50(pos_cnt,5);
                            uav2z = fuavpos50(pos_cnt,6);

                            uav3x = fuavpos50(pos_cnt,7);
                            uav3y = fuavpos50(pos_cnt,8);
                            uav3z = fuavpos50(pos_cnt,9);

                            uav4x = fuavpos50(pos_cnt,10);
                            uav4y = fuavpos50(pos_cnt,11);
                            uav4z = fuavpos50(pos_cnt,12);

                            uav5x = fuavpos50(pos_cnt,13);
                            uav5y = fuavpos50(pos_cnt,14);
                            uav5z = fuavpos50(pos_cnt,15);


                        elseif answer(4) == 70


                            uav1x = fuavpos70(pos_cnt,1);
                            uav1y = fuavpos70(pos_cnt,2);
                            uav1z = fuavpos70(pos_cnt,3);

                            uav2x = fuavpos70(pos_cnt,4);
                            uav2y = fuavpos70(pos_cnt,5);
                            uav2z = fuavpos70(pos_cnt,6);

                            uav3x = fuavpos70(pos_cnt,7);
                            uav3y = fuavpos70(pos_cnt,8);
                            uav3z = fuavpos70(pos_cnt,9);

                            uav4x = fuavpos70(pos_cnt,10);
                            uav4y = fuavpos70(pos_cnt,11);
                            uav4z = fuavpos70(pos_cnt,12);

                            uav5x = fuavpos70(pos_cnt,13);
                            uav5y = fuavpos70(pos_cnt,14);
                            uav5z = fuavpos70(pos_cnt,15);
                        end

                    end


                    vector_velocity = [-a*sin(t(i)/frame)  0 a*cos(t(i)/frame)];
                    vector_velocity2 = [-a*sin(t(i)/frame+pi/5*2)  0 a*cos(t(i)/frame+pi/5*2)];
                    vector_velocity3 = [-a*sin(t(i)/frame+pi/5*4) 0 a*cos(t(i)/frame+pi/5*4)];
                    vector_velocity4 = [-a*sin(t(i)/frame+pi/5*6) 0 a*cos(t(i)/frame+pi/5*6)];
                    vector_velocity5 = [-a*sin(t(i)/frame+pi/5*8) 0 a*cos(t(i)/frame+pi/5*8)];

                    cop_flag = 1;
                case 2 % 4대가 사각형 대형, 1대가 중앙에 위치한후 적진으로 truck과 같은 속도로 이동

                    % vector_position 조정하여 제트기 위치 조정
                    vector_position = [10*cos(t(i)/frame)+t(i)  b 10*sin(t(i)/frame)]; % UAV가 회전하면서 따라감
                    %vector_position = [t(i) b 0] % UAV가 일직선으로 따라감
                    vector_position2 = [a*cos(t(i)/frame)+t(i)  b a*sin(t(i)/frame)];
                    vector_position3 = [a*cos(t(i)/frame+pi/2)+t(i)  b a*sin(t(i)/frame+pi/2)];
                    vector_position4 = [a*cos(t(i)/frame+pi)+t(i)  b a*sin(t(i)/frame+pi)];
                    vector_position5 = [a*cos(t(i)/frame+pi/2*3)+t(i)  b a*sin(t(i)/frame+pi/2*3)];

                      radar_head.rotation = [0 1 0 (pi/20)*t(i)];

                      redline.translation = [70+t(i) 0-t(i)/20 0];
                    
                    if rem(i,k) == 0
                        if answer(4) == 30
                            uav1x = tuavpos30(pos_cnt,1);
                            uav1y = tuavpos30(pos_cnt,2);
                            uav1z = tuavpos30(pos_cnt,3);

                            uav2x = tuavpos30(pos_cnt,4);
                            uav2y = tuavpos30(pos_cnt,5);
                            uav2z = tuavpos30(pos_cnt,6);

                            uav3x = tuavpos30(pos_cnt,7);
                            uav3y = tuavpos30(pos_cnt,8);
                            uav3z = tuavpos30(pos_cnt,9);

                            uav4x = tuavpos30(pos_cnt,10);
                            uav4y = tuavpos30(pos_cnt,11);
                            uav4z = tuavpos30(pos_cnt,12);

                            uav5x = tuavpos30(pos_cnt,13);
                            uav5y = tuavpos30(pos_cnt,14);
                            uav5z = tuavpos30(pos_cnt,15);

                        elseif answer(4) == 50

                            uav1x = tuavpos50(pos_cnt,1);
                            uav1y = tuavpos50(pos_cnt,2);
                            uav1z = tuavpos50(pos_cnt,3);

                            uav2x = tuavpos50(pos_cnt,4);
                            uav2y = tuavpos50(pos_cnt,5);
                            uav2z = tuavpos50(pos_cnt,6);

                            uav3x = tuavpos50(pos_cnt,7);
                            uav3y = tuavpos50(pos_cnt,8);
                            uav3z = tuavpos50(pos_cnt,9);

                            uav4x = tuavpos50(pos_cnt,10);
                            uav4y = tuavpos50(pos_cnt,11);
                            uav4z = tuavpos50(pos_cnt,12);

                            uav5x = tuavpos50(pos_cnt,13);
                            uav5y = tuavpos50(pos_cnt,14);
                            uav5z = tuavpos50(pos_cnt,15);


                        elseif answer(4) == 70

                            uav1x = fuavpos70(pos_cnt,1);
                            uav1y = fuavpos70(pos_cnt,2);
                            uav1z = fuavpos70(pos_cnt,3);

                            uav2x = fuavpos70(pos_cnt,4);
                            uav2y = fuavpos70(pos_cnt,5);
                            uav2z = fuavpos70(pos_cnt,6);

                            uav3x = fuavpos70(pos_cnt,7);
                            uav3y = fuavpos70(pos_cnt,8);
                            uav3z = fuavpos70(pos_cnt,9);

                            uav4x = fuavpos70(pos_cnt,10);
                            uav4y = fuavpos70(pos_cnt,11);
                            uav4z = fuavpos70(pos_cnt,12);

                            uav5x = tuavpos70(pos_cnt,13);
                            uav5y = tuavpos70(pos_cnt,14);
                            uav5z = tuavpos70(pos_cnt,15);
                        end
                    end

                    %{
                % vector_velocity는 아직 잘모르겠음.
                vector_velocity = [1-a*sin(t(i)/frame+pi/2) 0 -a*cos(t(i)/frame+pi/2*3)];
                vector_velocity2 = [1-a*sin(t(i)/frame) 0 -a*cos(t(i)/frame+pi)];
                vector_velocity3 = [1-a*sin(t(i)/frame+pi/2) 0 a*cos(t(i)/frame+pi/2)];
                vector_velocity4 = [1-a*sin(t(i)/frame+pi) 0 a*cos(t(i)/frame+pi)];
                vector_velocity5 = [1-a*sin(t(i)/frame+pi/2*3) 0 a*cos(t(i)/frame+pi/2*3)];
                    %}
                    vector_velocity = [-sin(t(i)/frame) 0 cos(t(i)/frame)]; %UAV가 회전하면서 따라갈 때
                    %vector_velocity = [1 0 0]; %UAV가 일직선으로 따라갈 때
                    vector_velocity2 = [-sin(t(i)/frame) 0 -cos(t(i)/frame+pi)];
                    vector_velocity3 = [-sin(t(i)/frame+pi/2) 0 cos(t(i)/frame+pi/2)];
                    vector_velocity4 = [-sin(t(i)/frame+pi) 0 cos(t(i)/frame+pi)];
                    vector_velocity5 = [-sin(t(i)/frame+pi/2*3) 0 cos(t(i)/frame+pi/2*3)];

                    cop_flag = 1;

            end


            % Translation setting for the Plane node
            airpln.translation = vector_position;
            airpln2.translation = vector_position2;
            airpln3.translation = vector_position3;
            airpln4.translation = vector_position4;
            airpln5.translation = vector_position5;


            %test
            UavXposition.Text = sprintf('x : %0.2f y : %0.2f z : %0.2f',airpln.translation(1),airpln.translation(3),airpln.translation(2));
            UavXposition.FontColor = [1 0 0];
            Uav2Xposition.Text = sprintf('x : %0.2f y : %0.2f z : %0.2f',airpln2.translation(1),airpln2.translation(3),airpln2.translation(2));
            Uav2Xposition.FontColor = [1 0 0];
            Uav3Xposition.Text = sprintf('x : %0.2f y : %0.2f z : %0.2f',airpln3.translation(1),airpln3.translation(3),airpln3.translation(2));
            Uav3Xposition.FontColor = [1 0 0];
            Uav4Xposition.Text = sprintf('x : %0.2f y : %0.2f z : %0.2f',airpln4.translation(1),airpln4.translation(3),airpln4.translation(2));
            Uav4Xposition.FontColor = [1 0 0];
            Uav5Xposition.Text = sprintf('x : %0.2f y : %0.2f z : %0.2f',airpln5.translation(1),airpln5.translation(3),airpln5.translation(2));
            Uav5Xposition.FontColor = [1 0 0];


            if rem(i,k) == 0
                pos_cnt = pos_cnt + 1;
                if answer(4) == 30
                    if uavflag == 1
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',uav1x/1000, uav1y/1000, uav1z/1000);
                        xyzpositionLabel.FontColor = [1 0 0];
                    elseif uavflag == 2
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',uav2x/1000, uav2y/1000, uav2z/1000);
                        xyzpositionLabel.FontColor = [1 0 0];
                    elseif uavflag == 3
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',uav3x/1000, uav3y/1000, uav3z/1000);
                        xyzpositionLabel.FontColor = [1 0 0];
                    elseif uavflag == 4

                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',uav4x/1000, uav4y/1000, uav4z/1000);
                        xyzpositionLabel.FontColor = [1 0 0];
                    elseif uavflag == 5
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',uav5x/1000, uav5y/1000, uav5z/1000);
                        xyzpositionLabel.FontColor = [1 0 0];
                    elseif uavflag == 0
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',truckPos(pos_cnt,1)/1000, 0.00, 0.00);
                        xyzpositionLabel.FontColor = [1 0 0];
                    end

                elseif answer(4) == 50
                    if uavflag == 1
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',uav1x/1000, uav1y/1000, uav1z/1000);
                        xyzpositionLabel.FontColor = [1 0 0];
                    elseif uavflag == 2
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',uav2x/1000, uav2y/1000, uav2z/1000);
                        xyzpositionLabel.FontColor = [1 0 0];
                    elseif uavflag == 3
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',uav3x/1000, uav3y/1000, uav3z/1000);
                        xyzpositionLabel.FontColor = [1 0 0];
                    elseif uavflag == 4

                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',uav4x/1000, uav4y/1000, uav4z/1000);
                        xyzpositionLabel.FontColor = [1 0 0];
                    elseif uavflag == 5
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',uav5x/1000, uav5y/1000, uav5z/1000);
                        xyzpositionLabel.FontColor = [1 0 0];
                    elseif uavflag == 0
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',truckPos(pos_cnt,1)/1000, 0.00, 0.00);
                        xyzpositionLabel.FontColor = [1 0 0];
                    end


                elseif answer(4) == 70
                    if uavflag == 1
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',uav1x/1000, uav1y/1000, uav1z/1000);
                        xyzpositionLabel.FontColor = [1 0 0];
                    elseif uavflag == 2
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',uav2x/1000, uav2y/1000, uav2z/1000);
                        xyzpositionLabel.FontColor = [1 0 0];
                    elseif uavflag == 3
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',uav3x/1000, uav3y/1000, uav3z/1000);
                        xyzpositionLabel.FontColor = [1 0 0];
                    elseif uavflag == 4
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',uav4x/1000, uav4y/1000, uav4z/1000);
                        xyzpositionLabel.FontColor = [1 0 0];
                    elseif uavflag == 5
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',uav5x/1000, uav5y/1000, uav5z/1000);
                        xyzpositionLabel.FontColor = [1 0 0];
                    elseif uavflag == 0
                        xyzpositionLabel.Text = sprintf('x : %0.3f y : %0.3f z : %0.3f',truckPos(pos_cnt,1)/1000, 0.00, 0.00);
                        xyzpositionLabel.FontColor = [1 0 0];


                    end
                end
                pos_cnt = pos_cnt + 1;
            end






            % Compute the cross product and the amount of rotation theta
            vector = cross(vector_velocity, vector_z);
            vector2 = cross(vector_velocity2, vector_z);
            vector3 = cross(vector_velocity3, vector_z);
            vector4 = cross(vector_velocity4, vector_z);
            vector5 = cross(vector_velocity5, vector_z);

            vector = vector/norm(vector);
            vector2 = vector2/norm(vector2);
            vector3 = vector3/norm(vector3);
            vector4 = vector4/norm(vector4);
            vector5 = vector5/norm(vector5);

            theta = acos(dot(vector_velocity, vector_z)/(norm(vector_velocity)*norm(vector_z)));
            theta2 = acos(dot(vector_velocity2, vector_z)/(norm(vector_velocity2)*norm(vector_z)));
            theta3 = acos(dot(vector_velocity3, vector_z)/(norm(vector_velocity3)*norm(vector_z)));
            theta4 = acos(dot(vector_velocity4, vector_z)/(norm(vector_velocity4)*norm(vector_z)));
            theta5 = acos(dot(vector_velocity5, vector_z)/(norm(vector_velocity5)*norm(vector_z)));

            % Rotation setting for the Plane node
            airpln.rotation = [vector -theta];
            airpln2.rotation = [vector2 -theta2];
            airpln3.rotation = [vector3 -theta3];
            airpln4.rotation = [vector4 -theta4];
            airpln5.rotation = [vector5, -theta5];

            % uav_center_x = 0;
            % uav_center_z = 0;
            % BL_a= vector_position0(1) - uav_center_x;
            % BL_b= vector_position0(3) - uav_center_z;
            % BL_c= answer(5)*sqrt(BL_a^2+BL_b^2)-(BL_a*target_x+BL_b*target_z);
            %% drawCOP cop를 띄우면 렉이 걸리는 문제점 발생
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
     

      
        if rem(i,20) == 0
       drawCOP(vector_position, vector_position2, vector_position3, vector_position4, vector_position5, vector_position0);
        end
     

            if i >= 624
                % circle.translation = truck.translation;
            end

            % Update the figure
            vrdrawnow;
            % 반복문이 i로 돌고 있는데 그래프를 t로 출력하였기에 전체 시간에 대한 그래프가 나왔던 것임.
            %test
            if i==frame_t


                draw1t4(airpln, airpln2, airpln3, airpln4, ils1, ils2,ils3,ils4,GCS_1,GCS_2,GCS_3,GCS_4);
                draw(truck, ils1,ils2,ils3,ils4, airpln, airpln2, airpln3, airpln4);


                a1 = vrnode(world, 'S', 'Shape');

                %pause(1)
                delete(UAV_Position1);
                delete(UAV_Position2);
                delete(UAV_Position3);
                delete(UAV_Position4);
            end


        end

        %% 그래프 관련 코드


        if flag1==1
            if answer(4) == 30
                hdop = fdop30(:,4);
                pdop = fdop30(:,6);
                positionerr = fpose30;


            elseif answer(4) == 50
                hdop = fdop50(:,4);
                pdop = fdop50(:,6);
                positionerr = fpose50;



            elseif answer(4) == 70
                hdop = fdop70(:,4);
                pdop = fdop70(:,6);
                positionerr = fpose70;


            end
        elseif flag1==2
            if answer(4) == 30
                fhdop = fdop30(:,4);
                fpdop = fdop30(:,6);
                fpositionerr = fpose30;

                thdop = tdop30(:,4);
                tpdop = tdop30(:,6);
                tpositionerr = tpose30;



            elseif answer(4) == 50
                fhdop = fdop50(:,4);
                fpdop = fdop50(:,6);
                fpositionerr = fpose50;

                thdop = tdop50(:,4);
                tpdop = tdop50(:,6);
                tpositionerr = tpose50;


            elseif answer(4) == 70
                fhdop = fdop70(:,4);
                fpdop = fdop70(:,6);
                fpositionerr = fpose70;

                thdop = tdop70(:,4);
                tpdop = tdop70(:,6);
                tpositionerr = tpose70;

            end
        end








        if mod(temp,did) == 0
            graph_temp =graph_temp +1;

            if mod(graph_temp,4)==0
                if flag2 == 1

                    draw1t4(airpln, airpln2, airpln3, airpln4, ils1, ils2,ils3,ils4,GCS_1,GCS_2,GCS_3,GCS_4)
                    draw1t4(airpln, airpln2, airpln3, airpln4, ils1, ils2,ils3,ils4,GCS_1,GCS_2,GCS_3,GCS_4)

                    draw(truck, ils1,ils2,ils3,ils4, airpln, airpln2, airpln3, airpln4);
                    draw(truck, ils1,ils2,ils3,ils4, airpln, airpln2, airpln3, airpln4);

                    vrdrawnow

                else
                        ils1.coord.point = [airpln.translation; airpln2.translation; airpln3.translation; airpln4.translation; GCS_1.translation; airpln5.translation];
                    ils1.coordIndex = [0; 4; -1;1;4;-1;2;4;-1;3;4;-1;4;5;-1 ];
                    ils2.coord.point = [airpln.translation; airpln2.translation; airpln3.translation; airpln4.translation; GCS_2.translation; airpln5.translation];
                    ils2.coordIndex = [0; 4; -1;1;4;-1;2;4;-1;3;4;-1;4;5;-1 ];
                    ils3.coord.point = [airpln.translation; airpln2.translation; airpln3.translation; airpln4.translation; GCS_3.translation; airpln5.translation];
                    ils3.coordIndex = [0; 4; -1;1;4;-1;2;4;-1;3;4;-1;4;5;-1 ];
                    ils4.coord.point = [airpln.translation; airpln2.translation; airpln3.translation; airpln4.translation; GCS_4.translation; airpln5.translation];
                    ils4.coordIndex = [0; 4; -1;1;4;-1;2;4;-1;3;4;-1;4;5;-1 ];

                    ils1.color.color = [1 0.2 0.2;1 0.2 0.2;1 0.2 0.2;1 0.2 0.2;1 0.2 0.2];
                    ils2.color.color = [0 0 1;0 0 1;0 0 1;0 0 1;0 0 1];
                    ils3.color.color = [0 1 0;0 1 0;0 1 0;0 1 0;0 1 0];
                    ils4.color.color = [1 1 1;1 1 1;1 1 1;1 1 1;1 1 1];
                    vrdrawnow


                    pause(0.5);

                    ils1.coordIndex = [-1 ];
                    ils2.coordIndex = [-1 ];
                    ils3.coordIndex = [-1 ];
                    ils4.coordIndex = [-1 ];

                    vrdrawnow
                    

                    pause(1);
                     ils1.coord.point = [airpln.translation; airpln2.translation; airpln3.translation; airpln4.translation; GCS_1.translation; airpln5.translation];
                    ils1.coordIndex = [0; 4; -1;1;4;-1;2;4;-1;3;4;-1;4;5;-1 ];
                    ils2.coord.point = [airpln.translation; airpln2.translation; airpln3.translation; airpln4.translation; GCS_2.translation; airpln5.translation];
                    ils2.coordIndex = [0; 4; -1;1;4;-1;2;4;-1;3;4;-1;4;5;-1 ];
                    ils3.coord.point = [airpln.translation; airpln2.translation; airpln3.translation; airpln4.translation; GCS_3.translation; airpln5.translation];
                    ils3.coordIndex = [0; 4; -1;1;4;-1;2;4;-1;3;4;-1;4;5;-1 ];
                    ils4.coord.point = [airpln.translation; airpln2.translation; airpln3.translation; airpln4.translation; GCS_4.translation; airpln5.translation];
                    ils4.coordIndex = [0; 4; -1;1;4;-1;2;4;-1;3;4;-1;4;5;-1 ];

                    ils1.color.color = [1 0.2 0.2;1 0.2 0.2;1 0.2 0.2;1 0.2 0.2;1 0.2 0.2];
                    ils2.color.color = [0 0 1;0 0 1;0 0 1;0 0 1;0 0 1];
                    ils3.color.color = [0 1 0;0 1 0;0 1 0;0 1 0;0 1 0];
                    ils4.color.color = [1 1 1;1 1 1;1 1 1;1 1 1;1 1 1];
                    vrdrawnow

                    pause(0.5);
                    
                    ils1.coordIndex = [-1 ];
                    ils2.coordIndex = [-1 ];
                    ils3.coordIndex = [-1 ];
                    ils4.coordIndex = [-1 ];
                    vrdrawnow


                    pause(0.5);

                   
                    tils1.coord.point = [airpln.translation; airpln2.translation; airpln3.translation; airpln4.translation; truck.translation; airpln5.translation];
                    tils1.coordIndex = [0; 4; -1;1;4;-1;2;4;-1;3;4;-1;4;5;-1 ];
                    vrdrawnow
                    pause(0.5);

                    tils1.coordIndex = [-1 ];
                    vrdrawnow

                    pause(0.5);

                     tils1.coord.point = [airpln.translation; airpln2.translation; airpln3.translation; airpln4.translation; truck.translation; airpln5.translation];
                    tils1.coordIndex = [0; 4; -1;1;4;-1;2;4;-1;3;4;-1;4;5;-1 ];
                    vrdrawnow
                    pause(0.5);

                    tils1.coordIndex = [-1 ];
                    vrdrawnow




                end
            end


            if rem(graph_temp, 1) == 0
                cnt = cnt+1

                if flag1 == 1


                    ARPSlabel.Text = sprintf('HDOP : %0.2f\nPDOP : %0.2f\nPOS_ERROR : %0.2f',hdop(cnt),pdop(cnt),positionerr(cnt));


                elseif flag1==2

                    ARPSlabel.Text = sprintf('HDOP : %0.2f\nPDOP : %0.2f\nPOS_ERROR : %0.2f',fhdop(cnt),fpdop(cnt),fpositionerr(cnt));
                    EARPSlabel.Text = sprintf('HDOP : %0.2f\nPDOP : %0.2f\nPOS_ERROR : %0.2f',thdop(cnt),tpdop(cnt),tpositionerr(cnt));



                end



                if flag1==1
                    fig2 = figure(1);
                    
                    p1= plot(distance(graph_temp),hdop(cnt),'k*-','MarkerSize',3);
                    legend('ARPS');
                    subplot(3,1,1);

                    xlim([x_left1 100]);
                    ylim([0 10]);
                    xlabel('Distance(km)')
                    ylabel('HDOP')




                elseif flag1 ==2
                    fig2 = figure(1);
                    subplot(3,1,1);
                    p2= plot(distance(graph_temp),fhdop(cnt),'--k*',distance(graph_temp),thdop(cnt),'--r*');
                    legend('ARPS','E-ARPS');

                    xlim([x_left1 100]);
                    ylim([0 10]);
                    xlabel('Distance(km)')
                    ylabel('HDOP')
                end
                %         if flag == 1
                %             legend('Flat','Location','northwest');
                %         else
                %             legend([p1 p2],'Flat','3 Demension','Location','northwest');
                %         end

                hold on;

                if flag1==1
                    fig2= figure(1);
                    subplot(3,1,2);

                    p3=plot(distance(graph_temp),pdop(cnt),'--k*','MarkerSize',3);

                    %         if flag == 1
                    legend('ARPS');
                    %         end
                    xlim([x_left1 100]);
                    ylim([0 20]);
                    xlabel('Distance(km)')
                    ylabel('PDOP')

                elseif flag1 ==2
                    fig2 = figure(1);
                    subplot(3,1,2);


                    p4=plot(distance(graph_temp),fpdop(cnt),'--k*', distance(graph_temp),tpdop(cnt),'--r*' );
                    legend('ARPS','E-ARPS')
                    xlim([x_left1 100]);

                    ylim([0 20]);
                    xlabel('Distance(km)')
                    ylabel('PDOP')

                end

                hold on;
                if flag1==1
                    fig2 = figure(1);
                    subplot(3,1,3);

                    p5=plot(distance(graph_temp),positionerr(cnt),'--k*','MarkerSize',3);

                    %         if flag == 1
                    legend('ARPS');
                    %         end

                    xlim([x_left1 100]);
                    ylim([0 15]);
                    xlabel('Distance(km)')
                    ylabel('Position Error(m)')

                elseif flag1 ==2
                    fig2 = figure(1);
                    subplot(3,1,3);

                    p6= plot(distance(graph_temp),fpositionerr(cnt),'--k*', distance(graph_temp),tpositionerr(cnt),'--r*');
                    legend('ARPS','E-ARPS')
                    xlim([x_left1 100]);
                    ylim([0 15]);
                    xlabel('Distance(km)')
                    ylabel('Position Error(m)')
                end
                hold on;

                if flag1==1
                    TruckPDOP.Text = sprintf('%0.2f', pdop(cnt));
                    TruckHDOP.Text = sprintf('%0.2f',hdop(cnt));
                    Perror.Text = sprintf('%0.2f', positionerr(cnt));
                elseif flag1 ==2

                    TruckPDOP.Text = sprintf('%0.2f', tpdop(cnt));
                    TruckHDOP.Text = sprintf('%0.2f',thdop(cnt));
                    Perror.Text = sprintf('%0.2f', tpositionerr(cnt));
                end
            end
            %{

        hold on;
        if flag1 == 1
            TruckHDOP.Text = sprintf('%0.2f',ARPS_HDOP(graph_temp+1,trans_y));
            TruckPDOP.Text = sprintf('%0.2f',ARPS_PDOP(graph_temp+1,trans_y));
            Perror.Text = sprintf('%0.2f',ARPS_positionerror(graph_temp+1,trans_y));

        else 
            TruckHDOP.Text = sprintf('%0.2f',HDOP(graph_temp+1,trans_y));
            TruckPDOP.Text = sprintf('%0.2f',PDOP(graph_temp+1,trans_y));
            Perror.Text = sprintf('%0.2f',Positionerror(graph_temp+1,trans_y));

        end
            %}
            TruckHDOP.FontColor = [1 0 0];
            TruckPDOP.FontColor = [1 0 0];
            Perror.FontColor = [1 0 0];
            Terror.FontColor = [0 0 1];
            Truckposition.Text = sprintf('x : %0.2f y : %0.2f z : %0.2f',truck.translation(1),truck.translation(3),truck.translation(2));
            Truckposition.FontColor = [1 0 0];






        end

        drawnow
          
    end
    end

    start_flag = 0;
   

end
% Exit gracefully
% pause;
% close(world)
% delete(world)

