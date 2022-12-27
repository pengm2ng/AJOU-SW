%% dops 함수 호출해서 dop 띠우기

% 이거 실행시켜서 한번에 띄우기
clc;
clear;

n=30;
% t는 uav 속도
t=0:0.1:n;
t2 = 0: 0.12:n;
a=5;
b=10;

frame=15;
flat_tdop =[];
flat_gdop =[];
threed_tdop =[];
threed_gdop =[];
distance = 1:155;


% flat 일때
for i=1:length(t2)

%[x,z,y]

vector_position = [a*cos(t(i)*pi/frame) b a*sin(t(i)*pi/frame)]  %선회 비행
vector_position3 = [a*cos(t(i)*pi/frame+pi) b a*sin(t(i)*pi/frame+pi)];
vector_position4 = [a*cos(t(i)*pi/frame+pi/2) b a*sin(t(i)*pi/frame+pi/2)];
vector_position5 = [a*cos(t(i)*pi/frame+pi/2*3) b a*sin(t(i)*pi/frame+pi/2*3) ];
vector_position0 = [t2(i) 3 0]; 


%[x,y,z]
%{
vector_position = [a*cos(t(i)*pi/frame) a*sin(t(i)*pi/frame) b ];  %선회 비행
vector_position3 = [a*cos(t(i)*pi/frame+pi) a*sin(t(i)*pi/frame+pi) b ];
vector_position4 = [a*cos(t(i)*pi/frame+pi/2) a*sin(t(i)*pi/frame+pi/2) b ];
vector_position5 = [a*cos(t(i)*pi/frame+pi/2*3) a*sin(t(i)*pi/frame+pi/2*3) b ];
vector_position0 = [t2(i) 0 3]; 
%}


vector1=vector_position - vector_position0;
vector3=vector_position3 - vector_position0;
vector4=vector_position4 - vector_position0;
vector5=vector_position5 - vector_position0;
%svxyzmat =[vector1;vector3;vector4;vector5];

svxyzmat =[ vector_position; vector_position3; vector_position4; vector_position5];



dopvec = dops(svxyzmat, vector_position0);
%fprintf("flat\n")
%fprintf("%f %f %f %f %f %f %f\n",dopvec(1),dopvec(2),dopvec(3),dopvec(4),dopvec(5),dopvec(6),dopvec(7))
%fprintf("%.04f - tdop\n",dopvec(5))
%fprintf("%.04f - gdop\n\n",dopvec(7))
flat_vdop(i) =dopvec(3);
flat_hdop(i) = dopvec(4);
flat_tdop(i) = dopvec(5);
end


% 3d 일때
for i=1:length(t2)

%[x,z,y]

vector_position2_1 = [t(i) b 0 ];  %선회 비행
vector_position2_3 = [t(i)+20 b 0];
vector_position2_4 = [t(i)-10 b 15];
vector_position2_5 = [t(i)-10 b -15];
vector_position2_0 = [t2(i) 3 0]; 


%[x,y,z]
%{
vector_position2_1 = [t(i) 0 b ];  %선회 비행
vector_position2_3 = [t(i)+20 0 b ];
vector_position2_4 = [t(i)-10 15 b ];
vector_position2_5 = [t(i)-10 -15 b];
vector_position2_0 = [t(i) 0 3];
%}


vector2_1=vector_position2_1 - vector_position2_0;
vector2_3=vector_position2_3 - vector_position2_0;
vector2_4=vector_position2_4 - vector_position2_0;
vector2_5=vector_position2_5 - vector_position2_0;
%svxyzmat2 =[ vector2_1;vector2_3;vector2_4;vector2_5];


svxyzmat2 =[ vector_position2_1; vector_position2_3; vector_position2_4; vector_position2_5];



dopvec2 = dops(svxyzmat2, vector_position2_0);
%fprintf("3d\n")
%fprintf("%f %f %f %f %f %f %f\n",dopvec(1),dopvec(2),dopvec(3),dopvec(4),dopvec(5),dopvec(6),dopvec(7))
%fprintf("%.04f - tdop\n",dopvec(5))
%fprintf("%.04f - gdop\n\n",dopvec(7))
threed_vdop(i) =dopvec2(3);
threed_hdop(i) = dopvec2(4);
threed_tdop(i) = dopvec2(5);
end

figure(1)
plot(t2, flat_vdop ,t2,threed_vdop );
axis([0 20  0 5])
legend('flat','3d')
title("vdop");
figure(2)
plot(t2, flat_hdop ,t2,threed_hdop );
axis([0 20  0 20])
legend('flat','3d')
title("hdop")
figure(3)
plot(t2, flat_tdop ,t2,threed_tdop );
axis([0 20  0 20])
legend('flat','3d')
title("gdop")