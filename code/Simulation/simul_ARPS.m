%%%%%%%%%%% Simulator for ARPS Scheme %%%%%%%%%%

% 2013. 9. 30. ~ 
% Designed by Kyuman LEE

clear all
close all
clc

global avgDiff   

cnt = 0;

x_po = 150 * 1000;  % move Relay and User

%% Define User Position in sample space

% usrxyz = [100000 100000 25];
% usrxyz = [0 10 25];
% usrxyz = [0 50000 25];
% usrxyz = [1 51 25];
% usrxyz = [x_po 1 25];
% usrxyz = [75000 120000 25];
% usrxyz = [150000 0 25];
% usrxyz = [0 0 0];
%% Define Reference Node Position in sample space

    RefDia = 60*1000;                   % Diameter of Reference distribution
    NumRef = 4;                         % The number of Reference except for center reference

    for i = 1 : NumRef
        ang = (2*pi/NumRef)*i;
        ref(i,1) = RefDia*cos(ang);
        ref(i,2) = RefDia*sin(ang);
        ref(i,3) = randi([1 50]);
    end

   switch NumRef
        case 3
            refxyz = [ref(1,:); ref(2,:); 0 0 10; ref(3,:)];
        case 4
            refxyz = [ref(1,:); ref(2,:);  0 0 10; ref(4,:); ref(3,:)];
        case 5
            refxyz = [ref(1,:); ref(5,:); ref(2,:); 0 0 10; ref(4,:); ref(3,:)];
        case 6
            refxyz = [ref(2,:); ref(1,:); ref(6,:); 0 0 10; ref(3,:); ref(4,:); ref(5,:)];
        case 7
            refxyz = [ref(2,:); ref(1,:); ref(7,:); 0 0 10; ref(3,:); ref(4,:); ref(5,:); ref(6,:)];
   end
%     switch NumRef
%         case 3
%             refxyz = [ref(1,:); ref(2,:); ref(3,:)];
%         case 4
%             refxyz = [ref(1,:); ref(2,:); ref(4,:); ref(3,:)];
%         case 5
%             refxyz = [ref(1,:); ref(5,:); ref(2,:); ref(4,:); ref(3,:)];
%         case 6
%             refxyz = [ref(2,:); ref(1,:); ref(6,:); ref(3,:); ref(4,:); ref(5,:)];
%         case 7
%             refxyz = [ref(2,:); ref(1,:); ref(7,:); ref(3,:); ref(4,:); ref(5,:); ref(6,:)];
%     end
    [ref_row, ref_col] = size(refxyz);
    refid = 1:ref_row;
    
    
    A_rpt_results=[];
    A_Position_Err=[];
    A_Rel_DOP=[];
    rpt_cnt = 1;
    dopvec_usr = []
    relxyz_temp=[]
for rpt = 0:10:100  
    cnt=0;
for time = 0:15:300
% for time = 0:15:3000
    cnt = cnt+1;

%% Define Relay Node Position in sample space

    TraDia = 30*1000;                    % Diameter of relay node trajectory
    CRDia = 2*1000;                      % Diameter of center relay node
    TraCir = 2*pi*TraDia;                % Circumference of relay node trajectory
    CRCir = 2*pi*CRDia;                  % Circumference of center relay node
    relV = 60;                          % The velocity of relay node
    CrelV = 60;                          % The velocity of center relay node
    roundT = TraCir/relV;                % The time to round a trajectory once
    CroundT = CRCir/CrelV;
    AngV = 2*pi/roundT;                  % Angular speed of relay node
    CAngV = 2*pi/CroundT;                % Angular speed of center relay node
    NumRel = 5;                          % The number of relay node except for center node
    relH = 20*1000;                      % The Height of relay
    
    x_Rel = rpt *1000;
%     x_Rel = 40*1000;
%     x_Rel = 100*1000;
%      x_Rel = 150*1000;
%      usrxyz = [(rpt *1000)*cos(2*pi/12) (rpt *1000)*sin(2*pi/12) 0];
     usrxyz = [rpt*1000 0 0];
    c_speed  = 2.99792458e+8;            % The speed of light
    
    for i = 1 : NumRel
        ang = AngV*(roundT/NumRel*(i-1)+time);
        rel(i,1) = -TraDia*cos(ang) ;
%         rel(i,1) = -TraDia*cos(ang);
        rel(i,2) = TraDia*sin(ang);
        rel(i,3) = relH+randi([1 100]);
    end

    crel = [CRDia*cos(CAngV*time)+x_Rel CRDia*sin(CAngV*time) relH+randi([1 10])];       
%     crel = [CRDia*cos(CAngV*time) CRDia*sin(CAngV*time) relH+randi([1 10])];       
    
    switch NumRel
        case 3
            relxyz = [rel(3,:); crel; rel(2,:); rel(1,:)];
            NumRel = NumRel+1;
        case 4
%             relxyz = [rel(4,:); rel(3,:); crel; rel(2,:); rel(1,:)];
            relxyz = [rel(4,:); rel(3,:); rel(2,:); rel(1,:)];
        case 5
            relxyz = [rel(4,:); rel(3,:); rel(5,:); rel(2,:); rel(1,:)];
            relxyz_temp = [relxyz_temp ; relxyz];

        case 6
            relxyz = [rel(5,:); rel(4,:); rel(3,:); crel; rel(6,:); rel(1,:); rel(2,:)];
        case 7
            relxyz = [rel(5,:); rel(4,:); rel(6,:); crel; rel(3,:); rel(7,:); rel(2,:); rel(1,:)];
    end
    
    [rel_row, rel_col] = size(relxyz);
    relid = 1:rel_row;
% fprintf('Pseudolite Position Estimation');
%% Estimate Relay Node Position
    estrel = [];
    
    for n = 1:rel_row
        ref = [];
        prvec_rel = genrng(1,relxyz(n,:),refxyz,refid,0,[1 1 0 1 0]);
%         prvec_rel = genrng(1,relxyz(n,:),refxyz,refid,0,[0 0 0 0 0]);
        % Reordering of Relay node according to Pseudorange
        % Position change according to propagation time
%         pr_sort = sort(prvec_rel);
%         for j = 1: ref_row
%             order = find(prvec_rel == pr_sort(j));
%             ref(j,:) = refxyz(order,:);
%             pos_ch = (pr_sort(j)-pr_sort(1))/c_speed*relV;
%             pr_rel(j) = pr_sort(j)+pos_ch; 
%         end
%          estrel(n,:) = ARPS_iter(relxyz(n,:), ref, pr_sort);
         estrel(n,:) = ARPS(relxyz(n,:), refxyz, prvec_rel);
    end


    for k = 1:rel_row
        temprel(cnt, 3*k-2:3*k) = ( xyz2enu(estrel(k,:), relxyz(k,:)) )';
        tempxyz(cnt, 3*k-2:3*k) = relxyz(k,:) - estrel(k,:);
   
    % Calculate Relay distance and Declare a global variable
        RF_RL(k,:) = genrng(1,relxyz(k,:),refxyz,refid,time,[0 0 0 0 0]);
        RF_ES(k,:) = genrng(1,estrel(k,:),refxyz,refid,time,[0 0 0 0 0]);    
    end
    
    % Difference between estimated relay range and original relay range
        avgDiff = mean((RF_ES - RF_RL),2);
    
%% Plot Skyplot          
%     pause(0.001)
%     skyplot(relxyz,relid,usrxyz,0,0)  % Show the distribution of relay node
%     hold on

% fprintf('User Position Estimation');
%% Calculate pseudo range and Estimate position, Check error


      prvec = genrng(2,usrxyz,estrel,relid,0,[1 1 0 1 0]);
%     prvec = genrng(2,usrxyz,relxyz,relid,0,[1 1 0 1 0]);
%     prvec = genrng(2,usrxyz,relxyz,relid,0,[0 0 0 0 0]);
%     pr_master = genrng(2,masterxyz,relxyz,relid,time,[1 1 0 1 0]);
%     estusr = double_diff(usrxyz, estrel, masterxyz, prvec, pr_master);
    estusr = olspos(prvec,estrel, usrxyz);
    
%     prvec_po = prvec+avgDiff';
    prvec_po = prvec-avgDiff';
    estusr_po = olspos(prvec_po,estrel, usrxyz);
%     err(cnt,1:3) = ( xyz2enu(estusr(1:3), usrxyz) )';
    xyzerr(cnt,:) = usrxyz - estusr(1:3);
    
    relerr = relxyz - estrel;
    for rel_k = 1 : NumRel
        rel_err(cnt,rel_k) = sqrt(relerr(rel_k,1)^2+relerr(rel_k,2)^2+relerr(rel_k,3)^2);
        temp_rel_dop = dops(refxyz, relxyz(rel_k,:)); 
        dopvec_rel_cnt(cnt,rel_k) = temp_rel_dop(7);
    end
    
    xyzerr_po(cnt,:) = usrxyz - estusr_po(1:3);
    
    Po_err(cnt) = sqrt(xyzerr(cnt,1)^2+xyzerr(cnt,2)^2+xyzerr(cnt,3)^2);
    Po_err2(cnt) = sqrt(xyzerr_po(cnt,1)^2+xyzerr_po(cnt,2)^2+xyzerr_po(cnt,3)^2);
    
    terr(cnt,1) = estusr(4)/c_speed;  % true clk bias is zero
    dopvec_usr_cnt(cnt,:) = dops(relxyz,usrxyz);
%     dopvec_usr_cnt2(cnt,:) = dops_select(relxyz,usrxyz);
%     diff_dop(cnt,:) = dopvec_usr_cnt(cnt,6)-dopvec_usr_cnt2(cnt,8);
end

%% Calculate DOP 
for l = 1 : length(relid)
    dopvec_rel(l,:) = dops(refxyz,estrel(l,:));
    A_Rel_DOP(rpt_cnt,l)=dopvec_rel(l,7);
end
dopvec_est = dops(estrel,usrxyz);
dopvec_usr = [dopvec_usr; dops(relxyz,usrxyz)];
dopvec_usr2 = dops(relxyz,usrxyz);
%% Plot & Print result
Mean_PositionErr = mean(Po_err);
% Var_PositionErr = var(Po_err)
Std_PositionErr = std(Po_err);
Mean_DOP = mean(dopvec_usr_cnt(:,7));
Std_DOP = std(dopvec_usr_cnt(:,7));
A_Mean_results = [Mean_PositionErr Std_PositionErr Mean_DOP Std_DOP];
A_Position_Err(rpt_cnt, :) = Mean_PositionErr;
A_rpt_results(rpt_cnt, :) = A_Mean_results;
rpt_cnt=rpt_cnt+1
end
%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2)
subplot(3,1,1)
plot(0:time/(cnt-1):time,xyzerr(:,1))
ylabel('east error [m]')
title('Monte Carlo Results: Fixed Reference Locations')
grid on

subplot(3,1,2)
plot(0:time/(cnt-1):time,xyzerr(:,2))
ylabel('north error [m]')
grid on

subplot(3,1,3)
plot(0:time/(cnt-1):time,xyzerr(:,3))
ylabel('up error [m]')
xlabel('Monte Carlo trial number')
grid on
%}

DOP_rel = dopvec_rel;
DOP_usr = dopvec_usr

stdusrxyz = std(xyzerr);

HDRMS_mat = sqrt(stdusrxyz(1)^2+stdusrxyz(2)^2);
VDRMS_mat = sqrt(stdusrxyz(3)^2);
RMSE_mat(cnt) = sqrt(stdusrxyz(1)^2+stdusrxyz(2)^2+stdusrxyz(3)^2);

