%%%%%%%%%%% Simulator for ARPS Scheme %%%%%%%%%%

% 2013. 9. 30. ~ 
% Designed by Kyuman LEE

clear all
close all
clc

global avgDiff   

cnt = 0;

%% Define User Position in sample space

usrxyz = [70000 70000 25];

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
    [ref_row, ref_col] = size(refxyz);
    refid = 1:ref_row;

for time = 0:300:6000
    cnt = cnt+1;
      
%% Define Relay Node Position in sample space

    TraDia = 50*1000;                    % Diameter of relay node trajectory
    CRDia = 5*1000;                      % Diameter of center relay node
    TraCir = 2*pi*TraDia;                % Circumference of relay node trajectory
    CRCir = 2*pi*CRDia;                  % Circumference of center relay node
    relV = 60;                          % The velocity of relay node
    roundT = TraCir/relV;                % The time to round a trajectory once
    CroundT = CRCir/relV;
    AngV = 2*pi/roundT;                  % Angular speed of relay node
    CAngV = 2*pi/CroundT;                % Angular speed of center relay node
    NumRel = 3;                          % The number of relay node except for center node
    
    c_speed  = 2.99792458e+8;            % The speed of light

    for i = 1 : NumRel
        ang = AngV*(roundT/NumRel*(i-1)+time);
        rel(i,1) = -TraDia*cos(ang);
        rel(i,2) = TraDia*sin(ang);
        rel(i,3) = 10*1000;
    end

    crel = [CRDia*cos(CAngV*time) CRDia*sin(CAngV*time) 10*1000];       
    switch NumRel
        case 3
            relxyz = [rel(3,:); crel; rel(2,:); rel(1,:)];
        case 4
            relxyz = [rel(4,:); rel(3,:); crel; rel(2,:); rel(1,:)];
        case 5
            relxyz = [rel(4,:); rel(3,:); rel(5,:); crel; rel(2,:); rel(1,:)];
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
        if n == 3
            temp_prsum = prvec_rel;
        end
        % Reordering of Relay node according to Pseudorange
        % Position change according to propagation time
        pr_sort = sort(prvec_rel);
        for j = 1: ref_row
            order = find(prvec_rel == pr_sort(j));
            ref(j,:) = refxyz(order,:);
%             pos_ch = (pr_sort(j)-pr_sort(1))/c_speed*relV;
%             pr_rel(j) = pr_sort(j)+pos_ch; 
        end
         estrel(n,:) = ARPS_iter(relxyz(n,:), ref, pr_sort);
%          estrel(n,:) = ARPS(relxyz(n,:), refxyz, prvec_rel);
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

    prvec = genrng(2,usrxyz,relxyz,relid,0,[1 1 0 1 0]);
    avg_dis = ones(1,ref_row)*prvec(1);
    prvec_vir = temp_prsum + avg_dis;
%     pr_master = genrng(2,masterxyz,relxyz,relid,time,[1 1 0 1 0]);
%     estusr = double_diff(usrxyz, estrel, masterxyz, prvec, pr_master);
    estusr = olspos(prvec,estrel, usrxyz);
    virusr = olspos(prvec_vir, refxyz);
    
%     err(cnt,1:3) = ( xyz2enu(estusr(1:3), usrxyz) )';
    xyzerr(cnt,:) = usrxyz - estusr(1:3);
    terr(cnt) = estusr(4)/c_speed;  % true clk bias is zero
    v_terr(cnt) = virusr(4)/c_speed;
end

%% Calculate DOP 
for l = 1 : length(relid)
    dopvec_rel(l,:) = dops(refxyz,estrel(l,:));
end
dopvec_est = dops(estrel,usrxyz);
dopvec_usr = dops(relxyz,usrxyz)

%% Plot & Print result
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

DOP_rel = dopvec_rel
DOP_usr = dopvec_usr(1:7)

stdusrxyz = std(xyzerr);

HDRMS_mat = sqrt(stdusrxyz(1)^2+stdusrxyz(2)^2)
VDRMS_mat = sqrt(stdusrxyz(3)^2)
RMSE_mat(cnt) = sqrt(stdusrxyz(1)^2+stdusrxyz(2)^2+stdusrxyz(3)^2);