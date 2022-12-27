%%%%%%%%%% Simulation for TDMA MAC protocol embedded ARPS scheme %%%%%%%%%%

% 2015.9 10. ~
% Designed by Kyuman Lee

clear all
close all
clc

global avgDiff  

%% Time slot
time_slot = 7.8125*10^(-3);

%% Define User Position in sample space

usrxyz = [50000 50000 50];
origin = [1 3 10];

%% Define Reference Node Position in sample space

    RefDia = 55*1000;                   % Diameter of Reference distribution
    NumRef = 4;                         % The number of Reference except for center reference

    for i = 1 : NumRef-1
        ang = (2*pi/NumRef)*i;
        ref(i,1) = RefDia*cos(ang);
        ref(i,2) = RefDia*sin(ang);
        ref(i,3) = randi([1 50]);
    end

   switch NumRef
        case 4
            refxyz = [ref(1,:); ref(2,:); 0 0 10; ref(3,:)];
        case 5
            refxyz = [ref(1,:); ref(2,:);  0 0 10; ref(4,:); ref(3,:)];
        case 6
            refxyz = [ref(1,:); ref(5,:); ref(2,:); 0 0 10; ref(4,:); ref(3,:)];
        case 7
            refxyz = [ref(2,:); ref(1,:); ref(6,:); 0 0 10; ref(3,:); ref(4,:); ref(5,:)];
        case 8
            refxyz = [ref(2,:); ref(1,:); ref(7,:); 0 0 10; ref(3,:); ref(4,:); ref(5,:); ref(6,:)];
   end
   refid = 1:NumRef;

%% Relay setting
rel_vel = 100; % m/s
NumAir = 25;  % The number of total airborne nodes
NumRel = 5;   % The number of relays

cnt1 = 0;
err_cnt = 0;

for time = 0 : 5 : 1000
    cnt1 = cnt1+1;

%% Airborne node generation    
    init_pos = zeros(NumAir,3);

    quo_air = fix(NumAir/4);
    for i = 1 : quo_air
        init_pos(i,:) = [rand*90000 rand*90000 20000];
        init_pos(i+quo_air*1,:) = [-rand*90000 rand*90000 20000];
        init_pos(i+quo_air*2,:) = [rand*90000 -rand*90000 20000];
        init_pos(i+quo_air*3,:) = [-rand*90000 -rand*90000 20000];
    end
    if rem(NumAir, 4) ~= 0
        for i = 1 : rem(NumAir,4)
            Case = randi([1 4]);
                switch Case
                    case 1
                        init_pos(quo_air*4+i,:) = [rand*90000 rand*90000 20000];
                    case 2
                        init_pos(quo_air*4+i,:) = [-rand*90000 rand*90000 20000];
                    case 3
                        init_pos(quo_air*4+i,:) = [rand*90000 -rand*90000 20000];
                    case 4
                        init_pos(quo_air*4+i,:) = [-rand*90000 -rand*90000 20000];
                end
        end
    end
% πÊ«‚

%% Relay Selection
    % center relay
    distance = [];
    temp = abs(init_pos);
    for j = 1 : NumAir
        distance(j) =  sqrt(temp(j,1)^2+temp(j,2)^2);
    end
    min_dis = find(min(distance)==distance);
  
       while(1)
           quo = fix(NumAir/(NumRel+1));
           rel_index = min_dis;
   
            for j = 1 : quo-1
                rel_index = [rel_index randi([(NumRel+1)*(j-1)+1 (NumRel+1)*j])];
            end

            rel_index = [rel_index randi([(NumRel+1)*j+1 NumAir])];

             cnt = 0;
            for k = 1:length(rel_index)-1
                coll = find(rel_index(k) == rel_index(k+1:end)); 
                if length(coll)~= 0
                    cnt = cnt+1;
                end
            end
           % DOP check between relay and user(0,0,0) 
            relxyz = [];
            for m = 1 : NumRel
                relxyz = [relxyz; init_pos(rel_index(m),:)];
            end

            dop_check = dops(relxyz, usrxyz);
            if ((cnt == 0)&&(dop_check(6)<7))
                break
            end
    end

%     while(1)
%         index = randi([1 NumAir], 1, NumRel-1);
%         rel_index = [min_dis index];
%         cnt = 0;
%         for k = 1:length(index)
%             coll = find(rel_index(k) == rel_index(k+1:end)); 
%             if length(coll)~= 0
%                 cnt = cnt+1;
%             end
%         end
%        % DOP check between relay and user(0,0,0) 
%         relxyz = [];
%         for m = 1 : NumRel
%             relxyz = [relxyz; init_pos(rel_index(m),:)];
%         end
%         
%         dop_check = dops(relxyz, usrxyz);
%         if ((cnt == 0)&&(dop_check(6)<=8))
%             break
%         end
%     end
   
    relid = 1:NumRel;

   %% Estimate Relay Node Position
    estrel = [];
    theta = (randi([1 366],1,NumRel)-1)*pi/180;
  
    for n = 1:NumRel
        ref = [];
        % Relay node mobility model
        for i = 1 : NumRef
            rel(n,1) = relxyz(n,1)+cos(theta(n))*rel_vel*time_slot*(i-1);
            rel(n,2) = relxyz(n,2)+sin(theta(n))*rel_vel*time_slot*(i-1);
            rel(n,3) = 20000;
            temp_pr = genrng(1,rel(n,:),refxyz,refid,time,[1 1 0 1 0]);
       
            prvec_rel(i) = temp_pr(i);
        end

        % Reordering of Relay node according to Pseudorange
        % Position change according to propagation time
        pr_sort = sort(prvec_rel);
        for j = 1: NumRef
            order = find(prvec_rel == pr_sort(j));
            ref(j,:) = refxyz(order,:);
%             pos_ch = (pr_sort(j)-pr_sort(1))/c_speed*relV;
%             pr_rel(j) = pr_sort(j)+pos_ch; 
        end
         estrel(n,:) = ARPS(relxyz(n,:), ref, pr_sort);
%          estrel(n,:) = ARPS(relxyz(n,:), refxyz, prvec_rel);
    end


    for k = 1:NumRel
        temprel(cnt1, 3*k-2:3*k) = ( xyz2enu(estrel(k,:), relxyz(k,:)) )';
        tempxyz(cnt1, 3*k-2:3*k) = relxyz(k,:) - estrel(k,:);
   
    % Calculate Relay distance and Declare a global variable
        RF_RL(k,:) = genrng(1,relxyz(k,:),refxyz,refid,time,[0 0 0 0 0]);
        RF_ES(k,:) = genrng(1,estrel(k,:),refxyz,refid,time,[0 0 0 0 0]);    
    end

    % Difference between estimated relay range and original relay range
        avgDiff = mean((RF_ES - RF_RL),2);
    
    %% Plot Skyplot          
        pause(0.001)
        skyplot(relxyz,relid,usrxyz,0,0)  % Show the distribution of relay node
        hold on

    % fprintf('User Position Estimation');
    %% Calculate pseudo range and Estimate position, Check error

       % Relay node mobility model
        for i = 1 : NumRel
            relxyz(i,1) = relxyz(i,1)+cos(theta(i))*rel_vel*time_slot*(NumRef+i-1);
            relxyz(i,2) = relxyz(i,2)+sin(theta(i))*rel_vel*time_slot*(NumRef+i-1);
            tmp_prvec = genrng(2,usrxyz,relxyz,relid,time,[1 1 0 1 0]);
            
            prvec(i) = tmp_prvec(i);            
        end
    %     pr_master = genrng(2,masterxyz,relxyz,relid,time,[1 1 0 1 0]);
    %     estusr = double_diff(usrxyz, estrel, masterxyz, prvec, pr_master);
        estusr = olspos(prvec,estrel,usrxyz);%,usrxyz);
        
%         tol=1e-3;
%         initpos=[0 0 0 0];
%         [m,n]=size(initpos);
%         if m>n, estusr=initpos';else estusr=initpos;end
%         if max(size(estusr))<3,
%            error('must define at least 3 dimensions in INITPOS')
%         end
%         if max(size(estusr))<4,estusr=[estusr 0];end
%         numvis=max(size(relxyz));
%         beta=[1e9 1e9 1e9 1e9];
%         maxiter=10;
%         iter=0;
%         while ((iter<maxiter)&&(norm(beta)>tol)),
%             for N = 1:numvis,
%             pr0 = norm(relxyz(N,:)-estusr(1:3));
%             y(N,1) = prvec(N) - pr0 - estusr(4);
%             end,
%             H = hmat(relxyz,estusr(1:3));
%         %     beta = inv(transpose(H)*H)*transpose(H)*y;
%             beta = H\y;
%             estusr=estusr+beta'
%             iter=iter+1;
%         end
%         
%         stop
        
    %     err(cnt,1:3) = ( xyz2enu(estusr(1:3), usrxyz) )';
        xyzerr(cnt1,:) = usrxyz - estusr(1:3);
        % For the case that iteration process can not follow up a point
        if sqrt(xyzerr(cnt1,1)^2+xyzerr(cnt1,2)^2+xyzerr(cnt1,3)^2) >1000
            cnt1 = cnt1-1;
%             xyzerr(cnt1,:) = [0 0 0];
            err_cnt = err_cnt+1;
        end
    %     terr(i) = estusr(4);  % true clk bias is zero

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
plot(0:time/(cnt1-1):time,xyzerr(:,1))
ylabel('east error [m]')
title('Monte Carlo Results: Fixed Reference Locations')
grid on

subplot(3,1,2)
plot(0:time/(cnt1-1):time,xyzerr(:,2))
ylabel('north error [m]')
grid on

subplot(3,1,3)
plot(0:time/(cnt1-1):time,xyzerr(:,3))
ylabel('up error [m]')
xlabel('Monte Carlo trial number')
grid on

DOP_rel = dopvec_rel
DOP_usr = dopvec_usr(1:7)

stdusrxyz = std(xyzerr);

HDRMS_mat = sqrt(stdusrxyz(1)^2+stdusrxyz(2)^2)
VDRMS_mat = sqrt(stdusrxyz(3)^2)
