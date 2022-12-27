%%%%%%%%%%% Simulator for Link-16 in TDMA protocol %%%%%%%%%%

% 2015. 11. 6. ~ 
% Designed by Kyuman LEE

clear all
close all
clc

current_time = fix(clock);
fprintf(  'Clock %2d/%2d %2d:%2d:%2d\n',  current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );

%% Time slot
time_slot = 7.8125*10^(-3);

%% Define Reference Node Position in sample space

    RefDia = 60*1000;                   % Diameter of Reference distribution
    NumGRU = 5;                         % The number of Reference except for center reference

    for i = 1 : NumGRU-1
        ang = (2*pi/(NumGRU-1))*i;
        gru(i,1) = RefDia*cos(ang);
        gru(i,2) = RefDia*sin(ang);
        gru(i,3) = randi([1 50]);
    end

   switch NumGRU
        case 4
            gruxyz = [gru(1,:); gru(2,:); 0 0 10; gru(3,:)];
        case 5
            gruxyz = [gru(1,:); gru(2,:);  0 0 10; gru(4,:); gru(3,:)];
   end
   
   gruid = 1:NumGRU;

%% Define simulation parameters
pos_start = 0;
pos_step = 10000;
pos_end = 50000;

bar = waitbar(0,'Running Monte Carlo Trials...  ');
cnt1 = 0;
%% Define User Position in sample space
for usr_y = 0 : pos_step : 0
    cnt1 = cnt1+1;
    cnt2 = 0;
        for usr_x = pos_start : pos_step : pos_end
        cnt2 = cnt2+1;
        
        suxyz = [usr_x usr_y 5000];
        
        cnt = 0;
        for time = 0:4:2000
            cnt = cnt+1;
        %% Parameter for Primary user Position definition in sample space
            TraDia = 50*1000;                    % Diameter of relay node trajectory
            CPDia = 5*1000;                     % Diameter of center relay node
            TraCir = 2*pi*TraDia;                % Circumference of relay node trajectory
            CPCir = 2*pi*CPDia;                  % Circumference of center relay node
            puV = 50;                     % The velocity of relay node
            roundT = TraCir/puV;                % The time to round a trajectory once
            CroundT = CPCir/puV;
            AngV = 2*pi/roundT;                  % Angular speed of relay node
            CAngV = 2*pi/CroundT;                % Angular speed of center relay node
            NumPU = 4;                          % The number of relay node except for center node

            c_speed  = 2.99792458e+8;            % The speed of light
              
        %% Define Primary user Initial Position in sample space
         puxyz = [];
            for i = 1 : NumPU-1
                ang = AngV*(roundT/(NumPU-1)*(i-1)+time);
                pu(i,1) = -TraDia*cos(ang);
                pu(i,2) = TraDia*sin(ang);
                pu(i,3) = 10*1000;
            end

            cpu = [CPDia*cos(CAngV*time) CPDia*sin(CAngV*time) 10*1000];       
            switch NumPU
                case 4
                    puxyz = [pu(3,:); cpu; pu(2,:); pu(1,:)];
                case 5
                    puxyz = [pu(4,:); pu(3,:); cpu; pu(2,:); pu(1,:)];
            end
            
            puid = 1:NumPU;

        slot_num = randi([1 1],1,NumGRU)-1;
        delay = sort(time_slot*slot_num);
        
    %% Estimate Relay Node Position
        estpu_d = [];

%         for n = 1:NumPU
%             gru = [];
%             prvec_rel = genrng(1,puxyz(n,:),gruxyz,gruid,0,[1 1 0 1 0]);
%             % Reordering of Relay node according to Pseudorange
%             % Position change according to propagation time
%             pr_sort = sort(prvec_rel);
%             for j = 1: NumGRU
%                 order = find(prvec_rel == pr_sort(j));
%                 gru(j,:) = gruxyz(order,:);
%             end
%             pr_pu = [];
%             for k = 1:NumGRU
%                 dtime = time + delay(k);
%                 puxyz_d = [];
%                 pu_d = [];
%                 for j = 1 : NumPU-1
%                     ang = AngV*(roundT/(NumPU-1)*(j-1)+dtime);
%                     pu_d(j,1) = -TraDia*cos(ang);
%                     pu_d(j,2) = TraDia*sin(ang);
%                     pu_d(j,3) = 10*1000;
%                 end
% 
%                 cpu_d = [CPDia*cos(CAngV*dtime) CPDia*sin(CAngV*dtime) 10*1000]; 
% 
%                 switch NumPU
%                     case 4
%                         puxyz_d = [pu_d(3,:); cpu_d; pu_d(2,:); pu_d(1,:)];
%                     case 5
%                         puxyz_d = [pu_d(4,:); pu_d(3,:); cpu_d; pu_d(2,:); pu_d(1,:)];
%                     case 6
%                         puxyz_d = [pu_d(4,:); pu_d(3,:); pu_d(5,:); cpu_d; pu_d(2,:); pu_d(1,:)];
%                     case 7
%                         puxyz_d = [pu_d(5,:); pu_d(4,:); pu_d(3,:); cpu_d; pu_d(6,:); pu_d(1,:); pu_d(2,:)];
%                 end
%             temp_pr = genrng(1,puxyz_d(n,:),gru,gruid,0,[1 1 0 1 0]);
%             pr_pu(k) = temp_pr(k);            
%             end
% 
%               temp_estpu = olspos(pr_pu, gru, puxyz(n,:));
%               estpu_d(n,:) = temp_estpu(1:3);
% 
%         end

        for n = 1:NumPU
            prvec_pu = genrng(1,puxyz(n,:), gruxyz, gruid, time, [1 1 0 1 0]);
            estpu(n,:) = olspos(prvec_pu, gruxyz, puxyz(n,:));
        end

        for k = 1:NumPU
            tempxyz(cnt, 3*k-2:3*k) = puxyz(k,:) - estpu(k,1:3);
        end

        % %% Plot Skyplot          
        %     pause(0.001)
        %     skyplot(relxyz,relid,usrxyz,0,0)  % Show the distribution of relay node
        %     hold on

        %% Calculate pseudo range and Estimate position, Check error

            prvec = genrng(1,suxyz,puxyz,puid,0,[1 1 0 1 0]);
            estsu = olspos(prvec,estpu(:,1:3),suxyz);
            xyzerr(cnt,:) = suxyz - estsu(1:3);
        %     terr(i) = estusr(4);  % true clk bias is zero
        end

        HDRMS_mat(cnt1, cnt2) = sqrt(sum(xyzerr(:,1).^2+xyzerr(:,2).^2)/length(xyzerr));
        VDRMS_mat(cnt1, cnt2) = sqrt(sum(xyzerr(:,3).^2)/length(xyzerr));
        end
     waitbar(usr_y/pos_end)
end
close(bar);

%% Write file of simulation results
% file open start
Hfile = 'simul_Link16_distance(Horizontal)_';
Vfile = 'simul_Link16_distance(Vertical)_';
count = 1;
Hfilename = strcat(Hfile, sprintf('%02d', count), '.txt');
while fopen(Hfilename, 'rt') ~= -1
    fclose('all');
    count = count+1;
    Hfilename = strcat(Hfile, sprintf('%02d', count), '.txt');
end

count = 1;
Vfilename = strcat(Vfile, sprintf('%02d', count), '.txt');
while fopen(Vfilename, 'rt') ~= -1
    fclose('all');
    count = count+1;
    Vfilename = strcat(Vfile, sprintf('%02d', count), '.txt');
end

Hfd = fopen(Hfilename, 'w');
Vfd = fopen(Vfilename, 'w');
fprintf(Hfd, 'Clock %2d/%2d/%2d %2d:%2d:%2d\n',  current_time(1), current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );
fprintf(Hfd, 'simul_Link16_distance_%02d\n', count);
fprintf(Vfd, 'Clock %2d/%2d/%2d %2d:%2d:%2d\n',  current_time(1), current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );
fprintf(Vfd, 'simul_Link16_distance_%02d\n', count);
fprintf('simul_Link16_distance_%02d is saved !!\n', count);
% file open end

% write simulation parameters
fprintf(Hfd,'Relay node Speed: %d m/s\n', puV);
fprintf(Hfd,'Radius of Reference Station: %d km\n', RefDia/1000);
fprintf(Hfd,'=====\n');
fprintf(Vfd,'Relay node Speed: %d m/s\n', puV);
fprintf(Vfd,'Radius of Reference Station: %d km\n', RefDia/1000);
fprintf(Vfd,'=====\n');

% write results data
for usr_y = 1 : 1 : cnt1
        for usr_x = 1 : 1 : cnt2
        fprintf(Hfd,'%.10f\t',HDRMS_mat(usr_y, usr_x));
        fprintf(Vfd,'%.10f\t',VDRMS_mat(usr_y, usr_x));
        end
    fprintf(Hfd,'\n');
    fprintf(Vfd,'\n');
end

fprintf(Hfd,'=====\n');
fprintf(Hfd, 'Clock %2d/%2d/%2d %2d:%2d:%2d\n',  current_time(1), current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );

fprintf(Vfd,'=====\n');
fprintf(Vfd, 'Clock %2d/%2d/%2d %2d:%2d:%2d\n',  current_time(1), current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );

current_time = fix(clock);
fprintf(  'Clock %2d/%2d %2d:%2d:%2d\n',  current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );

