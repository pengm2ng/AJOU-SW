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

    RefDia = 55*1000;                   % Diameter of Reference distribution
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
vel_start = 1;
vel_step = 30;
vel_end = 151;

bar = waitbar(0,'Running Monte Carlo Trials...  ');
cnt1 = 0;
%% Define User Position in sample space
for vel= vel_start : vel_step : vel_end
    cnt1 = cnt1+1;
        
        usrxyz = [30000 40000 2500];
        puV = vel;
           
        switch puV
            case 1
                time_step = 1000; time_end = 200000;
            case 31
                time_step = 50; time_end = 12000;
            case 61
                time_step = 30; time_end = 6000;
            case 91
                time_step = 20; time_end = 4000;
            case 121
                time_step = 15; time_end = 3000;
            case 151
                time_step = 10; time_end = 2800;
        end
        
        cnt = 0;
        for time = 0:time_step:time_end
            cnt = cnt+1;
        %% Parameter for Primary user Position definition in sample space
            TraDia = 90*1000;                    % Diameter of relay node trajectory
            CPDia = 10*1000;                     % Diameter of center relay node
            TraCir = 2*pi*TraDia;                % Circumference of relay node trajectory
            CPCir = 2*pi*CPDia;                  % Circumference of center relay node
%             relV = 150;                     % The velocity of relay node
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
                pu(i,3) = 20*1000;
            end

            cpu = [CPDia*cos(CAngV*time) CPDia*sin(CAngV*time) 20*1000];       
            switch NumPU
                case 4
                    puxyz = [pu(3,:); cpu; pu(2,:); pu(1,:)];
                case 5
                    puxyz = [pu(4,:); pu(3,:); cpu; pu(2,:); pu(1,:)];
            end
            
            puid = 1:NumPU;

        slot_num = randi([1 128],1,NumGRU)-1;
        delay = sort(time_slot*slot_num);
        
    %% Estimate Relay Node Position
        estpu_d = [];

        for n = 1:NumPU
            gru = [];
            prvec_rel = genrng(1,puxyz(n,:),gruxyz,gruid,0,[1 1 0 1 0]);
            % Reordering of Relay node according to Pseudorange
            % Position change according to propagation time
            pr_sort = sort(prvec_rel);
            for j = 1: NumGRU
                order = find(prvec_rel == pr_sort(j));
                gru(j,:) = gruxyz(order,:);
            end
            pr_pu = [];
            for k = 1:NumGRU
                dtime = time + delay(k);
                puxyz_d = [];
                pu_d = [];
                for j = 1 : NumPU-1
                    ang = AngV*(roundT/(NumPU-1)*(j-1)+dtime);
                    pu_d(j,1) = -TraDia*cos(ang);
                    pu_d(j,2) = TraDia*sin(ang);
                    pu_d(j,3) = 20*1000;
                end

                cpu_d = [CPDia*cos(CAngV*dtime) CPDia*sin(CAngV*dtime) 20*1000]; 

                switch NumPU
                    case 4
                        puxyz_d = [pu_d(3,:); cpu_d; pu_d(2,:); pu_d(1,:)];
                    case 5
                        puxyz_d = [pu_d(4,:); pu_d(3,:); cpu_d; pu_d(2,:); pu_d(1,:)];
                    case 6
                        puxyz_d = [pu_d(4,:); pu_d(3,:); pu_d(5,:); cpu_d; pu_d(2,:); pu_d(1,:)];
                    case 7
                        puxyz_d = [pu_d(5,:); pu_d(4,:); pu_d(3,:); cpu_d; pu_d(6,:); pu_d(1,:); pu_d(2,:)];
                end
            temp_pr = genrng(1,puxyz_d(n,:),gru,gruid,0,[1 1 0 1 0]);
            pr_pu(k) = temp_pr(k);            
            end

              temp_estpu = olspos(pr_pu, gru, puxyz(n,:));
              estpu_d(n,:) = temp_estpu(1:3);

        end

        for k = 1:NumPU
            tempxyz(cnt, 3*k-2:3*k) = puxyz_d(k,:) - estpu_d(k,:);
        end

        % %% Plot Skyplot          
        %     pause(0.001)
        %     skyplot(relxyz,relid,usrxyz,0,0)  % Show the distribution of relay node
        %     hold on

        %% Calculate pseudo range and Estimate position, Check error

            prvec = genrng(1,usrxyz,puxyz,puid,0,[1 1 0 1 0]);
            estusr = olspos(prvec,estpu_d,usrxyz);
            xyzerr(cnt,:) = usrxyz - estusr(1:3);
        %     terr(i) = estusr(4);  % true clk bias is zero
        end

        avgusrxyz = mean(xyzerr);
        stdusrxyz = std(xyzerr);

        RMSE_mat(cnt1) = sqrt(stdusrxyz(1)^2+stdusrxyz(2)^2+stdusrxyz(3)^2);
 
    waitbar((vel-vel_start)/(vel_end-vel_start))
end
close(bar);

%% Write file of simulation results
% file open start
file = 'simul_Link16_TDMA_velocity_';
count = 1;
filename = strcat(file, sprintf('%02d', count), '.txt');
while fopen(filename, 'rt') ~= -1
    fclose('all');
    count = count+1;
    filename = strcat(file, sprintf('%02d', count), '.txt');
end

fd = fopen(filename, 'w');
fprintf(fd, 'Clock %2d/%2d/%2d %2d:%2d:%2d\n',  current_time(1), current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );
fprintf(fd, 'simul_Link16_TDMA_velocity_%02d\n', count);
fprintf('simul_Link16_TDMA_velocity_%02d is saved !!\n', count);
% file open end

% write simulation parameters
fprintf(fd,'User altitude: %d m\n', usrxyz(3));
fprintf(fd,'=====\n');

% write results data
count = 1;
fprintf(fd,'velocity(m/s)\t RMSE \n');
for vel = vel_start : vel_step : vel_end
    fprintf(fd,'%d\t %.10f\t\n',vel,RMSE_mat(count));
    count = count+1;
end


current_time = fix(clock);
fprintf(fd,'=====\n');
fprintf(fd, 'Clock %2d/%2d/%2d %2d:%2d:%2d\n',  current_time(1), current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );

fprintf(  'Clock %2d/%2d %2d:%2d:%2d\n',  current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );
