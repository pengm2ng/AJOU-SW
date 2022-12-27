%%%%%%%%%%% Simulator for ARPS Scheme in TDMA %%%%%%%%%%

% 2015. 11. 5. ~ 
% Designed by Kyuman LEE

clear all
close all
clc

current_time = fix(clock);
fprintf(  'Clock %2d/%2d %2d:%2d:%2d\n',  current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );

global avgDiff  

%% Time slot
time_slot = 7.8125*10^(-3);

%% Define Reference Node Position in sample space

    RefDia = 60*1000;                   % Diameter of Reference distribution
    NumRef = 5;                         % The number of Reference except for center reference

    for i = 1 : NumRef-1
        ang = (2*pi/(NumRef-1))*i;
        ref(i,1) = RefDia*cos(ang);
        ref(i,2) = RefDia*sin(ang);
        ref(i,3) = randi([1 50]);
    end

   switch NumRef
        case 4
            refxyz = [ref(1,:); ref(2,:); 0 0 10; ref(3,:)];
        case 5
            refxyz = [ref(1,:); ref(2,:);  0 0 10; ref(4,:); ref(3,:)];
   end
   
   refid = 1:NumRef;

%% Define simulation parameters
vel_start = 61;
vel_step = 90;
vel_end = 151;

bar = waitbar(0,'Running Monte Carlo Trials...  ');
cnt1 = 0;
%% Define User Position in sample space
for vel= vel_start : vel_step : vel_end
    cnt1 = cnt1+1;
        
        usrxyz = [10000 10000 2500];
        relV = vel;
           
        switch relV
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
        %% Define Relay Node Position in sample space
            TraDia = 90*1000;                    % Diameter of relay node trajectory
            CRDia = 10*1000;                     % Diameter of center relay node
            TraCir = 2*pi*TraDia;                % Circumference of relay node trajectory
            CRCir = 2*pi*CRDia;                  % Circumference of center relay node
%             relV = vel;                     % The velocity of relay node
            roundT = TraCir/relV;                % The time to round a trajectory once
            CroundT = CRCir/relV;
            AngV = 2*pi/roundT;                  % Angular speed of relay node
            CAngV = 2*pi/CroundT;                % Angular speed of center relay node
            NumRel = 4;                          % The number of relay node except for center node

            c_speed  = 2.99792458e+8;            % The speed of light
              
        %% Define Relay Node Initial Position in sample space
         relxyz = [];
            for i = 1 : NumRel-1
                ang = AngV*(roundT/(NumRel-1)*(i-1)+time);
                rel(i,1) = -TraDia*cos(ang);
                rel(i,2) = TraDia*sin(ang);
                rel(i,3) = 20*1000;
            end

            crel = [CRDia*cos(CAngV*time) CRDia*sin(CAngV*time) 20*1000];       
            switch NumRel
                case 4
                    relxyz = [rel(3,:); crel; rel(2,:); rel(1,:)];
                case 5
                    relxyz = [rel(4,:); rel(3,:); crel; rel(2,:); rel(1,:)];
            end
            
            relid = 1:NumRel;

%         slot_num = randi([1 32],1,NumRef)-1;
%         delay = sort(time_slot*slot_num);
        delay = time_slot*(refid-1);
        
    %% Estimate Relay Node Position
        estrel_d = [];

        for n = 1:NumRel
            ref = [];
            prvec_rel = genrng(1,relxyz(n,:),refxyz,refid,0,[1 1 0 1 0]);
            % Reordering of Relay node according to Pseudorange
            % Position change according to propagation time
            pr_sort = sort(prvec_rel);
            for j = 1: NumRef
                order = find(prvec_rel == pr_sort(j));
                ref(j,:) = refxyz(order,:);
            end
            pr_rel = [];
            for k = 1:NumRef
                dtime = time + delay(k);
                relxyz_d = [];
                rel_d = [];
                for j = 1 : NumRel-1
                    ang = AngV*(roundT/(NumRel-1)*(j-1)+dtime);
                    rel_d(j,1) = -TraDia*cos(ang);
                    rel_d(j,2) = TraDia*sin(ang);
                    rel_d(j,3) = 20*1000;
                end

                crel_d = [CRDia*cos(CAngV*dtime) CRDia*sin(CAngV*dtime) 20*1000]; 

                switch NumRel
                    case 4
                        relxyz_d = [rel_d(3,:); crel_d; rel_d(2,:); rel_d(1,:)];
                    case 5
                        relxyz_d = [rel_d(4,:); rel_d(3,:); crel; rel_d(2,:); rel_d(1,:)];
                    case 6
                        relxyz_d = [rel_d(4,:); rel_d(3,:); rel_d(5,:); crel_d; rel_d(2,:); rel_d(1,:)];
                    case 7
                        relxyz_d = [rel_d(5,:); rel_d(4,:); rel_d(3,:); crel_d; rel_d(6,:); rel_d(1,:); rel_d(2,:)];
                end
            temp_pr = genrng(1,relxyz_d(n,:),ref,refid,0,[1 1 0 1 0]);
            pr_rel(k) = temp_pr(k);            
            end
%                 estrel = olspos(pr_rel,ref,relxyz(n,:));
%                 estrel_d(n,:) = estrel(1:3);
             estrel_d(n,:) = ARPS(relxyz(n,:), ref, pr_rel);
    %          estrel(n,:) = ARPS(relxyz(n,:), refxyz, prvec_rel);
        end

        for k = 1:NumRel
            temprel(cnt, 3*k-2:3*k) = ( xyz2enu(estrel_d(k,:), relxyz_d(k,:)) )';
            tempxyz(cnt, 3*k-2:3*k) = relxyz_d(k,:) - estrel_d(k,:);

        % Calculate Relay distance and Declare a global variable
            RF_RL(k,:) = genrng(1,relxyz_d(k,:),refxyz,refid,time,[0 0 0 0 0]);
            RF_ES(k,:) = genrng(1,estrel_d(k,:),refxyz,refid,time,[0 0 0 0 0]);    
        end

            % Difference between estimated relay range and original relay range
                avgDiff = mean((RF_ES - RF_RL),2);

        % %% Plot Skyplot          
        %     pause(0.001)
        %     skyplot(relxyz,relid,usrxyz,0,0)  % Show the distribution of relay node
        %     hold on

        %% Calculate pseudo range and Estimate position, Check error

            prvec = genrng(2,usrxyz,relxyz,relid,0,[1 1 0 1 0]);
            estusr = olspos(prvec,estrel_d,usrxyz);
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
file = 'simul_ARPS_TDMA_velocity_';
count = 1;
filename = strcat(file, sprintf('%02d', count), '.txt');
while fopen(filename, 'rt') ~= -1
    fclose('all');
    count = count+1;
    filename = strcat(file, sprintf('%02d', count), '.txt');
end

fd = fopen(filename, 'w');
fprintf(fd, 'Clock %2d/%2d/%2d %2d:%2d:%2d\n',  current_time(1), current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );
fprintf(fd, 'simul_ARPS_TDMA_velocity_%02d\n', count);
fprintf('simul_ARPS_TDMA_velocity_%02d is saved !!\n', count);
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
