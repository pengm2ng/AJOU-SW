%%%%%%%%%%% Simulator for ARPS Scheme %%%%%%%%%%

% 2013. 9. 30. ~ 
% Designed by Kyuman LEE

clear all
close all
clc

current_time = fix(clock);
fprintf(  'Clock %2d/%2d %2d:%2d:%2d\n',  current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );

global avgDiff   

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

%% Define simulation parameters
velocity = 50; % m/s
fprintf('Relay node Speed: %d m/s\n', velocity);
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
        
        usrxyz = [usr_x usr_y 5000];
        
        cnt = 0;
        for time = 0:4:2000
            cnt = cnt+1;
        %% Define Relay Node Position in sample space
            TraDia = 50*1000;                    % Diameter of relay node trajectory
            CRDia = 5*1000;                     % Diameter of center relay node
            TraCir = 2*pi*TraDia;                % Circumference of relay node trajectory
            CRCir = 2*pi*CRDia;                  % Circumference of center relay node
            relV = velocity;                     % The velocity of relay node
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

        %% Estimate Relay Node Position
            estrel = [];

            for n = 1:rel_row
                ref = [];
                prvec_rel = genrng(1,relxyz(n,:),refxyz,refid,time,[1 1 0 1 0]);
                % Reordering of Relay node according to Pseudorange
            pr_sort = sort(prvec_rel);
            for j = 1: ref_row
                order = find(prvec_rel == pr_sort(j));
                ref(j,:) = refxyz(order,:);
                pos_ch = (pr_sort(j)-pr_sort(1))/c_speed*relV;
                pr_rel(j) = pr_sort(j)+pos_ch;
            end
                 estrel(n,:) = ARPS(relxyz(n,:), ref, pr_rel);
             end

            for k = 1:rel_row
                tempxyz(cnt, 3*k-2:3*k) = relxyz(k,:) - estrel(k,:);

            % Calculate Relay distance and Declare a global variable
                RF_RL(k,:) = genrng(1,relxyz(k,:),refxyz,refid,time,[0 0 0 0 0]);
                RF_ES(k,:) = genrng(1,estrel(k,:),refxyz,refid,time,[0 0 0 0 0]);    
            end

            % Difference between estimated relay range and original relay range
                avgDiff = mean((RF_ES - RF_RL),2);

        % %% Plot Skyplot          
        %     pause(0.001)
        %     skyplot(relxyz,relid,usrxyz,0,0)  % Show the distribution of relay node
        %     hold on

        %% Calculate pseudo range and Estimate position, Check error

            prvec = genrng(2,usrxyz,relxyz,relid,time,[1 1 0 1 0]);
            estusr = olspos(prvec,estrel,usrxyz);
            xyzerr(cnt,:) = usrxyz - estusr(1:3);
        %     terr(i) = estusr(4);  % true clk bias is zero
        end

        avgrelxyz = transpose(reshape(mean(tempxyz), rel_col, rel_row));

        avgusrxyz = mean(xyzerr);
        stdusrxyz = std(xyzerr);

        HDRMS_mat(cnt1, cnt2) = sqrt(sum(xyzerr(:,1).^2+xyzerr(:,2).^2)/length(xyzerr));
        VDRMS_mat(cnt1, cnt2) = sqrt(sum(xyzerr(:,3).^2)/length(xyzerr));
    end
    waitbar(usr_y/pos_end)
end
close(bar);

%% Write file of simulation results
% file open start
Hfile = 'simul_ARPS_contour(Horizontal)_';
Vfile = 'simul_ARPS_contour(Vertical)_';
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
fprintf(Hfd, 'simul_ARPS_contour_%02d\n', count);
fprintf(Vfd, 'Clock %2d/%2d/%2d %2d:%2d:%2d\n',  current_time(1), current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );
fprintf(Vfd, 'simul_ARPS_contour_%02d\n', count);
fprintf('simul_ARPS_contour_%02d is saved !!\n', count);
% file open end

% write simulation parameters
fprintf(Hfd,'Relay node Speed: %d m/s\n', velocity);
fprintf(Hfd,'Radius of Reference Station: %d km\n', RefDia/1000);
fprintf(Hfd,'=====\n');
fprintf(Vfd,'Relay node Speed: %d m/s\n', velocity);
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

