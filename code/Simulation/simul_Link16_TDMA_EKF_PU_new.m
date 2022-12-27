%%%%%%%%%% Simulation of Link-16 relative navigation embedded TDMA %%%%%%%%%%

% 2016. 5. ~
% Designed by Kyuman Lee

clear all
close all
clc

fprintf('simul_Link-16_TDMA_EKF_PU\n');
current_time = fix(clock);
fprintf(  'Clock %2d/%2d %2d:%2d:%2d\n',  current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );

%% Parameter for Ground Reference Unit Position definition in sample space
RefDia = 50*1000;                   % Diameter of Ground reference unit distribution
NumGRU = 4;                          % The number of Ground reference unit

for i = 1 : NumGRU-1
    ang = (2*pi/(NumGRU-1))*i;
    gru(i,1) = RefDia*cos(ang);
    gru(i,2) = RefDia*sin(ang);
    gru(i,3) = randi([1 50]);
end

switch NumGRU
    case 4
        gruxyz = [gru(1,:); gru(2,:); 1 1 10; gru(3,:)];
    case 5
        gruxyz = [gru(1,:); gru(2,:);  1 1 10; gru(4,:); gru(3,:)];
end

gruid = 1:NumGRU; 

    c_speed  = 2.99792458e+8;            % The speed of light

%% Parameter for Kalman filtering
    T = 1; % positioning intervals

    % Set f, see [1]
    f = @(X) ConstantVelocity(X, T);

    % Set Q, see [1]
    Sf = 0.018; Sg = 0.0355; Sp=10;         %state transition variance
    Qb = [Sf*T+Sg*T*T*T/3 Sg*T*T/2;
          Sg*T*T/2 Sg*T];
    Qxyz = Sp * [T^3/3 T^2/2;
                      T^2/2 T];
    Q = blkdiag(Qxyz,Qxyz,Qxyz,Qb);
    
%% Perform simulations

cnt1 = 0;
for ii = 1 : 20
   
    cnt1 = cnt1+1;

    %% Define Initial PU position in sample space
        init_puxyz = [-40000 -30000 3000];
        pu_vel = 50;
        theta = pi/4;

        Xu = zeros(8,1);
        Xu([1 3 5]) = [init_puxyz(1)+30 init_puxyz(2)+30 init_puxyz(3)+30];

        Pu = eye(8)*10;

    %% Define PPLI period and assigned slot number
        period = 3;
        run_time = fix(floor((100*10^3)/pu_vel)/period);
        coll = [];
        while(1)
           temp_slot = randi([1 period*128],NumGRU, 1);
           for n = 1:NumGRU-1
               coll(n) = sum(find(temp_slot(n)==temp_slot(n+1:end)));          % Check repetition
           end
           if sum(coll) == 0
               break
           end
        end
        t_num = sort(temp_slot)';
        slot_num = t_num;
        for i = 1 : run_time-1
            slot_num = [slot_num t_num+128*period*i];
        end
        time_slot = 7.8125*10^(-3);

        cnt = 0;
    for time = 0 : T : run_time*period
        cnt = cnt+1;

        %% Estimate PU position
            % Set g
            g = @(Xu) PseudorangeEquation(Xu, gruxyz);                     % pseudorange equations for each satellites

            % Set R
            Rhoerror = 1;                                                 % variance of measurement error(pseudorange error)
            R = eye(size(gruxyz, 1)) * Rhoerror; 

            % Set Z
            if time == 0
                Z = genrng(1,init_puxyz,gruxyz,gruid,time,[1 1 0 0 0]);            % measurement value
            else
                r_slot = find(slot_num > 128*(time-1) & slot_num < 129+(128*(time-1)));
                len_slot = length(r_slot);
                if len_slot ~= 0
                    for n = 1 : len_slot
                        switch rem(r_slot(n),NumGRU)
                            case 0
                                puxyz(1) = init_puxyz(1)+cos(theta)*pu_vel*time_slot*(slot_num(r_slot(n))-1);
                                puxyz(2) = init_puxyz(2)+sin(theta)*pu_vel*time_slot*(slot_num(r_slot(n))-1);
                                puxyz(3) = init_puxyz(3);
                                temp_z = genrng(1,puxyz,gruxyz,gruid,time,[1 1 0 0 0]);            % measurement value
                                Z(4) = temp_z(4);                           
                            case 1
                                puxyz(1) = init_puxyz(1)+cos(theta)*pu_vel*time_slot*(slot_num(r_slot(n))-1);
                                puxyz(2) = init_puxyz(2)+sin(theta)*pu_vel*time_slot*(slot_num(r_slot(n))-1);
                                puxyz(3) = init_puxyz(3);
                                temp_z = genrng(1,puxyz,gruxyz,gruid,time,[1 1 0 0 0]);            % measurement value
                                Z(1) = temp_z(1); 
                            case 2
                                puxyz(1) = init_puxyz(1)+cos(theta)*pu_vel*time_slot*(slot_num(r_slot(n))-1);
                                puxyz(2) = init_puxyz(2)+sin(theta)*pu_vel*time_slot*(slot_num(r_slot(n))-1);
                                puxyz(3) = init_puxyz(3);
                                temp_z = genrng(1,puxyz,gruxyz,gruid,time,[1 1 0 0 0]);            % measurement value
                                Z(2) = temp_z(2); 
                            case 3
                                puxyz(1) = init_puxyz(1)+cos(theta)*pu_vel*time_slot*(slot_num(r_slot(n))-1);
                                puxyz(2) = init_puxyz(2)+sin(theta)*pu_vel*time_slot*(slot_num(r_slot(n))-1);
                                puxyz(3) = init_puxyz(3);
                                temp_z = genrng(1,puxyz,gruxyz,gruid,time,[1 1 0 0 0]);            % measurement value
                                Z(3) = temp_z(3);   
                        end
                    end
                end
            end
            
            [Xu,Pu] = Extended_KF(f,g,Q,R,Z',Xu,Pu);                         % relay positioning using Kalman filter

            estpu = Xu([1 3 5]);

            puxyz = [init_puxyz(1)+cos(theta)*pu_vel*time init_puxyz(2)+sin(theta)*pu_vel*time init_puxyz(3)];
            xyzerr(cnt,:) = puxyz - estpu';
    end

    HDRMS_mat(cnt1) = sqrt(sum(xyzerr(:,1).^2+xyzerr(:,2).^2)/length(xyzerr));
    VDRMS_mat(cnt1) = sqrt(sum(xyzerr(:,3).^2)/length(xyzerr));
end

current_time = fix(clock);
fprintf(  'Clock %2d/%2d %2d:%2d:%2d\n',  current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );