%%%%%%%%%% Simulation of Link-16 relative navigation embedded TDMA %%%%%%%%%%

% 2016. 5. ~
% Designed by Kyuman Lee

clear all
close all
clc

fprintf('simul_Link-16_TDMA_EKF_SU_new\n');
current_time = fix(clock);
fprintf(  'Clock %2d/%2d %2d:%2d:%2d\n',  current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );

%% Parameter for Ground Reference Unit Position definition in sample space
RefDia = 52*1000;                   % Diameter of Ground reference unit distribution
NumGRU = 4;                         % The number of Ground reference unit

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

%% Define Primary Unit Position parameter
    NumAir = 25;                         % The number of total airborne nodes
    NumPU = 4;                          % The number of relay node except for center node
 
    c_speed  = 2.99792458e+8;            % The speed of light

%% Parameter for Kalman filtering
    T = 1; % positioning intervals

    % Set f, see [1]
    f = @(X) ConstantVelocity(X, T);

  % Set Q, see [1]
    Sf = 0.018; Sg = 0.0355; Sp = 10;         %state transition variance
    Qb = [Sf*T+Sg*T*T*T/3 Sg*T*T/2;
          Sg*T*T/2 Sg*T];
    Qxyz = Sp * [T^3/3 T^2/2;
                      T^2/2 T];
    Q = blkdiag(Qxyz,Qxyz,Qxyz,Qb);

%% Define Secondary Unit Position in sample space
    init_suxyz = [80000 50000 3000];
    su_vel = 50;
    theta = pi/2;
    
%% Define PPLI period and assigned slot number    
    period = 1;
    col1 = [];
    while(1)
       pu_slot = randi([1 period*128],NumPU, 1);
       for n = 1:NumPU-1
           col1(n) = sum(find(pu_slot(n)==pu_slot(n+1:end)));          % Check repetition
       end
       if sum(col1) == 0
           break
       end
    end
    pu_slnum = sort(pu_slot);
    col2 = [];
    while(1)
       gru_slot = randi([1 period*128],NumGRU, 1);
       for n = 1:NumGRU-1
           col2(n) = sum(find(gru_slot(n)==gru_slot(n+1:end)));          % Check repetition
       end
       if sum(col2) == 0
           break
       end
    end
    gru_slnum = sort(gru_slot);
    time_slot = 7.8125*10^(-3);
    
cnt1 = 0;
for ii = 1 : 1
    cnt1 = cnt1+1;
    
    %% Reset initial user position and state
        Xu = zeros(8,1);
        Xu([1 3 5]) = [init_suxyz(1)+300 init_suxyz(2)+300 init_suxyz(3)+300];
        Pu = eye(8)*10;
        
    %% Airborne node Generation
        pu_vel = 50;
        the_pu = (randi([1 366],1,NumPU)-1)*pi/180;
        the_air = (randi([1 366],1,NumAir)-1)*pi/180;
        Air_pos = zeros(NumAir,3);
        for i = 1 : NumAir
            Air_pos(i,:) = [(rand*2-1)*45000 (rand*2-1)*45000 (rand*2000)+9000];
        end
        
    %% Primary unit Selection
        coll = [];
        cnt2 = 0;
        
        while(1)
            cnt2 = cnt2+1;
            PU_ind = zeros(NumPU,1);
            PU_ind(1) = randi([1 NumAir],1);%find(max(ElAz(:,1))==ElAz(:,1));

            for n = 1 : NumPU-1
                d_v1 = Air_pos(PU_ind(n),:)-Xu([1 3 5])';
                for m = 1 : NumAir
                    d_v2 = Air_pos(m,:)-Xu([1 3 5])';
                    ang(m) = abs(180/pi*acos(dot(d_v1, d_v2)/(sqrt(d_v1(1)^2+d_v1(2)^2+d_v1(3)^2)*sqrt(d_v2(1)^2+d_v2(2)^2+d_v2(3)^2)))-90);
                end

            sort_ang = sort(ang);
            ind_temp = find(sort_ang(1)==ang);
            coll = [];
            coll = find(ind_temp == PU_ind);
            if isempty(coll) == 0
                    PU_ind(n+1) = find(sort_ang(2)==ang);
                else
                    PU_ind(n+1) = ind_temp;
                end
            end

            pu_entry = [];
            for m = 1 : NumPU
                pu_entry = [pu_entry; Air_pos(PU_ind(m),:)];
            end
            temp_pu{cnt2} = pu_entry;

            tmp_dop(cnt2,:) = dops(pu_entry, Xu([1 3 5])');
            if abs(tmp_dop(cnt2,6))<5 | cnt2 == 100
                if cnt2 == 100
                    ind = find(min(tmp_dop(:,6)) == tmp_dop(:,6));
                    puxyz = temp_pu{ind};
                else
                    puxyz= pu_entry;
                end      
                break
            end
        end

        puid = 1:NumPU;

        % Set initial values of X and P for Primary units    
        for n = 1:NumPU
            X{n} = [puxyz(n,1)+30 0 puxyz(n,2)+30 0 puxyz(n,3)+30 0 0 0]';
        end
        
        P = {[eye(8)*10] [eye(8)*10] [eye(8)*10] [eye(8)*10]};
    
    cnt = 0;
    for time = 1 : 1 : 100%(50*10^3)/su_vel
        cnt = cnt+1;
        if time > 1
            dop_move = dops(pu_cum{cnt-1}, Xu([1 3 5])')

            if abs(dop_move(6))>10
                cnt
                for n = 1 : NumAir
                    Air_new(n,1) = Air_pos(n,1)+cos(the_air(n))*pu_vel*(time-1);
                    Air_new(n,2) = Air_pos(n,2)+sin(the_air(n))*pu_vel*(time-1);
                    Air_new(n,3) = Air_pos(n,3);
                end

                cnt3 = 0;
                while(1)
                    cnt3 = cnt3+1;
                    PU_ind2 = zeros(NumPU,1);
                    PU_ind2(1) = randi([1 NumAir],1);%find(max(ElAz(:,1))==ElAz(:,1));

                    for n = 1 : NumPU-1
                        d_v3 = Air_new(PU_ind2(n),:)-Xu([1 3 5])';
                        for m = 1 : NumAir
                            d_v4 = Air_new(m,:)-Xu([1 3 5])';
                            ang2(m) = abs(180/pi*acos(dot(d_v3, d_v4)/(sqrt(d_v3(1)^2+d_v3(2)^2+d_v3(3)^2)*sqrt(d_v4(1)^2+d_v4(2)^2+d_v4(3)^2)))-90);
                        end

                    sort_ang2 = sort(ang2);
                    ind_temp2 = find(sort_ang2(1)==ang2);
                    coll = [];
                    coll = find(ind_temp2 == PU_ind2);
                    if isempty(coll) == 0
                            PU_ind2(n+1) = find(sort_ang2(2)==ang2);
                        else
                            PU_ind2(n+1) = ind_temp2;
                        end
                    end

                    pu_entry2 = [];
                    for m = 1 : NumPU
                        pu_entry2 = [pu_entry2; Air_new(PU_ind2(m),:)];
                    end
                    temp_pu2{cnt3} = pu_entry2;

                    tmp_dop2(cnt3,:) = dops(pu_entry2, Xu([1 3 5])');
                    if abs(tmp_dop2(cnt3,6))<10 | cnt3 == 100
                        puxyz = [];
                        if cnt3 == 100
                            ind = find(min(tmp_dop2(:,6)) == tmp_dop2(:,6));
                            puxyz = temp_pu2{ind};
                        else
                            puxyz= pu_entry2;
                        end   
                        break
                    end
                end
            end
        end
        
        pu_cum{cnt} = puxyz;
stop
        %% Estimate Primary unit positions
        estpu = [];
        for ii = 1:NumPU
            Xi = X{ii};
            Pi = P{ii};
            % Set g
            g = @(Xi) PseudorangeEquation(Xi, gruxyz);                  % pseudorange equations for each satellites

            % Set R
            Rhoerror = 1;                                              % variance of measurement error(pseudorange error)
            R = eye(size(gruxyz, 1)) * Rhoerror; 

            % Set Z
            Z = [];
            for n = 1 : NumGRU
                pu = [];
                pu(1,1) = puxyz(ii,1)+cos(the_air(PU_ind(n)))*pu_vel*(time_slot*gru_slnum(n));
                pu(1,2) = puxyz(ii,2)+sin(the_air(PU_ind(n)))*pu_vel*(time_slot*gru_slnum(n));
                pu(1,3) = puxyz(ii,3);
                temp_z = genrng(1,pu,gruxyz,gruid,0,[1 1 0 1 0]);       % measurement value
                Z(n) = temp_z(n);
            end

            [Xi,Pi] = Extended_KF(f,g,Q,R,Z',Xi,Pi);                      % relay positioning using Kalman filter
  
            estpu(ii,:) = Xi([1 3 5]);
            X{ii} = Xi;
            P{ii} = Pi;
        end

        %% Estimate Secondary unit position
            Xo = Xu;
            Po = Pu;
            
            % Set g
            g = @(Xo) PseudorangeEquation(Xo, estpu);                 % pseudorange equations for each satellites                

            % Set R
            Rhoerror = 1;                                             % variance of measurement error(pseudorange error)
            R = eye(size(puxyz, 1)) * Rhoerror; 

            % Set Z
            Zu = [];
            tmpxyz = [];
            for i = 1 : NumPU
                tmpxyz(i,1) = puxyz(i,1)+cos(the_air(PU_ind(n)))*pu_vel*(time_slot*pu_slnum(i));
                tmpxyz(i,2) = puxyz(i,2)+sin(the_air(PU_ind(n)))*pu_vel*(time_slot*pu_slnum(i));
                tmpxyz(i,3) = puxyz(i,3);
%                 temp_z = genrng(1,init_suxyz,puxyz,puid,time,[1 1 0 1 0]);            % measurement value
%                 Zu(i) = temp_z(i);
            end
            
            Zu = genrng(1,init_suxyz,tmpxyz,puid,time,[1 1 0 1 0]);
%             Zu = [Zu min(pr_ref)];

            [Xo,Po] = Extended_KF(f,g,Q,R,Zu',Xo,Po);                     % user positioning using Kalman filter
            estsu = Xu([1 3 5]);
            Xu = Xo;
            Pu = Po;

%             suxyz = [init_suxyz(1)+cos(theta)*su_vel*time init_suxyz(2)+sin(theta)*su_vel*time init_suxyz(3)];
            xyzerr(cnt,:) = init_suxyz - estsu';

    end

    HDRMS_mat(cnt1,:) = sqrt(sum(xyzerr(:,1).^2+xyzerr(:,2).^2)/length(xyzerr));
    VDRMS_mat(cnt1,:) = sqrt(sum(xyzerr(:,3).^2)/length(xyzerr));
end

current_time = fix(clock);
fprintf(  'Clock %2d/%2d %2d:%2d:%2d\n',  current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );