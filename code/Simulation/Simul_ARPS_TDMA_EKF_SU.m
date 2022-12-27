%%%%%%%%%% Simulation of ARPS scheme embedded to relative navigation %%%%%%%%%%

% 2016. 5. ~
% Designed by Kyuman Lee

clear all
close all
clc

fprintf('simul_ARPS_TDMA_EKF_SU\n');
current_time = fix(clock);
fprintf(  'Clock %2d/%2d %2d:%2d:%2d\n',  current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );

global avgDiff  

%% Define Reference Node Position in sample space

    RefDia = 52*1000;                   % Diameter of Reference distribution
    NumRef = 4;                         % The number of Reference
    
    for i = 1 : NumRef-1
        ang = (2*pi/NumRef)*i;
        ref(i,1) = RefDia*cos(ang);
        ref(i,2) = RefDia*sin(ang);
        ref(i,3) = randi([1 50]);
    end

   switch NumRef
        case 4
            refxyz = [ref(1,:); ref(2,:); 1 1 10; ref(3,:)];
        case 5
            refxyz = [ref(1,:); ref(2,:);  1 1 10; ref(4,:); ref(3,:)];
   end
   refid = 1:NumRef;

%% Define Relay Node Position parameter
    NumAir = 25;                         % The number of total airborne nodes
    NumRel = 4;                          % The number of relay node except for center node
 
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

%% Define User Position in sample space
    init_usrxyz = [80000 50000 3000];
    usr_vel = 50;
    theta = pi/2;

%% Define slot length and number
    time_slot = 7.8125*10^(-3);
    slot_num = 1:NumRel;
    slot_leng = time_slot;
    
cnt1 = 0;  
 
%% Perform simulations
for ii = 1 : 1
    cnt1 = cnt1+1;

    %% Airborne node Generation
        rel_vel = 50;
        the_rel = (randi([1 366],1,NumRel)-1)*pi/180;
        Air_pos = zeros(NumAir,3);
        ElAz = zeros(NumAir,2);
        for i = 1 : NumAir
            Air_pos(i,:) = [(rand*2-1)*45000 (rand*2-1)*45000 (rand*2000)+9000];
            tmp_Az = 180/pi*atan((Air_pos(i,2)-refxyz(3,2))/(Air_pos(i,1)-refxyz(3,1)));
            if Air_pos(i,1) < 0 && Air_pos(i,2) > 0
                Az(i) = tmp_Az+180;
            elseif Air_pos(i,1) < 0 && Air_pos(i,2) < 0
                Az(i) = tmp_Az+180;
            elseif Air_pos(i,1) > 0 && Air_pos(i,2) < 0
                Az(i) = tmp_Az+360;
            else
                Az(i) = tmp_Az;
            end
            dis(i) = norm(Air_pos(i,:)-refxyz(3,:));
        end
        Measure = [dis' Az'];

    %% Relay Selection
        Rel_ind = zeros(NumRel,1);
        Rel_ind(1) = find(min(Measure(:,1))==Measure(:,1));
        Rel_ind(2) = find(max(Measure(:,1))==Measure(:,1));
        
        ref_pos = [];
        for i = 1: NumRel-2
            ang = Measure(Rel_ind(2),2);
            max_dis = Measure(Rel_ind(2),1);
            G_ang = 360/(NumRel-1);
            ref_pos(i,:) = [max_dis*cos(pi/180*(ang+G_ang*i)) max_dis*sin(pi/180*(ang+G_ang*i))];
            for n = 1 : NumAir
                min_dis(n,i) = norm(Air_pos(n,1:2)-ref_pos(i,:));
            end
            Rel_ind(2+i) = find(min(min_dis(:,i))==min_dis(:,i));
        end

        relxyz = [];
        for m = 1 : NumRel
            relxyz = [relxyz; Air_pos(Rel_ind(m),:)];
        end

        relid = 1:NumRel;

    % Set initial values of X and P for Airborne relays    
        for n = 1:NumRel
            X{n} = [relxyz(n,1)+30 0 relxyz(n,2)+30 0 relxyz(n,3)+30 0 0 0]';
        end
        P = {[eye(8)*10] [eye(8)*10] [eye(8)*10] [eye(8)*10]};
        
    %% Reset initial user position and state
        Xu = zeros(8,1);
        Xu([1 3 5]) = [init_usrxyz(1)+300 init_usrxyz(2)+300 init_usrxyz(3)+300];
        Pu = eye(8)*10;

    cnt = 0;
    for time = 1 : 1 : 500%(50*10^3)/usr_vel
        cnt = cnt+1;

       %% Estimate Relay Node Positions
        estrel = [];

        for ii = 1:NumRel
            Xi = X{ii};
            Pi = P{ii};
            % Set g
            g = @(Xi) PseudorangeEquation_TDOA(Xi, refxyz);               % pseudorange equations for each satellites                

            % Set R
            Rhoerror = 1;                                               % variance of measurement error(pseudorange error)
            R = eye(size(refxyz, 1)-1) * Rhoerror; 

            % Set Z
            Z = [];
            for n = 1 : NumRef
                rel = [];
                rel(1,1) = relxyz(ii,1)+cos(the_rel(ii))*rel_vel*((time-1)+slot_leng*n);
                rel(1,2) = relxyz(ii,2)+sin(the_rel(ii))*rel_vel*((time-1)+slot_leng*n);
                rel(1,3) = relxyz(ii,3);
                tmp = genrng(1,rel,refxyz,refid,0,[1 1 0 1 0]);            % measurement value
                temp_z(n) = tmp(n);
            end
            sort_z = sort(temp_z);
            for k = 1 : length(temp_z)-1
                Z = [Z sort_z(k+1)-sort_z(1)];
            end

            [Xi,Pi] = Extended_KF(f,g,Q,R,Z',Xi,Pi);                      % relay positioning using Kalman filter

            estrel(ii,:) = Xi([1 3 5]);
            X{ii} = Xi;
            P{ii} = Pi;
        end

       for k = 1:NumRel
            RF_RL(k,:) = genrng(1,relxyz(k,:),refxyz,refid,0,[0 0 0 0 0]);
            RF_ES(k,:) = genrng(1,estrel(k,:),refxyz,refid,0,[0 0 0 0 0]);    
       end

        % Difference between estimated relay range and original relay range
            avgDiff = mean((RF_ES - RF_RL),2);

        %% Estimate User position
            Xo = Xu;
            Po = Pu;
            % Set g
            g = @(Xo) PseudorangeEquation(Xo, estrel);                    % pseudorange equations for each satellites                

            % Set R
            Rhoerror = 1;                                               % variance of measurement error(pseudorange error)
            Ru = eye(size(relxyz, 1)) * Rhoerror; 

            % Set Zu
            Zu = [];
            for i = 1 : NumRel
                tmpxyz(i,1) = relxyz(i,1)+cos(the_rel(i))*rel_vel*((time-1)+(slot_leng*NumRef)+(slot_num(i)*slot_leng));
                tmpxyz(i,2) = relxyz(i,2)+sin(the_rel(i))*rel_vel*((time-1)+(slot_leng*NumRef)+(slot_num(i)*slot_leng));
                tmpxyz(i,3) = relxyz(i,3);
%                 temp_z = genrng(2,tmpxyz,relxyz,relid,time,[1 1 0 1 0]);            % measurement value
%                 Zu(i) = temp_z(i);
            end
            Zu = genrng(1,init_usrxyz,tmpxyz,relid,time,[1 1 0 1 0]);


            [Xo,Po] = Extended_KF(f,g,Q,Ru,Zu',Xo,Po);                     % user positioning using Kalman filter
            estusr = Xo([1 3 5]);
            Xu = Xo;
            Pu = Po;
%             
%         %% Re-estimate User position
%             pr_ref = [];
%             for m = 1:NumRef
%                 pr_temp = norm(refxyz(m,:)-estusr_ini');
%                 pr_ref = [pr_ref pr_temp];
%             end 
% 
%             index = find(min(pr_ref) == pr_ref);
%             estrel_new = [estrel; refxyz(index,:)];
%         
%         % Decide Re-estimation
%             dop_current = dops(estrel,estusr_ini');
%             dop_reest = dops(estrel_new,estusr_ini');
% 
%             URE = PR_err(estusr_ini', estrel);
%             URE_re = PR_err(estusr_ini', estrel_new);
%             
%             if dop_reest(4)*URE_re < dop_current(4)*URE & dop_reest(3)*URE_re < dop_current(3)*URE
%                 % Set g
%                 g = @(Xu) PseudorangeEquation(Xu, estrel_new);               % pseudorange equations for each satellites                
% 
%                 % Set R
%                 Rhoerror = 3.5;                                               % variance of measurement error(pseudorange error)
%                 Rr = eye(size(relxyz, 1)+1) * Rhoerror; 
% 
%                 % Set Zr
%                 Zr = [];
%                 Zr = [Zu min(pr_ref)];                                         % measurement value
% 
%                 [Xo,Po] = Extended_KF(f,g,Q,Rr,Zr',Xu,Pu);                     % user positioning using Kalman filter
%                 Xu = Xo;
%                 Pu = Po;
%                 estusr = Xo([1 3 5]);
%             else
%                 Xu = Xo;
%                 Pu = Po;
%                 estusr = estusr_ini;
%             end
            
%         usrxyz = [init_usrxyz(1)+cos(theta)*usr_vel*time init_usrxyz(2)+sin(theta)*usr_vel*time init_usrxyz(3)];
        xyzerr(cnt,:) = init_usrxyz - estusr';
%             if abs(xyzerr(cnt,3))>1000
%                 stop
%             end
    end
    
    HDRMS_mat(cnt1,:) = sqrt(sum(xyzerr(:,1).^2+xyzerr(:,2).^2)/length(xyzerr));
    VDRMS_mat(cnt1,:) = sqrt(sum(xyzerr(:,3).^2)/length(xyzerr));
    
end

current_time = fix(clock);
fprintf(  'Clock %2d/%2d %2d:%2d:%2d\n',  current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );