%%%%%%%%%% Simulation of ARPS scheme embedded TDMA %%%%%%%%%%

% 2016. 5. ~
% Designed by Kyuman Lee

clear all
close all
clc

global avgDiff  

%% Time slot
time_slot = 7.8125*10^(-3);

%% Define Reference Node Position in sample space

    RefDia = 50*1000;                   % Diameter of Reference distribution
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
    Sf = 36;Sg = 0.01;sigma=5;         %state transition variance
    Qb = [Sf*T+Sg*T*T*T/3 Sg*T*T/2;
          Sg*T*T/2 Sg*T];
    Qxyz = sigma^2 * [T^3/3 T^2/2;
                      T^2/2 T];
    Q = blkdiag(Qxyz,Qxyz,Qxyz,Qb);

    Xu = zeros(8,1);
    Xu([1 3 5]) = [1000 1000 2000];
    P = {[eye(8)*10] [eye(8)*10] [eye(8)*10] [eye(8)*10]};
    Pu = eye(8)*10;

%% Define User Position in sample space

usrxyz = [1000 1000 2000];

%% Perform simulations
cnt = 0;

for time = 1 : 4 : 2800
    cnt = cnt+1;

 %% Airborne node Generation
    Air_pos = zeros(NumAir,3);
    ElAz = zeros(NumAir,2);
    for i = 1 : NumAir
        Air_pos(i,:) = [(rand*2-1)*35000 (rand*2-1)*35000 (rand*2000)+9000];
        [ElAz(i,1), ElAz(i,2)] = calElAz(Air_pos(i,:), refxyz(3,:));
    end

%% Relay Selection
    Rel_ind = zeros(NumRel,1);
    Rel_ind(1) = find(max(ElAz(:,1))==ElAz(:,1));

    Z_ang = 360/(NumRel-1);
    for i = 1 : NumRel-1
        temp = [];
        for j= 1 : NumAir
            if Z_ang*(i-1)<ElAz(j,2) && ElAz(j,2)<= Z_ang*i
                temp = [temp; ElAz(j,:)];
            end
        end
        Rel_ind(i+1) = find(min(temp(:,1))==ElAz(:,1));
    end
    
    relxyz = [];
    for m = 1 : NumRel
        relxyz = [relxyz; Air_pos(Rel_ind(m),:)];
    end

    relid = 1:NumRel;
    
    % Set initial values of X (Airborne relay)    
    for n = 1:NumRel
        X{n} = [relxyz(n,1)-10 0 relxyz(n,2)-10 0 relxyz(n,3)-10 0 0 0];
    end

   %% Estimate Relay Node Positions
    estrel = [];
    
    for ii = 1:NumRel
        Xi = X{ii};
        Pi = P{ii};
        % Set g
        g = @(Xi) PseudorangeEquation_TDOA(Xi, refxyz);             % pseudorange equations for each satellites                

        % Set R
        Rhoerror = 10;                                              % variance of measurement error(pseudorange error)
        R = eye(size(refxyz, 1)-1) * Rhoerror; 

        % Set Z
        Z = [];
        temp_z = genrng(1,relxyz(ii,:),refxyz,refid,0,[1 1 0 1 0]);       % measurement value
        sort_z = sort(temp_z);
        for k = 1 : length(temp_z)-1
            Z = [Z sort_z(k+1)-sort_z(1)];
        end

        [Xi,Pi] = Extended_KF(f,g,Q,R,Z',Xi',Pi);                      % relay positioning using Kalman filter

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
        % Set g
        g = @(Xu) PseudorangeEquation(Xu, estrel);                 % pseudorange equations for each satellites                

        % Set R
        Rhoerror = 10;                                             % variance of measurement error(pseudorange error)
        R = eye(size(relxyz, 1)) * Rhoerror; 

        % Set Z
        Z = genrng(2,usrxyz,relxyz,relid,time,[1 1 0 1 0]);            % measurement value

        [Xu,Pu] = Extended_KF(f,g,Q,R,Z',Xu,Pu);                     % user positioning using Kalman filter
        estusr_ini = Xu([1 3 5]);
        
    %% Re-estimate User position
        pr_ref = [];
        for m = 1:NumRef
            pr_temp = norm(refxyz(m,:)-estusr_ini');
            pr_ref = [pr_ref pr_temp];
        end 
        
        index = find(min(pr_ref) == pr_ref);
        estrel_new = [estrel; refxyz(index,:)];
    
        % Set g
        g = @(Xu) PseudorangeEquation(Xu, estrel_new);                 % pseudorange equations for each satellites                

        % Set R
        Rhoerror = 10;                                             % variance of measurement error(pseudorange error)
        R = eye(size(relxyz, 1)+1) * Rhoerror; 

        % Set Z
        Z = [Z min(pr_ref)];            % measurement value

        [Xu,Pu] = Extended_KF(f,g,Q,R,Z',Xu,Pu);                     % user positioning using Kalman filter
        estusr = Xu([1 3 5]);
        
        xyzerr(cnt,:) = usrxyz - estusr';

end

HDRMS_mat = sqrt(sum(xyzerr(:,1).^2+xyzerr(:,2).^2)/length(xyzerr));
VDRMS_mat = sqrt(sum(xyzerr(:,3).^2)/length(xyzerr));