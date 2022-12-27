%%%%%%%%%% Simulation of Link-16 relative navigation embedded TDMA %%%%%%%%%%

% 2016. 5. ~
% Designed by Kyuman Lee

clear all
close all
clc

%% Parameter for Ground Reference Unit Position definition in sample space
RefDia = 50*1000;                   % Diameter of Ground reference unit distribution
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
    
    cnt = 0;
for time = 1 : 4 : 2800
    cnt = cnt+1;
   
    %% Airborne node Generation
    Air_pos = zeros(NumAir,3);
    ElAz = zeros(NumAir,2);
    for i = 1 : NumAir
        Air_pos(i,:) = [(rand*2-1)*35000 (rand*2-1)*35000 (rand*2000)+9000];
        [ElAz(i,1), ElAz(i,2)] = calElAz(Air_pos(i,:), gruxyz(3,:));
    end

    %% Relay Selection
%     PU_ind = zeros(NumPU,1);
%     PU_ind(1) = find(max(ElAz(:,1))==ElAz(:,1));
%     while(1)
%         ind_temp = randi([1 NumPU]);
%         if ind_temp ~= PU_ind(1)
%             PU_ind(2) = ind_temp;
%             break
%         end
%     end

    PU_ind(1) = randi([1 NumPU]);
    
    for n = 1 : NumPU-1
        d_v1 = Air_pos(PU_ind(n),:)-Xu([1 3 5])';
        for m = 1 : NumAir
            d_v2 = Air_pos(m,:)-Xu([1 3 5])';
            ang(m) = abs(180/pi*acos(dot(d_v1, d_v2)/(sqrt(d_v1(1)^2+d_v1(2)^2+d_v1(3)^2)*sqrt(d_v2(1)^2+d_v2(2)^2+d_v2(3)^2)))-90);
        end
        sort_ang = sort(ang);
        ind_temp = find(sort_ang(1)==ang);
        coll = find(ind_temp == PU_ind);
        if coll ~= 0
            PU_ind(n+1) = find(sort_ang(2)==ang);
        else
            PU_ind(n+1) = ind_temp;
        end
    end

    puxyz = [];
    for m = 1 : NumPU
        puxyz = [puxyz; Air_pos(PU_ind(m),:)];
    end
    
    puid = 1:NumPU;
    
    
    % Set initial values of X (Primary unit)    
    for n = 1:NumPU
        X{n} = [puxyz(n,1)-10 0 puxyz(n,2)-10 0 puxyz(n,3)-10 0 0 0];
    end

    %% Estimate Primary unit positions
    estpu = [];
    for ii = 1:NumPU
        Xi = X{ii};
        Pi = P{ii};
        % Set g
        g = @(Xi) PseudorangeEquation(Xi, gruxyz);                  % pseudorange equations for each satellites

        % Set R
        Rhoerror = 10;                                              % variance of measurement error(pseudorange error)
        R = eye(size(gruxyz, 1)) * Rhoerror; 

        % Set Z
        Z = genrng(1,puxyz(ii,:),gruxyz,gruid,0,[1 1 0 1 0]);       % measurement value

        [Xi,Pi] = Extended_KF(f,g,Q,R,Z',Xi',Pi);                      % relay positioning using Kalman filter
        
        estpu(ii,:) = Xi([1 3 5]);
        X{ii} = Xi;
        P{ii} = Pi;
    end

    %% Estimate Secondary unit position
        % Set g
        g = @(Xu) PseudorangeEquation(Xu, estpu);                 % pseudorange equations for each satellites                

        % Set R
        Rhoerror = 10;                                             % variance of measurement error(pseudorange error)
        R = eye(size(puxyz, 1)) * Rhoerror; 

        % Set Z
        Z = genrng(1,usrxyz,puxyz,puid,0,[1 1 0 1 0]);            % measurement value

        [Xu,Pu] = Extended_KF(f,g,Q,R,Z',Xu,Pu);                     % user positioning using Kalman filter
        estusr = Xu([1 3 5]);

        xyzerr(cnt,:) = usrxyz - estusr';
end

HDRMS_mat = sqrt(sum(xyzerr(:,1).^2+xyzerr(:,2).^2)/length(xyzerr));
VDRMS_mat = sqrt(sum(xyzerr(:,3).^2)/length(xyzerr));