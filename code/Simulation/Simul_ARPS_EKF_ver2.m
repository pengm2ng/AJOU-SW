%%%%%%%%%% Simulation for TDMA MAC protocol embedded ARPS scheme %%%%%%%%%%

% 2015.9 10. ~
% Designed by Kyuman Lee

clear all
close all
clc

global avgDiff  

%% Time slot
time_slot = 7.8125*10^(-3);

%% Define Reference Node Position in sample space

    RefDia = 50*1000;                   % Diameter of Reference distribution
    NumRef = 4;                         % The number of Reference except for center reference

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

%% Define Relay Node Position in sample space
    TraDia = 35*1000;                    % Diameter of relay node trajectory
    CRDia = 3*1000;                      % Diameter of center relay node
    TraCir = 2*pi*TraDia;                % Circumference of relay node trajectory
    CRCir = 2*pi*CRDia;                  % Circumference of center relay node
    relV = 50;                          % The velocity of relay node
    roundT = TraCir/relV;                % The time to round a trajectory once
    CroundT = CRCir/relV;
    AngV = 2*pi/roundT;                  % Angular speed of relay node
    CAngV = 2*pi/CroundT;                % Angular speed of center relay node
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

    % Set initial values of X and P     
%     X{1} = [45000 0 77942 0 20000 0 0 0]';                 %Initial position
%     X{2} = [10000 0 0 0 20000 0 0 0]';
%     X{3} = [45000 0 -77942 0 20000 0 0 0]';
%     X{4} = [-90000 0 010 0 20000 0 0 0]';
    X = zeros(8,NumRel);
    X([1 3 5],1) = [17457 -30336 10000];                 %Initial position
    X([1 3 5],2) = [2999 50 10000];
    X([1 3 5],3) = [17543 30286 10000];
    X([1 3 5],4) = [-35000 50 10000];
    Xu = zeros(8,1);
    Xu([1 3 5]) = [50000 50000 2000];
    P = {[eye(8)*10] [eye(8)*10] [eye(8)*10] [eye(8)*10]};
    Pu = eye(8)*10;

%% Define User Position in sample space

usrxyz = [50000 50000 2000];

%% Perform simulations

cnt = 0;

for time = 1 : 4 : 2800
    cnt = cnt+1;

     for i = 1 : NumRel-1
        ang = AngV*(roundT/(NumRel-1)*(i-1)+time);
        rel(i,1) = -TraDia*cos(ang);
        rel(i,2) = TraDia*sin(ang);
        rel(i,3) = 10*1000;
    end

        crel = [CRDia*cos(CAngV*time) CRDia*sin(CAngV*time) 10*1000];       
        switch NumRel
            case 4
                relxyz = [rel(3,:); crel; rel(2,:); rel(1,:)];
            case 5
                relxyz = [rel(4,:); rel(3,:); crel; rel(2,:); rel(1,:)];
        end

        relid = 1:NumRel;
        
   %% Estimate Relay Node Positions
    estrel = [];
    
    for ii = 1:NumRel
%         Xi = X{ii};
        Xi = X(:,ii);
        Pi = P{ii};
        % Set g
        g = @(Xi) PseudorangeEquation(Xi, refxyz);             % pseudorange equations for each satellites                

        % Set R
        Rhoerror = 10;                                              % variance of measurement error(pseudorange error)
        R = eye(size(refxyz, 1)) * Rhoerror; 

        % Set Z
        Z = genrng(1,relxyz(ii,:),refxyz,refid,0,[1 1 0 1 0]);       % measurement value
        
        [Xi,Pi] = Extended_KF(f,g,Q,R,Z',Xi,Pi);                      % relay positioning using Kalman filter

        estrel(ii,:) = Xi([1 3 5]);
%         X{ii} = Xi;
        X(:,ii) = Xi;
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
        Z = genrng(1,usrxyz,relxyz,relid,time,[1 1 0 1 0]);            % measurement value

        [Xu,Pu] = Extended_KF(f,g,Q,R,Z',Xu,Pu);                     % user positioning using Kalman filter
        estusr = Xu([1 3 5]);
        
        xyzerr(cnt,:) = usrxyz - estusr';

end

HDRMS_mat = sqrt(sum(xyzerr(:,1).^2+xyzerr(:,2).^2)/length(xyzerr));
VDRMS_mat = sqrt(sum(xyzerr(:,3).^2)/length(xyzerr));