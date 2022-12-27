clear all
close all
clc

%% Parameter for Ground Reference Unit Position definition in sample space
RefDia = 50*1000;                   % Diameter of Reference distribution
NumGRU = 4;                         % The number of Reference except for center reference

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

%% Parameter for Primary user Position definition in sample space
    TraDia = 35*1000;                    % Diameter of relay node trajectory
    CPDia = 3*1000;                     % Diameter of center relay node
    TraCir = 2*pi*TraDia;                % Circumference of relay node trajectory
    CPCir = 2*pi*CPDia;                  % Circumference of center relay node
    puV = 50;                           % The velocity of relay node
    roundT = TraCir/puV;                 % The time to round a trajectory once
    CroundT = CPCir/puV;
    AngV = 2*pi/roundT;                  % Angular speed of relay node
    CAngV = 2*pi/CroundT;                % Angular speed of center relay node
    NumPU = 4;                           % The number of relay node except for center node

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
    X = zeros(8,NumPU);
    X([1 3 5],1) = [16457 -31336 10100];                 %Initial position
    X([1 3 5],2) = [1999 5 10100];
    X([1 3 5],3) = [16543 31286 10100];
    X([1 3 5],4) = [-34000 5 10100];
    Xu = zeros(8,1);
    Xu([1 3 5]) = [50000 50000 2000];

    P = {[eye(8)*10] [eye(8)*10] [eye(8)*10] [eye(8)*10]};
    Pu = eye(8)*10;
    
%% Define User Position in sample space
    usrxyz = [50000 50000 2000];  
    
    cnt = 0;
for time = 1 : 4 : 2800
    cnt = cnt+1;
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

    %% Estimate Airborne relay positions
    estpu = [];
    for ii = 1:NumPU
        Xi = X(:,ii);
        Pi = P{ii};
        % Set g
        g = @(Xi) PseudorangeEquation(Xi, gruxyz);                  % pseudorange equations for each satellites

        % Set R
        Rhoerror = 10;                                              % variance of measurement error(pseudorange error)
        R = eye(size(gruxyz, 1)) * Rhoerror; 

        % Set Z
        Z = genrng(1,puxyz(ii,:),gruxyz,gruid,0,[1 1 0 1 0]);       % measurement value

        [Xi,Pi] = Extended_KF(f,g,Q,R,Z',Xi,Pi);                      % relay positioning using Kalman filter
        
        estpu(ii,:) = Xi([1 3 5]);
        X(:,ii) = Xi;
        P{ii} = Pi;
    end

    %% Estimate User position
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