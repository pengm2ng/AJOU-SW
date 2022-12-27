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
    X = zeros(8,1);
    X([1 3 5],1) = [-100 100 -100];                 %Initial position
    P = eye(8)*10;

%% Define User Position in sample space
    usrxyz = [-30000 50 10000];  
    
cnt = 0;
%% Estimate Relay Node Positions
    
    for ii = 1:1000
        % Set g
%         g = @(X) PseudorangeEquation_TDOA(X, refxyz);             % pseudorange equations for each satellites                

        % Set R
        Rhoerror = 10;                                              % variance of measurement error(pseudorange error)
        R = eye(size(refxyz, 1)-1) * Rhoerror; 

        % Set Z
        Z = [];
        temp_z = genrng(1,usrxyz,refxyz,refid,0,[1 1 0 1 0]);       % measurement value
        sort_z = sort(temp_z);
        for k = 1 : length(temp_z)-1
            Z = [Z sort_z(k+1)-sort_z(1)];
        end

%         [Xi,Pi] = Extended_KF(f,g,Q,R,Z',Xi,Pi);                      % relay positioning using Kalman filter
        N_state = size(X, 1);    

        [Xp, ~] = f(X);%1

        [~, fy] = f(Xp);%2

        % [gXp, H] = g(Xp);%3
ref = [];
usr = [];
        usr = Xp([1 3 5]);
        pr = genrng(1,usr,refxyz,refid,0,[0 0 0 0 0]);
        pr_sort = sort(pr);
        for j = 1: length(refxyz)
            order = find(pr == pr_sort(j));
            ref(j,:) = refxyz(order,:);
        end

        for n = 1 : length(refxyz)
            e(n,1) = (usr(1) - ref(n,1))/pr_sort(n);
            e(n,2) = (usr(2) - ref(n,2))/pr_sort(n);
            e(n,3) = (usr(3) - ref(n,3))/pr_sort(n);
        end

        Jacob = zeros(size(refxyz, 1)-1, size(Xp, 1));
        for m = 1 : length(refxyz)-1
            dPR(m) = pr_sort(m+1)-pr_sort(1);
            Jacob(m, [1,3,5]) = [e(m+1,1)-e(1,1), e(m+1,2)-e(1,2), e(m+1,3)-e(1,3)];
        end

        gXp = (dPR)'; %+ Xp(7)
        Jacob(:, 7) = 0;

        H = Jacob;

        Pp = fy * P * fy.' + Q;%4

        K = Pp * H' / (H * Pp * H.' + R);%5

        X = Xp + K * (Z' - gXp)%6

        I = eye(N_state, N_state);
        P = (I - K * H) * Pp;%7

        estusr = X([1 3 5]);
        
        xyzerr(ii,:) = usrxyz - estusr';
%         if abs(xyzerr(ii,3)) > 1000
%             stop
%         end
end
