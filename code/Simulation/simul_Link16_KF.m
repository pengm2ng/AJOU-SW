%%%%%%%%%%% Simulator for Link-16 in TDMA protocol %%%%%%%%%%

% 2015. 11. 6. ~ 
% Designed by Kyuman LEE

clear all
close all
clc

current_time = fix(clock);
fprintf(  'Clock %2d/%2d %2d:%2d:%2d\n',  current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );

%% Define User Position in sample space
  init_usrxyz = [-40000 -30000 3000];
     usr_vel = 50;
    theta = pi/4;
    
%% Define Reference Node Position in sample space

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
   
%% Kalman filter parameters
   T = 1;       % positioning interval
   H = zeros(NumGRU,9);
   
   Phi = [1 0 0 T 0 0 T^2/2 0 0;    % state transition matrix
          0 1 0 0 T 0 0 T^2/2 0; 
          0 0 1 0 0 T 0 0 T^2/2; 
          0 0 0 1 0 0 T 0 0;
          0 0 0 0 1 0 0 T 0;
          0 0 0 0 0 1 0 0 T;
          0 0 0 0 0 0 1 0 0;
          0 0 0 0 0 0 0 1 0;
          0 0 0 0 0 0 0 0 1];
      
   Gamma = [T^2/2 0 0;              % plant noise transition matrix
            0 T^2/2 0;
            0 0 T^2/2;
            T 0 0;
            0 T 0;
            0 0 T;
            1 0 0;
            0 1 0;
            0 0 1];
        
    Q = [5*10^(-6) 0 0;               % plant covariance matrix
         0 5*10^(-6) 0;
         0 0 5*10^(-6)];
     
    X = [-40000 -30000 3000 0 0 0 0 0 0]';
    P = eye(9)*10;
cnt = 0;
 for time = 1 : 200
 cnt = cnt+1;
 
     for n = 1 : NumGRU
        Rc = norm(gruxyz(n,:)-X(1:3)');
        H(n,1:3)  = [gruxyz(n,1)-X(1)/Rc gruxyz(n,2)-X(2)/Rc gruxyz(n,3)-X(3)/Rc];
     end

     Pp = Phi*P*Phi.'+Gamma*Q*Gamma.';
    
     Rhoerror = 3.5;                                               % variance of measurement error(pseudorange error)
     R = Rhoerror^2 * eye(size(gruxyz, 1)); 

     K = Pp * H' / (H * Pp * H.' + R);
    
     Xp = Phi*X;
 
     Z = [];
     for i = 1 : NumGRU
         tmpxyz(1) = init_usrxyz(1)+cos(theta)*usr_vel*(time-1);
         tmpxyz(2) = init_usrxyz(2)+sin(theta)*usr_vel*(time-1);
         tmpxyz(3) = init_usrxyz(3);
         temp_z = genrng(1,tmpxyz,gruxyz,gruid,time,[1 0 0 1 0]);            % measurement value
         Z(i) = temp_z(i);
     end
%      Z = genrng(1,usrxyz,gruxyz,gruid,0,[1 0 0 0 0]);

     gXp = genrng(1,Xp(1:3)',gruxyz,gruid,0,[0 0 0 0 0]);

     X = Xp + K * (Z - gXp)';

     I = eye(size(X,1), size(X,1));
     P = (I - K * H) * Pp;
    
     est_error(:,cnt) = tmpxyz - X(1:3)';
 end

%Plot the results.
for ii = 1:3
    subplot(3,1,ii)
    plot(1:200, est_error(ii,:),'-r')
    legend('EKF')
    xlabel('Sampling index')
    ylabel('Error(meters)')
end
ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
text(0.5, 1,'\bf Relative positioning error in x,y and z directions','HorizontalAlignment','center','VerticalAlignment', 'top');

  
 