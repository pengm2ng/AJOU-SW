%%%%%%%%%%% Simulator for Link-16 in TDMA protocol %%%%%%%%%%

% 2015. 11. 6. ~ 
% Designed by Kyuman LEE

clear all
close all
clc

current_time = fix(clock);
fprintf(  'Clock %2d/%2d %2d:%2d:%2d\n',  current_time(2), current_time(3), current_time(4), current_time(5), current_time(6) );

%% Time slot
time_slot = 7.8125*10^(-3);

usrxyz = [30030 40030 2030];  

%% Define Reference Node Position in sample space

    RefDia = 100*1000;                   % Diameter of Reference distribution
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
   
for n = 1 : NumGRU
    GRU_Rho(n) = norm(gruxyz(n,:)-usrxyz);
end
   
%% Kalman filter parameters
   T = 1;       % positioning interval
   sigma = 3.5;   % variance of acceleration noise 
   H = zeros(NumGRU,11);
   H(1:end,1) = -1;
   

   Phi = [1 T 0 0 0 0 0 0 0 0 0;
          0 1 0 0 0 0 0 0 0 0 0;
          0 0 1 0 0 T 0 0 T^2/2 0 0;    % state transition matrix
          0 0 0 1 0 0 T 0 0 T^2/2 0; 
          0 0 0 0 1 0 0 T 0 0 T^2/2; 
          0 0 0 0 0 1 0 0 T 0 0;
          0 0 0 0 0 0 1 0 0 T 0;
          0 0 0 0 0 0 0 1 0 0 0;
          0 0 0 0 0 0 0 0 1 0 0;
          0 0 0 0 0 0 0 0 0 1 0;
          0 0 0 0 0 0 0 0 0 0 1];
      
   Gamma = [1 0 0 0 0;
            0 1 0 0 0;
            0 0 T^2/2 0 0;              % plant noise transition matrix
            0 0 0 T^2/2 0;
            0 0 0 0 T^2/2;
            0 0 T 0 0;
            0 0 0 T 0;
            0 0 0 0 T;
            0 0 1 0 0;
            0 0 0 1 0;
            0 0 0 0 1];
        
    Q = [5*10^(-6) 0 0 0 0;               % plant covariance matrix
         0 5*10^(-6) 0 0 0;
         0 0 5*10^(-6) 0 0;
         0 0 0 5*10^(-6) 0;
         0 0 0 0 5*10^(-6)];
     
    X = [0 0 30000 40000 2000 0 0 0 0 0 0]';
    P = eye(11)*10;

 for i = 1 : 2000
     
     for n = 1 : NumGRU
        Rc = norm(gruxyz(n,:)-X(3:5)');
        H(n, 3:5)  = [gruxyz(n,1)-X(1)/Rc gruxyz(n,2)-X(2)/Rc gruxyz(n,3)-X(3)/Rc];
     end
   
    Pp = Phi*P*Phi'+Gamma*Q*Gamma';
    
    Rhoerror = 3.5;                                               % variance of measurement error(pseudorange error)
    R = Rhoerror^2 * eye(size(gruxyz, 1)); 

    K = Pp * H'/ (H * Pp * H' + R);
    
    Xp = Phi*X;

    Z = genrng(1,usrxyz,gruxyz,gruid,0,[1 1 0 1 0]);
%     Z = GRU_Rho;

     gXp = genrng(1,Xp(3:5)',gruxyz,gruid,0,[0 0 0 0 0]);

     X = Xp + K * (Z - gXp)';

     I = eye(size(X,1), size(X,1));
     P = (I - K * H) * Pp;
     
     est_error(:,i) = usrxyz - X(3:5)';
 end

%Plot the results.
for ii = 1:3
    subplot(3,1,ii)
    plot(1:2000, est_error(ii,:),'-r')
    legend('EKF')
    xlabel('Sampling index')
    ylabel('Error(meters)')
end
ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
text(0.5, 1,'\bf Relative positioning error in x,y and z directions','HorizontalAlignment','center','VerticalAlignment', 'top');

  
 