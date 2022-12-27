%%%%%%%%%% Simulation for Selection algorithm %%%%%%%%%%

% 2016. 7. ~
% Designed by Kyuman Lee

clear all
close all
clc

fprintf('Simul_Selection Algorithm\n');

%% Define parameter for simulation
NumNode = 4;
NumAir = 25;

orgxyz = [1 1 1];

bar = waitbar(0,'Running Monte Carlo Trials...  ');
cnt = 0;
for ii = 1 : 100
    cnt = cnt+1;

    %% Airborne node Generation
        Air_pos = zeros(NumAir,3);
        ElAz = zeros(NumAir,2);
        for i = 1 : NumAir
            Air_pos(i,:) = [(rand*2-1)*50000 (rand*2-1)*50000 (rand*2000)+9000];
            [ElAz(i,1), ElAz(i,2)] = calElAz(Air_pos(i,:), orgxyz);
            dis(i) = norm(Air_pos(i,:)-orgxyz);
        end
        Measure = [dis' ElAz(:,2)];
        
    %% Relative Navigation  
        cnt1 = 0;
        while(1)
            cnt1 = cnt1+1;
            PU_ind = zeros(NumNode,1);
            PU_ind(1) = randi([1 NumAir],1);%find(max(ElAz(:,1))==ElAz(:,1));
            
            for n = 1 : NumNode-1
                d_v1 = Air_pos(PU_ind(n),:)-orgxyz;
                for m = 1 : NumAir
                    d_v2 = Air_pos(m,:)-orgxyz;
                    ang(m) = abs(180/pi*acos((d_v1(1)*d_v2(1)+d_v1(2)*d_v2(2)+d_v1(3)*d_v2(3))/(sqrt(d_v1(1)^2+d_v1(2)^2+d_v1(3)^2)*sqrt(d_v2(1)^2+d_v2(2)^2+d_v2(3)^2)))-90);
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

            puxyz = [];
            for m = 1 : NumNode
                puxyz = [puxyz; Air_pos(PU_ind(m),:)];
            end
            
            tmp_dop(cnt1,:) = dops(puxyz, orgxyz);
            
            if abs(tmp_dop(cnt1,6))<5 | cnt1 == 100
                if cnt1 == 100
                    dop_RelNav = min(tmp_dop(:,6));
                else
                    dop_RelNav = tmp_dop(cnt1,6);
                end
                break
            end
        end
        
    %% ARPS
        Rel_ind = zeros(NumNode,1);
        Rel_ind(1) = find(min(Measure(:,1))==Measure(:,1));

        Z_ang = 360/(NumNode-1);
        for i = 1 : NumNode-1
            temp{i} = [];
            for j= 1 : NumAir
                if Z_ang*(i-1)<Measure(j,2) && Measure(j,2)<= Z_ang*i
                    temp{i} = [temp{i}; Measure(j,:)];
                end
            end
            if isempty(temp{i}) == 1
                ind_new = find(max(temp{i-1}(:,2)) == temp{i-1});
                temp{i} = temp{i-1}(ind_new-length(temp{i-1}),:);
            end
                
            Rel_ind(i+1) = find(max(temp{i}(:,1))==Measure(:,1));
        end

        relxyz = [];
        for m = 1 : NumNode
            relxyz = [relxyz; Air_pos(Rel_ind(m),:)];
        end
        
   %% Random Selection
        RN_ind = zeros(NumNode,1);
       for k = 1 : NumNode
           while(1)
               tmp = randi([1 NumAir],1);
               coll_r = [];
               coll_r = find(RN_ind == tmp);
               if isempty(coll_r) == 1
                   RN_ind(k) = tmp;
                   break
               end
           end
       end
       
       rnxyz = [];
       for m = 1 : NumNode
           rnxyz = [rnxyz; Air_pos(RN_ind(m),:)];
       end
       
       dop_ARPS = dops(relxyz, orgxyz);
%        if dop_ARPS(6) >50
%            stop
%        end
       dop_Random = dops(rnxyz, orgxyz);
       
       DOP_mat(ii,:) = [dop_RelNav dop_ARPS(6) dop_Random(6)];
       waitbar(ii/1000)
end
close(bar);