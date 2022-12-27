%%%%%%%%%% Simulation for Selection algorithm %%%%%%%%%%

% 2016. 7. ~
% Designed by Kyuman Lee

clear all
close all
clc

fprintf('Simul_Selection Algorithm\n');

%% Define parameter for simulation
NumPU = 4;
NumRel = 4;
NumAir = 25;

pu_vel = 150;
rel_vel = 150;

orgxyz = [1 1 1];

bar = waitbar(0,'Running Monte Carlo Trials...  ');
cnt = 0;
for ii = 1 : 1000
    cnt = cnt+1;

    %% Airborne node Generation
        Air_pos = zeros(NumAir,3);
        for i = 1 : NumAir
            Air_pos(i,:) = [(rand*2-1)*45000 (rand*2-1)*45000 (rand*2000)+9000];
            tmp_Az = 180/pi*atan((Air_pos(i,2)-orgxyz(2))/(Air_pos(i,1)-orgxyz(1)));
            if Air_pos(i,1) < 0 && Air_pos(i,2) > 0
                Az(i) = tmp_Az+180;
            elseif Air_pos(i,1) < 0 && Air_pos(i,2) < 0
                Az(i) = tmp_Az+180;
            elseif Air_pos(i,1) > 0 && Air_pos(i,2) < 0
                Az(i) = tmp_Az+360;
            else
                Az(i) = tmp_Az;
            end
            dis(i) = norm(Air_pos(i,:)-orgxyz);
        end
        Measure = [dis' Az'];

        the_pu = (randi([1 366],1,NumPU)-1)*pi/180;
        
        the_rel = (randi([1 366],1,NumRel)-1)*pi/180;
        
    %% Relative Navigation  
        cnt1 = 0;
        while(1)
            cnt1 = cnt1+1;
            PU_ind = zeros(NumPU,1);
            PU_ind(1) = randi([1 NumAir],1);%find(max(ElAz(:,1))==ElAz(:,1));
            
            for n = 1 : NumPU-1
                d_v1 = Air_pos(PU_ind(n),:)-orgxyz;
                for m = 1 : NumAir
                    d_v2 = Air_pos(m,:)-orgxyz;
                    ang(m) = abs(180/pi*acos((d_v1(1)*d_v2(1)+d_v1(2)*d_v2(2)+d_v1(3)*d_v2(3))/(sqrt(d_v1(1)^2+d_v1(2)^2+d_v1(3)^2)*sqrt(d_v2(1)^2+d_v2(2)^2+d_v2(3)^2)))-90);
                end
                
                sort_ang = sort(ang);
                ind_temp = find(sort_ang(1)==ang);
                col = [];
                col = find(ind_temp == PU_ind);

                if isempty(col) == 0
                    PU_ind(n+1) = find(sort_ang(2)==ang);
                else
                    PU_ind(n+1) = ind_temp;
                end
            end

            puxyz = [];
            for m = 1 : NumPU
                puxyz = [puxyz; Air_pos(PU_ind(m),:)];
            end
            tmp_puxyz((cnt1-1)*NumPU+1:cnt1*NumPU,:) = puxyz;
            
            tmp_dop(cnt1,:) = dops(puxyz, orgxyz);
            
            if abs(tmp_dop(cnt1,6))<2 | cnt1 == 2000
                if cnt1 == 2000
                    RelNav_org = min(tmp_dop(:,6));
                    tmp_num = find(RelNav_org==tmp_dop(:,6));
                    puxyz = tmp_puxyz((tmp_num(1)-1)*NumPU+1:tmp_num(1)*NumPU,:);
                    newpu = [];
                    for n = 1 : NumPU
                        newpu(n,1) = puxyz(n,1)+cos(the_pu(n))*pu_vel*12;
                        newpu(n,2) = puxyz(n,2)+sin(the_pu(n))*pu_vel*12;
                        newpu(n,3) = puxyz(n,3);
                    end 
                else
                    RelNav_org = tmp_dop(cnt1,6); 
                    newpu = [];
                    for n = 1 : NumPU
                        newpu(n,1) = puxyz(n,1)+cos(the_pu(n))*pu_vel*12;
                        newpu(n,2) = puxyz(n,2)+sin(the_pu(n))*pu_vel*12;
                        newpu(n,3) = puxyz(n,3);
                    end 
                end
  
                RelNav_new = dops(newpu, orgxyz);
                break
            end
        end
        
    %% ARPS
        alr = 0;
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
           
%         Z_ang = 360/(NumRel-1);
%         for i = 1 : NumRel-1
%             temp{i} = [];
%             for j= 1 : NumAir
%                 if Z_ang*(i-1)<Measure(j,2) && Measure(j,2)<= Z_ang*i
%                     temp{i} = [temp{i}; Measure(j,:)];
%                 end
%             end
%             if isempty(temp{i}) == 1
%                 alr = 1;
%                 ind_new = find(max(temp{i-1}(:,2)) == temp{i-1});
%                 temp{i} = temp{i-1}(ind_new-length(temp{i-1}),:);
%             end
%                 
%             Rel_ind(i+1) = find(max(temp{i}(:,1))==Measure(:,1));
%         end

        relxyz = [];
        newrel = [];
        for m = 1 : NumRel
            relxyz = [relxyz; Air_pos(Rel_ind(m),:)];
            newrel(m,1) = relxyz(m,1)+cos(the_rel(m))*rel_vel*1;
            newrel(m,2) = relxyz(m,2)+sin(the_rel(m))*rel_vel*1;
            newrel(m,3) = relxyz(m,3);
        end
               
       ARPS_org = dops(relxyz, orgxyz);
       if alr == 1
           ARPS_org(6)
       end
       ARPS_new = dops(newrel, orgxyz);
       
       if ARPS_org(6) >50
           stop
       end
       
       DOP_org(ii,:) = [RelNav_org ARPS_org(6)];
       DOP_new(ii,:) = [RelNav_new(6) ARPS_new(6)];
       waitbar(ii/1000)
end
close(bar);