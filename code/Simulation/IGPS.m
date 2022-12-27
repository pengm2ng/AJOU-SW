function estrel = IGPS(relxyz, refxyz, prvec_rel)

% % Line of sight unit vector between RS and SV
% 
%         u = norm(refxyz-relxyz);
%         e(1) = (relxyz(1) - refxyz(1))/u;
%         e(2) = (relxyz(2) - refxyz(2))/u;
%         e(3) = (relxyz(3) - refxyz(3))/u;
%    
% %         PR = norm(relxyz-refxyz);
%         
%     estpos = refxyz+prvec_rel*e;

len_ref = length(refxyz);
% Initialization of H and PR vector
H = -1*ones(len_ref,4);
PR = [];

    for n = 1 : len_ref
        u_rs(n) = norm(refxyz(n,:)-relxyz);
        e_rs(n,1) = (relxyz(1) - refxyz(n,1))/u_rs(n);
        e_rs(n,2) = (relxyz(2) - refxyz(n,2))/u_rs(n);
        e_rs(n,3) = (relxyz(3) - refxyz(n,3))/u_rs(n);
        H(n,1:3) = e_rs(n,:);
        PR(n,:) = prvec_rel(n)+dot(e_rs(n,:),refxyz(n,:));
    end

        estpos = (H\PR)';
        estrel = estpos(1:3);