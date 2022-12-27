function estrel = IGPS_double(relxyz, refxyz, refPL, pr_rel, pr_REF)

len_ref = length(refxyz);
% Initialization of H and PR vector
H = [];
PR = [];

    for n = 1 : len_ref
    % unit vector between RS and PL
        u_pl(n) = norm(refxyz(n,:)-relxyz);
        e_pl(n,1) = (relxyz(1) - refxyz(n,1))/u_pl(n);
        e_pl(n,2) = (relxyz(2) - refxyz(n,2))/u_pl(n);
        e_pl(n,3) = (relxyz(3) - refxyz(n,3))/u_pl(n);
    % unit vector between RS and REF
        u_rf(n) = norm(refxyz(n,:)-refPL);
        e_rf(n,1) = (refPL(1) - refxyz(n,1))/u_rf(n);
        e_rf(n,2) = (refPL(2) - refxyz(n,2))/u_rf(n);
        e_rf(n,3) = (refPL(3) - refxyz(n,3))/u_rf(n);
    end
 
    for m = 1 : len_ref-1
       H(m,:) = e_pl(m+1,:) - e_pl(1,:);
       PR(m,:) = pr_rel(m+1)-pr_REF(m+1)-pr_rel(1)+pr_REF(1)+dot(e_pl(m+1,:),refxyz(m+1,:))+dot(e_rf(m+1,:),(refPL-refxyz(m+1,:)))...
           -dot(e_pl(1,:),refxyz(1,:))-dot(e_rf(1,:),(refPL-refxyz(1,:)));
    end
    
    estrel = (H\PR)';