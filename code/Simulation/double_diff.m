function estpl = double_diff(usrxyz, estPL, refPL, prvec, pr_rec)

len_pl = length(estPL);
% Initialization of H and PR vector
H = [];
PR = [];

    for n = 1 : len_pl
    % unit vector between PL and user
        u_usr(n) = norm(estPL(n,:)-usrxyz);
        e_usr(n,1) = (estPL(n,1) - usrxyz(1))/u_usr(n);
        e_usr(n,2) = (estPL(n,2) - usrxyz(2))/u_usr(n);
        e_usr(n,3) = (estPL(n,3) - usrxyz(3))/u_usr(n);
    % unit vector between PL and REF
        u_rf(n) = norm(estPL(n,:)-refPL);
        e_rf(n,1) = (estPL(n,1) - refPL(1))/u_rf(n);
        e_rf(n,2) = (estPL(n,2) - refPL(2))/u_rf(n);
        e_rf(n,3) = (estPL(n,3) - refPL(3))/u_rf(n);
    end
 
    for m = 1 : len_pl-1
       H(m,:) = e_usr(1,:) - e_usr(m+1,:);
       PR(m,:) = prvec(m+1)-pr_rec(m+1)-prvec(1)+pr_rec(1)-dot(e_usr(m+1,:),estPL(m+1,:))+dot(e_rf(m+1,:),(estPL(m+1,:)-refPL))...
           +dot(e_usr(1,:),estPL(1,:))-dot(e_rf(1,:),(estPL(1,:)-refPL));
    end
    
    estpl = (H\PR)';