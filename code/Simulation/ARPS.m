function estpos = ARPS(relxyz, refxyz, prvec_rel)

% Line of sight unit vector between RS and SV
    for n = 1 : length(refxyz)
        u = norm(refxyz(n,:)-relxyz);
        e(n,1) = (relxyz(1) - refxyz(n,1))/u;
        e(n,2) = (relxyz(2) - refxyz(n,2))/u;
        e(n,3) = (relxyz(3) - refxyz(n,3))/u;
    end
 
    for m = 1 : length(refxyz)-1
        PR(m,:) = prvec_rel(m+1)-prvec_rel(1)+dot(e(m+1,:),refxyz(m+1,:))-dot(e(1,:),refxyz(1,:));
        H(m,:) = e(m+1,:) - e(1,:);
    end
        estpos = H\PR;
%     estpos = inv(transpose(H)*H)*transpose(H)*PR;