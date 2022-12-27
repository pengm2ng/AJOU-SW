function estpos = ARPS_iter(relxyz, refxyz, prvec_rel)

% relxyz = relxyz - [30000 30000 10000];
tol=1e-3;
beta=[1e9 1e9 1e9 1e9];
maxiter = 10;
iter = 0;
% estpos = relxyz;
estpos = [-5000 5000 20000];

while ((iter<maxiter)&&(norm(beta)>tol))

% Line of sight unit vector between RS and SV
    for n = 1 : length(refxyz)
        u = norm(refxyz(n,:)-estpos);
        e(n,1) = (refxyz(n,1) - estpos(1))/u;
        e(n,2) = (refxyz(n,2) - estpos(2))/u;
        e(n,3) = (refxyz(n,3) - estpos(3))/u;
    end
 
    for m = 1 : length(refxyz)-1
        PR(m,:) = prvec_rel(m+1)-prvec_rel(1)-dot(e(m+1,:),refxyz(m+1,:))+dot(e(1,:),refxyz(1,:));
        H(m,:) = e(1,:) - e(m+1,:);
    end
        beta = H\PR;
        estpos = beta';
        iter = iter+1;     
end
