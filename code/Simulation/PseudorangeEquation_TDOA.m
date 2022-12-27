% Compute Val = || Xs - X || + b and its Jacobian.
function [Val, Jacob] = PseudorangeEquation_TDOA(X, refxyz)

relid = 1:length(refxyz);
rel = X([1 3 5]);
pr = genrng(1,rel,refxyz,relid,0,[0 0 0 0 0]);
pr_sort = sort(pr);
for j = 1: length(refxyz)
    order = find(pr == pr_sort(j));
    ref(j,:) = refxyz(order,:);
end

for n = 1 : length(refxyz)
    e(n,1) = (rel(1) - ref(n,1))/pr_sort(n);
    e(n,2) = (rel(2) - ref(n,2))/pr_sort(n);
    e(n,3) = (rel(3) - ref(n,3))/pr_sort(n);
end

Jacob = zeros(size(refxyz, 1)-1, size(X, 1));
for m = 1 : length(refxyz)-1
    dPR(m) = pr_sort(m+1)-pr_sort(1);
    Jacob(m, [1,3,5]) = [e(m+1,1)-e(1,1), e(m+1,2)-e(1,2), e(m+1,3)-e(1,3)];
end

Val = (dPR)'; %+ X(7))';
Jacob(:, 7) = 0;

end