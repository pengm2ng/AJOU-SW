function estusr = toapos(prvec, refxyzmat)

x_ = []; y_ = []; z_ = []; k = []; r = []; A = []; b = []; xyz = [];

x = refxyzmat(:,1);
y = refxyzmat(:,2);
z = refxyzmat(:,3);

    for i = 1:length(prvec)-1
        x_ = [x_ x(i+1)-x(1)];
        y_ = [y_ y(i+1)-y(1)];
        z_ = [z_ z(i+1)-z(1)];
    end
    
    for j = 1:length(prvec)
        r = [r prvec(j)];
    end

    for l = 1:length(prvec)
        k = [k x(l)^2+y(l)^2+z(l)^2];
    end
    
    for m = 1:length(prvec)-1
        A = [A; x_(m) y_(m) z_(m)];
    end
    
    for n = 1:length(prvec)-1
        b = [b; r(1)^2-r(n+1)^2+k(n+1)-k(1)];
    end
    
    estusr = (1/2*inv(transpose(A)*A)*transpose(A)*b)';
