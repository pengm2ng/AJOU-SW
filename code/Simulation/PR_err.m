function pr_err = PR_err(usrxyz, relxyz)

for i = 1 : length(relxyz)
    pr = norm(relxyz(i,:)-usrxyz);
    el = asin((relxyz(i,3)-usrxyz(3))/pr);

    %% Airborne receiver error
    a0_a = 0.15;
    a1_a = 0.43;
    t_c_a = 6.9;

    rn_a = a0_a + a1_a*exp(-el*(180/pi)/t_c_a);
    mp_a = 0.13 + 0.53*exp(-el*(180/pi)/10);

    err_air = sqrt(rn_a.^2+mp_a.^2);

    %% Ground station receiver error
    a0_gn = 0.093;
    a1_gn = 0.26;
    t_c_gn = 16.4;
    M = 2;
    Re = 6378136.3;
    hI = 350000;

    rn_g = a0_gn + a1_gn*exp(-el*(180/pi)/t_c_gn);

    a0_gm = 0.50;
    a1_gm = 1.64;
    a2_g = 0.08;
    a3_g = 0.02;
    t_c_gm = 14.5;
    mp_g = a0_gm+a1_gm*exp(-el*(180/pi)/t_c_gm);
    Fpp = (1-(Re*cos(el)/(Re+hI)))^(-1/2);

    pr_gnd = sqrt(rn_g.^2+mp_g.^2);
    err_SIS = sqrt(a2_g^2+a3_g^2*Fpp^2);

    err_gnd = sqrt(pr_gnd.^2/M+err_SIS^2);

    %% Tropospheric delay error

    N = 34;
    ho = 100;
   
    err_trop = ((N*ho*10^(-6))/sqrt(0.002+sin(el)^2))*(1-exp(-(relxyz(1,3)-usrxyz(3))/ho));

%     err_trop = (2.312 ./ sin(sqrt(el .* el + 1.904e-3)))*0.1 + ...
%                    0.084 ./ sin(sqrt(el .* el + 0.6854e-3));

    PRE(i) = sqrt(err_air.^2+err_gnd.^2+err_trop.^2);

end
pr_err = mean(PRE);


