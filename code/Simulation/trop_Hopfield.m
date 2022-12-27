function troperr = trop_Hopfield(usrxyz, svmat)

% Tropospheric Error model for Pseusolite

% reference : Tropospheric Delay Estimation for Pseudolite Positioning

% Standard atmosphere model
T = 18 + 273.15;  % temperature in Kelvin
P = 1013.25;      % air pressure in mbar
f = 50;           % relative humidity

% Fixed scaled height
hd = 42700;  % for dry component
hw = 12000;  % for wet component

Nd = 77.6*(P/T);  % refractive index for dry component
Nw = 22770*(f/T^2)*10^(7.4475*(T-273)/(T-38.3));  % refractive index for wet component
R = norm(usrxyz - svmat);  % slant range

hp = usrxyz(3);
e = asin((svmat(3)-hp)/R)*(180/pi);  % elevation angle between relay and reference
% if e < 5
%     usrxyz;
%     svmat;
%     error('Elevation angle Lower than 5 degree exist');
% end
hr = R * sin(e*(pi/180))+hp;

troperr = -((10^-6*Nd*(hd-hp))/(5*(hr-hp))*((1-hr/(hd-hp))^5-(1-hp/(hd-hp))^5)*R + (10^-6*Nw*(hw-hp))/(5*(hr-hp))*((1-hr/(hw-hp))^5-(1-hp/(hw-hp))^5)*R);

