% Function enu = wgsxyz2enu(xyz, reflat, reflon, refalt) returns
% a 3 x 1 vector enu which represents the East, North, and Up
% coordinates (in meters) of a point with WGS84 xyz coordinates
% xyz (3 x 1 vector, units in meters) in an ENU coordinate system
% located at latitude reflat (degrees), longitude reflon (degrees)
% and altitude above the WGS84 ellipsoid refalt (meters)
%
% Note: Requires functions wgslla2xyz.m and rot.m to be in the 
% same directory

function enu = xyz2enu(usrxyz, orgxyz)


tmpxyz = usrxyz;
tmporg = orgxyz;
if size(tmpxyz) ~= size(tmporg)
    tmporg=tmporg'; 
end
% Difference xyz from reference point
diffxyz = tmpxyz - tmporg;
[m,n] = size(diffxyz); 
if m<n
    diffxyz=diffxyz';
end

refllh = xyz2llh(orgxyz);

% First, calculate the xyz of reflat, reflon, refalt
reflat = refllh(1);
reflon = refllh(2);

% Now rotate the (often short) diffxyz vector to enu frame

R1=rot(90+reflon, 3);
R2=rot(90-reflat, 1);
R=R2*R1;

enu=R*diffxyz;

return;

           