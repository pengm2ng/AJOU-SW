function [elevation, azimuth] = calElAz(posXYZ, refPosXYZ)%, rotationMatrix)
%---------------------------------------------------------
% Calculate elevation from position in ECEF XYZ
% Ju-Yong Do,  Feb 21, 2004
%---------------------------------------------------------
% input  : target ECEF XYZ, reference ECEF XYZ
% output : elevation
%=========================================================
%=========================================================
[numPos numDim] = size(posXYZ);
elevation       = zeros(numPos,1);

%=========================================================
%Calculate ENU, AZE and return
if numDim~=3, disp('Invalid target coordinate'); return; end

enu = (xyz2enu(posXYZ, refPosXYZ))';
% rotationMatrix = calRotMat(posXYZ);
% enu       = (rotationMatrix*(posXYZ'-refPosXYZ'*ones(1,numPos)))';
elevation = 90-180/pi*atan2(sqrt(enu(:,1).^2+enu(:,2).^2),enu(:,3));
azimuth   = mod(180/pi*atan2(enu(:,1),enu(:,2)), 360);

return;

%=========================================================
%=========================================================
