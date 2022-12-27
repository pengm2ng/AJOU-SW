%                           eleva.m
%  Scope:   This MATLAB macro computes the elevation angle and the ECEF unit 
%           line-of-sight vector, when the ECEF user and satellite positions are
%           known; WGS-84 constants are used.
%  Usage:   [eangle,ulos] = eleva(user,sv)
%  Description of parameters:
%           user   -  input, ECEF user position vector
%           sv     -  input, ECEF satellite position vector
%           eangle -  output, elevation angle in radians
%           ulos   -  output, ECEF unit line-of-sight vector
%  Remark:  The components of the input vectors should have the same unit,
%           e.g. meter, foot.
%  External Matlab macros used:  uverv, wgs84con
%  Last update:  05/22/99
%  Copyright (C) 1996-99 by LL Consulting. All Rights Reserved.

function  [eangle,ulos] = eleva(user,sv)

ulos = sv - user;
temp = sqrt(ulos' * ulos);
ulos = ulos / temp;

w1 = uverv(user);
eangle = asin(w1' * ulos);
