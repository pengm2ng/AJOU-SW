%                             genrn.m
%  Scope:   This MATLAB macro generates random numbers with normal
%           (Gaussian) distribution, with mean and standard deviation 
%           specified. 
%  Usage:   x = genrn(n,xmean,xstd,iseed)   when the seed is reset to iseed 
%           x = genrn(n,xmean,xstd) 
%  Description of parameters:
%           n     - input, number of random numbers to be generated
%           xmean - input, specified mean
%           xstd  - input, specified standard deviation
%           iseed - input, initial value of the seed (optional); if it 
%                   is specified the seed is reset to iseed value
%           x     - output, random numbers with specified mean and
%                   standard deviation, vector with n elements
%  Last update: 06/27/00
%  Copyright (C) 1996-00 by LL Consulting. All Rights Reserved.

function  x = rngen(n,xmean,xstd,iseed)
        
if ( (nargin < 3) | (nargin > 4) )
   disp('Error - GENRN.m  - check the argument list');
   disp('  ');
   return
elseif  (nargin == 4)
   rand('seed',iseed);     
end

clear x
x = zeros(n,1);

for  k = 1:n
   xsum = - 6.;
   for kk = 1:12
      xsum = rand(1) + xsum;
   end
   x(k) = xsum * xstd + xmean;
end