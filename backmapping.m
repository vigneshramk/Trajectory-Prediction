function [ u,v ] = backmapping( x,y )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
r = sqrt(x.^2 + y.^2);
global k z;

u = k*x.*atan2(r/z,1)./r;
v = k*y.*atan2(r/z,1)./r;

end

