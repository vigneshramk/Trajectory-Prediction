function [ x, y, xdot, ydot ] = transformation( u, v, udot, vdot )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global focal_length distance;

x = (4*focal_length*distance*u)./(u.^2 + v.^2 - 4*focal_length^2);
y = (4*focal_length*distance*v)./(u.^2 + v.^2 - 4*focal_length^2);

r = sqrt(x.^2 + y.^2 + distance^2);
den = r.^2 + distance^2 -2*distance*r;


x_size = size(x); x_size = x_size(1);
for k=1:x_size
    
    A = [(2*focal_length*(r(k)-distance) - (2*focal_length*(x(k)^2))/r(k)) -1*(2*focal_length*(y(k)*x(k)))/r(k);
    -1*(2*focal_length*(x(k)*y(k)))/r(k) (2*focal_length*(r(k)-distance) - (2*focal_length*(y(k)^2))/r(k))];

    B = [udot(k)*den(k); vdot(k)*den(k)];
    X = inv(A)*B;
    xdot(k) = X(1);
    ydot(k) = X(2);
end
end

