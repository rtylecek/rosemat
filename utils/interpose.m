function [ T, R, q, tdif ] = interpose( x1, pos1, x2, pos2, x )
%INTERPOSE Interpolate two poses (linear);
%   quaternion + translation
assert(x1<=x2);
%assert(x1<=x);
%assert(x<=x2);
tdif = min(x-x1,x-x2);

if x<x1
  x = x1;
  %warning('interPose: x<x1');
end
if x>x2
  x = x2;
  %warning('interPose: x>x2');
end

q1 = ([pos1.Orientation.W pos1.Orientation.X pos1.Orientation.Y pos1.Orientation.Z]);
T1 = [pos1.Position.X pos1.Position.Y pos1.Position.Z];

q2 = ([pos2.Orientation.W pos2.Orientation.X pos2.Orientation.Y pos2.Orientation.Z]);
T2 = [pos2.Position.X pos2.Position.Y pos2.Position.Z];

%% T
w2 = (x-x1)/(x2-x1);
w1 = 1-w2;
T = w1*T1 + w2*T2;
%% R
q = slerp2(q1,q2,w2);
R = quat2rotm(q);
end

%% from http://www.alecjacobson.com/weblog/?p=981
function [c] = slerp(a,b,t)
angle = acos( dot(a,b) );
% easy degenerate case
if(0==angle)
  c = a;
  % hard case
elseif(pi==angle)
  error('SLERP: angle between vectors cannot be exactly PI...');
else
  c = (sin((1.0-t)*angle)/sin(angle))*a + (sin(t*angle)/sin(angle))*b;
end
end

function [ q3 ] = slerp2( q1, q2, t )
%SLERP quaternion slerp
%   computes the slerp of value t between quaternions q1 and q2
%%
q1 = q1 ./ norm(q1);
q2 = q2 ./ norm(q2);

one = 1.0 - eps;
d = q1*q2';
absD = abs(d);

if(absD >= one)
  scale0 = 1 - t;
  scale1 = t;
else
  % theta is the angle between the 2 quaternions
  theta = acos(absD);
  sinTheta = sin(theta);
  
  scale0 = sin( ( 1.0 - t ) * theta) / sinTheta;
  scale1 = sin( ( t * theta) ) / sinTheta;
end
if(d < 0)
  scale1 = -scale1;
end

q3 = scale0 * q1 + scale1 * q2;
q3 = q3 ./ norm(q3);
end