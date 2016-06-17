%% FULL WINDED TITLE 
% INSTRUCTIONS
%{
% Call Using: 
%               current values for the 3-Solenoids (currx, curry, currz)
%               current position(x,y)
%               NOTE: phi & psi in the world-frame
%}

function [phi, psi] = fwdMagneticField( currx, curry, currz, x, y )
%Print Task Name
Task = 'Running Find Magnetic Field from Currents'
%---------------------

% Mapping of Magnetic Field to Current Based on Physical 
% Attributes of Solenoid
M = eye(3)
% position of the ball center
pos = [x,y,0];
%Define Current-Vector(I)
I = [currx; curry; currz];

%% Eqn I => B
% Unit Vecotr for pos
p_hat = pos/norm(pos);
% Constant of Permeability
mu = 4*(10^-7)*pi;
% Eqn parts for B => I 
temp = 2*pi*(norm(pos)^3)*M\(3*p_hat*(p_hat') - 2*eye(3))/mu
% Current Vector
B = temp\I;
% Unit Vector in the Direction of B
u = B/norm(B);

%% Output 
% Angle from the world-x-axis to B in world-x-z-plane
phi = atan2(u(2),u(1));
% Angle from world-x-axis to B in world-x-y-plane
psi = atan2(u(3),sqrt(u(2)^2+u(1)^2));


end

