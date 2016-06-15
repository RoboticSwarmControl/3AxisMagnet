%% Current Required for This orientation of magnetic field  
% INSTRUCTIONS
%{
% Call using the current pos(x,y) of the ball 
% and orientation of the magnetic field(phi,psi)
%}

function [ currx, curry, currz ] = inverseMagneticField(x,y,phi,psi)
%Print Task Name
Task = 'Running Find Required Current'
%---------------------

% Mapping of Magnetic Field to Current Based on Physical 
% Attributes of Solenoid
M = eye(3);
% position of the ball center
pos = [x,y,0];
% Magnitude of Field
a = 1;
% Magnetic Field with constant magnitude
B = rotz(psi)*roty(phi)*[0;0;a];
 
%% Eqn B => I
% Unit Vecotr for pos
p_hat = pos/norm(pos);
% Constant of Permeability
mu = 4*(10^-7)*pi;
% Eqn parts for B => I
I = (2*pi*(norm(pos)^3)*M\(3*p_hat*(p_hat') - 2*eye(3))/mu)*B;

%% Output 
currx = I(1);
curry = I(2);
currz = I(3);

end

