%% Solenoid-current Required for a Specific Orientation of Magnetic-Field
%% <- in the Workspace
% Author: Mohamed Ghori
% Reference Material: 
% A. J. Petruska, J. B. Brink, and J. J. Abbott, "First Demonstration of a Modular and Reconfigurable Magnetic-Manipulation System," IEEE Int. Conf. Robotics and Automation, 2015 (to appear). 
% A. J. Petruska, A. W. Mahoney, and J. J. Abbott, "Remote Manipulation with a Stationary Computer-Controlled Magnetic Dipole Source," IEEE Trans. Robotics, 30(5):1222-1227, 2014. 
% A. J. Petruska and J. J. Abbott, "Omnimagnet: An Omnidirectional Electromagnet for Controlled Dipole-Field Generation," IEEE Trans. Magnetics, 50(7):8400810(1-10), 2014. 

% INSTRUCTIONS
%{
% Call using the current pos(x,y) of the ball 
% and orientation of the magnetic field(phi,psi)
%}

function [ currx, curry, currz ] = inverseMagneticField(x,y,phi,psi)
%Print Task Name
Task = 'Running Find Required Current'
%---------------------
% 
% inverseMagneticField uses the net-magnetic-field equations given by the 
% papers above to find what solenoid-current combinations result in a 
% desired orientation
%
%   [ currx, curry, currz ] = inverseMagneticField(x,y,phi,psi)
%   "Returns current in each solenoid given the an input of current
%    position of the magnet-ball and orientation of the magnetic field
%    'x' 'y' 'phi' & 'psi' "
%
%   EX___
%     [ currx, curry, currz ] = inverseMagneticField(0,0,pi,pi)
%

%% inverseMagneticField 
% Enough Inputs EXCEPTION
if nargin == 5
    % Mapping of Magnetic Field to Current Based on Physical 
    % Attributes of Solenoid
    M = eye(3);
    % position of the ball center
    pos = [x,y,0];
    % Magnitude of Field
    a = 100;
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
else
    ERROR = 'Not Enough Input Arguments'
end

end

