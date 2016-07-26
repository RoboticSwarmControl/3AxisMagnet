%% Orientatoin of Magnetic-Field from Specific Solenoid-current Values
%                           Author: Mohamed Ghori
%                          -----------------------
% Reference Material: 
% A. J. Petruska, J. B. Brink, and J. J. Abbott, "First Demonstration of a Modular and Reconfigurable Magnetic-Manipulation System," IEEE Int. Conf. Robotics and Automation, 2015 (to appear). 
% A. J. Petruska, A. W. Mahoney, and J. J. Abbott, "Remote Manipulation with a Stationary Computer-Controlled Magnetic Dipole Source," IEEE Trans. Robotics, 30(5):1222-1227, 2014. 
% A. J. Petruska and J. J. Abbott, "Omnimagnet: An Omnidirectional Electromagnet for Controlled Dipole-Field Generation," IEEE Trans. Magnetics, 50(7):8400810(1-10), 2014. 
% Link: http://www.telerobotics.utah.edu/index.php/Research/Omnimagnets

function [phi, psi, B, Task] = fwdMagneticField( currx, curry, currz, x, y )
%Print Task Name
Task = 'Running Find Magnetic Field from Currents';
%---------------------
%
% fwdMagneticField returns the magnetic-field orientation that coresponds
% to specific solenoid-current vlaues and position of the magnetic ball
%
%   [phi, psi] = fwdMagneticField( currx, curry, currz, x, y )
%   "Returns the magnetic-field orientation at the location of the 
%    ball given the solenoid-current of the Omnimagnet 'x' 'y' 'currx' 
%    'curry' 'currz'"
%
% EX___
%   [phi, psi, B, Task] = fwdMagneticField( 1, 2, 3, 0, 0 )
% 

%% fwdMagneticField
% Enough Inputs EXCEPTION
if nargin == 5
 % Mapping of Magnetic Field to Current Based on Physical 
    % Attributes of Solenoid
    M = eye(3);
    % position of the ball center
    pos = [x,y,0]
    %Define Current-Vector(I)
    I = [currx; curry; currz]

    %% Eqn I => B
    
    % Unit Vecotr for pos
    if norm(pos)==0
    p_hat  = [0;0;0];
    else
    p_hat = pos/norm(pos);
    % Constant of Permeability
    mu = 4*(10^-7)*pi;
    % Eqn parts for B => I 
    temp = (2*pi/mu)*(norm(pos)^3)*(M\(3*p_hat*(p_hat') - 2*eye(3)));
    % Current Vector
    B = temp\I;
    % Unit Vector in the Direction of B
    u = B/norm(B);
    end
    %% Output 
    % Angle from the world-x-axis to B in world-x-z-plane
    phi = atan2(u(3),sqrt(u(2)^2+u(1)^2));
    if phi<0 
        phi = phi+2*pi;
    end
    % Angle from world-x-axis to B in world-x-y-plane
    psi = atan2(u(2),u(1));
    if psi<0
        psi = psi +2*pi;
    end
    % Magnet off exception
    if norm(I) == 0
        phi = 0;
        psi = 0;
    end
else
    ERROR = 'Not Enough Input Arguments';
    display(ERROR);
    
end
end

