%% Solenoid-current Required for a Specific Orientation of Magnetic-Field
%% <- in the Workspace
%                           Author: Mohamed Ghori
%                          -----------------------
% Reference Material: 
% A. J. Petruska, J. B. Brink, and J. J. Abbott, "First Demonstration of a Modular and Reconfigurable Magnetic-Manipulation System," IEEE Int. Conf. Robotics and Automation, 2015 (to appear). 
% A. J. Petruska, A. W. Mahoney, and J. J. Abbott, "Remote Manipulation with a Stationary Computer-Controlled Magnetic Dipole Source," IEEE Trans. Robotics, 30(5):1222-1227, 2014. 
% A. J. Petruska and J. J. Abbott, "Omnimagnet: An Omnidirectional Electromagnet for Controlled Dipole-Field Generation," IEEE Trans. Magnetics, 50(7):8400810(1-10), 2014. 
% Link: http://www.telerobotics.utah.edu/index.php/Research/Omnimagnets

function [ currx, curry, currz, Task ] = inverseMagneticField(H,H0)
%Print Task Name
Task = 'Running Find Required Current';
%---------------------
% 
% inverseMagneticField uses the net-magnetic-field equations given by the 
% papers above to find what solenoid-current combinations result in a 
% desired orientation
%
%   [ currx, curry, currz ] = inverseMagneticField(x,y,phi,psi)
%   "Returns the current in each solenoid needed to produce a certain
%    orientation of magnetic-field at a specified positoin
%    'phi' & 'psi' given 'x' 'y' "
%
%   EX___
%     [ currx, curry, currz ] = inverseMagneticField(0,0,pi,pi)
%

%% inverseMagneticField 
% Enough Inputs EXCEPTION
if nargin == 2
    % Column of Homogeneous
        %xcol= 0;
        %ycol= 4;
        zcol= 8;
        pcol= 12; 
    % ----------------------
    
    
    
    
    
    % Mapping of Magnetic Field to Current Based on Physical 
    % Attributes of Solenoid
    M = [25.1 0 0;0 25.8 0; 0 0 26.3];
    % position of the ball center
    pos = H(pcol+1:15)';
    B = H(zcol+1:zcol+3)';
    %% Eqn B => I
    % Unit Vector for pos
    p_hat = pos/norm(pos);
    % Constant of Permeability
    mu = 4*(10^-7)*pi;
    % Eqn parts for B => I     I = (2pi/mu)*(||P||^3)(M^-1)(3*p^*p^T-2I)*B
    I = (2*pi/mu)*(norm(pos)^3)*(M\(3*p_hat*(p_hat') - 2*eye(3)))*B;
    H0;
    %% Output 
    currx = I(1);
    curry = I(2);
    currz = I(3);
else
    ERROR = 'Not Enough Input Arguments';
    display(ERROR);
end

end

