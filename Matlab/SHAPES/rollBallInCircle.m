%% Rolls a Magnet Ball in a Circle Trajectory Using Magnetic Field
% Author: Mohamed Ghori
% Reference Material: 
% A. J. Petruska, J. B. Brink, and J. J. Abbott, "First Demonstration of a Modular and Reconfigurable Magnetic-Manipulation System," IEEE Int. Conf. Robotics and Automation, 2015 (to appear). 
% A. J. Petruska, A. W. Mahoney, and J. J. Abbott, "Remote Manipulation with a Stationary Computer-Controlled Magnetic Dipole Source," IEEE Trans. Robotics, 30(5):1222-1227, 2014. 
% A. J. Petruska and J. J. Abbott, "Omnimagnet: An Omnidirectional Electromagnet for Controlled Dipole-Field Generation," IEEE Trans. Magnetics, 50(7):8400810(1-10), 2014. 

% INSTRUCTIONS
%{              
%               Init-x(x0) Init-y(y0) Init-phi(phi) Init-psi(psi) 
%               NOTE: "Init position and Orientation"
%               Radius of circle(radius)
%               NOTE: "radius of circle about origin"
%               Period(T) Time-Step(dt)
%               NOTE: "The total time to complete the Circle and the number
%                      of steps to be displayed"
%}

function [ currX, currY, currZ ] = rollBallInCircle(x0,y0,phi,psi,radius,T,dt,speed,ballsize)
%Print Task Name
Task = 'Running Roll Ball in Circle';
%---------------------
% rollBallInCircle Returns the Circle path to follow
%
%   rollBallInCircle() 
%   "Returns a path of a Circle shape with a radius length of 1"
%   
%   rollBallInCircle(x0,y0,phi,psi,radius) 
%   "Returns a path of a Circular shape with an init position and orientation 
%    and far radius 'x0' 'y0' 'phi' 'psi' & 'radius' "
%
%   rollBallInCircle(x0,y0,phi,psi,radius,T,dt) 
%   "Returns a path of a Circular shape with an init position and orientation 
%    and far radius 'x0' 'y0' 'phi' 'psi' & 'radius' "
%    with Time to completion and timestep 'T' & 'dt' "
%
%   [ currX, currY, currZ ] = rollBallInCircle(x0,y0,phi,psi,radius,T,dt,speed,ballsize) 
%   "Returns a path of a Circular shape with an init position and orientation 
%    and far radius 'x0' 'y0' 'phi' 'psi' & 'radius' "
%    with Time to completion and timestep 'T' & 'dt' 
%    with ball-size and video speed 'ballsize' 'speed'"
%
% EX___  
%   [ currX, currY, currZ ] = rollBallInCircle(0,0,pi,pi,1,10,0.1,1,1);
%

%% rollBallInCircle
% Enough Inputs EXCEPTION
if nargin == 0||nargin == 5||nargin == 7||nargin == 9
    % Default Variables 
    if nargin == 0
        x0 = 10;
        y0 = 0;
        phi = 0;
        psi = 0;
        radius = 10;
        speed = 1;
        ballsize = 1;
        T = 10;
        dt = 0.1;
    end
    %% rollBallInCircle
    % Initiate current vectors
    currX = [];
    currY = [];
    currZ = [];

    % First Orientation
    wRb = rotz(psi)*roty(phi);

    % Angle Step size
    del = 2*pi/(T/dt);

    pos1 = [x0;y0;0];
    % Full Cirlce
    for Q = 0:del:2*pi
        % Pos current
        pos2 = [radius*cos(Q);radius*sin(Q);0]
        if isnan(pos1)==0
        % Use ballfwd Control
        [ currx, curry, currz, wRb] = ballfwd(pos1,pos2,wRb,dt,dt,speed,ballsize);
        % Set Required Current Vecotrs 
        currX = [currX;currx];
        currY = [currY;curry];
        currZ = [currZ;currz];
        end
        % set Old pos
        pos1 = pos2
    end
else
    ERROR = 'Not Enough Input Arguments'
end
end

