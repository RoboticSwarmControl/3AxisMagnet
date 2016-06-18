%% Rolls a Magnet Ball in a Rectangular Trajectory Using Magnetic Field
% Author: Mohamed Ghori
% Reference Material: 
% A. J. Petruska, J. B. Brink, and J. J. Abbott, "First Demonstration of a Modular and Reconfigurable Magnetic-Manipulation System," IEEE Int. Conf. Robotics and Automation, 2015 (to appear). 
% A. J. Petruska, A. W. Mahoney, and J. J. Abbott, "Remote Manipulation with a Stationary Computer-Controlled Magnetic Dipole Source," IEEE Trans. Robotics, 30(5):1222-1227, 2014. 
% A. J. Petruska and J. J. Abbott, "Omnimagnet: An Omnidirectional Electromagnet for Controlled Dipole-Field Generation," IEEE Trans. Magnetics, 50(7):8400810(1-10), 2014. 

% INSTRUCTIONS
%{
% Call Using: 
%               Init-x(x0) Init-y(y0) Init-phi(phi) Init-psi(psi) 
%               NOTE: "Init position and Orientation"
%               Oposite-Corner-x(corner(1)) Oposition-Corner-y(corner(2))
%               NOTE: "Position and Orientation of 'far' or 3rd corner"
%               Period(T) Time-Step(dt)
%               NOTE: "The total time to complete the square and the number
%                      of steps to be displayed"
% Help:         [ Current, currY, currZ ] = rollBallInSquare( x0, y0, phi, psi,corner,speed,ballsize,T,dt)
%               
%}

function [ currX, currY, currZ ] = rollBallInSquare(x0,y0,phi,psi,corner,T,dt,speed,ballsize)
%Print Task Name
Task = 'Running Roll Ball in Square'
%---------------------
% rollBallInSquare Returns the square path to follow
%
%   [ currX, currY, currZ ] = rollBallInSquare() 
%   "Returns a path of a square shape with a side length of 1"
%   
%   [ currX, currY, currZ ] = rollBallInSquare(x0,y0,phi,psi,corner) 
%   "Returns a path of a rectangular shape with an init position and orientation 
%    and far corner 'x0' 'y0' 'phi' 'psi' & 'corner' "
%
%   [ currX, currY, currZ ] = rollBallInSquare(x0,y0,phi,psi,corner,T,dt) 
%   "Returns a path of a rectangular shape with an init position and orientation 
%    and far corner 'x0' 'y0' 'phi' 'psi' & 'corner' "
%    with Time to completion and timestep 'T' & 'dt' "
%
%   [ currX, currY, currZ ] = rollBallInSquare(x0,y0,phi,psi,corner,T,dt,speed,ballsize) 
%   "Returns a path of a rectangular shape with an init position and orientation 
%    and far corner 'x0' 'y0' 'phi' 'psi' & 'corner' "
%    with Time to completion and timestep 'T' & 'dt' 
%    with ball-size and video speed 'ballsize' 'speed'"
%
% EX___  
%   [ currX, currY, currZ ] = rollBallInSquare(0,0,pi,pi,[10;10],10,0.1,1,1);
%

%% rollBallInSquare
% Enough Inputs EXCEPTION
if nargin == 0||nargin == 5||nargin == 7||nargin == 9
    % Default Params____ 
    % Zero Params
    if nargin == 0
        x0 = 0;
        y0 = 0;
        phi = 0;
        psi = 0;
        corner = [1;1];
        T = 1;
        dt = 0.1;
        speed = 1;
        ballsize = 1;
    end

    % 5 Params "Square Dimensions Only"
    if nargin == 5
        T = 1;
        dt = 0.1;
        speed = 1;
        ballsize = 1;    
    end

    % 7 params "Size of Ball & Video Speed Unknown"
    if nargin == 7
        speed = 1;
        ballsize = 1;
    end

    %% rollBallInSquare
    %  --------
    % |        |  W
    % |        |
    %  --------
    %     L
    % Length and Widht of the "Square"
    L = abs(x0-corner(1));
    W = abs(y0-corner(2));

    % Time per Meter 
    %for a constant velocity accross each leg
    delT = T/(2*W + 2*L);

    % Period Per Leg
    TW = delT*W;
    TL = delT*L;

    % Initialize Currents
    currX = [];
    currY = [];
    currZ = [];

    % First Orientation
    wRb = rotz(psi)*roty(phi);

    % Four Corners of Rectangle
    corners = [x0 y0 0; corner(1) y0 0;corner(1) corner(2) 0; x0 corner(2) 0];


    %% Rolling Ball in Square

    % First Leg
    % Use ballfwd Control
    [ currx, curry, currz, wRb] = ballfwd(corners(1,:)',corners(2,:)',wRb,TL,dt,speed,ballsize)
    % Set Required Current Vecotrs 
    currX = [currX;currx];
    currY = [currY;curry];
    currZ = [currZ;currz];
    %}
    % Second Leg
    [ currx, curry, currz, wRb] = ballfwd(corners(2,:)',corners(3,:)',wRb,TW,dt,speed,ballsize)
    % Set Required Current Vecotrs
    currX = [currX;currx];
    currY = [currY;curry];
    currZ = [currZ;currz];

    % Third Leg
    [ currx, curry, currz, wRb] = ballfwd(corners(3,:)',corners(4,:)',wRb,TL,dt,speed,ballsize)
    % Set Required Current Vecotrs
    currX = [currX;currx];
    currY = [currY;curry];
    currZ = [currZ;currz];

    % Fourth Leg
    [ currx, curry, currz, wRb] = ballfwd(corners(4,:)',corners(1,:)',wRb,TW,dt,speed,ballsize)
    % Set Required Current Vecotrs
    currX = [currX;currx];
    currY = [currY;curry];
    currZ = [currZ;currz];
else
    ERROR = 'Not Enough Input Arguments'
end
end


