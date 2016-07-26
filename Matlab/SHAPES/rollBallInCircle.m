%% Rolls a Magnet Ball in a Circle Trajectory Using Magnetic Field
% Author: Mohamed Ghori
% Reference Material: 
% A. J. Petruska, J. B. Brink, and J. J. Abbott, "First Demonstration of a Modular and Reconfigurable Magnetic-Manipulation System," IEEE Int. Conf. Robotics and Automation, 2015 (to appear). 
% A. J. Petruska, A. W. Mahoney, and J. J. Abbott, "Remote Manipulation with a Stationary Computer-Controlled Magnetic Dipole Source," IEEE Trans. Robotics, 30(5):1222-1227, 2014. 
% A. J. Petruska and J. J. Abbott, "Omnimagnet: An Omnidirectional Electromagnet for Controlled Dipole-Field Generation," IEEE Trans. Magnetics, 50(7):8400810(1-10), 2014. 
% Link: http://www.telerobotics.utah.edu/index.php/Research/Omnimagnets

function [ currX, currY, currZ ,Task] = rollBallInCircle(wHb,radius,T,dt,speed,ballsize)
%Print Task Name
Task = 'Running Roll Ball in Circle';
%---------------------
% rollBallInCircle Returns rolls the ballmagnet in a cirucular path and 
% returns the the required solenoid-current coresponding to each
% orientation at each step in the path
%
%   rollBallInCircle() 
%   "Returns a path of a Circle shape"
%   
%   rollBallInCircle(wHb,radius) 
%   "Returns a path of a rectangular shape with an init Homogeneous
%    from the world frame to the ball frame of 'wHb' and with a specific 
%    'radius' "
%
%   rollBallInCircle(wHb,radius,T,dt) 
%   "Returns a path of a rectangular shape with an init Homogeneous
%    from the world frame to the ball frame of 'wHb' and with a specific 
%    'radius'
%    with a period to complete path and timestep 'T' & 'dt'
%
%   rollBallInCircle(wHb,radius,T,dt,speed,ballsize) 
%   "Returns a path of a rectangular shape with an init Homogeneous
%    from the world frame to the ball frame of 'wHb' and with a specific 
%    'radius'
%    with a period to complete path and timestep 'T' & 'dt'
%    with ball-size and video speed as 'ballsize' 'speed'"
%
% EX___  
%   [ currX, currY, currZ ] = rollBallInCircle(0,0,pi,pi,1,10,0.1,1,1);
%

%% rollBallInCircle
% Enough Inputs EXCEPTION
if nargin == 0||nargin == 5||nargin == 7||nargin == 9
    % Default Variables 
    %% 0 Input 
    if nargin == 0
        % init x
        x0 = 10;
        % init y
        y0 = 0;
        % Init pos
        p0 = [x0;y0;0];
        % init latitude
        phi = 0;
        % init longitude
        psi = 0;
        % Init rot vector(rotation in world-z then magnetic-y)
        R0 = roty(phi)*rotz(psi);
        % Init Homgeneous 
        wHb = [R0,p0;0 0 0 1];
        % radius of circle
        radius = 10;
        % speed of video
        speed = 1;
        % tool size
        ballsize = 1;
        % time to completion
        T = 10;
        % time step at which to reccord
        dt = 0.1;
    end
    %% 5 Input 
    if nargin == 0
        % time to completion
        T = 10;
        % time step at which to reccord
        dt = 0.1;    
        % speed of video
        speed = 1;
        % tool size
        ballsize = 1;
    end
    %% 7 Input 
    if nargin == 0
        % speed of video
        speed = 1;
        % tool size
        ballsize = 1;
    end
    
    % Column of Homogeneous Matrix
        %xcol= 0;
        %ycol= 4;
        %zcol= 8;
        pcol= 12; 
    % ----------------------
    
    %% rollBallInCircle
    % Initiate current vectors
    currX = [];
    currY = [];
    currZ = [];

    % Angle Step size
    del = 2*pi/(T/dt);
    
    % Full Cirlce
    for Q = 0:del:2*pi
        % Pos current
        pos2 = [radius*cos(Q);radius*sin(Q);0];
        if isnan(wHb(pcol+1:15))==0
            % Time to completion point to point movement
            T = dt;
            % add 4 steps between points
            dt = dt/4;
            % Use ballfwd Control
            [ currx, curry, currz, wHb] = ballfwd(wHb,pos2,T,dt,speed,ballsize);
            % Set Solenoid Current Vecotrs 
            currX = [currX;currx];
            currY = [currY;curry];
            currZ = [currZ;currz];
        end
    end
else
    display('ERROR:Not Enough Input Arguments');
end
end

