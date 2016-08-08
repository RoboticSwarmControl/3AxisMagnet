%% Rolls a Magnet Ball in a Rectangular Trajectory Using Magnetic Field
% Author: Mohamed Ghori
% Reference Material: 
% A. J. Petruska, J. B. Brink, and J. J. Abbott, "First Demonstration of a Modular and Reconfigurable Magnetic-Manipulation System," IEEE Int. Conf. Robotics and Automation, 2015 (to appear). 
% A. J. Petruska, A. W. Mahoney, and J. J. Abbott, "Remote Manipulation with a Stationary Computer-Controlled Magnetic Dipole Source," IEEE Trans. Robotics, 30(5):1222-1227, 2014. 
% A. J. Petruska and J. J. Abbott, "Omnimagnet: An Omnidirectional Electromagnet for Controlled Dipole-Field Generation," IEEE Trans. Magnetics, 50(7):8400810(1-10), 2014. 
% Link: http://www.telerobotics.utah.edu/index.php/Research/Omnimagnets

function [ curra, currb, currc, wHb, Task ] = rollBallInRectangle(wHb,corner,T,dt,speed,ballsize)
%Print Task Name
Task = 'Running Roll Ball in Rectangle';
%---------------------
% rollBallInRectangle rolls the ballmagnet in a rectangular path and 
% returns the the required solenoid-current coresponding to each
% orientation at each step in the path
%
%   rollBallInRectangle() 
%   "Returns a path of a rectangle shape"
%   
%   rollBallInRectangle(wHb,corner) 
%   "Returns a path of a rectangular shape with an init Homogeneous 
%    transformation from the world frame to the ball frame of 'wHb'
%    and a 3rd corner [x2,y2] 'corner' "
%
%   rollBallInRectangle(wHb,corner,T,dt) 
%   "Returns a path of a rectangular shape with an init Homogeneous 
%    transformation from the world frame to the ball frame of 'wHb'
%    and a 3rd corner [x2,y2] 'corner'
%    with a period to complete path and timestep 'T' & 'dt' "
%
%   rollBallInRectangle(wHb, corner,T,dt,speed,ballsize) 
%   "Returns a path of a rectangular shape with an init Homogeneous 
%    transformation from the world frame to the ball frame of 'wHb'
%    and a 3rd corner [x2,y2] 'corner'
%    with a period to complete path and timestep 'T' & 'dt'
%    with ball-size and video speed as 'ballsize' 'speed'"
%
% EX___  
%   [ currX, currY, currZ ] = rollBallInRectangle([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1],[10;10],10,0.1,1,1);
%

%% rollBallInRectangle
% Enough Inputs EXCEPTION
if nargin == 0||nargin == 5||nargin == 7||nargin == 9
    % Default Params____ 
    
    %% Zero Params
    if nargin == 0
        % Init x
        x0 = 5;
        % Init y
        y0 = 5;
        % Init pos
        p0 = [x0;y0;0];
        % Init latitude
        phi = 0;
        % Init longitude
        psi = 0;
        % Init rot vector(rotation in world-z then magnetic-y)
        R0 = roty(phi)*rotz(psi);
        % Init Homgeneous 
        wHb = [R0,p0;0 0 0 1];
        % 3rd Corner of the rectangular trajectory 
        corner = [-5;-5];
        % Time to completion of trajectory
        T = 10;
        % time step at which to reccord
        dt = 0.05;
        % speed of video
        speed = 0.5;
        % tool size
        ballsize = 1;
    end

    %% 2 Params "Rectangle Dimensions Only Given"
    if nargin == 2
        %  Time  t completion of trajectory
        T = 10;
        % time step at which to reccord
        dt = 0.1;
        % speed of video
        speed = 100;
        % tool size
        ballsize = 1;    
    end

    %% 4 params "Size of Ball & Video Speed Unknown"
    if nargin == 4
        % speed of video
        speed = 1;
        % size of tool 
        ballsize = 1;
    end

    % Column of Homogeneous
        %xcol= 0;
        %ycol= 4;
        %zcol= 8;
        pcol= 12; 
    % ----------------------
    
    %% rollBallInRectangle
    %  --------
    % |        |  W
    % |        |
    %  --------
    %     L
    % Length and Width of the "Rectangle"
    L = abs(wHb(pcol+1)-corner(1));
    W = abs(wHb(pcol+2)-corner(2));

    % Time per Meter 
    %for a constant velocity accross each leg
    delT = T/(2*W + 2*L);

    % Period Per Leg
    TW = delT*W;
    TL = delT*L;

    % Initialize Currents
    curra = [];
    currb = [];
    currc = [];

    % Four Corners of Rectangle
    corners = [wHb(pcol+1) wHb(pcol+2) 0; corner(1) wHb(pcol+2) 0;corner(1) corner(2) 0; wHb(pcol+1) corner(2) 0];


    %% Rolling Ball in Rectangle

    % First Leg
    % Use ballfwd Control
    [ currX, currY, currZ, wHb] = ballfwd(wHb,corners(2,:)',TL,dt,speed,ballsize);
    % Set Required Current Vecotrs 
    curra = [curra;currX];
    currb = [currb;currY];
    currc = [currc;currZ];
    %}
    % Second Leg
    [ currX, currY, currZ, wHb] = ballfwd(wHb,corners(3,:)',TW,dt,speed,ballsize);
    % Set Required Current Vecotrs
    curra = [curra;currX];
    currb = [currb;currY];
    currc = [currc;currZ];

    % Third Leg
    [ currX, currY, currZ, wHb] = ballfwd(wHb,corners(4,:)',TL,dt,speed,ballsize);
    % Set Required Current Vecotrs
    curra = [curra;currX];
    currb = [currb;currY];
    currc = [currc;currZ];

    % Fourth Leg
    [ currX, currY, currZ] = ballfwd(wHb,corners(1,:)',TW,dt,speed,ballsize);
    % Set Required Current Vecotrs
    curra = [curra;currX];
    currb = [currb;currY];
    currc = [currc;currZ];
else
    print('ERROR = Not Enough Input Arguments')
end
end


