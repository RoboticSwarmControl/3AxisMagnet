%% Play Back Function that reconstructs a path based on recquired current 
%% <- for each orientation of the magnetic Ball
%                           Author: Mohamed Ghori
%                          -----------------------
% Reference Material: 
% A. J. Petruska, J. B. Brink, and J. J. Abbott, "First Demonstration of a Modular and Reconfigurable Magnetic-Manipulation System," IEEE Int. Conf. Robotics and Automation, 2015 (to appear). 
% A. J. Petruska, A. W. Mahoney, and J. J. Abbott, "Remote Manipulation with a Stationary Computer-Controlled Magnetic Dipole Source," IEEE Trans. Robotics, 30(5):1222-1227, 2014. 
% A. J. Petruska and J. J. Abbott, "Omnimagnet: An Omnidirectional Electromagnet for Controlled Dipole-Field Generation," IEEE Trans. Magnetics, 50(7):8400810(1-10), 2014. 
% Link: http://www.telerobotics.utah.edu/index.php/Research/Omnimagnets

function [wHb,Task] = playback(currx,curry,currz,wHb, T, dt, speed, ballsize)
%Print Task Name
Task = 'Running playback';
%---------------------
% playback runs through the 3 solenoids' current combinations returned from
% the simulation and computes the orientatatoin that corespond to these 
% combinations. These orientataions are used to try and remap the path 
% that was previously simulated. 
%
%   playback( currx,curry,currz,wHb)
%   "runs through a array of current values for the Omnimagnet's
%    3-Solenoids 'currx' 'curry' 'currz' starting wit an initial
%    orientation and position described by wHb"
%
%   playback( currx,curry,currz,wHb,T,dt)
%   "runs through a array of current values for the Omnimagnet's
%    3-Solenoids 'currx' 'curry' 'currz' starting wit an initial
%    orientation and position described by wHb
%    with period to rotate between orientations and timestep 'T' & 'dt' "
%
%   playback( currx,curry,currz,wHb,T,dt, ballsize,speed)
%   "runs through a array of current values for the Omnimagnet's
%    3-Solenoids 'currx' 'curry' 'currz' starting wit an initial
%    orientation and position described by wHb
%    with period to rotate between orientations and timestep 'T' & 'dt' 
%    with ball-size and video speed 'ballsize' 'speed'"
%
% EX___  
%   [wHb, Task] = playback([0;0;0],1,2,3,10,0.1,1,1);
%
% Column of Homogeneous
        %xcol= 0;
        %ycol= 4;
        %zcol= 8;
        %pcol= 12; 
    % ----------------------
    
%% playback
% Enough Inputs EXCEPTION
if nargin == 8 ||nargin == 3 ||nargin == 4 ||nargin == 6
    %% 4 Params
    if nargin == 4
        % Time to completion betweeen points
        T = .1;
        % time step to run play back
        dt = 0.05;
        % speed of the video
        speed = 1;
        % tool size
        ballsize = 1;
    end
    %% 6 Params
    if nargin ==6
        % size of tool
        ballsize = 1;
        % speed of video
        speed =1;
    end
    
    % Number of recorded positions
    arraysize = size(currx);
    
    % Loop through all orientations
    for n = 1:1:arraysize-1
        % Initial position Current 
        I0 = 10*[currx(n);curry(n);currz(n)];
        magfield(I0);
        % Next position Current
        If = 10*[currx(n+1);curry(n+1);currz(n+1)];
        magfield(If);
        % Move ball according to current 
        [ wHb ] = fwdcurrent(I0, If,wHb,T,dt,speed,ballsize);
    end

else
    display('ERROR: Not Enough Input Arguments');
end
end

