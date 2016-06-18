%% Play Back Function that reconstructs a path based on recquired current 
%% <- for each orientation of the magnetic Ball
% Author: Mohamed Ghori
% Reference Material: 
% A. J. Petruska, J. B. Brink, and J. J. Abbott, "First Demonstration of a Modular and Reconfigurable Magnetic-Manipulation System," IEEE Int. Conf. Robotics and Automation, 2015 (to appear). 
% A. J. Petruska, A. W. Mahoney, and J. J. Abbott, "Remote Manipulation with a Stationary Computer-Controlled Magnetic Dipole Source," IEEE Trans. Robotics, 30(5):1222-1227, 2014. 
% A. J. Petruska and J. J. Abbott, "Omnimagnet: An Omnidirectional Electromagnet for Controlled Dipole-Field Generation," IEEE Trans. Magnetics, 50(7):8400810(1-10), 2014. 

% INSTRUCTIONS
%{
% Call Using:
%               Current position (p0)
%               Solenoid current vector(currx, curry, currz)
%               Ball size(size)
%               speed of video(speed)
%               Period(T)
%               time step(dt)
%}

function  playback(currx,curry,currz,p0, T,dt, ballsize,speed)
%Print Task Name
Task = 'Running playback'
%---------------------
% playback runs through a array of current values for the Omnimagnet's
% 3-Solenoids in order to reconstruct a path
% 
%   playback( p0,currx,curry,currz,T,dt, ballsize,speed)
%   "runs through a array of current values for the Omnimagnet's
%    3-Solenoids 'currx' 'curry' 'currz' given an intial position 'p0'
%    with Time to completion and timestep 'T' & 'dt' 
%    with ball-size and video speed 'ballsize' 'speed'"
%
% EX___  
%   playback([0;0;0],1,2,3,10,0.1,1,1);
%

%% playback
% Enough Inputs EXCEPTION
if nargin == 8
    % Number of recorded positions 
    arraysize = size(currx)
    % Init rotation matrix
    wRb = eye(3);
    % Loop through all orientations
    for n = 1:1:arraysize-1
        % Initial position Current 
        I0 = [currx(n);curry(n);currz(n)]
        % Next position Current
        If = [currx(n+1);curry(n+1);currz(n+1)]
        % Move ball according to current 
        [ pf, wRb ] = fwdcurrent(I0, If,p0,wRb,T,dt,speed,ballsize)
        % set next initial position
        p0 = pf
    end

else
    ERROR = 'Not Enough Input Arguments'
end
end

