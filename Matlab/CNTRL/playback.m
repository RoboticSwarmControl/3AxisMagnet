%% FULL WINDED TITLE 
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

function  playback( p0,currx,curry,currz,ballsize,speed,T,dt)
%Print Task Name
Task = 'Running playback'
%---------------------

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
    [ pf, wRb ] = fwdcurrent(I0, If,wRb,p0,speed,ballsize,T,dt)
    % set next initial position
    p0 = pf
end

end

