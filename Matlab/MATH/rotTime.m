%% FULL WINDED TITLE 
%%                      Author: Mohamed K. Ghori B.S. M.E.
%                      ------------------------------------
% 
% Acknowledgements:
%


function [ t, Task ] = rotTime(deltheta,omega0, alph)
%Print Task Name
Task = 'Finding time to complete rotation';
%---------------------
% Desccription of Function
%   rotTime(theta to travel, current angular velocity, Torque applied, Moment of Inertia)
%
% EX__
%  [t,Task] = rotTime(pi,1,1) 
%   
% Compact Text Format
format compact

%% rotTime
% Enough Inputs EXCEPTION
if nargin == 3
    
    % aph = [a;b;c];
    % omega = [d;e;f];
    % axis; angle 
    % delangle is set to anglef i guess 
    % thf = th0+om0*t+0.5*aph*t^2
    % omega = omega0 + aph*t
    %end
    % 
    t = max((-omega0+sqrt(omega0^2-4*alph*deltheta))/(2*alph),(-omega0+sqrt(omega0^2-4*alph*deltheta))/(2*alph));
else
    display('ERROR: Not Enough Input Arguments');
end
end

