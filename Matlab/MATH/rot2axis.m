%% Rotation in Axis Angle notation
%%                      Author: Mohamed K. Ghori B.S. M.E.
%                      ------------------------------------
% 


function [ axis, angle, Task ] = rot2axis(R)
%Print Task Name
Task = 'Converting to Axis Angle';
%---------------------
% Desccription of Function
%   rot2axis(Rotation Matrix)
%
% EX__
%  [axis, angle,Task] = rot2axis(eye(3)) 
%   
% Compact Text Format
format compact

%% rot2axis
% Enough Inputs EXCEPTION
if nargin == 1
%% Quaternion Version of Axis Angle
        % Quaternion
        Q = rot2quat(R);
        % Angle of Rotation
        angle = 2*atan2(norm(Q(2:4)),Q(1));
        % Define Axis
        if angle == 0
            % scalar multiplier
            axis = [0;0;0];
        else
            % scalar multiplier
            axis = Q(2:4)/sin(angle/2); 
        end
else
    display('ERROR: Not Enough Input Arguments');
end
end

