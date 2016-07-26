%% Rotation Matrix About X-Axis
% Author: Mohamed Ghori
% INSTRUCTIONS
%{
% Call Using the Angle of Rotation about X-Axis (Roll)
%}

function [ R,Task ] = rotx(roll)
%Print Task Name
Task = 'Constructing Rotation-X Matrix';
%---------------------

%% X-Axis Rotation Matrix
R = [1 0 0; 0 cos(roll) -sin(roll);...
            0 sin(roll) cos(roll)];

end

