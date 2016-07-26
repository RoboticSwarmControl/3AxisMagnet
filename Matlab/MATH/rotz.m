%% Rotation Matrix About z-Axis
% Author: Mohamed Ghori
% INSTRUCTIONS
%{
% Call Using the Angle of Rotation about Z-Axis (yaw)
%}

function [ R, Task ] = rotz(yaw)
%Print Task Name
Task = 'Constructing Rotation-z Matrix';
%---------------------

%% z-Azis Rotation Matriz
R = [cos(yaw) -sin(yaw) 0;...
     sin(yaw) cos(yaw) 0; 0 0 1];

end

