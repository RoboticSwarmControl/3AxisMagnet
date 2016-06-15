%% Rotation Matrix About Y-Axis
% INSTRUCTIONS
%{
% Call Using the Angle of Rotation about Y-Axis (pitch)
%}

function [ R ] = roty(pitch)
%Print Task Name
Task = 'Constructing Rotation-Y Matrix'
%---------------------

%% Y-Axis Rotation Matrix
R = [ cos(pitch) 0 sin(pitch);...
      0 1 0; -sin(pitch) 0 cos(pitch)];

end

