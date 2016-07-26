%% Finds the angle and direction of rotation to go from vect1 to vect2 all in the world frame
% Author: Mohamed Ghori
% INSTRUCTIONS
%{
% Call using two vectors vect1(current) vect2(destination)
%}

function [ theta,Task ] = anglediff( vect1,vect2 )
%Print Task Name
Task = 'Running Find the Angle between two Vectors';
%---------------------

% Angle Between world x-axis and vect1
theta1 = atan2(vect1(2),vect1(1));
% Angles past 180 degrees EXCEPTION
if theta1 <0 
    theta1 = 2*pi + theta1;
end

% Angle Between world y-axis and vect2
theta2 = atan2(vect2(2),vect2(1));
% Angles past 180 degrees EXCEPTION
if theta2 <0
    theta2 = 2*pi + theta2;
end

% Difference Angle between two vectors in the world x-y plane 
theta = theta1 - theta2;
%  if theta <0
%      theta = theta + 2*pi;
%  end
end

