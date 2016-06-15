%% Finds the angle and direction of rotation to go from vect1 to vect2 all in the world frame
% INSTRUCTIONS
%{
% Call using two vectors vect1(current) vect2(destination)
%}

function [ theta ] = anglediff( vect1,vect2 )
%Print Task Name
Task = 'Running Find the Angle between two Vectors'
%---------------------
% world x-axis
xaxis = [1;0;0];
% Angle Between ball x-axis and vect1
theta1 = acos(dot(vect1,xaxis)/(norm(xaxis)*norm(vect1)));
% Angle Between ball x-axis and vect1
theta2 = acos(dot(vect2,xaxis)/(norm(xaxis)*norm(vect2)));

% Difference Angle between two vectors in the world x-y plane 
theta = theta2-theta1;

end

