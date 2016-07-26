%% Find a phi and psi coresponding to a Rotatoin matrix
% Author: Mohamed Ghori
% INSTRUCTIONS
%{
% Call using current Rotation matrix
% decomposes R into phi(rotation about world-y-axis ) and psi(rotation about world-z-axis)
%}

function [ phi, psi, Task] = findPhiPsi( wHb )
%Print Task Name
Task = 'Running Find Phi and Psi';
%---------------------
wRb = wHb(1:3,1:3);
%% Find phi and psi
% Components of magnet-x-axis in the world-x-z plane
comp = [wRb(1);0;wRb(3)];
% world-x-axis unit vector in world
x_axis = [1;0;0];
% Phi angle about world-y-axis between magnet-x-axis and world x-axis 
phi = acos(dot(comp,x_axis)/(norm(x_axis)*norm(comp)));
% Correcting for Negative Values
if wRb(3) <0
    phi = phi+pi;
end


% components of magnet-x-axis in the world-x-y plane
comp = [wRb(1);wRb(2);0];
% x-axis unit vector in world
x_axis = [1;0;0];
% Psi angle about world-z-axis between magnet-x-axis and world x-axis 
psi = acos(dot(comp,x_axis)/(norm(x_axis)*norm(comp)));
% Correcting for Negative Values
if wRb(2) < 0
    psi = psi + pi;
end
end

