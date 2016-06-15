%% Find a phi and psi coresponding to a Rotatoin matrix
% INSTRUCTIONS
%{
% Call using current Rotation matrix
% decomposes R into phi(rotation about world-y-axis ) and psi(rotation about world-z-axis)
%}

function [ phi, psi ] = findPhiPsi( wRb )
%Print Task Name
Task = 'Running Find Phi and Psi'
%---------------------

%% Find phi and psi
% Components of magnet-x-axis in the world-x-z plane
comp = [wRb(1);0;wRb(3)];
% world-x-axis unit vector in world
x_axis = [1;0;0];
% Phi angle about world-y-axis between magnet-x-axis and world x-axis 
phi = acos(dot(comp,x_axis)/(norm(x_axis)*norm(comp)));
% components of magnet-x-axis in the world-x-y plane
comp = [wRb(1);wRb(2);0];
% x-axis unit vector in world
x_axis = [1;0;0];
% Psi angle about world-z-axis between magnet-x-axis and world x-axis 
psi = acos(dot(comp,x_axis)/(norm(x_axis)*norm(comp)));

% Correcting for Negative Values
if wRb(3) <0
    phi = 2*pi - phi;
end
% Correcting for Negative Values
if wRb(2) < 0
    psi = 2*pi - psi;
end
end

