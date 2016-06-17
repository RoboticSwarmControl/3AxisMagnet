%% Converts current to change in orientation and position
% INSTRUCTIONS
%{
% Call Using: 
%               Initial Current(I0)
%               Final Current(If)
%               Initial Position(p0)
%}

function [ pf, wRb ] = fwdcurrent(I0, If,wRb,p0,speed,size,T,dt)
%Print Task Name
Task = 'Running Current to Step'
%---------------------

%% Find Rotation
% Initial Orientation
[phi1, psi1] = fwdMagneticField( I0(1), I0(2), I0(3), p0(1), p0(2) )
% Final Orientation
[phi2, psi2] = fwdMagneticField( If(1), If(2), If(3), p0(1), p0(2) )
% Rotation about world-z-axis
psi = psi2 -psi1;
% Rotation about world-y-axis
phi = phi2-phi1;

%% Rotate Ball NOTE: Rotation takes T/4 time and Translate takes 3T/4 time
    %% Visualization of Rotation
    % Number of steps in rotation
    rotsteps = floor(T/(4*dt));
    % Angular Velocity of rotation
    omegaz = psi*4/T;
    % Seperate Rotation Matrix for visualization
    Rrot = wRb;
    for n = 1 : rotsteps
        % Rotation
        Rrot = rotz(omegaz*dt)*Rrot;
        % Visualization Function of ball after Rotation
        plot_ball(size,p0,Rrot,dt,speed);
    end
    % Pre multiply rotation for fixed frame
    wRb = rotz(psi)*wRb
    % Visualization Function of ball after Rotation 
    plot_ball(size,p0,wRb,0,speed);
    
%% Translation    
    %% Visualization of Translation
    % Number of steps in Translation
    transteps = floor(3*T/(4*dt));
    % Angular Velocity of rotation
    omegay = phi*4/(3*T)
    % Linear Velocity of ball
    vel = omegay*size;
    % Rotation matrix for visualization
    Rtrans = wRb
    % Position vector for visualization
    p = p0;
    for n = 1 : transteps
        % Rotation
        Rtrans = roty(omegay*dt)*Rtrans;
        % Position of Ball
        p = p + vel*[0;1;0]*dt; 
        % Visualization Function of ball after Rotation
        plot_ball(size,p,Rtrans,dt,speed);
        
    end
    % Pre multiply rotation for fixed frame
    wRb = roty(phi)*wRb;
    % Visualization Function of ball after Rotation 
    plot_ball(size,p,wRb,0,speed)
    % Final position
    pf = p;

end

