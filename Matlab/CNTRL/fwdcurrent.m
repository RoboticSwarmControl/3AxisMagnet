%% Converts Current to Change in Orientation and Position
% Author: Mohamed Ghori
% Reference Material: 
% A. J. Petruska, J. B. Brink, and J. J. Abbott, "First Demonstration of a Modular and Reconfigurable Magnetic-Manipulation System," IEEE Int. Conf. Robotics and Automation, 2015 (to appear). 
% A. J. Petruska, A. W. Mahoney, and J. J. Abbott, "Remote Manipulation with a Stationary Computer-Controlled Magnetic Dipole Source," IEEE Trans. Robotics, 30(5):1222-1227, 2014. 
% A. J. Petruska and J. J. Abbott, "Omnimagnet: An Omnidirectional Electromagnet for Controlled Dipole-Field Generation," IEEE Trans. Magnetics, 50(7):8400810(1-10), 2014. 


% INSTRUCTIONS
%{
% Call Using: 
%               Initial Current(I0)
%               Final Current(If)
%               Initial Position(p0)
%}

function [ pf, wRb ] = fwdcurrent(I0, If,p0,wRb,T,dt,speed,ballsize)
%Print Task Name
Task = 'Running Current to Step'
%---------------------
%
% fwdcurrent first rotates the ball such that the y-axis of coresponding to
% the orientatoin of the initial and final vector of the 3-solenoid-currents
% coincide second it rolls the ball about the y axis such that the x&z-axies
% coincide 
%
%   [ pf, wRb ] = fwdcurrent(I0, If,p0,wRb,T,dt,speed,ballsize)
%   "Returns a resultant position and orientation vector and matrix,
%    respectiely, when given a current and final solenoid-current
%    vector,current position and orientation
%    a Time to completion and time step, and video speed and ball size:
%    'I0' & 'If' , 'p0' & 'wRb' , 'T' & 'dt' , 'speed' & 'ballsize' 
%
% EX___
%   [ pf, wRb ] = fwdcurrent([1;2;3], [4;;6],[0;0;0],eye(3),10,0.1,1,1)
%   
%% fwdcurrent
% Enough Inputs EXCEPTION
if nargin == 8
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
            plot_ball(ballsize,p0,Rrot,dt,speed);
        end
        % Pre multiply rotation for fixed frame
        wRb = rotz(psi)*wRb
        % Visualization Function of ball after Rotation 
        plot_ball(ballsize,p0,wRb,0,speed);

    %% Translation    
        %% Visualization of Translation
        % Number of steps in Translation
        transteps = floor(3*T/(4*dt));
        % Angular Velocity of rotation
        omegay = phi*4/(3*T)
        % Linear Velocity of ball
        vel = omegay*ballsize;
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
            plot_ball(ballsize,p,Rtrans,dt,speed);

        end
        % Pre multiply rotation for fixed frame
        wRb = roty(phi)*wRb;
        % Visualization Function of ball after Rotation 
        plot_ball(ballsize,p,wRb,0,speed)
        % Final position
    pf = p;
else
    ERROR = 'Not Enough Input Arguments'
end
end

