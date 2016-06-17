%% Moves Ball Forward in a Straight Line
% INSTRUCTIONS
%{
% This function rolls a ball from p1 to p2 in a line
% Call Using:
%               Init-pos(p1) Init-pos(p2) Init-Orientation(wRb)
%               NOTE: "Init position and Orientation"
%               Period(T) Time-Step(dt) speed ballsize
%               NOTE: "The total time to complete a line, the number
%                      of steps to be displayed, speed of video, 
%                      and ball ballsize "
%}

function [ currx, curry, currz, wRb ] = ballfwd( T,p1,p2,wRb,dt,speed,ballsize)
%Print Task Name
Task = 'Running Move Ball Fwd'
%---------------------

%% Initiate Current
% Decompose wRb into phi & psi components
[phi, psi] = findPhiPsi(wRb);
% Finding required current for phi and psi orientation
[currX, currY, currZ] = inverseMagneticField(p1(1), p1(2), phi, psi)
currx = currX;
curry = currY;
currz = currZ;

%% Rotate Ball about world-z-axis
% direction vector from p1 to p2 in world
direction = (p2-p1)/norm(p2-p1)
% unit vector of Magnetic-Field y-axis in world x-y-plane
yaxis = [wRb(4:5)';0];
% Angle Between Magnetic-Field y-axis and vector pointing to next position
theta = (pi/2 - anglediff(direction,yaxis))
% For no rotation needed EXCEPTION
if isnan(direction)
    theta = 0;
end

    %% Visualization of Rotation
    % Number of steps in rotation
    rotsteps = floor(T/(4*dt));
    % Angular Velocity of rotation
    omegaz = theta*4/T;
    % Seperate Rotation Matrix for visualization
    Rrot = wRb;
    % Visualization Function of ball after Rotation
    for n = 1 : rotsteps
        % Rotation
        Rrot = rotz(omegaz*dt)*Rrot;
        % Visualization Function of ball after Rotation
        plot_ball(ballsize,p1,Rrot,dt,speed);
        % Find the Neccessary Current and add it to current set 
            % Decompose wRb into phi & psi components    
            [phi, psi] = findPhiPsi(Rrot);
            % Finding required current for phi and psi orientation of
            % magnet-field
            [currX, currY, currZ] = inverseMagneticField(p1(1), p1(2), phi, psi)
            currx = [currx;currX];
            curry = [curry;currY];
            currz = [currz;currZ];

    end
    
    % Pre multiply rotation for fixed frame
    wRb = rotz(theta)*wRb;
    % Visualization Function of ball after Rotation 
    plot_ball(ballsize,p1,wRb,0,speed);
    % Find the Neccessary Current and add it to current set 
        % Decompose wRb into phi & psi components    
        [phi, psi] = findPhiPsi(Rrot);
        % Finding required current for phi and psi orientation of
        % magnet-field
        [currX, currY, currZ] = inverseMagneticField(p1(1), p1(2), phi, psi);
        currx = [currx;currX];
        curry = [curry;currY];
        currz = [currz;currZ];

%% Translation
% Rotation required to reach p2 with no slip condition
gama = norm(p2-p1)/(ballsize);

    %% Visualization of Translation
    % Number of steps in Translation
    transteps = floor(3*T/(4*dt));
    % Angular Velocity of rotation
    omegay = gama*4/(3*T)
    % Linear Velocity of ball
    vel = abs(omegay)*ballsize;
    % Rotation matrix for visualization
    Rtrans = wRb; 
    % Position vector for visualization
    p = p1;
    for n = 1 : transteps
        % Rotation
        Rtrans = Rtrans*roty(omegay*dt);
        % Position of Ball
        p = p + vel*direction*dt; 
        % Visualization Function of ball after Rotation
        plot_ball(ballsize,p,Rtrans,dt,speed);
        % Find the Neccessary Current and add it to current set 
            % Decompose wRb into phi & psi components    
            [phi, psi] = findPhiPsi(Rrot);
            % Finding required current for phi and psi orientation of
            % magnet-field
            [currX, currY, currZ] = inverseMagneticField(p1(1), p1(2), phi, psi);
            currx = [currx;currX];
            curry = [curry;currY];
            currz = [currz;currZ];

    end
    
    % Pre multiply rotation for fixed frame
    wRb = wRb*roty(gama);
    % Visualization Function of ball after Rotation 
    plot_ball(ballsize,p2,wRb,0,speed)
    % Find the Neccessary Current and add it to current set 
        % Decompose wRb into phi & psi components    
        [phi, psi] = findPhiPsi(Rrot);
        % Finding required current for phi and psi orientation of
        % magnet-field
        [currX, currY, currZ] = inverseMagneticField(p1(1), p1(2), phi, psi);
        currx = [currx;currX];
        curry = [curry;currY];
        currz = [currz;currZ];

    

end

