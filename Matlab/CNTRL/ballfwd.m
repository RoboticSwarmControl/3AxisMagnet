%% Moves Ball Forward in a Straight Line
% INSTRUCTIONS
%{
% This function rolls a ball from p1 to p2 in a line
% Call Using:
%               Init-pos(p1) Init-pos(p2) Init-Orientation(wRb)
%               NOTE: "Init position and Orientation"
%               Period(T) Time-Step(dt) speed size
%               NOTE: "The total time to complete a line, the number
%                      of steps to be displayed, speed of video, 
%                      and ball size "
%}

function [ currx, curry, currz, wRb ] = ballfwd( T,p1,p2,wRb,dt,speed,size)
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

%% Rotate Ball
% direction vector from p1 to p2 in world
direction = (p2-p1)/norm(p2-p1);
% unit vector of Magnetic Field x-axis in world x-y-plane
xaxis = [wRb(1:2)';0];

% Angle Between ball x-axis and vector between initial and final point
theta = anglediff(xaxis,direction);

    %% Visualization of Rotation
    % Number of steps in rotation
    rotsteps = floor(T/(4*dt));
    % Angular Velocity of rotation
    omegax = theta*4/T;
    % Seperate Rotation Matrix for visualization
    Rrot = wRb;
    for n = 1 : rotsteps
        % Rotation
        Rrot = rotz(omegax*dt)*Rrot;
        % Visualization Function of ball after Rotation
        plot_ball(size,p1,Rrot,dt,speed);
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
    plot_ball(size,p1,wRb,0,speed);
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
gama = norm(p2-p1)/(size);
    
    %% Visualization of Translation
    % Number of steps in Translation
    transteps = floor(3*T/(4*dt));
    % Angular Velocity of rotation
    omegay = gama*4/(3*T)
    % Linear Velocity of ball
    vel = omegay*size;
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
        plot_ball(size,p,Rtrans,dt,speed);
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
    plot_ball(size,p2,wRb,0,speed)
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

