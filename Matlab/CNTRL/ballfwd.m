%% Moves Ball Forward in a Straight Line from point1 to point2
% Author: Mohamed Ghori
% Reference Material: 
% A. J. Petruska, J. B. Brink, and J. J. Abbott, "First Demonstration of a Modular and Reconfigurable Magnetic-Manipulation System," IEEE Int. Conf. Robotics and Automation, 2015 (to appear). 
% A. J. Petruska, A. W. Mahoney, and J. J. Abbott, "Remote Manipulation with a Stationary Computer-Controlled Magnetic Dipole Source," IEEE Trans. Robotics, 30(5):1222-1227, 2014. 
% A. J. Petruska and J. J. Abbott, "Omnimagnet: An Omnidirectional Electromagnet for Controlled Dipole-Field Generation," IEEE Trans. Magnetics, 50(7):8400810(1-10), 2014. 
% Link: http://www.telerobotics.utah.edu/index.php/Research/Omnimagnets

function [ currX, currY, currZ, wHb, Task ] = ballfwd(wHb, p2,T,dt,speed,ballsize)
%Print Task Name
Task = 'Running Move Ball Fwd';
%---------------------
%
% ballfwd rotates the ball such that the y-axis is 90 degreees
% perpendicular to the direction of piont2 from point1. second it rolls the
% ball about the y-axis until the ball reaches point2
%
%   [ currx, curry, currz, wRb ] = ballfwd(p1,p2,wRb,T,dt,speed,ballsize)
%   "Returns the solenoid-current combination coresponding to 
%    the series of orientations the magneticc-field must go through during 
%    the ball's path from point1 to point2, Given the 
%    initial point and final ponit, the initial orientation, the time to 
%    complete path, the time step, the speed of the video, and the ball size:
%    'p1' 'p2' 'wRb' 'T' 'dt' 'speed' 'ballsize' respectively
%
% EX__
%   [ currx, curry, currz, wRb ] = ballfwd([0;0;0],[1;1;1],eye(3),10,0.1,1,1)
%   

%% ballfwd
if nargin == 6
    
    % Column of Homogeneous
        %xcol= 0;
        ycol= 4;
        %zcol= 8;
        pcol= 12; 
    % ----------------------
    
    %% Initiate Current    
    % Finding required current for phi and psi orientation
    [currx, curry, currz] = inverseMagneticField(wHb);
    currX = currx;
    currY = curry;
    currZ = currz;

    %% Rotate Ball about world-z-axis
    % direction vector from p1 to p2 in world
    direction = (p2-wHb(pcol+1:15)')/norm(p2-wHb(pcol+1:15)');
    % unit vector of Magnetic-Field y-axis in world x-y-plane
    yaxis = wHb(ycol+1:ycol+3)';
    % Angle Between Magnetic-Field y-axis and vector pointing to next position
    theta = (pi/2 +anglediff(direction,yaxis));
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
        Hrot = wHb;
        % Visualization Function of ball after Rotation
        for n = 1 : rotsteps
            % Rotation
            Rrot = rotz(omegaz*dt);
            % Convert Rrot to Homegeneous Matrix
            Hrot(1:3,1:3) = Rrot*Hrot(1:3,1:3);
            % Visualization Function of ball after Rotation
            plot_ball(ballsize,Hrot,dt,speed);
            % Find the Neccessary Current and add it to current set 
                % magnet-field
                [currx, curry, currz] = inverseMagneticField(Hrot);
                currX(n+1,1) = currx;
                currY(n+1,1) = curry;
                currZ(n+1,1) = currz;

        end

        % Pre multiply rotation for fixed frame
        wHb(1:3,1:3) = rotz(theta)*wHb(1:3,1:3);%[rotz(theta)',[0;0;0];0 0 0 1];
        % Visualization Function of ball after Rotation 
        plot_ball(ballsize,wHb,0.1,speed);
        % Find the Neccessary Current and add it to current set 
            % magnet-field
            [currx, curry, currz] = inverseMagneticField(wHb);
            currX = [currX;currx];
            currY = [currY;curry];
            currZ = [currZ;currz];
    
    %% Translation
    % Rotation required to reach p2 with no slip condition
    gama = norm(p2-wHb(pcol+1:15)')/(ballsize);

        %% Visualization of Translation
        % Number of steps in Translation
        transteps = floor(3*T/(4*dt));
        % Angular Velocity of rotation
        omegay = gama*4/(3*T);
        % Linear Velocity of ball
        vel = omegay*ballsize;
        % Rotation matrix for visualization
        Htrans = wHb; 
        % size of current vectors
        cursize = size(currX);
        % Transformation        
        for n = 1 : transteps
            %  Rotation
            Htrans = Htrans*[roty(omegay*dt),[0;0;0];0 0 0 1];
            %  Translation
            Htrans(pcol+1:15) = Htrans(pcol+1:15)' + vel*direction*dt;
            % Visualization Function of ball after Rotation
            plot_ball(ballsize,Htrans,dt,speed);
            % Find the Neccessary Current and add it to current set 
                % magnet-field
                [currx, curry, currz] = inverseMagneticField(Htrans);
                currX(cursize(1)+n,1) = currx;
                currY(cursize(1)+n,1) = curry;
                currZ(cursize(1)+n,1) = currz;
        end

        % Pre multiply rotation for fixed frame
        wHb= wHb*[roty(gama),[0;0;0];0 0 0 1];
        %  Translation
        wHb(pcol+1:15) = p2;
        % Visualization Function of ball after Rotation 
        plot_ball(ballsize,wHb,0.1,speed)
        % Find the Neccessary Current and add it to current set 
            % magnet-field
            [currx, curry, currz] = inverseMagneticField(wHb);
            currX = [currX;currx];
            currY = [currY;curry];
            currZ = [currZ;currz];  
else
    display('ERROR: Not Enough Input Arguments');
end
end

