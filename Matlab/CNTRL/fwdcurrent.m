%% Converts Current to Change in Orientation and Position
%                           Author: Mohamed Ghori
%                          -----------------------
% Reference Material: 
% A. J. Petruska, J. B. Brink, and J. J. Abbott, "First Demonstration of a Modular and Reconfigurable Magnetic-Manipulation System," IEEE Int. Conf. Robotics and Automation, 2015 (to appear). 
% A. J. Petruska, A. W. Mahoney, and J. J. Abbott, "Remote Manipulation with a Stationary Computer-Controlled Magnetic Dipole Source," IEEE Trans. Robotics, 30(5):1222-1227, 2014. 
% A. J. Petruska and J. J. Abbott, "Omnimagnet: An Omnidirectional Electromagnet for Controlled Dipole-Field Generation," IEEE Trans. Magnetics, 50(7):8400810(1-10), 2014. 
% Link: http://www.telerobotics.utah.edu/index.php/Research/Omnimagnets

function [ wHb, Task] = fwdcurrent(I0, If,wHb,T,dt,speed,ballsize)
%Print Task Name
Task = 'Running Current to Step';
%---------------------
%
% fwdcurrent first rotates the ball such that the y-axis of coresponding to
% the orientatoin of the initial and final vector of the 3-solenoid-currents
% coincide second it rolls the ball about the y axis such that the x&z-axies
% coincide 
%
%   [ pf, wRb ] = fwdcurrent(I0, If,p0,wRb,T,dt,speed,ballsize)
%   "Returns the resultant position and orientation of the ball
%    when the solnoid-currents change from an Inital to a final state 
%    given this initial and final solenoid-current state the current
%    orientatoin, a time to completion and time step, and ballsize and 
%    speed of the video:
%    'I0' & 'If' , 'p0' & 'wRb' , 'T' & 'dt' , 'speed' & 'ballsize' 
%
% EX___
%   [ pf, wRb ] = fwdcurrent([1;2;3], [4;;6],[0;0;0],eye(3),10,0.1,1,1)
%   
% Column of Homogeneous
        %xcol= 0;
        ycol= 4;
        zcol= 8;
        pcol= 12; 
    % ----------------------
    
%% fwdcurrent
% Enough Inputs EXCEPTION
if nargin == 7
   
    %% Find Rotation
    %{
    % Initial Orientation
    [phi1, psi1] = fwdMagneticField( I0(1), I0(2), I0(3), wHb(pcol+1), wHb(pcol+2));
    % Final Orientation
    [phi2, psi2] = fwdMagneticField( If(1), If(2), If(3), wHb(pcol+1), wHb(pcol+2));
    % Rotation about world-z-axis
    %}     
        % Shows direction of Init North Pole Orientation
    quiver3(0,0,5, I0(1),I0(2),I0(3));
%% I0 Angle 1

    % Find latitude and longitude of Initial and final ball z-axis
    phi1 = atan2(I0(3),sqrt(I0(2)^2+I0(1)^2));
    % Sign correction
    if phi1<0 
    phi1 = phi1+2*pi;
    end
    % Angle from world-x-axis to B in world-x-y-plane
    psi1 = atan2(I0(2),I0(1));
    % Sign correction
    if psi1<0
        psi1 = psi1 +2*pi;
    end
    %% If Angle 2
    phi2 = atan2(If(3),sqrt(If(2)^2+If(1)^2));
    % Sign correction
    if phi2<0 
        phi2 = phi2+2*pi;
    end
    % Angle from world-x-axis to B in world-x-y-plane
    psi2 = atan2(If(2),If(1));
    % Sign correction
    if psi2<0
        psi2 = psi2 +2*pi;
    end
    %}
   
% Magnet off exception
    if norm(I0) == 0
        phi1 = 0;
        psi1 = 0;
    end
    % Magnet off exception
    if norm(If) == 0
        phi2 = 0;
        psi2 = 0;
    end
    phi1
    phi2
    %% Diff in Anlge
    psi = psi2-psi1
    % Rotation about world-y-axis (negativd phi2 due to backward rotation)
    phi = phi2-phi1
    if phi<-pi
        phi = phi + 2*pi
    else if phi>pi
            phi = phi-2*pi;
        end
    end
    % No rotation occurs
    if phi==0 && psi ==0
    else        
        %% Decompose whole into velocities

            % direction of linear movemnt
            wHb(1:3,1:3) = rotz(psi)*wHb(1:3,1:3);
            %% vsiualization
            plot_ball(ballsize,wHb,0.001,speed);
            % direction of roll
            direction = -rotz(pi/2)*[wHb(ycol+1);wHb(ycol+2);0];
            % velocity in this direction
            vel = -direction*phi/T;
       
        %% whole Move
            % Resultant Pos & Orientation
            wHb(pcol+1:15) = wHb(pcol+1:15)' + vel*T;
            wHb(1:3,1:3) = wHb(1:3,1:3)*roty(-phi);
           % if norm(wHb(zcol+1:zcol+3)-If)>0
   
           % end
    end      
            %% vsiualization
            plot_ball(ballsize,wHb,0.001,speed);
else
    display('ERROR: Not Enough Input Arguments');
end

