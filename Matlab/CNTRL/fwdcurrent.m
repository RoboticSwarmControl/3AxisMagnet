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
    
    %%[phi,psi,~] = Rot_in_phipsi(I0,If);
    % No rotation occurs
    %if phi==0 && psi ==0
    %else        
    %R = roty(phi)*rotz(psi);
    % Rotation Matrix from latitude and longitude
%     R = [((cos(psi))^2+cos(phi)*(sin(psi))^2)  ((1-cos(phi))*cos(psi)*sin(psi))   (sin(phi)*sin(psi));...
%          ((1-cos(phi))*cos(psi)*sin(psi))      (cos(phi)*(cos(psi))^2+(sin(psi))^2) (-cos(psi)*sin(phi));...
%           (-sin(phi)*sin(psi))                 (cos(psi)*sin(phi))                 cos(phi)];
    



    % point to point roll
    [R,alph,L,~] = point2pointroll(I0'*ballsize,If'*ballsize);
    % Convert Rotation to axis angle    
    [u,th,~] = rot2axis(R);
    % mass of ball in 1kg/radius
    Trq = 1;
    mass = ballsize*1;
    Imoment = 0.4*mass*ballsize^2;
    % Find angular acceleration
    aph = Trq/Imoment;
    % Torque assumed to be 1
    [t1,~] = rotTime(th,0,aph);
    [t2,~] = rotTime(th,aph*t1,-aph);
   
    
    % Use ODE 45 to see the resultant change in angle 
    %[~,X] = ode45(@(t,X) dX_dt(t,X), 0:0.1:t, X0)
    % find rotation
    %Rroll = axis2rot(u,X(size(X,1)));
    % Rotation
    wHb(1:3,1:3) = R*wHb(1:3,1:3);
    % Translation
    wHb(13:15) = wHb(13:15)' + rotz(alph+pi/2)*[1;0;0]*L*ballsize;
    wHb = round(wHb,2);
    
    display(u,'Torque axis')
    display(1, 'Torque magnitude');
    display(t1, 'Positive Torque Period')
    display(t2, 'Negative Torque Period')
    
%         %% Decompose whole into velocities
%             % Show Mag Field
%             Torque  = [0;0;1];
%             mt = wHb(zcol+1:zcol+3)';
%             [h,g] = makePlotsMagfield3D(mt, Torque);
%             
%             % direction of linear movemnt
%             wHb(1:3,1:3) = rotz(psi)*wHb(1:3,1:3);
%             
            %% vsiualization
            %plot_ball(ballsize,wHb,dt,speed);
            
            %             plot_ball(ballsize,wHb,0.001,speed);
%             % direction of roll
%             direction = rotz(pi/2)*[wHb(ycol+1);wHb(ycol+2);0];
%             % velocity in this direction
%             vel = direction*phi/T;
%             
%             %Show Mag Field
%             Torque = wHb(ycol+1:ycol+3)';
%             mt = wHb(zcol+1:zcol+3)';
%             [h,g] = makePlotsMagfield3D(mt, Torque);
%        
%             %% whole Move
%             % Resultant Pos & Orientation
%             wHb(pcol+1:15) = wHb(pcol+1:15)' + vel*T;
%             wHb(1:3,1:3) = wHb(1:3,1:3)*roty(-phi);
%            % if norm(wHb(zcol+1:zcol+3)-If)>0
%    
%            % end
%    end      
            %% vsiualization
            plot_ball(ballsize,wHb,0.001,speed);
            
else
    display('ERROR: Not Enough Input Arguments');
end

