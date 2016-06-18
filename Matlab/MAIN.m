%% This is the Main Function that Runs all other functions for the Omnimagnet Simulation
% Author: Mohamed Ghori
% Reference Material: 
% A. J. Petruska, J. B. Brink, and J. J. Abbott, "First Demonstration of a Modular and Reconfigurable Magnetic-Manipulation System," IEEE Int. Conf. Robotics and Automation, 2015 (to appear). 
% A. J. Petruska, A. W. Mahoney, and J. J. Abbott, "Remote Manipulation with a Stationary Computer-Controlled Magnetic Dipole Source," IEEE Trans. Robotics, 30(5):1222-1227, 2014. 
% A. J. Petruska and J. J. Abbott, "Omnimagnet: An Omnidirectional Electromagnet for Controlled Dipole-Field Generation," IEEE Trans. Magnetics, 50(7):8400810(1-10), 2014. 

% INSTRUCTIONS
%{
% This is the main function where any comands can be written
%}

function MAIN(type,x0,y0,phi,psi,ShapeSize,T,dt,speed,ballsize)
%Print Task Name
Task = 'Running Main Function'
%---------------------
% Main calls the path to follow and then plays back the path resulting
% from the returned current matrix
%
%   MAIN() 
%   "Returns a path of a random shape with a defining size of 1"
%   
%   MAIN(type) 
%   "Returns a path of shape(1=Square)(2=Circle) with a 
%    defining size of 1"
%   
%   MAIN(type,x0,y0,phi,psi,ShapeSize) 
%   "Returns a path of shape(1=Square)(2=Circle) with
%    an init position and orientation and over-all size of 'x0' 'y0' 'phi' 
%    'psi' & 'ShapeSize' "
%
%   MAIN(type,x0,y0,phi,psi,ShapeSize,T,dt) 
%   "Returns a path of shape(1=Square)(2=Circle) 
%    with an init position and orientation and over-all size 
%    of 'x0' 'y0' 'phi' 'psi' & 'ShapeSize' 
%    with Time to completion and timestep 'T' & 'dt' "
%
%   MAIN(type,x0,y0,phi,psi,ShapeSize,T,dt,speed,ballsize) 
%   "Returns a path of shape(1=Square)(2=Circle) 
%    with an init position and orientation and over-all size 
%    of 'x0' 'y0' 'phi' 'psi' & 'ShapeSize' 
%    with Time to completion and timestep 'T' & 'dt' 
%    with ball-size and video speed 'ballsize' 'speed'"
%
% EX___  
%   MAIN(2,0,0,pi,pi,[10;10],10,0.1,1,1);
%

%% MAIN
% Enough Inputs EXCEPTION
if nargin == 0||nargin == 1||nargin == 6||nargin == 8||nargin == 10
    %% Add Paths 
    % MATLAB add path function
    addpath('MATH','CNTRL','PLOT','SHAPES');
    
    %% Init Graph
    if nargin ==10
        plot_ball(ballsize);
    else
        plot_ball(1);
    end
    %% Following Shape
    % 0 Param "Blind"
    if nargin ==0 
        a = ceil(2*rand);
        if a==1
        [currX, currY, currZ] = rollBallInSquare();
        else
        [currX, currY, currZ] = rollBallInCircle();
        end
    end

    % 1 Param "Given Path Shape"
    if nargin == 1
        if type==1
            [currX, currY, currZ] = rollBallInSquare();
        else
            [currX, currY, currZ] = rollBallInCircle();
        end
    end

    % 6 Param "Given Path Shape, & Path Parameters"
    if nargin ==6
        if type==1
            [currX, currY, currZ] = rollBallInSquare(x0,y0,phi,psi,ShapeSize);
        else
            [currX, currY, currZ] = rollBallInCircle(x0,y0,phi,psi,ShapeSize);
        end
    end

    % 8 Param "Given Path Shape, Path Params, & Time Params"
    if nargin == 8
        if type==1
            [currX, currY, currZ] = rollBallInSquare(x0,y0,phi,psi,ShapeSize,T,dt);
        else
            [currX, currY, currZ] = rollBallInCircle(x0,y0,phi,psi,ShapeSize,T,dt);
        end
    end

    % 10 Param "Given Path Shape, Path Params, & Time Params, Video Params"
    if nargin == 10
        if type==1
            [currX, currY, currZ] = rollBallInSquare(x0,y0,phi,psi,ShapeSize,T,dt,speed,ballsize);
        else
            [currX, currY, currZ] = rollBallInCircle(x0,y0,phi,psi,ShapeSize,T,dt,speed,ballsize);
        end
    end



    %% Play it Back
    playback(currX,currY,currZ,[x0; y0; 0],T,dt,speed,ballsize);

else
    ERROR = 'Not Enough Input Arguments'
end

end

