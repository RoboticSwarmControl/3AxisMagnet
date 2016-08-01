%% This is the Main Function that Runs all other functions for the Omnimagnet Simulation
%                           Author: Mohamed Ghori
%                          -----------------------
% Reference Material: 
% A. J. Petruska, J. B. Brink, and J. J. Abbott, "First Demonstration of a Modular and Reconfigurable Magnetic-Manipulation System," IEEE Int. Conf. Robotics and Automation, 2015 (to appear). 
% A. J. Petruska, A. W. Mahoney, and J. J. Abbott, "Remote Manipulation with a Stationary Computer-Controlled Magnetic Dipole Source," IEEE Trans. Robotics, 30(5):1222-1227, 2014. 
% A. J. Petruska and J. J. Abbott, "Omnimagnet: An Omnidirectional Electromagnet for Controlled Dipole-Field Generation," IEEE Trans. Magnetics, 50(7):8400810(1-10), 2014. 
% Link: http://www.telerobotics.utah.edu/index.php/Research/Omnimagnets

%% NOTE:should I remove mov?  


function [curra, currb, currc, type, mov, Task] =  MAIN(type,x0,y0,phi,psi,ShapeSize,T,dt,speed,ballsize)
% add folders
addpath('./CLEAN');addpath('./SHAPES');addpath('./PLOT');addpath('./MATH');addpath('./CNTRL')


%Print Task Name
Task = 'Running Main';
%---------------------
% Main calls the shape function that generates a trajectory for the ball to
% follow. This shape function returns a set of current vectors coresponding
% the required-current values of [currx, curry,currz] for the Omnimagnet at 
% each step along the path  
%
%   MAIN() 
%   "Returns a path of rectangle or circle at random"
%   
%   MAIN(type) 
%   "Returns a path of shape(1=Rectangle)(2=Circle)"
%   
%   MAIN(type,x0,y0,phi,psi,ShapeSize) 
%   "Returns a path of shape(1=Rectangle)(2=Circle) with
%    an initital position & orientation of 'x0' 'y0' 'phi' 
%    'psi'(phi is the latitude measured from the x-axis & psi is the 
%    longitude measured from the x-axis) and shape deffinition 
%    (squre => 3rd cornere[x2,y2]; circle => Radius) 'ShapeSize'"
%
%   MAIN(type,x0,y0,phi,psi,ShapeSize,T,dt) 
%   "Returns a path of shape(1=Rectangle)(2=Circle) with
%    an initital position & orientation of 'x0' 'y0' 'phi' 
%    'psi'(phi is the latitude measured from the x-axis & psi is the 
%    longitude measured from the x-axis) and shape deffinition 
%    (squre => 3rd cornere[x2,y2]; circle => Radius) 'ShapeSize'"
%    and with a period to complete path and timestep of 'T' & 'dt' "
%
%   MAIN(type,x0,y0,phi,psi,ShapeSize,T,dt,speed,ballsize) 
%   "Returns a path of shape(1=Rectangle)(2=Circle) with
%    an initital position & orientation of 'x0' 'y0' 'phi' 
%    'psi'(phi is the latitude measured from the x-axis & psi is the 
%    longitude measured from the x-axis) and shape deffinition 
%    (squre => 3rd cornere[x2,y2]; circle => Radius) 'ShapeSize'"
%    and with a period to complete path and timestep of 'T' & 'dt' "
%    and with ball-size and video speed of 'ballsize' 'speed'"
%
% EX___  
%   rectangle
%   [curra, currb, currc, type, mov, Task] = MAIN(2,0,0,pi,pi,[10;10],10,0.1,1,1);
%
%   circle
%   [curra, currb, currc, type, mov, Task] = MAIN(2,0,0,pi,pi,10,10,0.1,1,1);
%
% Compact Text Format
format compact

%% MAIN
% Declare Movie as Global variable
global mov

% Enough Inputs EXCEPTION
if nargin == 0||nargin == 1||nargin == 6||nargin == 8||nargin == 10 
    %% Close all Figure
    CleanUpQ(2);
    
    %% Add Paths
    % MATLAB add path function
    %addpath(genpath('C:\Users\alighori\Documents\2) MENU\1) OCCUPATION\JOBS\JOB(05.26.2016) - RA (Dr. Aaron Becker)\Test Project (Omni-Magnet & Magnet Ball)\MATLAB CODE\Omnimagnet_GIT\Omnimagnet-Git\MAIN'))
    
    %% Init Graph
    % If ballsize given
    if nargin ==10
        % Accomodate Plot for Ballsize
        plot_ball(ballsize);
    else
        % Assume Ballsize == 1
        plot_ball(1);
    end
    
%% Following Shape
    %% 0 Param "Blind"
    % check input 
    if nargin ==0 
        % randomly chose a shape (rectangle 2-circle)
        a = ceil(2*rand);
        % assign shapee
        if a==1
            % run rectangular trajectory 
            [curra, currb, currc,wHb] = rollBallInRectangle();
            
            % playback trajectory
            prompt = 'Run playback?\n 1-Yes or 0-No\n Then Press Enter\n';
            prompt = input(prompt)==1;
            if prompt ==1
                wHb = [1 0 0 5;0 1 0 5;0 0 1 0; 0 0 0 1];
                playback(curra,currb,currc,wHb);
            end
        else
            % run Circle trajectory
            [curra, currb, currc,wHb] = rollBallInCircle();
            % playback trajectory
            prompt = 'Run playback?\n 1-Yes or 0-No\n Then Press Enter\n';
            prompt = input(prompt)==1;
            if prompt ==1
                wHb = [1 0 0 5;0 1 0 5;0 0 1 0; 0 0 0 1];
                playback(curra,currb,currc,wHb);
            end
        end
    end

    %% 1 Param "Given Path Shape Type"
    % check input 
    if nargin == 1
        if type==1
            % run rectangle trajectory
            [curra, currb, currc,~] = rollBallInRectangle()
            % playback trajectory
            
            
            
            prompt = 'Run playback?\n 1-Yes or 0-No\n Then Press Enter\n';
            prompt = input(prompt)==1;
            if prompt ==1                
                wHb = [1 0 0 5;0 1 0 5;0 0 1 0; 0 0 0 1];
                playback(curra,currb,currc,wHb);
            end
        else
            % run circle trajectory
            [curra, currb, currc,wHb] = rollBallInCircle();
            % playback trajectory
            prompt = 'Run playback?\n 1-Yes or 0-No\n Then Press Enter\n';
            prompt = input(prompt)==1;
            if prompt ==1
                wHb = [1 0 0 5;0 1 0 5;0 0 1 0; 0 0 0 1];
                playback(curra,currb,currc,wHb);
            end
        end
    end

    %% 6 Param "Given Path Shape, & Path Parameters"
    % check input 
    if nargin ==6
        % Init Pos vector
        p0 = [x0;y0;0];
        % Init rot vector(rotation in world-z then magnetic-y)
        R0 = roty(phi)*rotz(psi);
        % Init Homogeneous Vector
        wHb =  [R0,p0;0 0 0 1];
        
        if type==1
            % run rectangle trajectory (specified rectangle)
            [curra, currb, currc,wHb] = rollBallInRectangle(wHb, ShapeSize);
            % playback trajectory
            prompt = 'Run playback?\n 1-Yes or 0-No\n Then Press Enter\n';
            prompt = input(prompt)==1;
            if prompt ==1
                wHb = [1 0 0 5;0 1 0 5;0 0 1 0; 0 0 0 1];
                playback(curra,currb,currc,wHb);
            end
        else
            % run circle trajectory (specified circle)
            [curra, currb, currc,wHb] = rollBallInCircle(wHb,ShapeSize);
           % playback trajectory
            prompt = 'Run playback?\n 1-Yes or 0-No\n Then Press Enter\n';
            prompt = input(prompt)==1;
            if prompt ==1
                wHb = [1 0 0 5;0 1 0 5;0 0 1 0; 0 0 0 1];
                playback(curra,currb,currc,wHb);
            end
        end
    end
    

    %% 8 Param "Given Path Shape, Path Params, & Time Params"
    % check input 
    if nargin == 8
        % Init Pos vector
        p0 = [x0;y0;0];
        % Init rot vector(rotation in world-z then magnetic-y)
        R0 = roty(phi)*rotz(psi);
        % Init Homogeneous Vector
        wHb =  [R0,p0;0 0 0 1];

        if type==1
            % run rectangle trajectory (specified rectangle, Time to complete & time step to reccord)
            [curra, currb, currc,wHb] = rollBallInRectangle(wHb,ShapeSize,T,dt);
            % playback trajectory
            prompt = 'Run playback?\n 1-Yes or 0-No\n Then Press Enter\n';
            prompt = input(prompt)==1;
            if prompt ==1
                wHb = [1 0 0 5;0 1 0 5;0 0 1 0; 0 0 0 1];
                playback(curra,currb,currc,wHb);
            end
        else
            % run circle trajectory (specified Circle, Time to complete & time step to reccord)
            [curra, currb, currc,wHb] = rollBallInCircle(wHb,ShapeSize,T,dt);
            % playback trajectory
            prompt = 'Run playback?\n 1-Yes or 0-No\n Then Press Enter\n';
            prompt = input(prompt)==1;
            if prompt ==1
                wHb = [1 0 0 5;0 1 0 5;0 0 1 0; 0 0 0 1];
                playback(curra,currb,currc,wHb);
            end
        end
    end

    %% 10 Param "Given Path Shape, Path Params, & Time Params, Video Params"
    % check input 
    if nargin == 10
        % Init Pos vector
        p0 = [x0;y0;0];
        % Init rot vector(rotation in world-z then magnetic-y)
        R0 = roty(phi)*rotz(psi);
        % Init Homogeneous Vector
        wHb =  [R0,p0;0 0 0 1];

        if type==1
            % run rectangle trajectory (specified rectangle, Time to complete & time step to reccord,tool size & speed of video)
            [curra, currb, currc,wHb] = rollBallInRectangle(wHb,ShapeSize,T,dt,speed,ballsize);
            % playback trajectory
            prompt = 'Run playback?\n 1-Yes or 0-No\n Then Press Enter\n';
            prompt = input(prompt)==1;
            if prompt ==1
                wHb = [1 0 0 5;0 1 0 5;0 0 1 0; 0 0 0 1];
                playback(curra,currb,currc,wHb);
            end
        else
            % run circle trajectory (specified Circle, Time to complete & time step to reccord,tool size & speed of video)
            [curra, currb, currc,wHb] = rollBallInCircle(wHb,ShapeSize,T,dt,speed,ballsize);
            % playback trajectory
            prompt = 'Run playback?\n 1-Yes or 0-No\n Then Press Enter\n';
            prompt = input(prompt)==1;
            if prompt ==1
                wHb = [1 0 0 5;0 1 0 5;0 0 1 0; 0 0 0 1];
                playback(curra,currb,currc,wHb);
            end
        end
    end
    % Remove Ball from graph
    plot_ball();
    % Play movie back
    figure;
    %movie(mov);

    %% Write Solenoid-Current Values to MS-Excel File     
    % If playback run
    if prompt ==1
    % create File
    filename = 'currentdata.xlsx';
    % write curra (inner solenoid)
    xlswrite(filename,curra,'currx','A1');
    % write currb (middle solenoid)
    xlswrite(filename,currb,'currx','B1');
    % write currc (Outer solenoid)
    xlswrite(filename,currc,'currx','C1');
    end 

    %% Close all Figure
    CleanUpQ(2);
    %% Cleaning all Data
    CleanUpQ(1);
    
else % Incorect parameters used while calling 'MAIN'
    display('ERROR: Not Enough Input Arguments');
end
end

