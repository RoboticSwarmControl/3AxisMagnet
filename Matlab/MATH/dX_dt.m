%% FULL WINDED TITLE 
%%                      Author: Mohamed K. Ghori B.S. M.E.
%                      ------------------------------------
% 
% Acknowledgements:
%


function [ dXdt, Task ] = dX_dt(X,T,I)
%Print Task Name
Task = 'Running ODE';
%---------------------
% Desccription of Function
%   dX_dt(state,Torque,Moment of Inertia)
%
% EX__
%  [outputargs,Task] = Untitled2(1) 
%   
% Compact Text Format
format compact

%% dX_dt
% Enough Inputs EXCEPTION
if nargin == 0
  dXdt = [t + X(2);1];
else
    display('ERROR: Not Enough Input Arguments');
end
end

