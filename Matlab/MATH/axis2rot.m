%% Axis Angle notation to Rotation Matrix 
%%                      Author: Mohamed K. Ghori B.S. M.E.
%                      ------------------------------------
% 
% Acknowledgements:
%


function [ R, Task ] = axis2rot( u,th )
%Print Task Name
Task = 'Converting Axis Angle to Rotation Matrix';
%---------------------
% Desccription of Function
%   axis2rot(axis, angle)
%
% EX__
%  [R,Task] = axis2rot([1,0,0],pi) 
%   
% Compact Text Format
format compact

%% axis2rot
% Enough Inputs EXCEPTION
if nargin == 0
    % u_hat to skew symetric matrix
            u_skew = vect2skew(u);
        % Use rodruigez formula to rotate about singular axis
        R = (eye(3)*cos(th)+u*u'*(1-cos(th))+u_skew*sin(th));
else
    display('ERROR: Not Enough Input Arguments');
end
end

