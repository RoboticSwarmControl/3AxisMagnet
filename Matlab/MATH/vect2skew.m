%% Creates a skew symmetric matrix from a vector 
%%                      Author: Mohamed K. Ghori B.S. M.E.
%                      ------------------------------------
% 
% Acknowledgements:
%


function [ skew,Task ] = vect2skew(vector)
%Print Task Name
Task = 'Running vect2skew';
%---------------------
% INSTRUCTIONS
%{
% Call using a vector 
%}

% Enough Inputs EXCEPTION
if nargin == 1
% components of vector
    a = vector(1);
    b = vector(2);
    c = vector(3);
% skew symmetric matrix of vector
    skew =[0   -c   b ;
           c    0  -a ;
          -b    a   0  ];

else
    display('ERROR: Not Enough Input Arguments');
end
end

