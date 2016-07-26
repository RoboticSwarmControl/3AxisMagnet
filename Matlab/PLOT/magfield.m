%% show magnetic field for certain Omnimagnet Current values 
%%                      Author: Mohamed K. Ghori B.S. M.E.
%                      ------------------------------------
% 
% Acknowledgements:
%


function [ Task ] = magfield(I)
%Print Task Name
Task = 'Running Show MAG Field';
%---------------------
% Display magntic vector field for specific set of currents for the Omnimagnet  
%   magfield(I)
%   Call with input current vector applied to Omnimagnet
%
% EX__
%  [Task] = magfield([1,0,0]);
%   
% Compact Text Format
format compact

%% magfield
% Enough Inputs EXCEPTION
if nargin == 1
    % Mapping of Magnetic Field to Current Based on Physical 
    K = eye(20);
    %% Eqn I => B
    % position of the ball center
    for k = 0:2
        for j = -10:2:10
            for i = -10:2:10
            % pose 
            pos = [i,j,k];
            % pose unit vector
            p_hat = pos/norm(pos);
            % Attributes of Solenoid
            M = eye(3);
            % Constant of Permeability
            mu = 4*(10^-7)*pi;
            % Eqn parts for B => I 
            temp = (2*pi/mu)*(norm(pos)^3)*(M\(3*p_hat*(p_hat') - 2*eye(3)));
            % Current Vector
            B = inv(temp)*I;
            % Show magnectic field vector
            quiver3(pos(1), pos(2), pos(3),B(1),B(2),B(3));            
            drawnow
            hold on
            end
        end
    end
else
    display('ERROR: Not Enough Input Arguments');
end
end

