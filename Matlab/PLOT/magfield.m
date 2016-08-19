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
    %% Eqn I => B
    % position of the ball center
    for k = -10:1:10
        for j = -10:1:10
            for i = -10:1:10
            % pose 
            pos = [i,j,k]
            % pose unit vector
            p_hat = pos/norm(pos)'
            % Attributes of Solenoid
            %M = [25.1 0 0;0 25.8 0; 0 0 26.3];
            % Constant of Permeability
            mu = 4*(10^-7)*pi;
            % Eqn parts for B => I 
            temp = (mu/(2*pi*(norm(pos)^3)))*((3*p_hat*(p_hat') - eye(3)));
            %temp = (2*pi/mu)*(norm(pos)^3)*((3*p_hat*(p_hat') - 2*eye(3)));
            % Current Vector
            B = (mu/(2*pi*(norm(pos)^3)))*((3*p_hat*(p_hat') - eye(3)))*I
            % Direction of B
            arr = B/sqrt(sum(abs(B).^2,1))
            % Show magnectic field vector
            quiver3(pos(1), pos(2), pos(3),arr(1),arr(2),arr(3));            
%             starty = -10:1:10;
%             startz = starty;
%             startx = starty;
%             h = streamline(pos(1), pos(2), pos(3),arr(1),arr(2),arr(3),startx,starty,startz);%,[.1,1000]);
%             set(h,'Color','red')

            drawnow
            hold on
            end
        end
    end
else
    display('ERROR: Not Enough Input Arguments');
end
end

