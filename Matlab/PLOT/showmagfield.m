%% Visualization for the Magnetic field line from the Omnimagnet to the Magnet-tool 
%%                      Author: Mohamed K. Ghori B.S. M.E.
%                      ------------------------------------
% 
% Acknowledgements:
%

%% NOTE: NOT TESTED TO WORK
function [ Task ] = showmagfield( currx,curry,currz,p0)
%Print Task Name
Task = 'Running Show Magnetic Field';
%---------------------
% INSTRUCTIONS
%{
% PUT INFO ON HOW TO USE FUNCTION
%}
   
% Column of Homogeneous
        %xcol= 0;
        %ycol= 4;
        %zcol= 8;
        %pcol= 12; 
    % ----------------------

% Enough Inputs EXCEPTION
if nargin == 4
    % Step size between quivers
    delp = norm(p0)/10; 
    % Initialize posB (Position of Quiver)
    posB(1,:) = p0;
    direction =[];
    % Draw 200 quivers to outline magnetic field lines
    for n = 0:1000
        % Magnetic Field Value at position
        [~,~,B] = fwdMagneticField(currx,curry,currz,posB(n+1,1),posB(n+1,2));
        % Orientation Matrix
        %R = rotz(psi)*roty(phi);
        % Direction of Magnetic Field line
        direction(n+1,:) = B/norm(B)';
        % why is it not working 
        % Iterate through positions along the magnetic field line
        posB(n+2,:) = direction(n+1,:)'*delp+posB(n+1,:)';
    end
    % Draw arrow tangent to magnetic field line
    10*direction;
    size(direction)
    size(posB)
        quiv = quiver3(posB(1:1001,1),posB(1:1001,2),posB(1:1001,3),direction(1:1001,1),direction(1:1001,2),direction(1:1001,3));
        hold on
        pause(0.2)
        delete(quiv);
else
    ERROR = 'Not Enough Input Arguments';
    display(ERROR);
end
end

