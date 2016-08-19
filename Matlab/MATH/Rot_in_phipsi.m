%% FULL WINDED TITLE 
%%                      Author: Mohamed K. Ghori B.S. M.E.
%                      ------------------------------------
% 
% Acknowledgements:
%


function [ phi, psi, Task ] = Rot_in_phipsi( vect1, vect2)
%Print Task Name
Task = 'Running Rot_in_phipsi';
%---------------------
% Desccription of Function
%   Rot_in_phipsi(input_args)
%   The initial vector(vect1) and final vector(vect2) 
%
% EX__
%  [phi,psi] = Rot_in_phipsi([1;2;3],[4;5;6]) 
%   
% Compact Text Format
format compact

%% vect1 Angle 1
if nargin ==2
    % Find latitude and longitude of Initial and final ball z-axis
    phi1 = atan2(vect1(3),sqrt(vect1(2)^2+vect1(1)^2));
    % Sign correction
    if phi1<0 
    phi1 = phi1+2*pi;
    end
    % Angle from world-x-axis to B in world-x-y-plane
    psi1 = atan2(vect1(2),vect1(1));
    % Sign correction
    if psi1<0
        psi1 = psi1 +2*pi;
    end
    %% vect2 Angle 2
    phi2 = atan2(vect2(3),sqrt(vect2(2)^2+vect2(1)^2));
    % Sign correction
    if phi2<0 
        phi2 = phi2+2*pi;
    end
    % Angle from world-x-axis to B in world-x-y-plane
    psi2 = atan2(vect2(2),vect2(1));
    % Sign correction
    if psi2<0
        psi2 = psi2 +2*pi;
    end
    %}
   
% Magnet off exception
    if norm(vect1) == 0
        phi1 = 0;
        psi1 = 0;
    end
    % Magnet off exception
    if norm(vect2) == 0
        phi2 = 0;
        psi2 = 0;
    end
    
    %% Diff in Anlge
    % change in longitude
    Ang1 = min([norm(psi2-psi1); norm(psi2-psi1 + 2*pi); norm(psi2-psi1 - 2*pi)]);
    % choose shortest rout between angles
    switch Ang1
        case norm(psi2-psi1)
            psi = psi2-psi1;
        case norm(psi2-psi1 + 2*pi)
            psi = psi2-psi1 + 2*pi;
        case norm(psi2-psi1 - 2*pi)
            psi = psi2-psi1 - 2*pi;
    end
    
    % change in lattitude
    Ang2 = min([norm(phi2-phi1); norm(phi2-phi1 + 2*pi); norm(phi2-phi1 - 2*pi)]);
    % choose shortest rout between angles
    switch Ang2
        case norm(phi2-phi1)
            phi = phi2-phi1;
        case norm(phi2-phi1 + 2*pi)
            phi = phi2-phi1 + 2*pi;
        case norm(phi2-phi1 - 2*pi)
            phi = phi2-phi1 - 2*pi;
    end

else
    display('ERROR: Not Enough Input Arguments');
end

