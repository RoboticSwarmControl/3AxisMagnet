%% Dr. Becker's  Roll Any Point on the Sphere ot Any Desired 
%% Latitude-Longitude Coordinates with one Straight-Line Roll
%%                      Author: Mohamed K. Ghori B.S. M.E.
%                      ------------------------------------
% 
% Acknowledgements:
%   Implemented Dr. Becker's Code: "Roll Any Point on the Sphere ot Any Desired 
%  Latitude-Longitude Coordinates with one Straight-Line Roll"


function [ R,alph,L, Task ] = point2pointroll(point1,point2)
%Print Task Name
Task = 'Roll a sphere point to point';
%---------------------
% Desccription of Function
%   point2pointroll(point1,point2)
%
% EX__
%  [R,Task] = point2pointroll([1;2;3],[4;5;6]) 
%   
% Compact Text Format
format compact

%% point2pointroll
% Enough Inputs EXCEPTION
if nargin == 2
    y0 = point1(2);
    x0 = point1(1);
    y1 = point2(2);
    x1 = point2(1);
    
    % Find Alpha 
    if norm([x0;y0]-[x1;y1])==0
        if norm([x0;y0])==0
            alph = 0;
        else
            alph = atan(y0/x0);
        end
    else
        alph = atan((-y0+y1)/(x0-x1));
    end
   
    % Find L
    d2 = norm([x0;y0]-[x1;y1])^2;
    
    if d2 == 0
        Opoint = [0;0;0];
    else
        Opoint = [(y0-y1)*(x1*y0-x0*y1)/d2; (x0-x1)*(-x1*y0+x0*y1)/d2;0];
    end
    
    % Vectors from the intersection point to the start & end north poles
    w0 = point1-Opoint;
    w1 = point2-Opoint;
    
    % Find Cosin L
    %cosL = w0*w1/(norm(w0)*norm(w1));
    %sinL = sqrt(1-cosL^2);
    if d2 ==0
        L = 0;
    else
        L = acos(dot(w0,w1)/(norm(w0)*norm(w1)));
        if L>pi
            L = L - 2*pi;
        elseif L<-pi
            L = L +2*pi;
%         elseif (w1(2)>0) && (w0(2)>0) && (w1(1)-w0(1))<0
%             L = -L;
%         elseif (w1(2)<0) && (w0(2)<0) && (w1(1)-w0(1))>0
%             L = -L
        end
        
    end
    
    R = [(cos(alph))^2+cos(L)*(sin(alph))^2 (1-cos(L))*cos(alph)*sin(alph)      sin(L)*sin(alph);...
     (1-cos(L))*cos(alph)*sin(alph)     cos(L)*(cos(alph))^2+(sin(alph))^2  -cos(alph)*sin(L);...
     -sin(L)*sin(alph)                  cos(alph)*sin(L)                    cos(L)];
    
    Rm = [(cos(alph))^2+cos(-L)*(sin(alph))^2 (1-cos(-L))*cos(alph)*sin(alph)      sin(-L)*sin(alph);...
         (1-cos(-L))*cos(alph)*sin(alph)     cos(-L)*(cos(alph))^2+(sin(alph))^2  -cos(alph)*sin(-L);...
         -sin(-L)*sin(alph)                  cos(alph)*sin(-L)                    cos(-L)];

 if round(norm(Rm*point1 - point2),3)> round(norm(R*point1 - point2),3)
 L = -L;
 end
 
else
    display('ERROR: Not Enough Input Arguments');
end
end




