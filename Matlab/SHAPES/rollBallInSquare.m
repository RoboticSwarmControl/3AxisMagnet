
%% Roll a Magnet Ball in a Square Trajectory Using a Magnetic Field
% INSTRUCTIONS
%{
% Call Using: 
%               Init-x(x0) Init-y(y0) Init-phi(phi) Init-psi(psi) 
%               NOTE: "Init position and Orientation"
%               Oposite-Corner-x(corner(1)) Oposition-Corner-y(corner(2))
%               NOTE: "Position and Orientation of 'far' or 3rd corner"
%               Period(T) Time-Step(dt)
%               NOTE: "The total time to complete the square and the number
%                      of steps to be displayed"
%}

function [ currX, currY, currZ ] = rollBallInSquare( x0, y0, phi, psi,corner,speed,ballsize,T,dt)
%Print Task Name
Task = 'Running Roll Ball in Square'
%---------------------

%  --------
% |        |  W
% |        |
%  --------
%     L
% Length and Widht of the "Square"
L = abs(x0-corner(1));
W = abs(y0-corner(2));

% Time per Meter 
%for a constant velocity accross each leg
delT = T/(2*W + 2*L);

% Period Per Leg
TW = delT*W;
TL = delT*L;

% Initialize Currents
currX = [];
currY = [];
currZ = [];

% First Orientation
wRb = rotz(psi)*roty(phi);

% Four Corners of Rectangle
corners = [x0 y0 0; corner(1) y0 0;corner(1) corner(2) 0; x0 corner(2) 0];


%% Rolling Ball in Squre

% First Leg
% Use ballfwd Control
[ currx, curry, currz, wRb] = ballfwd( TL,corners(1,:)',corners(2,:)',wRb,dt,speed,ballsize)
% Set Required Current Vecotrs 
currX = [currX;currx];
currY = [currY;curry];
currZ = [currZ;currz];
%}
% Second Leg
[ currx, curry, currz, wRb] = ballfwd( TW,corners(2,:)',corners(3,:)',wRb,dt,speed,ballsize)
% Set Required Current Vecotrs
currX = [currX;currx];
currY = [currY;curry];
currZ = [currZ;currz];

% Third Leg
[ currx, curry, currz, wRb] = ballfwd( TL,corners(3,:)',corners(4,:)',wRb,dt,speed,ballsize)
% Set Required Current Vecotrs
currX = [currX;currx];
currY = [currY;curry];
currZ = [currZ;currz];

% Fourth Leg
[ currx, curry, currz, wRb] = ballfwd( TW,corners(4,:)',corners(1,:)',wRb,dt,speed,ballsize)
% Set Required Current Vecotrs
currX = [currX;currx];
currY = [currY;curry];
currZ = [currZ;currz];

end


