%% This is the Main Function that runs all other functions 
% INSTRUCTIONS
%{
% This is the main function where any comands can be written
%}

function MAIN( )
%Print Task Name
Task = 'Running Main Function'
%---------------------
% Initi Graphing Area with ball size of 1
plot_ball(1);
% Roll ball in square
[currx, curry, currz] = rollBallInSquare(5,5,0,0,[10;10],10,0.1)

end

