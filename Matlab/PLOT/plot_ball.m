%% PLOT MAGNET BALL POSITION
% INSTRUCTIONS 
%{
% 2-Steps
% 1)Call plot_ball(size) "with only the size of the ball"
%   This creates the plot window
% 
% 2)Call plot_ball(x0,y0,size,dt,speed) "with the current x,y-position &"
%                                       "the size of the ball & "
%                                       "the time step & the scale"
%----------------------------------------------------------------------
%}
function  plot_ball(size,pos,R,dt,speed)
%Print Task Name
Task = 'Running Plot Magnet Ball'
%---------------------

%% Draw Sphere
if nargin ==1
    %% Creating Figure
    % Enforce that one unit is displayed 
    % equivalently in x, y, and z.
    axis equal;
    % Set the viewing volume
    % height : ensures ample height to see whole picture
    height = size*5;
    % -x x -y y -z z step
    axis([-20 20 -20 20 -0 height])
    % Set the viewing angle
    view(-135, 40)
    % Label the axes.
    xlabel('x0 (m)')
    ylabel('y0 (m)')
    zlabel('z0 (m)')
    % Turn on Grid
    grid on
    % Title of Figure
    title('Ball Trajectory')
    hold on
    
else
    %% Draw Sphere
    [x,y,z] = sphere;
    % Draw Function
    % set(s(1), 'xdata',size*x+x0 ,'ydata', size*y+y0,'zdata',size*z+size)
    s = surf(size*x+pos(1),size*y+pos(2),size*z+size)
    
    %% Draw Arrow Pointing North(magnet-Z-Axis) & AxisOfRolling(magnet-Y-Axis)
    % Elongate Quiver
    R(7:9) = 4*size*R(7:9);
    % Draw z-Arrow
    quiver3(pos(1),pos(2),pos(3),R(7),R(8),R(9))
    % Draw x-Arrow
    quiver3(pos(1),pos(2),pos(3),R(4),R(5),R(6))
    
    %% sets time delay
    pause(dt/speed);
    
    %% makes prvious ball invisible & Marks Position
    % Ball Visibility
    set(s,'visible','off')
    % Draws a red dot to denote poition already visited
    scatter3(pos(1),pos(2),0,'.','red')
end   
end

