    % Axis set Equal
    axis equal;
    % Set the viewing volume
    % height : ensures ample height to see whole picture
    % -x x -y y -z z step
    axis([-10 10 -10 10 0 1])
    % Set the viewing angle
    view(-135, 20)
    % Label the axes.
    xlabel('x0 (m)')
    ylabel('y0 (m)')
    zlabel('z0 (m)')
    % Turn on Grid
    grid on
    % Title of Figure
    title('Mag Field')
    hold on
    
    magfield([1;0;0])
    drawnow
    