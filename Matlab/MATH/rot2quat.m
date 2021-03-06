%% Rotation Matrix to Quaternion
% Author: Mohamed Ghori
function [quat] = rot2quat(R)
%Print Task Name
Task = 'Running Rot 2 Quat'
%---------------------
    %% Quaternion Decomposition && Assignment    
        %Angle
    Tau = trace(R)
    if Tau == -1
        Tau = 3;
        R = eye(3);
    end
    %theta = acos((Tau-1)/2);
    %w = theta/2;
        %Axis    
    
    %u = (1/(2*sin(theta)))*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
        %Quaternion
    qw = sqrt(1+Tau)/2;
     quat = [qw;(R(3,2)-R(2,3))/(4*qw);(R(1,3)-R(3,1))/(4*qw);(R(2,1)-R(1,2))/(4*qw)];   
    %quat = [cos(w);sin(w)*u(1);sin(w)*u(2); sin(w)*u(3)];

end