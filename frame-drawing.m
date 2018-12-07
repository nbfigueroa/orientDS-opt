%% Draw 6DOF frames
figure('Color',[1 1 1])

% % Example 1: Frames to visualize
t_1 = [0.25 0 0]';
t_2 = [0.25 0.25 0]';
roll = -0.015; pitch = 0.001; yaw= 3.13535;
% R_1 = eul2rotm([yaw,pitch,roll])
R_1 = eul2rotm([yaw,pitch,roll])*rotationMatrx('z',deg2rad(180));
roll = -0.0014; pitch = -0.0026; yaw= -1.69614;
% R_2 = eul2rotm([yaw,pitch,roll])
R_2 = eul2rotm([yaw,pitch,roll])*rotationMatrx('z',deg2rad(180));


% Example 1: Frames to visualize
t_1 = [0.25 0 0]';
t_2 = [0.25 0.25 0]';
R_1 = [0.999  -0.0083 0.0021; 0.00837 0.9999 0.0029; -0.00215 -0.00292 0.999]
R_2 = [0.2901 -0.9569 -0.00157; 0.9569 0.2901 0.003; -0.00271 -0.00246 0.999]

R_1 = [0.999 -0.0199 0; 0.0199 0.9999 -0.0108; 0.00028 0.0108 0.999]
R_2 = R_1*rotationMatrx('z',deg2rad(-48.4149))

% Desired Angular Velocity
Omega_X = logm(R_2 * R_1')

% Create Homogeneous MatriX Representation
H = zeros(4,4,2);
H(1:3,1:3,1) = R_1;
H(1:3,4,1)   = t_1;
H(1:3,1:3,2) = R_2;
H(1:3,4,2)   = t_2;

% Draw World Reference Frame
drawframe(eye(4),0.1); hold on;

for i=1:2
    
    % Draw Frame
    drawframe(H(:,:,i),0.1); hold on;
    
    % Draw Robot
    [xyz] = R2rpy(H(1:3,1:3,i));
    xyz = xyz*180/pi;
    t = H(1:3,4,i);
    drawCuboid([t(1) t(2) t(3) 0.05 0.05 0.1 xyz(3) xyz(2) xyz(1)], 'FaceColor', 'g');
    alpha(.5)
    hold on;
end

view([-66 11]);
title('Position/Orientation Data $\xi = [x,y,z,s_x,s_y,s_z,\theta]$','Interpreter','LaTex')
xlabel('$x_1$','Interpreter','LaTex');
ylabel('$x_2$','Interpreter','LaTex');
zlabel('$x_3$','Interpreter','LaTex');
grid on;
axis equal;