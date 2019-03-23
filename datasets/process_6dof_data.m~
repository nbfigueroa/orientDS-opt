%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%       Load data from position/orientation trajectories    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all; clear all; clc
pkg_dir         = '/home/nbfigueroa/Dropbox/PhD_papers/CoRL-2018/code/ds-opt/';
load(strcat(pkg_dir,'datasets/icub_gazebo_demos/icub_data'))

% Position/Velocity Trajectories
vel_samples = 50; vel_size = 0.75; 
[h_data, h_att, h_vel] = plot_reference_trajectories_DS(Data, att, vel_samples, vel_size);

% Draw Wall
rectangle('Position',[-1 1 6 1], 'FaceColor',[.85 .85 .85]); hold on;
limits = axis;

% Extract Position and Velocities
M          = size(Data,1)/2;    
Xi_ref     = Data(1:M,:);
Xi_dot_ref = Data(M+1:end,:);   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Create orientation/6DoF data    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Rdata = []; Hdata = [];
for o=1:length(odata)
    theta_angles = odata{o};
    R = zeros(3,3,length(theta_angles));
    H = zeros(4,4,length(theta_angles));
    for r=1:length(theta_angles)
        % Populate R matrix
        R(:,:,r)  = eul2rotm([theta_angles(r),0,0]);
        
        % Populate H matrix
        H(:,:,r)     = eye(4);
        H(1:3,1:3,r) = R(:,:,r);
        H(1:3,4,r)   = [data{o}(1:2,r)+ att; 0] ;        
        
    end            
    Rdata{o} = R;
    Hdata{o} = H;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize 6DoF data in 3d %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 3;
Xi_ref(M,:)     = 0;
Xi_dot_ref(M,:) = 0;
Data_xi = [Xi_ref; Xi_dot_ref];
att_xi = att; att_xi(M,1) = 0;

%%%%% Plot 3D Position/Velocity Trajectories %%%%%
vel_samples = 50; vel_size = 0.75; 
[h_data, h_att, h_vel] = plot_reference_trajectories_DS(Data_xi, att_xi, vel_samples, vel_size); 
hold on;


%%%%%% Plot Wall %%%%%%
cornerpoints = [-1 1 0;  5 1 0; 5 2 0; -1 2 0;
                -1 1 0.25;  5 1 0.25; 5 2 0.25; -1 2 0.25];

            
plotminbox(cornerpoints,[0.5 0.5 0.5]); hold on;


%%%%% Plot 6DoF trajectories %%%%%
ori_samples = 200; ori_size = 0.25;
for h=1:length(Hdata)
    H = Hdata{h};
    
    for i=1:ori_samples:length(H)
        
        % Draw Frame
        drawframe(H(:,:,i),ori_size); hold on;
        
        % Draw Robot
        [xyz] = R2rpy(H(1:3,1:3,i));
        xyz = xyz*180/pi;
        t = H(1:3,4,i);
        drawCuboid([t(1) t(2) t(3) 0.45 0.15 0.05 xyz(3) xyz(2) xyz(1)], 'FaceColor', 'g');
        alpha(.5)
        hold on;
    end    
end