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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize 6DoF data in 3d %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 3;
Xi