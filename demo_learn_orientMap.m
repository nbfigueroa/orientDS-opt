%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo Script for Couple Orientation Dynamics Learning introduced in :    %
% ........                                                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2018 Learning Algorithms and Systems Laboratory,          %
% EPFL, Switzerland                                                       %
% Author:  Nadia Figueroa                                                 % 
% email:   nadia.figueroafernandez@epfl.ch                                %
% website: http://lasa.epfl.ch                                            %
%                                                                         %
% This work was supported by the EU project Cogimon H2020-ICT-23-2014.    %
%                                                                         %
% Permission is granted to copy, distribute, and/or modify this program   %
% under the terms of the GNU General Public License, version 2 or any     %
% later version published by the Free Software Foundation.                %
%                                                                         %
% This program is distributed in the hope that it will be useful, but     %
% WITHOUT ANY WARRANTY; without even the implied warranty of              %
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General%
% Public License for more details                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Demo Script for Learning Couple Orientation Dynamics from Demos %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    Step 1: Load 6DOF Dataset     %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;
pkg_dir         = '/home/nbfigueroa/Dropbox/PhD_papers/CoRL-2018-Extension/code/orientDS-opt/';
%%%%%%%%%%%%%%%%%%% Choose a Dataset %%%%%%%%%%%%%%%%%%%%%                     
choosen_dataset = 1; % 1: Demos from Gazebo Simulations
                     % 2: Demos from Real iCub 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sub_sample      = 5; % To sub-sample trajectories                   
[Data, Data_sh, att, x0_all, ... 
dt, data, qdata, Hdata, Data_QX] = load_6DOF_datasets(pkg_dir, choosen_dataset, sub_sample);

%%%%% Plot Position/Velocitty Trajectories %%%%%
vel_samples = 80; vel_size = 0.75; 
[h_data, h_att, h_vel] = plot_reference_trajectories_DS(Data, att, vel_samples, vel_size);
axis equal;
limits = axis;

%%%%% Draw Obstacle %%%%%
rectangle('Position',[-1 1 6 1], 'FaceColor',[.85 .85 .85]); hold on;
h_att = scatter(att(1),att(2), 150, [0 0 0],'d','Linewidth',2); hold on;

%%%%% Plot 6DoF trajectories %%%%%
ori_samples = 300; frame_size = 0.25; box_size = [0.45 0.15 0.05];
plot_6DOF_reference_trajectories(Hdata, ori_samples, frame_size, box_size); 

%%%%% Plot Quaternion trajectories %%%%%
title_name = 'Quaternion Trajectories from Gazebo Demonstrations ';
[h] = plot_Quaternion_trajectories(Data_QX, title_name);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    Step 2: Learn Joint Distr. of Quats+Pos as GMM of p(quat,\xi)     %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  OPTION 1 (STANDARD GMM FITTING): Fit GMM to Trajectory Data %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Standard GMM Estimation Algorithm %%%%%%%%%%%%%%%
% 0: Physically-Consistent Non-Parametric (Collapsed Gibbs Sampler)
% 1: GMM-EM Model Selection via BIC
% 2: CRP-GMM (Collapsed Gibbs Sampler)
est_options = [];
est_options.type             = 1;   % GMM Estimation Alorithm Type   
est_options.maxK             = 20;  % Maximum Gaussians for Type 1
est_options.fixed_K          = [];  % Fix K and estimate with EM for Type 1
est_options.samplerIter      = 50;  % Maximum Sampler Iterations
est_options.do_plots         = 1;   % Plot Estimation Statistics
est_options.sub_sample       = 5;   % Size of sub-sampling of trajectories
                                    % 1/2 for 2D datasets, >2/3 for real    
% Metric Hyper-parameters
est_options.estimate_l       = 1;   % '0/1' Estimate the lengthscale, if set to 1
est_options.l_sensitivity    = 2;   % lengthscale sensitivity [1-10->>100]
est_options.length_scale     = [];  % if estimate_l=0 you can define your own

% Fit GMM to Trajectory Data
[Priors, Mu, Sigma] = fit_gmm(Data_QX, [], est_options);

%% Generate GMM data structure for Mapping Function
clear qx_gmm; qx_gmm.Mu = Mu; qx_gmm.Sigma = Sigma; qx_gmm.Priors = Priors; 
qx_gmr          = @(x) gmr_regressor(qx_gmm.Priors, qx_gmm.Mu, qx_gmm.Sigma,x, 5:6, 1:4);

% Compute quat_dist error on training data
mean_quat_error = mean(quat_error(qx_gmr, Data_QX));
fprintf('Standard GMR with got quat_dist on training set: %d \n', mean_quat_error);

% Compare Quaternions from Demonstration vs Regressor
h_quat = visualizeEstimatedQuaternions(Data_QX, qx_gmr);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    Step 3: Simulate 6DOF learned motions    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ==> Assume an lpv-ds model was learned or pre-loaded <== %%
DS_name = 'icub-Object-DS-1';
matfile = strcat(pkg_dir,'/models/', DS_name,'.mat');
load(matfile)
if constr_type == 1
    ds_lpv = @(x) lpv_ds(x-repmat(att,[1 size(x,2)]), ds_gmm, A_g, b_g);
else
    ds_lpv = @(x) lpv_ds(x, ds_gmm, A_k, b_k);
end

%% % Fill in plotting options
ds_plot_options = [];
ds_plot_options.sim_traj  = 1;            % To simulate trajectories from x0_all
ds_plot_options.x0_all    = x0_all;       % Intial Points
ds_plot_options.init_type = 'cube';       % For 3D DS, to initialize streamlines
                                          % 'ellipsoid' or 'cube'  
ds_plot_options.nb_points = 30;           % No of streamlines to plot (3D)
ds_plot_options.plot_vol  = 1;            % Plot volume of initial points (3D)
ds_plot_options.limits    = limits;

[hd, hs, hr, x_sim] = visualizeEstimatedDS(Xi_ref, ds_lpv, ds_plot_options);
rectangle('Position',[-1 1 6 1], 'FaceColor',[.85 .85 .85]); hold on;
h_att = scatter(0,3, 150, [0 0 0],'d','Linewidth',2); hold on;
h_orig = scatter(0,0, 150, [0 1 0],'d','Linewidth',2); hold on;
axis tight;

%% Plot 6DOF Frames and object on top of DS
demo_quats = qx_gmr(Data_QX(5:6,:));


% 
% switch constr_type
%     case 0
%         title('GMM-based LPV-DS with QLF', 'Interpreter','LaTex','FontSize',20)
%     case 1
%         title('GMM-based LPV-DS with P-QLF (v0) ', 'Interpreter','LaTex','FontSize',20)
%     case 2
%         title('GMM-based LPV-DS with P-QLF', 'Interpreter','LaTex','FontSize',20)
% end




