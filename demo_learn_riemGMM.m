%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo Script for Learning a Riemmanian GMM on Quat+Euclidean Data        %%
% Author: Nadia Figueroa                                                  %%
% Disclaimer: This code was adapted from the pbdlib-matlab demo script    %%
% demo_Riemannian_quat_GMR02.m which implements the algorithm below       %%
%  "An Approach for Imitation Learning on Riemannian Manifolds",          %%
%  Zeestraten, M. J. A etal.                                              %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Step 1b: Load 6DoF Dataset %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
plot_6DOF_reference_trajectories(Hdata, ori_samples, frame_size, box_size, 'r'); 

%%%%% Plot Quaternion trajectories %%%%%
title_name = 'Quaternion Trajectories from Gazebo Demonstrations ';
[h] = plot_Quaternion_trajectories(Data_QX, title_name);

%%%% Setting some variables and Prepare Data %%%%%
xIn  = Data_QX(5:6,:);
xOut = Data_QX(1:4,:);
M_in     = size(xIn,1);
N     = size(xIn,2);
regress_approach = 'Riemannian GMM';

% Position Input, Quaternion Output
x = [xIn; xOut];

% Tangent Space of Riemmanian Manifold (quat looses a dim, euclidean same)
uIn = xIn;
uOut = logmap(xOut,[0;1;0;0]);
u = [uIn; uOut];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Step 2: Learn Riemannian GMM   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set Estimation Parameters   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbIter    = 10; %Number of iteration for the Gauss Newton algorithm
nbIterEM  = 10; %Number of iteration for the EM algorithm
clear model
model.nbStates = 6; %Number of states in the GMM
switch M_in
    case 2
        model.nbVar    = 5; %Dimension of the tangent space (incl. time)
        model.nbVarMan = 6; %Dimension of the manifold (incl. time)
    case 3
        model.nbVar    = 6; %Dimension of the tangent space (incl. 2D Euclidean)
        model.nbVarMan = 7; %Dimension of the manifold (incl. 2D Euclidean)
end
model.params_diagRegFact = 1E-4; %Regularization of covariance

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Estimate initial GMM on tangent space %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
est_options = [];
est_options.type             = 1;   % GMM Estimation Alorithm Type
est_options.maxK             = 20;  % Maximum Gaussians for Type 1
est_options.fixed_K          = model.nbStates;  % Fix K and estimate with EM for Type 1
est_options.do_plots         = 0;   % Plot Estimation Statistics
est_options.sub_sample       = 1;   % Size of sub-sampling of trajectories
[Priors_tang, Mu_tang, Sigma_tang] = fit_gmm(u, [], est_options);

% Populate tangent space GMM model parameters
model.Priors = Priors_tang;
model.Mu = Mu_tang;
model.Sigma = Sigma_tang;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fit Riemmanian GMM with Gauss-Newton method %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
[model, U0, uTmp] = fit_rgmm(model, xIn, xOut, M_in, N, nbIter, nbIterEM);
toc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     Step 4: Regress Quats from Euclidean Space with Riemannian GMM    %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch M_in
    case 2
        % 2d- Euclidean Manifold
        in=1:2; out=3:5; outMan=3:6;
    case 3
        % 3d- Euclidean Manifold
        in=1:3; out=4:6; outMan=3:7;
end

% Create regressor function handle
clear qx_rgmr
qx_rgmr  = @(x) rgmr_regressor(model, x, in, out, outMan, U0, nbIter);

% Compute quat_dist error on training data
mean_quat_error = mean(quat_error(qx_rgmr, Data_QX(:,1:10:end)));
fprintf('Riemannian GMR with got quat_dist on training set: %d \n', mean_quat_error);

% Compare Quaternions from Demonstration vs Regressor
h_quat = visualizeEstimatedQuaternions(Data_QX, qx_rgmr, regress_approach);

%% Create 6DoF trajectories from Regressed Orientations %%%%%
demo_quats = qx_rgmr(Data_QX(5:6,:));
demo_x     = Data_QX(5:6,:)+att;
demo_H     = get_Hdata(demo_quats, demo_x);
demo_Hdata{1} = demo_H; 

%% %%% Plot Position/Velocity Trajectories %%%%%
vel_samples = 80; vel_size = 0.75; 
[h_data, h_att, h_vel] = plot_reference_trajectories_DS(Data, att, vel_samples, vel_size);
axis equal;
limits = axis;

%%%%% Draw Obstacle %%%%%
rectangle('Position',[-1 1 6 1], 'FaceColor',[.85 .85 .85]); hold on;
h_att = scatter(att(1),att(2), 150, [0 0 0],'d','Linewidth',2); hold on;

%%%%% Plot 6DoF trajectories %%%%%
ori_samples = 50; frame_size = 0.25; box_size = [0.45 0.15 0.05];
plot_6DOF_reference_trajectories(demo_Hdata, ori_samples, frame_size, box_size, 'r');  
title('Learned Orientations with Riem-GMM', 'Interpreter','LaTex','FontSize',20)
