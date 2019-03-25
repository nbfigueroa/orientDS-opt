%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%        Data Processing Script      %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all

% Load Data from Michael's Demonstrations
load('rawdata_icubwMichael')

% Trajectories to use
raw_data(1:3) = [];
raw_data(2) = [];

data = []; 
odata = [];
window_size = 1051; crop_size = (window_size+1)/2; 
dt = raw_data{1}.dt;
pre_subsample = 3;
pos_cutting = 1e-3;
vel_cutting = 1e-4;
for i=1:length(raw_data)
        ee_pose = raw_data{i}.ee_pose;
        dx_nth = sgolay_time_derivatives(ee_pose(1:3,:)', dt, 2, 2, window_size);
        X     = dx_nth(:,:,1)';
        X_dot = dx_nth(:,:,2)';    
        q     = ee_pose(4:end-1,crop_size:end-crop_size);
        
        % Trim data with zero-velocity
        zero_vel_idx = [];
        for ii=1:length(X_dot)
            if (norm(X_dot(:,ii)) < vel_cutting)
                zero_vel_idx = [zero_vel_idx ii];
            end
        end
        fprintf('Measurements removed %d \n', length(zero_vel_idx));
        X(:,zero_vel_idx) = [];
        X_dot(:,zero_vel_idx) = [];  
        q(:,zero_vel_idx) = [];
        
        % Check final measurements
        [idx_end, dist_end] = knnsearch( X(:,end-round(1/dt):end)', X(:,end)', 'k',round(1/dt));
        idx_zeros = idx_end(dist_end < pos_cutting);
        X(:,idx_zeros)     = [];
        X_dot(:,idx_zeros) = [];
        q(:,idx_zeros)  = [];
        
        % Make last measurment 0 velocity and scale the previous
        X_dot(:,end)   = zeros(size(X,1),1);
        for k =1:20
            X_dot(:,end-k) = (X_dot(:,end-k) +  X_dot(:,end-(k-1)))/2;
        end
               
        data{i} = [X(:,1:pre_subsample:end); X_dot(:,1:pre_subsample:end)];                
        
        % Make x-axis of rotation the heading of the robot
        q_tmp = q(:,1:pre_subsample:end);
        R_tmp = quaternion(q_tmp,1);

        % Populate H matrix
        H = zeros(4,4,size(R_tmp,3));
        for r=1:length(R_tmp)       
            R(:,:,r) =  R_tmp(:,:,r)*rotationMatrx('z',deg2rad(90));            
            H(:,:,r)     = eye(4);
            H(1:3,1:3,r) = R(:,:,r);
            H(1:3,4,r)   = data{i}(1:3,r) ;            
        end
        
        Rdata{i} = R;
        Hdata{i} = H;
        q = quaternion(R,1);
        qdata{i} = R;
        
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     Sub-sample measurements and Process for Learning      %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sub_sample = 5;
[Data, Data_sh, att, x0_all, dt, data, Hdata] = processDataStructureOrient(data, Hdata, sub_sample);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%       Visualize 6DoF data in 3d       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Extract Position and Velocities
M          = size(Data,1)/2;    
Xi_ref     = Data(1:M,:);
Xi_dot_ref = Data(M+1:end,:); 


%%%%% Plot 3D Position/Velocity Trajectories %%%%%
vel_samples = 50; vel_size = 0.5; 
[h_data, h_att, h_vel] = plot_reference_trajectories_DS(Data, att, vel_samples, vel_size); 
hold on;

%%%%%% Plot Wall %%%%%%
cornerpoints = [-6.35 -1.55 0;  -6.85 -1.55 0;  -6.85 -2.05 0; -6.35 -2.05 0;
                -6.35 -1.55 0.5;  -6.85 -1.55 0.5;  -6.85 -2.05 0.5; -6.35 -2.05 0.5];            
plotminbox(cornerpoints,[0.5 0.5 0.5]); hold on;


%%%%% Plot 6DoF trajectories %%%%%
ori_samples = 200; frame_size = 0.25; box_size = [0.15 0.1 0.05];
plot_6DOF_reference_trajectories(Hdata, ori_samples, frame_size, box_size); 

