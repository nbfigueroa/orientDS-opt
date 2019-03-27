%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%        Data Processing Script      %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all

% Load Data from Michael's Demonstrations
load('rawdata_icubwMichael')

% Trajectories to use
raw_data(1:3) = [];
raw_data(3) = [];

data = []; 
odata = [];
window_size = 1051; crop_size = (window_size+1)/2; 
window_size_q = 251; crop_size_q = (window_size_q+1)/2; 
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
        
        dq_nth = sgolay_time_derivatives(q', dt, 2, 2, window_size_q);
        q     = dq_nth(:,:,1)';
        X     = X(:,crop_size_q:end-crop_size_q);
        X_dot = X_dot(:,crop_size_q:end-crop_size_q);        
        
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
               
        data{i} = [X(1:2,1:pre_subsample:end); X_dot(1:2,1:pre_subsample:end)];                                
        
        % Make x-axis of rotation the heading of the robot
        q_tmp = q(:,1:pre_subsample:end);        
        
        % Remove Measurements from 2nd demo
        if i==2
            xdata = data{i};
            xdata(:,3250:3788) = [];
            q_tmp(:,3250:3788) = [];
            
            
            dx_nth = sgolay_time_derivatives(xdata(1:2,:)', dt, 2, 2, window_size);
            X     = dx_nth(:,:,1)';
            X_dot = dx_nth(:,:,2)';
            xdata = [X;X_dot];
            q_tmp = q_tmp(:,crop_size:end-crop_size);                        
            
            xdata(:,6599:end) = []; 
            q_tmp(:,6599:end) = [];            
            data{i} = xdata;
        elseif i ==1
            xdata = data{i};
            xdata(:,6157:end) = []; 
            q_tmp(:,6157:end) = [];            
            data{i} = xdata;
        end
        
        R_tmp = quaternion(q_tmp,1);

        % Populate H matrix
        H = zeros(4,4,size(R_tmp,3));
        for r=1:length(R_tmp)       
            R(:,:,r)     = R_tmp(:,:,r)*rotationMatrx('z',deg2rad(90));            
            H(:,:,r)     = eye(4);
            H(1:3,1:3,r) = R(:,:,r);
            H(1:2,4,r)   = data{i}(1:2,r) ;            
        end
        
        Rdata{i} = R;
        Hdata{i} = H;
        q = quaternion(R,1);
        qdata{i} = q;
        
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     Sub-sample measurements and Process for Learning      %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sub_sample = 5;
[Data, Data_sh, att, x0_all, ~, data, Hdata] = processDataStructureOrient(data, Hdata, sub_sample);

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

%%%%% Draw Table %%%%%
rectangle('Position',[-6.75 -2.15 0.5 0.5], 'FaceColor',[.85 .85 .85]); hold on;

%%%%% Plot 6DoF trajectories %%%%%
ori_samples = 200; frame_size = 0.25; box_size = [0.15 0.1 0.05];
plot_6DOF_reference_trajectories(Hdata, ori_samples, frame_size, box_size, 'r'); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Playing around with quaternions   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Quaternions
figure('Color',[1 1 1])
for i=1:length(qdata)  
% for i=2:2
    qData = qdata{i};
    plot(1:length(qData),qData(1,:),'r-.','LineWidth',2); hold on;
    plot(1:length(qData),qData(2,:),'g-.','LineWidth',2); hold on;
    plot(1:length(qData),qData(3,:),'b-.','LineWidth',2); hold on;
    plot(1:length(qData),qData(4,:),'m-.','LineWidth',2); hold on;
    legend({'$q_1$','$q_2$','$q_3$','$q_4$'},'Interpreter','LaTex', 'FontSize',14)
    xlabel('Time-stamp','Interpreter','LaTex', 'FontSize',14);
    ylabel('Reference Quaternions','Interpreter','LaTex', 'FontSize',14);    
    grid on;
    axis tight;
end
title('Demonstrations from Real iCub Co-Manipulation','Interpreter','LaTex', 'FontSize',14);


%% Compute Omega from qdata
Odata = [];
for i=1:length(qdata)   
    qData = qdata{i}; 
    RData = Rdata{i};
    Omega = zeros(3,length(qData));
    for ii=2:length(qData)
        if true
            q_2 = quat_conj(qData(:,ii-1));
            q_1 = qData(:,ii);
            
            % Partitioned product
            % delta_q = quat_prod(q_1,q_2);            
            % Matrix product option 1
            % Q = QuatMatrix(q_1);
            % delta_q = Q*q_2;
            
            % Matrix product option 2
            delta_q = quat_multiply(q_1',q_2');            
            Omega(:,ii) = 2*quat_logarithm(delta_q)/dt;
        else
            % Using Rotation matrices
            R_2 = RData(:,:,ii-1);
            R_1 = RData(:,:,ii);
            Omega(:,ii) = rot_logarithm(R_1*R_2');
        end
    end                
    Odata{i} = Omega;
end

% Plot Angular Velocities
figure('Color',[1 1 1])
% for i=1:length(Odata)   
for i=1:1
    Omega = Odata{i};            
    plot(1:length(Omega),Omega(1,:),'r-.','LineWidth',2); hold on;
    plot(1:length(Omega),Omega(2,:),'g-.','LineWidth',2); hold on;
    plot(1:length(Omega),Omega(3,:),'b-.','LineWidth',2); hold on;
    legend({'$\omega_1$','$\omega_2$','$\omega_3$'},'Interpreter','LaTex', 'FontSize',14)
    xlabel('Time-stamp','Interpreter','LaTex', 'FontSize',14);
    ylabel('Angular Velocity (rad/s)','Interpreter','LaTex', 'FontSize',14);    
    grid on;
    axis tight;
end
title('Demonstrations from Real iCub Co-Manipulation','Interpreter','LaTex', 'FontSize',14);


%% Plot Integrated Quaternions
figure('Color',[1 1 1])
for i=1:length(Odata)   
    Omega = Odata{i};
    qData = qdata{i};
    qData_hat = zeros(4,length(Omega));    
    
    % Forward integration
    qData_hat(:,1) = qData(:,1); 
    for ii=2:length(qData_hat)
        omega_exp = quat_exponential(Omega(:,ii), dt);
        qData_hat(:,ii) = real(quat_multiply(omega_exp',qData_hat(:,ii-1)'));
    end
    
    % Plot Forward integrated quaternions
    plot(1:length(qData_hat),qData_hat(1,:),'r-.','LineWidth',2); hold on;
    plot(1:length(qData_hat),qData_hat(2,:),'g-.','LineWidth',2); hold on;
    plot(1:length(qData_hat),qData_hat(3,:),'b-.','LineWidth',2); hold on;
    plot(1:length(qData_hat),qData_hat(4,:),'m-.','LineWidth',2); hold on;
    legend({'$\hat{q}_1$','$\hat{q}_2$','$\hat{q}_3$','$\hat{q}_4$'},'Interpreter','LaTex', 'FontSize',14)
    xlabel('Time-stamp','Interpreter','LaTex', 'FontSize',14);
    ylabel('Forward Inegrated Quaternion $q(t + \Delta t)$','Interpreter','LaTex', 'FontSize',14);    
    grid on;
    axis tight;
end
title('Demonstrations from Real iCub Co-Manipulation','Interpreter','LaTex', 'FontSize',14);
