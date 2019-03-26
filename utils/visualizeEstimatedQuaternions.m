function [h] = visualizeEstimatedQuaternions(Data_QX, quat_regr)
h = figure('Color',[1 1 1]);
qdata_hat = quat_regr(Data_QX(5:6,:));    

% Reference Quaternion Trajectories
plot(Data_QX(1,:)', '.-','Color',[1 0 1], 'LineWidth',1); hold on;
plot(Data_QX(2,:)', '.-','Color',[0 1 0], 'LineWidth',1); hold on;
plot(Data_QX(3,:)', '.-','Color',[0 0 1], 'LineWidth',1); hold on;
plot(Data_QX(4,:)', '.-','Color',[1 0 1], 'LineWidth',1); hold on;


% Estimated Quaternion Trajectories
plot(qdata_hat(1,:)','--','Color',[1 0 0], 'LineWidth', 1); hold on;
plot(qdata_hat(2,:)','--','Color',[0 1 0], 'LineWidth', 1); hold on;
plot(qdata_hat(3,:)','--','Color',[0 0 1], 'LineWidth', 1); hold on;
plot(qdata_hat(4,:)','--','Color',[1 0 1], 'LineWidth',1); hold on;

legend({'$q^{ref}_{1}$','$q^{ref}_{2}$','$q^{ref}_{3}$','$q^{ref}_{4}$', ...,
    '$q^{d}_{1}$','$q^{d}_{2}$', '$q^{d}_{3}$', '$q^{d}_{4}$'}, 'Interpreter', 'LaTex', 'FontSize', 15)
grid on;
    
title('Real vs Estimated Quaternions', 'Interpreter', 'LaTex', 'FontSize', 15)

end