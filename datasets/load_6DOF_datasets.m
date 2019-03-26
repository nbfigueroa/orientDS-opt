function [Data, Data_sh, att, x0_all, dt, data, qdata, Hdata, Data_QX] = load_6DOF_datasets(pkg_dir, choosen_dataset, sub_sample)

switch  choosen_dataset
    case 1             
             load(strcat(pkg_dir,'datasets/procdata_icubGazebo_right'))
    case 2
end

% Process and sub-sample trajectories... might need to add qdata
[Data, Data_sh, att, x0_all, ~, data, Hdata] = processDataStructureOrient(data, Hdata, sub_sample);

% Create dataset for learning joint distribution (Quaternion/Xi)
Data_QX = [];
for i=1:length(data)
    q_data     = qdata{i}(:,1:sub_sample:end);
    xi_data    = data{i}(1:2,1:sub_sample:end);    
    q_xi       = [q_data; xi_data];
    Data_QX    = [Data_QX q_xi];  
end


end