function [ joint_states ] = readJointsfromBag(bag, joint_state_topic)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

% Functions for each topic type
joint_pos = @(j) j.position;

% Read Pose Topics
[msgs, meta]      = bag.readAll(joint_state_topic);
[joint_positions] = ros.msgs2mat(msgs, joint_pos);
joint_t = cellfun(@(x) x.time.time, meta); % Time Stamps

joint_states = [joint_positions;joint_t];

end

