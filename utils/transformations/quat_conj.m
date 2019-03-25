function [q_bar] = quat_conj(q)
q_bar = q;
q_bar(1:3,:) = -q_bar(1:3,:);
end