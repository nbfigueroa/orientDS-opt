function [ H ] = convert2H( X )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

H = zeros(4,4,length(X));
H(1:3,1:3,1:end) = reshape(quaternion(X(4:7,1:end)),[3 3 length(X)]);
H(1:3,4,1:end) = reshape(X(1:3,1:end),[3 1 length(X)]);   

end

