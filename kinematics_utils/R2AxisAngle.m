function [ w,phi ] = R2AxisAngle( R )
% convert from rotation matrix to angle-axis representation

T = trace(R);
if T < -1
    R = -R;
    T = trace(R);
end
phi = acos ( ( T - 1)/2);

w = zeros(3,1);
w(1) = (R(3,2) - R(2,3))/sqrt((R(3,2) - R(2,3))^2+(R(1,3) - R(3,1))^2+(R(2,1) - R(1,2))^2);
w(2) = (R(1,3) - R(3,1))/sqrt((R(3,2) - R(2,3))^2+(R(1,3) - R(3,1))^2+(R(2,1) - R(1,2))^2);
w(3) = (R(2,1) - R(1,2))/sqrt((R(3,2) - R(2,3))^2+(R(1,3) - R(3,1))^2+(R(2,1) - R(1,2))^2);

end

