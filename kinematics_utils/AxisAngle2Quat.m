function q = AxisAngle2Quat(s,p)
   q = [sin(p/2)*s(1); sin(p/2)*s(2); sin(p/2)*s(3); cos(p/2)];
end