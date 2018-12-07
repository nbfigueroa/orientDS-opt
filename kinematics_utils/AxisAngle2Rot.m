function R = AxisAngle2Rot(s,phi)
    

r_3 = s(3);
r_2 = s(2);
r_1 = s(1);

if phi == 0
    R = eye(3);
else
    n_x = r_1/phi;
    n_y = r_2/phi;
    n_z = r_3/phi;
    R =[ 1 - 2*(n_y^2 + n_z^2)*sin(.5*phi)^2       -n_z *sin(phi) + 2*n_x*n_y *sin(.5*phi)^2    n_y*sin(phi) + 2*n_z*n_x *sin(.5*phi)^2;
         n_z *sin(phi) + 2*n_x*n_y*sin(.5*phi)^2   1 - 2*(n_z^2 + n_x^2)*sin(.5*phi)^2         -n_x*sin(phi) + 2*n_y*n_z *sin(.5*phi)^2;
        -n_y *sin(phi) + 2*n_z*n_x*sin(.5*phi)^2    n_x *sin(phi) + 2*n_y*n_z *sin(.5*phi)^2    1 - 2*(n_x^2 + n_y^2)*sin(.5*phi)^2];
end    
    
      
    
    
    