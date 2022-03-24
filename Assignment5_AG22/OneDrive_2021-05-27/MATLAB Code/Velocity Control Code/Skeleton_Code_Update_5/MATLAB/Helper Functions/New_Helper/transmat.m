function [TransMat,R_x,D_x,D_z,R_z] = transmat(alpha,a,d,theta)
    %This function is used to perform matrix multiplication to obtain the
    %full
    R_x = [1,0,0,0;0,cos(alpha),-sin(alpha),0;0,sin(alpha),cos(alpha),0;0,0,0,1]
    D_x = [1,0,0,a;,0,1,0,0;0,0,1,0;0,0,0,1]
    D_z = [1,0,0,0;0,1,0,0;0,0,1,d;0,0,0,1]
    R_z = [cos(theta),-sin(theta),0,0;sin(theta),cos(theta),0,0;0,0,1,0;0,0,0,1]
    TransMat = R_x*D_x*D_z*R_z
end