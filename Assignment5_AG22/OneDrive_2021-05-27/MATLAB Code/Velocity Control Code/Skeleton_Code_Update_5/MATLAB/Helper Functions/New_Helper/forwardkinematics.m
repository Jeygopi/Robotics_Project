function outputmat = forwardkinematics(a1,a2,a3,a4,a5)
    %Given a set of angles, output the task space pose
    syms Q1 Q2 Q3 Q4 Q5 l1 l2 l3 l4 l5
    
    %% Produce transformation matrices of frame i with respect to i-1
    syms l1 l2 l3 l4 l5 Q1 Q2 Q3 Q4 Q5 P_x P_y P_z real


    l4l5_4 = [0 0 -(l4 + l5)];
    'hello'
    %%DH Table
    DH_Mat = [0,0,l1,Q1;pi/2,0,0,Q2;0,l2,0,Q3;0,l3,0,Q4;pi/2,0,l4,Q5;0,0,l5,0];

    %conversion of DH table into transformation matricies 
    [T_1to0,R_1to0_x,D_1to0_x,D_1to0_z,R_1to0_z] = transmat(DH_Mat(1,1),DH_Mat(1,2),DH_Mat(1,3),DH_Mat(1,4));

    [T_2to1,R_2to1_x,D_2to1_x,D_2to1_z,R_2to1_z] = transmat(DH_Mat(2,1),DH_Mat(2,2),DH_Mat(2,3),DH_Mat(2,4));

    [T_3to2,R_3to2_x,D_3to2_x,D_3to2_z,R_3to2_z] = transmat(DH_Mat(3,1),DH_Mat(3,2),DH_Mat(3,3),DH_Mat(3,4));

    [T_4to3,R_4to3_x,D_4to3_x,D_4to3_z,R_4to3_z] = transmat(DH_Mat(4,1),DH_Mat(4,2),DH_Mat(4,3),DH_Mat(4,4));

    [T_5to4,R_5to4_x,D_5to4_x,D_5to4_z,R_5to4_z] = transmat(DH_Mat(5,1),DH_Mat(5,2),DH_Mat(5,3),DH_Mat(5,4));

    [T_eto5,R_eto5_x,D_eto5_x,D_eto5_z,R_eto5_z] = transmat(DH_Mat(6,1),DH_Mat(6,2),DH_Mat(6,3),DH_Mat(6,4));

    clc

    T_eto0 = T_1to0*T_2to1*T_3to2*T_4to3*T_5to4*T_eto5;
    
    %substitutes the values into the T_eto0 matrix
    ou = (subs(T_eto0,[Q1,Q2,Q3,Q4,Q5,l1,l2,l3,l4,l5],[a1,a2,a3,a4,a5,34,34,34,7.5,7.5]))
    %This is to convert all the values to decimal values
    for i = size(ou,1)
        for j = size(ou,2)
            ou(i,j) = double(ou(i,j))
        end
    end
    
    outputmat = vpa(ou);
end