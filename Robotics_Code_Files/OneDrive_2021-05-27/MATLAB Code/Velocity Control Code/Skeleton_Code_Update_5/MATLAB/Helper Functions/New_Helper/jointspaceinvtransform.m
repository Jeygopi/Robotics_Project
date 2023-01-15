function [Q1i,Q2i,Q3i,Q4i,Q5i] = jointspaceinvtransform(Q1,Q2,Q3,Q4,Q5)
    
    Q1i = -Q1;
    Q2i = -Q2;
    Q3i = -(Q3 + pi/2);
    Q4i = -Q4;
    Q5i = Q5 - pi/2;
    
    