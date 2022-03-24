function [Q1t,Q2t,Q3t,Q4t,Q5t] = jointspacetransform(Q1,Q2,Q3,Q4,Q5)
    Q1t = -Q1;
    Q2t = -Q2;
    Q3t = -Q3 - pi/2;
    Q4t = Q4 - pi/2;
    Q5t = Q5 + pi/2;