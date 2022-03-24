function [Q1d,Q2d,Q3d,Q4d,Q5d] = jointspaceveltransform(Q1dot,Q2dot,Q3dot,Q4dot,Q5dot)

    Q1d = -Q1dot;
    Q2d = -Q2dot;
    Q3d = -Q3dot;
    Q4d = -Q4dot;
    Q5d = Q5dot;