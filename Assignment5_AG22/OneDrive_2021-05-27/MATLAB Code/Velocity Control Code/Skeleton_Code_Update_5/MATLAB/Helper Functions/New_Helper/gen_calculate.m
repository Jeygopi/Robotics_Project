function output = gen_calculate(tinit,tf,df,vf,dinit,vinit)
    A = [1 tinit tinit^2 tinit^3;
    0 1 2*tinit 3*tinit^2;
    1 tf tf^2 tf^3;
    0 1 2*tf 3*tf^2];
    C = [dinit;
    vinit;
    df;
    vf];
%getting the results of the coefficients
    output = inv(A) * C;
end