function output = cal_velocity(coefficients,t)
    coefficients;
    output = [];
        coef = coefficients;
        output = coef(2) + 2*coef(3)*t + 3*coef(4)*t^2;