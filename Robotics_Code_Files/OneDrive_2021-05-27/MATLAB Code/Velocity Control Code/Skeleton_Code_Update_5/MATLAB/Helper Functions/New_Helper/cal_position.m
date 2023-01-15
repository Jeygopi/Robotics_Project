function output = cal_position(coefficients,t)
    coefficients;
    output = [];
        coef = coefficients;
        output = coef(1) + coef(2)*t + coef(3) * t^2 + coef(4) * t^3;
end