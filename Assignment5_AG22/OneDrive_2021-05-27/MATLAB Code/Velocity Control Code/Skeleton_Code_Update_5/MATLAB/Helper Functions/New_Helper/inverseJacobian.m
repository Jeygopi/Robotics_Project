function result = inverseJacobian(x,y,z,xo,yo,zo,Q1,Q2,Q3,Q4,Q5)

    l1 = 19.8;
    l2 = 27.2;
    l3 = 29.4;
    l45 = 19.8;
    %UNCOMMENT THIS LINE NORMALLY
    %Q2 = 0;
    delta1 = sin(Q1)*(l45);
    delta2 = cos(Q1)*(l45);
    delta3 = sin(Q2+Q3)*l3 + sin(Q2)*l2;
    delta5 = cos(Q2 + Q3)*l3;
    delta4 = delta5 + cos(Q2)*l2;
    
    J = [ (-sin(Q1)*delta4),(delta2 -cos(Q1)*delta3),(delta2 -sin(Q2+Q3)*cos(Q1)*l3 + delta2), (delta2),0;
        (cos(Q1)*delta4), (-sin(Q1)*delta3 + delta1), (-sin(Q2+Q3)*sin(Q1)*l3 + delta1), delta1,0;
        0, delta4, delta5, 0, 0;
        0, sin(Q1), sin(Q1), sin(Q1) 0;
        0, -cos(Q1), -cos(Q1), -cos(Q1), 0;
        1, 0, 0, 0, -1];
    
    Jpseudoinv = inv(J'*J)*J';
    
    result = Jpseudoinv*[x,y,z,xo,yo,zo]';
    
    Q1dot = result(1);
    Q2dot = result(2);
    Q3dot = result(3);
    Q4dot = result(4);
    Q5dot = result(5);
    
    