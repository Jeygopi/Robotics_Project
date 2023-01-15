function [x,y,z] = fwdkinematics(Q1,Q2,Q3,Q4,Q5)

    l1 = 19.8;
    l2 = 27.2;
    l3 = 29.4;
    l4 = 4.2;
    l5 = 19.8-4.2;
    delta1 = cos(Q2 + Q3 + Q4);
    delta3 = sin(Q2+Q3+Q4);
    delta2 = l3*cos(Q2 + Q3) + l2*cos(Q2) + (l4+l5)*delta3;
    
    TE_to0 = [sin(Q1)*sin(Q5) + cos(Q1)*cos(Q5)*delta1, cos(Q5)*sin(Q1) - cos(Q1)*delta1*sin(Q5), cos(Q1)*delta3, cos(Q1)*delta2;
        cos(Q5)*delta1*sin(Q1)-cos(Q1)*sin(Q5), -cos(Q1)*cos(Q5) - delta1*sin(Q1)*sin(Q5) sin(Q1)*delta3, sin(Q1)*delta2;
        cos(Q5)*delta3, -sin(Q5)*delta3, -delta1, l1 + l3*sin(Q2+Q3)-(l4*delta1)-(l5*delta1)+l2*sin(Q2);
        0,0,0,1]
    
    outputvec = TE_to0*[0,0,0,1]';
    x = outputvec(1);
    y = outputvec(2);
    z = outputvec(3);
    %x = cos(Q1)*delta2;
    %y = sin(Q1)*delta2;
    %z = l1 + l3*sin(Q2+Q3) - l45*delta1 + l2*sin(Q2);