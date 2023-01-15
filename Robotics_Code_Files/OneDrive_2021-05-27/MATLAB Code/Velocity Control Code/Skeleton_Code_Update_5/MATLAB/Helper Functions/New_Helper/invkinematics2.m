function [Q1,Q2,Q3,Q4,Q5] = invkinematics2(x,y,z,state)
    %This is used to calculate the inverse kinematics for the robotic
    %system, note that l_1=34,l_2=34,l_3=34,l_4=7.5,l_5=7.5 have been already substituted in
    %as their chosen values. 
    
    %state input determines which configuration is requested. state = 1
    %returns the elbow down configuration, and state = -1 returns the elbow
    %up configuration
    
    l1 = 19.8;
    l2 = 27.2;
    l3 = 29.4;
    l4 = 4.2;
    l5 = 19.8-4.2;
    
    %%Measurements are wrong 
    zp = 4.2 +15.6 + z - 19.8;
    zp;
    Q1 = atan2(y,x);
    xp = x*cos(Q1) + y*sin(Q1);
    xp;
    %Only the negative solution manifold is considered for Q_3, as
    %discussed in the report. This is so that the elbow up configuration is
    %obtained. 
    Q3 = state*acos((xp*xp + zp*zp - (28^2 + 25.4^2))/(2*28*25.4)) ;
    Q3;
    Q2 = atan2(double((28 + 25.4*cos(Q3))*zp - 25.4*sin(Q3)*xp),double((28 + 25.4*cos(Q3))*xp + 25.4*sin(Q3)*zp));
    Q4 = -(Q3+Q2);
    
    Q5 = 0;
end