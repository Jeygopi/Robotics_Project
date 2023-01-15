function [Q1,Q2,Q3,Q4,Q5] = invkinematics(x,y,z,state)
    %This is used to calculate the inverse kinematics for the robotic
    %system, note that l_1=34,l_2=34,l_3=34,l_4=7.5,l_5=7.5 have been already substituted in
    %as their chosen values. 
    
    %state input determines which configuration is requested. state = 1
    %returns the elbow down configuration, and state = -1 returns the elbow
    %up configuration
    zp = 7.5 + 7.5 + z - 23.5;
    Q1 = atan2d(y,x);
    xp = x*cosd(Q1) + y*sind(Q1);
    %Only the negative solution manifold is considered for Q_3, as
    %discussed in the report. This is so that the elbow up configuration is
    %obtained. 
    Q3 = state*acosd((xp*xp + zp*zp - (28^2 + 25.4^2))/(2*28*25.4)) ;
    Q2 = atan2d((28 + 25.4*cosd(Q3))*zp - 25.4*sind(Q3)*xp,(28 + 25.4*cosd(Q3))*xp + 25.4*sind(Q3)*zp);
        Q4 = -Q3-Q2;
    
    Q5 = 0;
end