function [Q1,Q2,Q3,Q4,Q5] = invkinematics(x,y,z,state)
    %This is used to calculate the inverse kinematics for the robotic
    %system, note that l_1=24,l_2=28,l_3=25.4,l_4=19.8 have been already substituted in
    %as their chosen values. 
    
    %state input determines which configuration is requested. state = 1
    %returns the elbow down configuration, and state = -1 returns the elbow
    %up configuration
    z = -z;
    zp = 19.8 + z - 24;
    Q1 = atan2d(y,x);
    xp = x*cosd(Q1) + y*sind(Q1);
    %Only the negative solution manifold is considered for Q_3, as
    %discussed in the report. This is so that the elbow up configuration is
    %obtained. 
    Q3 = state*acosd((xp*xp + zp*zp - (29.4^2 + 25.4^2))/(2*29.4*25.4)) ;
    Q2 = atan2d((29.4 + 27.2*cosd(Q3))*zp - 27.2*sind(Q3)*xp,(29.4 + 27.2*cosd(Q3))*xp + 27.2*sind(Q3)*zp);
    
    
    Q4 = -Q3-Q2-21-90;
%     if Q3 > 35
%         Q4 = -Q3-Q2-21+(Q3 - 35)-90;
%     else
%         Q4 = -Q3-Q2-21-90;
%     end
        
    
    Q5 = Q1-90;
end