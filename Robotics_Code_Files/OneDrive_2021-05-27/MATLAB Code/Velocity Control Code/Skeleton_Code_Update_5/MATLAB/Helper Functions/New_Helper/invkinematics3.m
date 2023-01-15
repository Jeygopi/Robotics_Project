function [Q1,Q2,Q3,Q4,Q5] = invkinematics(a14,a24,a34,state)
    %This is used to calculate the inverse kinematics for the robotic
    %system, note that l_1=34,l_2=34,l_3=34,l_4=7.5,l_5=7.5 have been already substituted in
    %as their chosen values. 
    
    %state input determines which configuration is requested. state = 1
    %returns the elbow down configuration, and state = -1 returns the elbow
    %up configuration
    zp = 7.5 + 7.5 + a34 - 34
    Q1 = atan2d(a24,a14)
    xp = a14*cosd(Q1) + a24*sind(Q1)
    %Only the negative solution manifold is considered for Q_3, as
    %discussed in the report. This is so that the elbow up configuration is
    %obtained. 
    Q3 = state*acosd((xp*xp + zp*zp - (34^2 + 34^2))/(2*34*34)) 
    Q2 = atan2d((34 + 34*cosd(Q3))*zp - 34*sind(Q3)*xp,(34 + 34*cosd(Q3))*xp + 34*sind(Q3)*zp)
    Q4 = -Q3-Q2
    Q5 = Q1
end