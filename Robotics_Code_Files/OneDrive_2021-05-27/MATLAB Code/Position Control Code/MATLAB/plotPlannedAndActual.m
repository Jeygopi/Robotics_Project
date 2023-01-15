%this is a code generates the actual trajectory and the planned trajectory.
%The actual trajectory is calculated by the forward kinematics, The joint
%space readings are obtained by the feedback of the servo motor. 

clear all
close all
clc

[startPoint,offset1] = target_position("G1",11,"knight");
[endPoint,offset2] = target_position("F2",3,"release_knight");
[viaPoint1,viaPoint2] = gen_viapoint(startPoint,endPoint)
pointList = [startPoint,viaPoint1,viaPoint2,endPoint];
vInit = [[0;0;0],[0;0;0], [2;0;0]];
vEnd = [[0;0;0.76],[2;0;0],[0;0;0.76]];
tStart = [0 1 4];
tEnd = [1 4 5];
coefficientList = [];
p = 0
q = 0
start = 1
index = 1
for i = 1:3
    sp = pointList(:,i);
    ep = pointList(:,i+1);
    vi = vInit(:,i);
    ve = vEnd(:,i);
    ts = tStart(i);
    te = tEnd(i);
    temp = [];
    for j = 1:3
        temp(:,j) = gen_calculate(ts,te,ep(j,:),ve(j,:),sp(j,:),vi(j,:));
    end
    temp;
    for t = ts:0.01:te
        position = [];
        for j = 1:3
            position(j) = cal_position(temp(:,j),t);
        end
        position;
        [Q1,Q2,Q3,Q4,Q5] = invkinematics(position(1),position(2),position(3),1)
        record(:,index) = [Q1,Q2,Q3,Q4,Q5];
        record1(:,index) = [position(1),position(2),position(3)]
        index = index + 1;
      
        q = q+1
    end
end
figure;
plot(record1(1,:),record1(3,:))


hold on
load('Q1i.csv');
load('Q2i.csv');
load('Q3i.csv');
load('Q4i.csv');
load('Q5i.csv');

%Transformational Matrix for the calculation of inverse kinematics and
%forward kinematics
syms Q1 Q2 Q3 Q4 Q5
l1 = 24;
l2 = 29.4;
l3 = 25.4;
l4 = 9.9;
l5 = 9.9;

Q4 = -Q2 - Q3;

T01 = [cos(Q1) -sin(Q1) 0 0;
    sin(Q1) cos(Q1) 0 0;
    0 0 1 l1;
     0 0 0 1];
 
T12 = [1 0 0 0; 0 cosd(90) -sind(90) 0; 0 sind(90) cosd(90) 0;  0 0 0 1] * [cos(Q2) -sin(Q2) 0 0; sin(Q2) cos(Q2) 0 0; 0 0 1 0; 0 0 0 1];
 
T23 = [cos(Q3) -sin(Q3) 0 l2;
     sin(Q3) cos(Q3) 0 0;
     0 0 1 0;
     0 0 0 1];
 
T34 = [cos(Q4) -sin(Q4) 0 l3;
     sin(Q4) cos(Q4) 0 0;
     0 0 1 0;
     0 0 0 1];
 
T45 = [1 0 0 0; 0 cosd(90) -sind(90) 0; 0 sind(90) cosd(90) 0;  0 0 0 1] * [cos(Q5) -sin(Q5) 0 0; sin(Q5) cos(Q5) 0 0; 0 0 1 l4; 0 0 0 1];
 
T5e = [1 0 0 0;
     0 1 0 0;
     0 0 1 l5;
     0 0 0 1];
 
 T0e = simplify(T01*T12*T23*T34*T45*T5e);
 
 displacement =  T0e(1:3,4);
 
 for i = 1:503
     ans = subs(displacement,[Q1,Q2,Q3,Q4,Q5],[Q1i(i),Q2i(i),Q3i(i),Q4i(i),Q5i(i)]);
     x(i) = ans(1);
     y(i) = -ans(2);
     z(i) = ans(3);
 end
 title('Comparison between the Actual Trajectory and Planned Trajectory ')
 xlabel('x-axis(cm)')
 ylabel('z-axis(cm)')
 legend('Planned Trajectory','Actual Trajectory')
 plot(x,z)
 
 hold off
 
 figure;
 tspan = 0.01:0.01:5.03;
 
 hold on
 plot(tspan,record1(1,:))
 plot(tspan,x)
 title('x value vs time')
 xlabel('time(s)')
 ylabel('x value(cm)')
 legend('Planned Trajectory','Actual Trajectory')
 
  figure;

 hold on
 plot(tspan,record1(2,:))
  plot(tspan,y)
 title('y value vs time')
 xlabel('time(s)')
 ylabel('y value(cm)')
 legend('Planned Trajectory','Actual Trajectory')
 
  figure;

 hold on
 plot(tspan,record1(3,:))
  plot(tspan,z)
 title('z value vs time')
 xlabel('time(s)')
 ylabel('z value(cm)')
 legend('Planned Trajectory','Actual Trajectory')
 