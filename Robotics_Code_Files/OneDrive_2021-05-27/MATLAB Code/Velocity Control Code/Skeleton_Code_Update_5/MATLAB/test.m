clear all
clc
close all

startPoint = [20;20;0];
endPoint = [27;20;0];

[viaPoint1,viaPoint2] = gen_viapoint(startPoint,endPoint);
pointList = [startPoint,viaPoint1,viaPoint2,endPoint];
vInit = [[0;0;0],[0;0;0], [2;0;0]];
vEnd = [[0;0;0.76],[2;0;0],[0;0;0.76]];
tStart = [0 1 4];
tEnd = [1 4 5];
coefficientList = [];
p = 0;
record = [];
recordDegree = [];
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
        p = p + 1;
        record(p,:) = position;
        position;
        [Q1,Q2,Q3,Q4,Q5] = invkinematics(position(1),position(2),position(3),1);
        recordDegree(p,:) = [Q1,Q2,Q3,Q4,Q5];
%         pause(0.01)
%         sendJointPos(s,[deg2rad(Q3), deg2rad(Q4), deg2rad(Q1), deg2rad(Q2), deg2rad(Q5)],numID)
    end
end

figure;
plot(record(:,1),record(:,3))