function coefficientList = traj_gen(startPoint, endPoint)
    [viaPoint1,viaPoint2] = gen_viapoint(startPoint,endPoint);
    pointList = [startPoint,viaPoint1,viaPoint2,endPoint];
    vInit = [[0;0;0],[0;0;15.76], [5.5;0;0]];
    vEnd = [[0;0;15.76],[5.5;0;0],[0;0;-15.76]];
    tStart = [0 1 4];
    tEnd = [1 4 5];
    coefficientList = [];
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
        for t = ts:0.01:te
            for j = 1:3
                [xpos,ypos,zpos] = cal_position(temp(:,j),t);
            end
        end
    end
end