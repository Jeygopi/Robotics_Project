%% Hacky way to do the task 2 
tic;
for i = 1:300
    [fbb,e] = readFB(s,6);
    Q1r = fbb(3);
    Q2r = fbb(4);
    Q3r = fbb(1);
    Q4r = fbb(2);
    Q5r = fbb(6);
    Q6r = fbb(5);
    
    [xJoy, yJoy, eJoy] = readJoy(s);
    xJoy
    yJoy
    dt = toc;
    if i > 1    
        Q1d = Q1r - Q1ro;
        Q2d = Q2r - Q2ro;
        Q3d = Q3r - Q3ro;
        Q4d = Q4r - Q4ro;
        Q5d = Q5r - Q5ro;
        
        xrefv = xJoy - xJoyo;
        yrefv = yJoy - yJoyo;
        zrefv = 5;
        
        [Q1,Q2,Q3,Q4,Q5] = invkinematics(xrefv*dt,yrefv*dt,zrefv,1)
        
        
        sendJointPos(s,[Q3,Q4,Q1,Q2,Q6,Q5] , numID);
        
    end
    
    tic;
    
    
    
    Q1ro = Q1r;
    Q2ro = Q2r;
    Q3ro = Q3r;
    Q4ro = Q4r;
    Q5ro = Q5r;
    Q6ro = Q6r;
    
    xJoyo = xJoy;
    yJoyo = yJoy;
end
    