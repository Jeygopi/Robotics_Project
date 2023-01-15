%% Section 0 

startPoint = [0;0;0];
% endPoint = [45.72;-4.6;1];
viapoint = [6;5;-10];
endPoint = [6;5;-10]

pointList = [startPoint,viapoint,endPoint];
vInit = [[0;0;0],[0;0;0]];
vEnd = [[0;0;0],[0;0;0]];
tStart = [0 2];
tEnd = [2 30];

coefficientList = [];
p = 0
q = 0
start = 1
count1 = 0
tex = 0;
tey = 0;
tez = 0;
count = 0
xtr = [];
ytr = [];
ztr = [];
xve = [];
yve = [];
zve = [];

for i = 1:2
    sp = pointList(:,i);
    ep = pointList(:,i+1);
    vi = vInit(:,i);
    ve = vEnd(:,i);
    ts = tStart(i);
    te = tEnd(i);
    temp = [];
    temp2 = [];
    [fbb,e] = readFB(s,6);
            
    
    %[Q1r,Q2r,Q3r,Q4r,Q5r,Q6r] = [fba[3],fba[4],fba[1],fba[2],fba[6],fba[5]];
    tic;
    Q1old = fbb(3);
    Q2old = fbb(4);
    Q3old = fbb(1);
    Q4old = fbb(2);
    Q5old = fbb(6);
    Q6old = fbb(5);
    
    oldxvele = 0;
    oldyvele = 0;
    oldzvele = 0;
    
    oldxvel = 0;
    oldyvel = 0;
    oldzvel = 0;
        
    for j = 1:3
        temp(:,j) = gen_calculate(ts,te,ep(j,:),ve(j,:),sp(j,:),vi(j,:)); %calculate co-efficients for polynomial
    end
    temp;
    
    for t = ts:0.01:te
%         [fbb,e] = readFB(s,6);
        count = count + 1;
            %[Q1r,Q2r,Q3r,Q4r,Q5r,Q6r] = [fba[3],fba[4],fba[1],fba[2],fba[6],fba[5]];
%         Q1r = fbb(3);
%         Q2r = fbb(4);
%         Q3r = fbb(1);
%         Q4r = fbb(2);
%         Q5r = fbb(6);
%         Q6r = fbb(5);
        
        
        for j = 1:3
                position(j) = cal_position(temp(:,j),t);
                velocity(j) = cal_velocity(temp(:,j),t);
        end
            
         hello = toc;
         
         xtr(count) = position(1);
         ytr(count) = position(2);
         ztr(count) = position(3);
         
         xve(count) = velocity(1);
         yve(count) = velocity(2);
         zve(count) = velocity(3);
         
         %[Q1,Q2,Q3,Q4,Q5] = invkinematics(position(1),position(2),position(3),1)
         
         %[Q1dot, Q2dot, Q3dot, Q4dot, Q5dot] = invJacobian([velocity(1),velocity(2),velocity(3),0,0,0],Q1r,Q2r,Q3r,Q4r,Q5r);
         
         % Inverse kinematics for position controller 
         
         %[Q1ref,Q2ref,Q3ref,Q4ref,Q5ref] = invkinematics(position(1),position(2),position(3),1);
         
         %Q1err = Q1ref - Q1r;
         %Q2err = Q1ref - Q2r;
         %Q3err = Q1ref - Q3r;
         %Q4err = Q4ref - Q4r;
         %Q5err = Q5ref - Q5r;
         
         %sendJointVel(s,[0.2(Q3err/dt), 0.2(Q4err/dt), 0.2(Q1err/dt), 0.2(Q2err/dt), 0,0.2(Q5err/dt)],numID)
         
         tic;
         
         %This bit needs to be written in task space
         
         %jvQ1 = Q1r-Q1old;
         %jvQ2 = Q2r-Q2old;
         %jvQ3 = Q3r-Q3old;
         %jvQ4 = Q4r-Q4old;
         %jvQ5 = Q5r-Q5old;
         
         %xvector = jacobian(Q1r,Q2r,Q3r,Q4r,Q5r,jvQ1,jvQ2,jvQ3,jvQ4,jvQ5)
         
         %xvele = xvector(1) - velocity(1);
         %yvele = xvector(2) - velocity(2);
         %zvele = xvector(3) - velocity(3);
         
%          xveled = xvele - oldxvele;
%          yveled = yvele - oldyvele;
%          zveled = zvele - oldzvele;
%          
%          
%          oldxvele = xvele;
%          oldyvele = yvele;
%          oldzvele = zvele;
%          
%          tex = tex + xvele;
%          tey = tey + yvele;
%          tez = tez + zvele;
%          
%          xvel = velocity(1) + 6*xvele + 0.01*tex + 1*xveled;
%          yvel = velocity(2) + 6*yvele + 0.01*tey + 1*yveled;
%          zvel = velocity(3) + 13*zvele + 0.1*tez + 10*zveled;
%          
%          xvelf = 0.9*oldxvel + 0.1*xvel;
%          yvelf = 0.9*oldyvel + 0.1*yvel;
%          zvelf = 0.9*oldzvel + 0.1*zvel;
%          
%          oldxvel = xvel;
%          oldyvel = yvel;
%          oldzvel = zvel;
%          
%          
%          te(1:3,count) = [xvele,yvele,zvele];
%          tev(1:3,count) = [xvelf,yvelf,zvelf];
%          
%          [Q1dot, Q2dot, Q3dot, Q4dot, Q5dot] = invJacobian([xvelf,yvelf,zvelf,0,0,0],Q1r,Q2r,Q3r,Q4r,Q5r);
%          
%          
% %          Q1error = (Q1r - Q1old) - Q1dot;
% %          Q2error = (Q2r - Q2old) - Q2dot;
% %          Q3error = (Q3r - Q3old) - Q3dot;
% %          Q4error = (Q4r - Q4old) - Q4dot;
% %          Q5error = (Q5r - Q5old) - Q5dot;
% %          
% %          teQ1 = teQ1 + Q1error;
% %          teQ2 = teQ2 + Q2error;
% %          teQ3 = teQ3 + Q3error;
% %          teQ4 = teQ4 + Q4error;
% %          teQ5 = teQ5 + Q5error;
% %          
% %          Q1dot = Q1dot + 1*Q1error + 0.15*Q1error;
%          
%          %sendJointVel(s,[Q3dot, Q4dot, Q1dot, Q2dot, 0,Q5dot],numID)
%          
%          Q1old = Q1r;
%          Q2old = Q2r;
%          Q3old = Q3r;
%          Q4old = Q4r;
%          Q5old = Q5r;
         
         
         
    end
end




%% Section 1
txe = 0;
tye = 0;
tze = 0;

oldvec1 = 0;
oldvec2 = 0;
oldvec3 = 0;

tic;
[fbb,e] = readFB(s,6);
    tic;
    Q1old = fbb(3);
    Q2old = fbb(4);
    Q3old = fbb(1);
    Q4old = fbb(2);
    Q5old = fbb(6);
    Q6old = fbb(5);
    
    
tic;    
[fba,e] = readFB(s,6);

    Q1old2 = fba(3);
    Q2old2 = fba(4);
    Q3old2 = fba(1);
    Q4old2 = fba(2);
    Q5old2 = fba(6);
    Q6old2 = fba(5);
dt2 = toc; 
    Q1dotold = (Q1old - Q1old2)/dt2;
    Q2dotold = (Q2old - Q2old2)/dt2;
    Q3dotold = (Q3old - Q3old2)/dt2;
    Q4dotold = (Q4old - Q4old2)/dt2;
    Q5dotold = (Q5old - Q5old2)/dt2;
    
tic;

olderQ1pos = 0;
olderQ2pos = 0;
olderQ3pos = 0;
olderQ4pos = 0;
olderQ5pos = 0;

xerr = zeros(700);
yerr = zeros(700);
zerr = zeros(700);

Q1a = zeros(700);
Q2a = zeros(700);
Q3a = zeros(700);
Q4a = zeros(700);
Q5a = zeros(700);
for i = 1:3000
    
    
    [fbb,e] = readFB(s,6);
    Q1r = fbb(3);
    Q2r = fbb(4);
    Q3r = fbb(1);
    Q4r = fbb(2);
    Q5r = fbb(6);
    Q6r = fbb(5);
    
    [x(i),y(i),z(i)] = fwdkinematics(Q1r,Q2r,Q3r,Q4r,Q5r)
    
    
    
    [Q1,Q2,Q3,Q4,Q5] = invkinematics2(xtr(i),ytr(i),ztr(i),1);
    
    xerr(i) = x(i) - xtr(i);
    yerr(i) = y(i) - ytr(i);
    zerr(i) = z(i) - ztr(i);
    
    Q1a(i) = Q1;
    Q2a(i) = Q2;
    Q3a(i) = Q3;
    Q4a(i) = Q4;
    Q5a(i) = Q5;
    
    erQ1pos = Q1 - Q1r;
    erQ2pos = Q2 - Q2r;
    erQ3pos = Q3 - Q3r;
    erQ4pos = Q4 - Q4r;
    erQ5pos = Q5 - Q5r;
    
    %sendJointVel(s,[3.5*erQ3pos, 0.7*erQ4pos,0.7*erQ1pos, 1*erQ2pos, 0,0],numID)
    
    olderQ1pos = erQ1pos;
    olderQ2pos = erQ2pos;
    olderQ3pos = erQ3pos;
    olderQ4pos = erQ4pos;
    olderQ5pos = erQ5pos;
    
    
    % These are all fine
%     Q1rs(i) = Q1r;
%     Q2rs(i) = Q2r;
%     Q3rs(i) = Q3r;
%     Q4rs(i) = Q4r;
%     Q5rs(i) = Q5r;

    [fbc,e] = readFB(s,6);
    Q1rr = fbc(3);
    Q2rr = fbc(4);
    Q3rr = fbc(1);
    Q4rr = fbc(2);
    Q5rr = fbc(6);
    Q6rr = fbc(5);
    
    dt = toc;
    
    Q1dot = (Q1rr - Q1old);
    Q2dot = (Q2rr - Q2old);
    Q3dot = (Q3rr - Q3old);
    Q4dot = (Q4rr - Q4old);
    Q5dot = (Q5rr - Q5old);
%     
%     Q1ddd(i) = Q1dot;
%     Q2ddd(i) = Q2dot;
%     Q3ddd(i) = Q3dot;
%     Q4ddd(i) = Q4dot;
%     Q5ddd(i) = Q5dot;
%     
    tic;
    
%     if (abs(Q1dot - Q1dotold) > 0.05)
%         Q1dot = 0.9*Q1dotold + 0.1*Q1dot;
%     end
%     
%     if (abs(Q2dot - Q2dotold) > 0.05)
%         Q2dot = 0.9*Q2dotold + 0.1*Q2dot;
%     end
%     
%     if (abs(Q3dot - Q3dotold) > 0.05)
%         Q3dot = 0.9*Q3dotold + 0.1*Q3dot;
%     end
%     
%     if (abs(Q4dot - Q4dotold) > 0.05)
%         Q4dot = 0.9*Q4dotold + 0.1*Q4dot;
%     end
%     
%     if (abs(Q5dot - Q5dotold) > 0.05)
%         Q5dot = 0.9*Q5dotold + 0.1*Q5dot;
%     end
    
    Q1ds(i) = Q1dot;
    Q2ds(i) = Q2dot;
    Q3ds(i) = Q3dot;
    Q4ds(i) = Q4dot;
    Q5ds(i) = Q5dot;
    
    vector = jacobian(Q1rr,Q2rr,Q3rr,Q4rr,Q5rr,Q1dot,Q2dot,Q3dot,Q4dot,Q5dot);
    
%     if (abs(vector(1) - oldvec1) > 5)
%         vector(1) = 0.9*oldvec1 + 0.1*vector(1);
%     end
%     if (abs(vector(2) - oldvec2) > 5)
%         vector(2) = 0.9*oldvec2 + 0.1*vector(2);
%     end
%     if (abs(vector(3) - oldvec3) > 5)
%         vector(3) = 0.9*oldvec3 + 0.1*vector(3);
%     end
        
    oldvec1 = vector(1);
    oldvec2 = vector(2);
    oldvec3 = vector(3);
    
    xv(i) = vector(1);
    yv(i) = vector(2);
    zv(i) = vector(3);
    
    
    
    % keep reference trajectories at 0
    %-10 z axis
    txe = txe + (xve(i) - xv(i));
    tye = tye + (yve(i)-yv(i));
    tze = tze + (zve(i) - zv(i)); 
    
    velerrx(i) = (xve(i) - xv(i));
    velerry(i) = (yve(i) - yv(i));
    velerrz(i) = (zve(i) - zv(i));
    
    
    
    
    %dxvel = xve(i) + 1.6*(xve(i) - xv(i)); %xv(i) + 1.4*(xve(i)-xv(i));
    %dyvel = yve(i) + 2*(yve(i)-yv(i)); %+ 0.01*tye; %+ 1*dt*tye;
    % here vector(3) - -(10) results in upwards movement of Q1
    %dzvel = zve(i) + 3.1*(zve(i)-zv(i)); %+ 0.01*tze; %+ 1*dt*tze;
    if (abs((xve(i) - xv(i)))>1.5)
        dxvel = xve(i) + 1.3*(xve(i) - xv(i)) + 0.02*txe;
    else
        dxvel = xve(i);
    end
        
    if (abs((yve(i)-yv(i))) > 1.5)
        dyvel = yve(i) + 1.2*(yve(i)-yv(i))+ 0.02*tye;
    else
        dyvel = yve(i);
    end
    
    if (abs((zve(i)-zv(i))) > 1.5)
        dzvel = zve(i) + 1.3*(zve(i)-zv(i)) + 0.014*tze;
    else
        dzvel = zve(i);
    end
   
    
    dxv(i) = dxvel; 
    dyv(i) = dyvel;
    dzv(i) = dzvel;
    
     if (i > 1)
        if (abs(dyv(i-1) - dyv(i)) > 1)
            dyvel = 0.99*dyv(i-1) + 0.01*dyv(i);
        end
    end
    
    if (i > 1)
        if (abs(dzv(i-1) - dzv(i)) > 1)
            dzvel = 0.99*dzv(i-1) + 0.01*dzv(i);
        end
    end
    
    if (i > 1)
        if (abs(dxv(i-1) - dxv(i)) > 1)
            dxvel = 0.99*dxv(i-1) + 0.01*dxv(i);
        end
    end
    
    
    [Q1dot, Q2dot, Q3dot, Q4dot, Q5dot] = invJacobian([dxvel,dyvel,dzvel,0,0,0],Q1r,Q2r,Q3r,Q4r,Q5r);
    
    Q1dd(i) = Q1dot;
    Q2dd(i) = Q2dot;
    Q3dd(i) = Q3dot;
    Q4dd(i) = Q4dot;
    Q5dd(i) = Q5dot;
    
    Q1old = Q1rr;
    Q2old = Q2rr;
    Q3old = Q3rr;
    Q4old = Q4rr;
    Q5old = Q5rr;
    
%     if (abs(Q1dot - Q1dotold) > 0.7)
%         Q1dot = 1*Q1dotold;
%     end
%     
%     if (abs(Q2dot - Q2dotold) > 1)
%         Q2dot = 1*Q2dotold;
%     end
%     
%     if (abs(Q3dot - Q3dotold) > 0.5)
%         Q3dot = 1*Q3dotold;
%     end
%     
%     if (abs(Q4dot - Q4dotold) > 0.5)
%         Q4dot = 1*Q4dotold;
%     end
%     
%     if (abs(Q5dot - Q5dotold) > 0.5)
%         Q5dot = 1*Q5dotold;
%     end
%     
    
    Q1dot = (Q1dotold + Q1dot)/2;
    Q2dot = (Q2dotold + Q2dot)/2;
    Q3dot = (Q3dotold + Q3dot)/2;
    Q4dot = (Q4dotold + Q4dot)/2;
    Q5dot = (Q5dotold + Q5dot)/2;
    
    Q1dotold = Q1dot;
    Q2dotold = Q2dot;
    Q3dotold = Q3dot;
    Q4dotold = Q4dot;
    Q5dotold = Q5dot;
    
    Q1dd(i) = Q1dot;
    Q2dd(i) = Q2dot;
    Q3dd(i) = Q3dot;
    Q4dd(i) = Q4dot;
    Q5dd(i) = Q5dot;
    
    [fbe,e] = readFB(s,6);
    Q1g = fbe(3);
    Q2g = fbe(4);
    Q3g = fbe(1);
    Q4g = fbe(2);
    Q5g = fbe(6);
    Q6g = fbe(5);
    
    % Joint space control here%
    sendJointVel(s,[Q3dot, Q4dot, Q1dot, Q2dot, 0,pi/2],numID);
    tic;
    for k = 1:100
    end
    
    [fbd,e] = readFB(s,6);
    tic;
    Q1f = fbd(3);
    Q2f = fbd(4);
    Q3f = fbd(1);
    Q4f = fbd(2);
    Q5f = fbd(6);
    Q6f = fbd(5);
    newdt = toc;
    
    sendJointVel(s,[(Q3dot + 0.5*((Q3f - Q3g) - (Q3g + newdt*Q3dot))),(Q4dot + 0.5*((Q4f - Q4g) - (Q4g + newdt*Q4dot))),(Q1dot + 0.5*((Q1f - Q1g) - (Q1g + newdt*Q1dot))),(Q2dot + 0.5*((Q2f - Q2g) - (Q2g + newdt*Q2dot))),0,0],numID);
    
    
end

sendJointVel(s,[0, 0, 0, 0, 0,0],numID)

%% test robot

for i = 1:10
    
    sendJointVel(s,[-0.5, 0, 0, 0, 0,0],numID)
    
end

sendJointVel(s,[0, 0, 0, 0, 0,0],numID)
         
    
    