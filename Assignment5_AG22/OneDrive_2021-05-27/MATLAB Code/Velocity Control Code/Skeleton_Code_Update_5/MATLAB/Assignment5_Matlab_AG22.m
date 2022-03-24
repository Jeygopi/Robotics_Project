%% Position control

% Set motor EPROM to velocity control mode
%setControlMode(s, "position");

%% Position control code

Q1 = 0;
Q2 = 50;
Q3 = -45;
Q4 = 0;
Q5 = 90;

[Q1s,Q2s,Q3s,Q4s,Q5s] = jointspacetransform(deg2rad(Q1),deg2rad(Q2),deg2rad(Q3),deg2rad(Q4),deg2rad(Q5));

sendJointPos(s,[Q3s, Q4s, Q1s, Q2s, pi/4, Q5s],numID);

%% Posn control 

x = 43;
y = 1;
z = 27;


[Q1p,Q2p,Q3p,Q4p,Q5p] = invkinematics2(x,y,z,-1)

[Q1p,Q2p,Q3p,Q4p,Q5p] = jointspaceinvtransform(Q1p,Q2p,Q3p,Q4p,Q5p);

sendJointPos(s,[Q3p, Q4p, Q1p, Q2p, pi/4, Q5p],numID);
i = 1;
[fbb,e] = readFB(s,6);
Q1pp(i) = fbb(3);
Q2pp(i) = fbb(4);
Q3pp(i) = fbb(1);
Q4pp(i) = fbb(2);
Q5pp(i) = fbb(6);

[Q1pp(i),Q2pp(i),Q3pp(i),Q4pp(i),Q5pp(i)] = jointspaceinvtransform(Q1pp(i),Q2pp(i),Q3pp(i),Q4pp(i),Q5pp(i));

[x,y,z] = fwdkinematics(Q1pp(i),Q2pp(i),Q3pp(i),Q4pp(i),Q5pp(i));

for i = 1:500
    % Set Motor internal PID controller. Gains must be integers.
    [fbb,e] = readFB(s,6);
    Q1pp(i+1) = fbb(3);
    Q2pp(i+1) = fbb(4);
    Q3pp(i+1) = fbb(1);
    Q4pp(i+1) = fbb(2);
    Q5pp(i+1) = fbb(6);
    [Q1pp(i+1),Q2pp(i+1),Q3pp(i+1),Q4pp(i+1),Q5pp(i+1)] = jointspaceinvtransform(Q1pp(i+1),Q2pp(i+1),Q3pp(i+1),Q4pp(i+1),Q5pp(i+1));
    
    [x(i),y(i),z(i)] = fwdkinematics(Q1pp(i+1),Q2pp(i+1),Q3pp(i+1),Q4pp(i+1),Q5pp(i+1));
    
    if i > 2
        xd = x(i) - x(i-1);
        yd = y(i) - y(i-1);
        zd = z(i) - z(i-1);
        
        if (xd > 1)
        end
    end
end
    
    tmpID = 1;
Kp = 5;
Ki = 0;
Kd = 1;
setPID(s, tmpID, Kp, Ki, Kd)



%% Code test
[fbb,e] = readFB(s,6);
      

for i = 1:10000  
    %[Q1r,Q2r,Q3r,Q4r,Q5r,Q6r] = [fba[3],fba[4],fba[1],fba[2],fba[6],fba[5]];
    [fbb,e] = readFB(s,6);
    Q1r(i) = fbb(3);
    Q1r(i)
    Q2r(i) = fbb(4);
    Q3r(i) = fbb(1);
    Q4r(i) = fbb(2);
    Q5r(i) = fbb(6);
    Q6r(i) = fbb(5);
    
    [Q1r(i),Q2r(i),Q3r(i),Q4r(i),Q5r(i)] =jointspacetransform(Q1r(i),Q2r(i),Q3r(i),Q4r(i),Q5r(i));
    
end

%% Velocity control

% Set motor EPROM to velocity control mode
setControlMode(s, "velocity");

%% Velocity Commands Trajectory generation

startPoint = [2;2;1];
% endPoint = [45.72;-4.6;1];
viapoint = [0;0;15];
endPoint = [0;0;15];

pointList = [startPoint,viapoint,endPoint];
vInit = [[0;0;0],[0;0;0]];
vEnd = [[0;0;0],[0;0;0]];
tStart = [0 1];
tEnd = [1 50];
coefficientList = [];
p = 0
q = 0
start = 1
count1 = 0
tex = 0;
tey = 0;
tez = 0;
for i = 1:2
    sp = pointList(:,i);
    ep = pointList(:,i+1);
    vi = vInit(:,i);
    ve = vEnd(:,i);
    ts = tStart(i);
    te = tEnd(i);
    temp = [];
    temp2 = [];
%     [fbb,e] = readFB(s,6);
%             
%     
%     %[Q1r,Q2r,Q3r,Q4r,Q5r,Q6r] = [fba[3],fba[4],fba[1],fba[2],fba[6],fba[5]];
%     tic;
%     Q1old = fbb(3);
%     Q2old = fbb(4);
%     Q3old = fbb(1);
%     Q4old = fbb(2);
%     Q5old = fbb(6);
%     Q6old = fbb(5);
    
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
    count = 0
    for t = ts:0.01:te
        count1 = count1 + 1;
        for j = 1:3
                position(j) = cal_position(temp(:,j),t);
                velocity(j) = cal_velocity(temp(:,j),t);
                p(j,count1) = position(j);
                v(j,count1) = velocity(j);
        end
    end
end


[fbb,e] = readFB(s,6);
    Q1ro(i) = fbb(3);
    Q1ro(i)
    Q2ro(i) = fbb(4);
    Q3ro(i) = fbb(1);
    Q4ro(i) = fbb(2);
    Q5ro(i) = fbb(6);
    
    
%% Calculating joint velocities

for i = 1:length(v)
    pvec = [p(1,i),p(2,i),p(3,i),0,0,0];
    [Q1p(i),Q2p(i),Q3p(i),Q4p(i),Q5p(i)]=invkinematics2(pvec(1),pvec(2),pvec(3),-1);
    inputvec = [v(1,i),v(2,i),v(3,i),0,0,0];
    [Q1dot(i), Q2dot(i), Q3dot(i), Q4dot(i), Q5dot(i)] = invJacobian(inputvec',Q1p(i),Q2p(i),Q3p(i),Q4p(i),Q5p(i));
    
end


%% Sending Velcoities 
%This code is run to send joint velocities
tic;
count2 = 0;
Q1r = zeros(1,length(v));
Q2r = zeros(1,length(v));
Q3r = zeros(1,length(v));
Q4r = zeros(1,length(v));
Q5r = zeros(1,length(v));

for i = 1:length(p(1,:))
    
    [Q11(i),Q22(i),Q33(i),Q44(i),Q55(i)] = invkinematics2(p(1,i),p(2,i),p(3,i),-1);
    
    [Q1(i),Q2(i),Q3(i),Q4(i),Q5(i)] = jointspacetransform(Q11(i),Q22(i),Q33(i),Q44(i),Q55(i));
    
end

tex = 0;
tey = 0;
tez = 0;
teox = 0;
teoy = 0;
teoz = 0;

dt = 0.03;
for i = 1:length(v(1,:))
    
    [Q111(i),Q222(i),Q333(i),Q444(i),Q555(i)] = invkinematics2(p(1,i),p(2,i),p(3,i),-1);
    
    [Q111(i),Q222(i),Q333(i),Q444(i),Q555(i)] = jointspacetransform(Q111(i),Q222(i),Q333(i),Q444(i),Q555(i));
    
    if i > 2
        Q1vel(i-1) = (Q111(i) - Q111(i-1))/dt;
        Q2vel(i-1) = (Q222(i) - Q222(i-1))/dt;
        Q3vel(i-1) = (Q333(i) - Q333(i-1))/dt;
        Q4vel(i-1) = (Q444(i) - Q444(i-1))/dt;
        Q5vel(i-1) = (Q555(i) - Q555(i-1))/dt;
        
        pp(i-1,1:5) = inverseJacobian(v(1,i),v(2,i),v(3,i),0,0,0,Q1vel(i-1),Q2vel(i-1),Q3vel(i-1),Q4vel(i-1),Q5vel(i-1));

    end
     
end


for i = 1:length(v)
    tic;
    count2 = count2 + 1;
    
    [fbb,e] = readFB(s,6);
    Q1r(i) = fbb(3);
    Q2r(i) = fbb(4);
    Q3r(i) = fbb(1);
    Q4r(i) = fbb(2);
    Q5r(i) = fbb(6);
    
    if i > 2
        if (abs(Q1r(i) - Q1r(i-1)) > 1)
            Q1r(i) = Q1r(i-1);
        end
        if (abs(Q2r(i) - Q2r(i-1)) > 1)
            Q2r(i) = Q2r(i-1);
        end
        if (abs(Q3r(i) - Q3r(i-1)) > 1)
            Q3r(i) = Q3r(i-1);
        end
        if (abs(Q4r(i) - Q4r(i-1)) > 1)
            Q4r(i) = Q4r(i-1);
        end
        if (abs(Q5r(i) - Q5r(i-1)) > 1)
            Q5r(i) = Q5r(i-1);
        end
    end
    
%     [Q111(i),Q222(i),Q333(i),Q444(i),Q555(i)] = invkinematics2(p(1,i),p(2,i),p(3,i),-1);
%     
%     if i > 1
%         Q1vel(i-1) = (Q111(i) - Q111(i-1))/dt;
%         Q2vel(i-1) = (Q222(i) - Q222(i-1))/dt;
%         Q3vel(i-1) = (Q333(i) - Q333(i-1))/dt;
%         Q4vel(i-1) = (Q444(i) - Q444(i-1))/dt;
%         Q5vel(i-1) = (Q555(i) - Q555(i-1))/dt;
%         
%         pp(i-1,1:5) = inverseJacobian(v(1,i),v(2,i),v(3,i),0,0,0,Q1vel(i-1),Q2vel(i-1),Q3vel(i-1),Q4vel(i-1),Q5vel(i-1));
% 
%     end
    
    % Find the required joint angles here using inverse kinematics
    
    pvec = [p(1,i),p(2,i),p(3,i),0,0,0];
    
    %[Q1p(i),Q2p(i),Q3p(i),Q4p(i),Q5p(i)]=invkinematics2(pvec(1),pvec(2),pvec(3),-1);
    
    %Don't need to transform to joint co-ords as it's already there
    
    [Q1t(i),Q2t(i),Q3t(i),Q4t(i),Q5t(i)] = jointspaceinvtransform(Q1r(i),Q2r(i),Q3r(i),Q4r(i),Q5r(i));
    
%     Q1t(i) = Q1r(i);
%     Q2t(i) = Q2r(i);
%     Q3t(i) = Q3r(i);
%     Q4t(i) = Q4r(i);
%     Q5t(i) = Q5r(i);
    
    
    %[x(i),y(i),z(i)] = fwdkinematics(Q1t,Q2t,Q3t,Q4t,Q5t);
    
    Qd1s = 0;
    Qd2s = 0;
    Qd3s = 0;
    Qd4s = 0;
    Qd5s = 0;
    
    
    if i > 1
        
        Q1dot(i-1) = (Q1t(i) - Q1t(i-1))/dt;
        Q2dot(i-1) = (Q2t(i) - Q2t(i-1))/dt;
        Q3dot(i-1) = (Q3t(i) - Q3t(i-1))/dt;
        Q4dot(i-1) = (Q4t(i) - Q4t(i-1))/dt;
        Q5dot(i-1) = (Q5t(i) - Q5t(i-1))/dt;
        if i > 3
            Q1davg(i-3) = (Q1dot(i-3) + Q1dot(i-2) + Q1dot(i-1))/3;
            Q2davg(i-3) = (Q2dot(i-3) + Q2dot(i-2) + Q2dot(i-1))/3;
            Q3davg(i-3) = (Q3dot(i-3) + Q3dot(i-2) + Q3dot(i-1))/3;
            Q4davg(i-3) = (Q4dot(i-3) + Q4dot(i-2) + Q4dot(i-1))/3;
            Q5davg(i-3) = (Q5dot(i-3) + Q5dot(i-2) + Q5dot(i-1))/3;
        
            %Jacobian is used here to transform the joint space velocities
            %to task space velocities
            vector = jacobian(Q1t(i-3),Q2t(i-3),Q3t(i-3),Q4t(i-3),Q5t(i-3),Q1davg(i-3),Q2davg(i-3),Q3davg(i-3),Q4davg(i-3),Q5davg(i-3));
            xact(i-3) = vector(1);
            yact(i-3) = vector(2);
            zact(i-3) = vector(3);
            xori(i-3) = vector(4);
            yori(i-3) = vector(5);
            zori(i-3) = vector(6);

            
            xerr(i-3) = v(1,i-3) - vector(1);
            yerr(i-3) = v(2,i-3) - vector(2);
            zerr(i-3) = v(3,i-3) - vector(3);
            
            
            xorierr(i-3) = 0 - vector(4);
            yorierr(i-3) = 0 - vector(5);
            zorierr(i-3) = 0 - vector(6);
            
            tex = tex + xerr(i-3);
            tey = tey + yerr(i-3);
            tez = tez + zerr(i-3);
            teox = teox + xorierr(i-3);
            teoy = teoy + yorierr(i-3);
            teoz = teoz + zorierr(i-3);
            
            %Rotation for bishop 
            %xs(i-3) = cos(0.46)*v(1,i-3) - sin(0.46)*v(2,i-3);
            %ys(i-3) = sin(0.46)*v(1,i-3) + cos(0.46)*v(2,i-3);

            xs(i-3) = v(1,i-3) + 4*xerr(i-3) + 0.05*tex; %1.8, 2.5 gains, 0.05 integral, 0.3 derivative
            ys(i-3) = v(2,i-3) + 0*yerr(i-3) +0.0*tey;
            zs(i-3) = 1*v(3,i-3); + 4.8*zerr(i-3) + 0.05*tez;
            xo(i-3) = 0 + 1*xorierr(i-3); %+ 0.01*teox;
            yo(i-3) = 0 + 1*yorierr(i-3); %+ 0.01*teoy;
            zo(i-3) = 0 + 1*zorierr(i-3); %+ 0.01*teoz;
            
            
            
            if i > 4
               xs(i-3) = v(1,i-3) + 3.8*xerr(i-3) + 0.065*tex + 0.3*(xerr(i-3) - xerr(i-4))/dt;
               ys(i-3) = v(2,i-3) + 0*yerr(i-3) + 0.0*tey +0*(yerr(i-3) - yerr(i-4))/dt; %3.5 0.05 0.3
               zs(i-3) = 1*v(3,i-3) + 4.2*zerr(i-3) + 0.05*tez+ 0.33*(zerr(i-3) - zerr(i-4))/dt;
            end
            
            
            
            % Rotation matrix for bishop
            

            velocityvec = [xs(i-3),ys(i-3),zs(i-3),0,0,0];

            [Q1rr(i-3),Q2rr(i-3),Q3rr(i-3),Q4rr(i-3),Q5rr(i-3)] = invkinematics2(p(1,i-3),p(2,i-3),p(3,i-3),-1);

            %outputs = inverseJacobian(xs(i),ys(i),zs(i),0,0,0,Q1rr(i),Q2rr(i),Q3rr(i),Q4rr(i),Q5rr(i));
            
            %Inverse jacobian is used here to transform the velocities from
            %task space to joint space
            oo(i-3,1:5) = inverseJacobian(xs(i-3),ys(i-3),zs(i-3),xo(i-3),yo(i-3),zo(i-3),Q1rr(i-3),Q2rr(i-3),Q3rr(i-3),Q4t(i-3),Q5rr(i-3));

            Qd1s = -oo(i-3,1);
            Qd2s = -oo(i-3,2);
            Qd3s = -oo(i-3,3);
            Qd4s = -oo(i-3,4);
            Qd5s = oo(i-3,5);
        end
        
        
        
        
        
        
        
        if i > 7
            Qd1s = -(oo(i-4,1) + oo(i-5,1) + oo(i-6,1) + oo(i-7,1))/4;
            Qd2s = -(oo(i-4,2) + oo(i-5,2) + oo(i-6,2) + oo(i-7,2))/4;
            Qd3s = -(oo(i-4,3) + oo(i-5,3) + oo(i-6,3) + oo(i-7,3))/4;
            Qd4s = -(oo(i-4,4) + oo(i-5,4) + oo(i-6,4) + oo(i-7,4))/4;
            Qd5s = (oo(i-4,5) + oo(i-5,5) + oo(i-6,5) + oo(i-7,5))/4;
        end
    end
    
    %Joint Space correction
%     if i > 1
%         Q1veler(i-1) = Q1vel(i-1) - Q1dot(i-1);
%         Q2veler(i-1) = Q2vel(i-1) - Q2dot(i-1);
%         Q3veler(i-1) = Q3vel(i-1) - Q3dot(i-1);
%         Q4veler(i-1) = Q4vel(i-1) - Q4dot(i-1);
%         Q5veler(i-1) = Q5vel(i-1) - Q5dot(i-1);
%         
%         Qd1s = Qd1s + 0.8*Q1veler(i-1);%+0.02*sum(Q1veler);
%         Qd2s = Qd2s + 3*Q2veler(i-1);%+0.02*sum(Q2veler);
%         Qd3s = Qd3s + 0.8*Q3veler(i-1);%+0.02*sum(Q3veler);
%         Qd4s = Qd4s + 0.8*Q4veler(i-1);%+0.02*sum(Q4veler);
%         Qd5s = Qd5s + 0.8*Q5veler(i-1);%+0.02*sum(Q5veler);
%     end
    
    
    
%     [Q1dot(i), Q2dot(i), Q3dot(i), Q4dot(i), Q5dot(i)] = invJacobian(inputvec',Q1p(i),Q2p(i),Q3p(i),Q4p(i),Q5p(i));
%     
%     [Q1dot(i), Q2dot(i), Q3dot(i), Q4dot(i), Q5dot(i)] = jointspaceveltransform(Q1dot(i),Q2dot(i),Q3dot(i),Q4dot(i),Q5dot(i))
%     
%     Q1dots = Q1dot(i);
%     Q2dots = Q2dot(i);
%     Q3dots = Q3dot(i);
%     Q4dots = Q4dot(i);
%     Q5dots = Q5dot(i);
    
    %[Qd1s,Qd2s,Qd3s,Qd4s,Qd5s] = jointspaceveltransform(Qd1s,Qd2s,Qd3s,Qd4s,Qd5s);

    
%         errQ1(i) = Q1(i)-Q1r(i);
%         errQ2(i) = Q2(i)-Q2r(i);
%         errQ3(i) = Q3(i)-Q3r(i);
%         errQ4(i) = Q4(i)-Q4r(i);
%         errQ5(i) = Q5(i)-Q5r(i);
    if i>7
        
%         errQ1d(i) = (errQ1(i) - errQ1(i-1))/dt;
%         errQ2d(i) = (errQ2(i) - errQ2(i-1))/dt;
%         errQ3d(i) = (errQ3(i) - errQ3(i-1))/dt;
%         errQ4d(i) = (errQ4(i) - errQ4(i-1))/dt;
%         errQ5d(i) = (errQ5(i) - errQ5(i-1))/dt;
%         
%         Q1ds(i) = Qd1s + 0.5*errQ1d(i);
%         Q2ds(i) = Qd2s + 0.5*errQ2d(i);
%         Q3ds(i) = Qd3s + 0.5*errQ3d(i);
%         Q4ds(i) = Qd4s + 0.5*errQ4d(i);
%         Q5ds(i) = Qd5s + 0.5*errQ5d(i);
        
        if (Qd2s*dt + Q2r > 0.8 & Qd2s*dt + Q2r < - 1.5)
            Q2ds = 0;;
        end
        
        if (Qd1s*dt + Q1r > 1.5 & Qd1s*dt + Q1r < - 1.5)
            Q1ds = 0;
        end
        
        if (Qd3s*dt + Q3r > 1.3 & Qd3s*dt + Q3r < - 1.5)
            Qd3s = 0;
        end
        
        %errQ4 = 0 - Q4r;
        
        %Qd4s = Qd4s + 0.01*errQ4; 
        
        %Delete this for better performance
        Qd4s = Qd4s + 1*((pp(i,4)) - Qd4s);
        %Qd3s = Qd3s + 1*((pp(i,3)) - Qd3s);
        
        sendJointVel(s, [Qd3s,Qd4s, Qd1s,Qd2s,0,0] , numID);
    
    end    
    
    
    
    dt = toc;

    
end
 sendJointVel(s, [0,0, 0,0,0,0] , numID);
    

%% Filtering values from pseudo inverse

for i = 1:length(v(1,:))
    
    [Q111(i),Q222(i),Q333(i),Q444(i),Q555(i)] = invkinematics2(p(1,i),p(2,i),p(3,i),-1);
    if i > 2
        Q1vel(i-1) = (Q111(i) - Q111(i-1))/dt;
        Q2vel(i-1) = (Q222(i) - Q222(i-1))/dt;
        Q3vel(i-1) = (Q333(i) - Q333(i-1))/dt;
        Q4vel(i-1) = (Q444(i) - Q444(i-1))/dt;
        Q5vel(i-1) = (Q555(i) - Q555(i-1))/dt;
        
        pp(i-1,1:5) = inverseJacobian(v(1,i),v(2,i),v(3,i),0,0,0,Q1vel(i-1),Q2vel(i-1),Q3vel(i-1),Q4vel(i-1),Q5vel(i-1));

    end
     
end
%% test
tic;
[fbb,e] = readFB(s,6);
    Q1rrr = fbb(3);
    Q2rrr = fbb(4);
    Q3rrr = fbb(1);
    Q4rrr = fbb(2);
    Q5rrr = fbb(6);
%% Simulation results  

for i = 1:length(p(1,:))
    
    [Q11(i),Q22(i),Q33(i),Q44(i),Q55(i)] = invkinematics2(p(1,i),p(2,i),p(3,i),-1);
    
    [Q1(i),Q2(i),Q3(i),Q4(i),Q5(i)] = jointspacetransform(Q11(i),Q22(i),Q33(i),Q44(i),Q55(i));
    
end

%% Joint torque test

for i = 1:500
    
    sendJointVel(s, [0,0, 0,-2.4,0,0] , numID);
    
end

sendJointVel(s,[0,0,0,0,0,0],numID);
