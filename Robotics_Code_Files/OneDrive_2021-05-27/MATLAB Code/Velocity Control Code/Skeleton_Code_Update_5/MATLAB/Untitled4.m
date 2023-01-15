%% % Open the port to which the Arduino is connected and create a serial object.

% @ port                - COM port address of the Arduino. This will change depending on which USB port
%                   the arduino is connected to, and the exact structure of the address will vary between
%                   operating systems

% @ baudrate            - BaudRate used between MATLAB and Arduino, which is limited to a max of 
%                   230400 by MATLAB.

% @ numID               - Number of detected motors.

% @ ID                  - Vector containing ID of each detected motor.

% @ establishSerial()   - A helper function that creates and returns a serial object while also performing
%                   a handshake with the Arduino to confim connection.

clear all
close all 
clc
instrreset

% Specify COM Port
port = 'COM7';                  % MacOS Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS
% port = 'COM1';                                 % Windows Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS

% Setup Serial Connection
baudrate = 230400;
s = establishSerial(port, baudrate); 

% Read Connected Motor IDs
[numID, ID] = getMotorIDs(s)

% Set Change Motor ID flag - CHANGE TO ADJUST MOTOR ID
changeMotorID = false;
%% % Set motor EPROM to position control mode
setControlMode(s, "position");
%% Trying with joint home position:

Q1 = 0;
Q2 = -70;
Q3 = 0;
Q4 = -Q2-Q3-21-90;
Q5 = 0



sendJointPos(s,[deg2rad(Q3), deg2rad(Q4), deg2rad(Q1), deg2rad(Q2),deg2rad(-90), deg2rad(Q5)],numID);


%% Test joint control mode 

[fbb,e] = readFB(s,6);
            
    
    %[Q1r,Q2r,Q3r,Q4r,Q5r,Q6r] = [fba[3],fba[4],fba[1],fba[2],fba[6],fba[5]];
    tic;
    Q1r = fbb(3);
    Q2old = fbb(4);
    Q3r = fbb(1);
    Q4r = fbb(2);
    Q5r = fbb(6);
    Q6r = fbb(5);
        
total_time = 0;
totalerror=0;

for i = 1:100
    
    
    Q3dot = 0;
    Q4dot = 0;
    Q1dot = 0;
    Q5dot = 0;
    
    [fbb,e] = readFB(s,6);
    Q2r = fbb(4)
    Q2rs(i) = Q2r
    hello = toc;
    Q2e = (Q2r - Q2old) - pi/18;
    totalerror = totalerror + Q2e;
    
    total_time = total_time + hello;
    tic;
    Qestored(i) = Q2e;
    
    Q2dot = -pi/18 + 1*Q2e + 0.1*totalerror;
  
    Q2stored(i) = Q2dot;
    sendJointVel(s,[Q3dot, Q4dot, Q1dot, Q2dot, 0,Q5dot],numID)
    
    Q2old = Q2r
    
    actualerror(i) = (Q2dot) - (Q2r);
    
end

sendJointVel(s,[0,0,0,0,0,0],numID)
%% % Set motor EPROM to velocity control mode
setControlMode(s, "velocity");
%% pointList = [startPoint,viaPoint1,viaPoint2,endPoint];

startPoint = [17;19;-10];
% endPoint = [45.72;-4.6;1];
viapoint = [27;23;-18];
endPoint = [27;23;-18]


pointList = [startPoint,viapoint,endPoint];
vInit = [[0;0;0],[0;0;1]];
vEnd = [[0;0;1],[1;0;0]];
tStart = [0 5];
tEnd = [5 1000];
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
    count = 0
    for t = ts:0.01:te
        [fbb,e] = readFB(s,6);
        count = count + 1;
            %[Q1r,Q2r,Q3r,Q4r,Q5r,Q6r] = [fba[3],fba[4],fba[1],fba[2],fba[6],fba[5]];
        Q1r = fbb(3);
        Q2r = fbb(4);
        Q3r = fbb(1);
        Q4r = fbb(2);
        Q5r = fbb(6);
        Q6r = fbb(5);
        
        
        for j = 1:3
                position(j) = cal_position(temp(:,j),t);
                velocity(j) = cal_velocity(temp(:,j),t);
        end
            
         hello = toc;
         
         %[Q1,Q2,Q3,Q4,Q5] = invkinematics(position(1),position(2),position(3),1)
         
         %[Q1dot, Q2dot, Q3dot, Q4dot, Q5dot] = invJacobian([velocity(1),velocity(2),velocity(3),0,0,0],Q1r,Q2r,Q3r,Q4r,Q5r);
         
         % Inverse kinematics for position controller 
         
         [Q1ref,Q2ref,Q3ref,Q4ref,Q5ref] = invkinematics(position(1),position(2),position(3),1)
         
         Q1err = Q1ref - Q1r;
         Q2err = Q1ref - Q2r;
         Q3err = Q1ref - Q3r;
         Q4err = Q4ref - Q4r;
         Q5err = Q5ref - Q5r;
         
         %sendJointVel(s,[0.2(Q3err/dt), 0.2(Q4err/dt), 0.2(Q1err/dt), 0.2(Q2err/dt), 0,0.2(Q5err/dt)],numID)
         
         tic;
         
         %This bit needs to be written in task space
         
         jvQ1 = Q1r-Q1old;
         jvQ2 = Q2r-Q2old;
         jvQ3 = Q3r-Q3old;
         jvQ4 = Q4r-Q4old;
         jvQ5 = Q5r-Q5old;
         
         xvector = jacobian(Q1r,Q2r,Q3r,Q4r,Q5r,jvQ1,jvQ2,jvQ3,jvQ4,jvQ5)
         
         xvele = xvector(1) - velocity(1);
         yvele = xvector(2) - velocity(2);
         zvele = xvector(3) - velocity(3);
         
         xveled = xvele - oldxvele;
         yveled = yvele - oldyvele;
         zveled = zvele - oldzvele;
         
         
         oldxvele = xvele;
         oldyvele = yvele;
         oldzvele = zvele;
         
         tex = tex + xvele;
         tey = tey + yvele;
         tez = tez + zvele;
         
         xvel = velocity(1) + 6*xvele + 0.01*tex + 1*xveled;
         yvel = velocity(2) + 6*yvele + 0.01*tey + 1*yveled;
         zvel = velocity(3) + 13*zvele + 0.1*tez + 10*zveled;
         
         xvelf = 0.9*oldxvel + 0.1*xvel;
         yvelf = 0.9*oldyvel + 0.1*yvel;
         zvelf = 0.9*oldzvel + 0.1*zvel;
         
         oldxvel = xvel;
         oldyvel = yvel;
         oldzvel = zvel;
         
         
         te(1:3,count) = [xvele,yvele,zvele];
         tev(1:3,count) = [xvelf,yvelf,zvelf];
         
         [Q1dot, Q2dot, Q3dot, Q4dot, Q5dot] = invJacobian([xvelf,yvelf,zvelf,0,0,0],Q1r,Q2r,Q3r,Q4r,Q5r);
         
         
%          Q1error = (Q1r - Q1old) - Q1dot;
%          Q2error = (Q2r - Q2old) - Q2dot;
%          Q3error = (Q3r - Q3old) - Q3dot;
%          Q4error = (Q4r - Q4old) - Q4dot;
%          Q5error = (Q5r - Q5old) - Q5dot;
%          
%          teQ1 = teQ1 + Q1error;
%          teQ2 = teQ2 + Q2error;
%          teQ3 = teQ3 + Q3error;
%          teQ4 = teQ4 + Q4error;
%          teQ5 = teQ5 + Q5error;
%          
%          Q1dot = Q1dot + 1*Q1error + 0.15*Q1error;
         
         %sendJointVel(s,[Q3dot, Q4dot, Q1dot, Q2dot, 0,Q5dot],numID)
         
         Q1old = Q1r;
         Q2old = Q2r;
         Q3old = Q3r;
         Q4old = Q4r;
         Q5old = Q5r;
         
         
         
    end
end

%% Section 2 Testing

Q1 = pi/7;
Q2 = pi/3;
Q3 = pi/2;
Q4 = 0;
Q5 = 0;

[Q1t,Q2t,Q3t,Q4t,Q5t] = jointspacetransform(Q1,Q2,Q3,Q4,Q5);

[x,y,z] = fwdkinematics(-Q1t, -Q2t, -Q3t - pi/2, -Q4t, Q5t);

[Q1i,Q2i,Q3i,Q4i,Q5i] = jointspaceinvtransform(Q1t,Q2t,Q3t,Q4t,Q5t);

xi = 35;
yi = 0;
zi = 10;

[Q1,Q2,Q3,Q4,Q5] = invkinematics2(xi,yi,zi,-1);

[Q11,Q22,Q33,Q44,Q55] = jointspacetransform(Q1,Q2,Q3,Q4,Q5);

    