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
port = 'COM5';                  % MacOS Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS
% port = 'COM1';                                 % Windows Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS

% Setup Serial Connection
baudrate = 230400;
s = establishSerial(port, baudrate); 

% Read Connected Motor IDs
[numID, ID] = getMotorIDs(s)

% Set Change Motor ID flag - CHANGE TO ADJUST MOTOR ID
changeMotorID = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% NOTE: Remember to fclose(s) before disconnecting your Arduino!!! %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% section for running code

% startPoint = [25.0;-6.8;1];
startPoint = [15;25;10];
% endPoint = [45.72;-4.6;1];
endPoint = [25;19;10];

[viaPoint1,viaPoint2] = gen_viapoint(startPoint,endPoint)
pointList = [startPoint,viaPoint1,viaPoint2,endPoint];
vInit = [[0;0;0],[0;0;3], [2;0;0]];
vEnd = [[0;0;3],[2;0;0],[0;0;2]];
tStart = [0 5 500];
tEnd = [5 10 1000];
coefficientList = [];
p = 0
q = 0
start = 1
count1 = 0
for i = 1:3
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
    Q1r = fbb(3);
    Q2r = fbb(4);
    Q3r = fbb(1);
    Q4r = fbb(2);
    Q5r = fbb(6);
    Q6r = fbb(5);
        
    for j = 1:3
        temp(:,j) = gen_calculate(ts,te,ep(j,:),ve(j,:),sp(j,:),vi(j,:)); %calculate co-efficients for polynomial
    end
    temp;
    
    
    for t = ts:0.01:te
        % this is all for velocity
        Q1old = Q1r;
        Q2old = Q2r;
        Q3old = Q3r;
        Q4old = Q4r;
        Q5old = Q5r;
        [fbb,e] = readFB(s,6);
            
            %[Q1r,Q2r,Q3r,Q4r,Q5r,Q6r] = [fba[3],fba[4],fba[1],fba[2],fba[6],fba[5]];
        Q1r = fbb(3);
        Q2r = fbb(4);
        Q3r = fbb(1);
        Q4r = fbb(2);
        Q5r = fbb(6);
        Q6r = fbb(5);
        
        count1 = count1 + 1;
            position = [];
            orientation = [];
            velocity = [];
            angvel = [];
            for j = 1:3
                position(j) = cal_position(temp(:,j),t);
                velocity(j) = cal_velocity(temp(:,j),t);
            end
            [x,y,z] = fwdkinematics(Q1r,Q2r,Q3r,Q4r,Q5r);
            K_pos = [0,0,0;0,4,0;0,0,4];
            %x-velocity(1)
            %K_pos*[x-velocity(1),y-velocity(2),z-velocity(3)]'
            xe = 0*(x-velocity(1));
            ye = 4*(y-velocity(2));
            ze = 4*(z-velocity(3));
            %The above is just the error
            
            xtr(count) = position(1);
            ytr(count) = position(2);
            ztr(count) = position(3);
            
            %[xe,ye,ze] = (K_pos*[x-velocity(1),y-velocity(2),z-velocity(3)]')';
            %[Q1e,Q2e,Q3e,Q4e,Q5e] = invkinematics2(xe,ye,ze,1);
            
            [Q1,Q2,Q3,Q4,Q5] = invkinematics2(position(1),position(2),position(3),1);
            
            % Velocity control is here
            
            dt = toc;
        
            Q1dot = (Q1r - Q1old)/dt;
            Q2dot = (Q2r - Q2old)/dt;
            Q3dot = (Q3r - Q3old)/dt;
            Q4dot = (Q4r - Q4old)/dt;
            Q5dot = (Q5r - Q5old)/dt;
            
            Q1dd(count1) = Q1dot;
            Q2dd(count1) = Q2dot;
            Q3dd(count1) = Q3dot;
            Q4dd(count1) = Q4dot;
            Q5dd(count1) = Q5dot;

            tic
            
            [Q1des, Q2des, Q3des, Q4des, Q5des] = invJacobian([velocity(1),velocity(2),velocity(3),0,0,0],Q1r,Q2r,Q3r,Q4r,Q5r)
            
            Q1act(count1) = Q1des;
            Q2act(count1) = Q2des;
            Q3act(count1) = Q3des;
            Q4act(count1) = Q4des;
            Q5act(count1) = Q5des;
            
            vectorout = jacobian(Q1r,Q2r,Q3r,Q4r,Q5r,Q1dot,Q2dot,Q3dot,Q4dot,Q5dot);
            
            vel_error = [velocity(1) - vectorout(1),velocity(2) - vectorout(2), velocity(3) - vectorout(3)];
            
            adjvel = [velocity(1),velocity(2),velocity(3)] + (K_pos*vel_error')';
            
            adjvel(4:6) = [0,0,0];
            adjvel
            
            [Q1dot,Q2dot,Q3dot,Q4dot,Q5dot] = invJacobian(adjvel,Q1r,Q2r,Q3r,Q4r,Q5r);
            
            pos_e = [Q1-Q1r,Q2-Q2r,Q3-Q3r,Q4-Q4r,Q5-Q5r]'
            
            Q1c(count1) = Q1;
            Q2c(count1) = Q2;
            Q3c(count1) = Q3;
            Q4c(count1) = Q4;
            Q5c(count1) = Q5;
            
            Q1s(count1) = Q1r;
            Q2s(count1) = Q2r;
            Q3s(count1) = Q3r;
            Q4s(count1) = Q4r;
            Q5s(count1) = Q5r;
            
            pos_es(1:5,count1) = pos_e;
            
            %compcommand = [Q1,Q2,Q3,Q4,Q5] + K_pos*pos_e;
            
%             Q1r = compcommand(1);
%             Q2r = compcommand(2);
%             Q3r = compcommand(3);
%             Q4r = compcommand(4);
%             Q5r = compcommand(5);
        
      
        
        %unnessecary and just checking if it returns the inverse
        
            
            
            
            
%             positionx(count1) = position(1);
%             positiony(count1) = position(2);
%             positionz(count1) = position(3);
%             
%             velocityx(count1) = velocity(1);
%             velocityy(count1) = velocity(2);
%             velocityz(count1) = velocity(3);
%             
%             [Q1,Q2,Q3,Q4,Q5] = invkinematics(position(1),position(2),position(3),1)
%             
%             Q1c(count1) = Q1;
%             Q2c(count1) = Q2;
%             Q3c(count1) = Q3;
%             Q4c(count1) = Q4;
%             Q5c(count1) = Q5;
%             
%             velocitycomb = [velocity(1),velocity(2),velocity(3),0,0,-1]
%             
%             [Q1dot,Q2dot,Q3dot,Q4dot,Q5dot] = invJacobian(velocitycomb,Q1,Q2,Q3,Q4,Q5)
%             
%             Q1d(count1) = Q1dot;
%             Q2d(count1) = Q2dot;
%             Q3d(count1) = Q3dot;
%             Q4d(count1) = Q4dot;
%             Q5d(count1) = Q5dot;
%             
%             vectorout = jacobian(Q1,Q2,Q3,Q4,Q5,Q1dot,Q2dot,Q3dot,Q4dot,Q5dot);
%             
%             ab(count1,1:6) = vectorout;
            %sendJointVel(s,[Q3dot, Q4dot, Q1dot, Q2dot, 0,Q5dot],numID)
    end
end

%% seems like I can use the pseudo inverse, and filter out the output from there, 

plot(positionx)


