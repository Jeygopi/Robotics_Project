clear all
clc
%% Assignment 1 Forward Kinematics
%% Produce transformation matrices of frame i with respect to i-1
syms l1 l2 l3 l4 l5 Q1 Q2 Q3 Q4 Q5 P_x P_y P_z real


l4l5_4 = [0 0 -(l4 + l5)];
%%DH Table
DH_Mat = [0,0,l1,Q1;pi/2,0,0,Q2;0,l2,0,Q3;0,l3,0,Q4;pi/2,0,l4,Q5;0,0,l5,0];

%conversion of DH table into transformation matricies 
[T_1to0,R_1to0_x,D_1to0_x,D_1to0_z,R_1to0_z] = transmat(DH_Mat(1,1),DH_Mat(1,2),DH_Mat(1,3),DH_Mat(1,4));

[T_2to1,R_2to1_x,D_2to1_x,D_2to1_z,R_2to1_z] = transmat(DH_Mat(2,1),DH_Mat(2,2),DH_Mat(2,3),DH_Mat(2,4));

[T_3to2,R_3to2_x,D_3to2_x,D_3to2_z,R_3to2_z] = transmat(DH_Mat(3,1),DH_Mat(3,2),DH_Mat(3,3),DH_Mat(3,4));

[T_4to3,R_4to3_x,D_4to3_x,D_4to3_z,R_4to3_z] = transmat(DH_Mat(4,1),DH_Mat(4,2),DH_Mat(4,3),DH_Mat(4,4));

[T_5to4,R_5to4_x,D_5to4_x,D_5to4_z,R_5to4_z] = transmat(DH_Mat(5,1),DH_Mat(5,2),DH_Mat(5,3),DH_Mat(5,4));

[T_eto5,R_eto5_x,D_eto5_x,D_eto5_z,R_eto5_z] = transmat(DH_Mat(6,1),DH_Mat(6,2),DH_Mat(6,3),DH_Mat(6,4));

clc

T_eto0 = T_1to0*T_2to1*T_3to2*T_4to3*T_5to4*T_eto5;

%% Output the end effector pose given joint space displacement

% Refer to the forwardkinematics custom function to see how this function
% works. 
% Main functionality is it substitutes values given for angles into the
% T_eto0 matrix.
ou = forwardkinematics(0,0,0,0,0,T_eto0);
%Uses the function forwardkinematics


%% Transformation matricies with respect to initial frames and final transformation matrix E with respect to 0

'transformation matrices from frame i to i-1'

T_1to0
T_2to1
T_3to2
T_4to3
T_5to4
T_eto5

T_eto0

'end effector pose given trivial joint space displacement'
ou = forwardkinematics(0,0,0,0,0,T_eto0);

'inverse kinematics example'

% Refer to function invkinematics for details on how this operates
% Select -1 for elbow up configuration, 1 for elbow down configuration
% Main functionality is it uses the derived inverse kinematic equations to
% provide a solution for the joint space angles.
invkinematics(19.08,0,3.74,-1)

%% Drawing for X-Z Plane
%The following method is for drawing XZ Plane
%index is for outputing the data for testing
index = 1;
%  
king_height=9.29;
king_pickup=6.47;
pawn_pickup=2.14;
D1=45;
D2=32.96;
D3=D2/8;
m=12;

k0=sqrt((m+(D1-D2)/2+D3/3)^2+(D3/3)^2)
k1=sqrt((m-(D1-D2)/2-D3/3+D1)^2+(4*D3-D3/3)^2)
k2=sqrt((m+D1)^2+(D1/2)^2)


% %Draw the figure of the chessboard
% line([m,m],[2*king_height,0],'Color','red','LineWidth',2)
% line([m,k2],[2*king_height,2*king_height],'Color','red','LineWidth',2)
% line([k2,k2],[0,2*king_height],'Color','red','LineWidth',2)
% line([m,k2],[0,0],'Color','red','LineWidth',2)
% 
% line([k0,k0],[king_height+king_pickup,pawn_pickup],'Color','blue','LineWidth',2)
% line([k0,k1],[king_height+king_pickup,king_height+king_pickup],'Color','blue','LineWidth',2)
% line([k1,k1],[pawn_pickup,king_height+king_pickup],'Color','blue','LineWidth',2)
% line([k0,k1],[pawn_pickup,pawn_pickup],'Color','blue','LineWidth',2)
% 
% %For loop that allows theta2 and theta3 (Q2 and Q3) to have all the possible values
% for theta2 = 0/180*pi:0.1: 70 / 180 * pi
%     for theta3 = -160/180*pi:0.1:-30 / 180 * pi
%         theta4 = -theta3 - theta2;
% 
%         %calculate the corresponding position of the end-effector
%         spot = subs(T0e,{Q1,Q2,Q3,Q4,Q5,L1,L2,L3,L4,L5},{0,theta2,theta3,theta4,0,34,34,34,7.5,7.5});
%         
%         plot(spot(1,4),spot(3,4),'.k')
%         %testing code: the 'list' variable contains [x value, z value,theta2, theta3] in each row
% 
%         list(index,:) = [vpa(simplify(spot(1,4))) vpa(simplify(spot(3,4))) vpa(theta2) vpa(theta3)];
%         index = index+1;
%         %End of test
%     end
% end
% 
% %Define the maximum value along x-axis and y-axis
% xlim([0 70])
% ylim([-20 50])
% 
% %Label and title the figure
% title('X-Z Plane')
% xlabel('x axis(cm)')
% ylabel('z axis(cm)')



%% Drawing for X-Y Plane

'xy plane drawing'
% Create the window for the plot
figure(1);
hold on


%The following part is for drawing the XY Plane, method is the same as the XZ Plane above
for theta1 = -pi/2:0.1:pi / 2
    for theta2 = 0/180*pi:0.2: 70 / 180 * pi
        for theta3 = -160/180*pi:0.2:-30 / 180 * pi
            theta4 = -theta3 - theta2;
            spot = subs(T_eto0,{Q1,Q2,Q3,Q4,Q5,l1,l2,l3,l4,l5},{theta1,theta2,theta3,theta4,0,34,34,34,7.5,7.5});

              %Only draw the position of the end-effector that has z-value lower than 15 and higher than 0.
            if(spot(3,4)<=15 && spot(1,4) >= 0 && spot(3,4) >= 0 && spot(3,4) >= 0)
                plot(spot(1,4),spot(2,4),'.k')
            end
        end
    end
end

%Draw the figure of the chessboard
line([m,m+D1],[D1/2,D1/2],'Color','red','LineWidth',2)
line([m,m],[D1/2,-D1/2],'Color','red','LineWidth',2)
line([m+D1,m+D1],[D1/2,-D1/2],'Color','red','LineWidth',2)
line([m,m+D1],[-D1/2,-D1/2],'Color','red','LineWidth',2)

line([m+(D1-D2)/2,m+(D1-D2)/2+D2],[D2/2,D2/2],'Color','blue','LineWidth',2)
line([m+(D1-D2)/2,m+(D1-D2)/2],[D2/2,-D2/2],'Color','blue','LineWidth',2)
line([m+(D1-D2)/2+D2,m+(D1-D2)/2+D2],[D2/2,-D2/2],'Color','blue','LineWidth',2)
line([m+(D1-D2)/2,m+(D1-D2)/2+D2],[-D2/2,-D2/2],'Color','blue','LineWidth',2)

xlim([0 70])
ylim([-35 35])
title('X-Y Plane')
xlabel('x axis(cm)')
ylabel('y axis(cm)')