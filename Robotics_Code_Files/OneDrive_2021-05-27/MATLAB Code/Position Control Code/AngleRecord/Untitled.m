close all
clear all 
clc
load('Move1_pawn.mat')
t = 0.01:0.01:5.03;
figure;
hold on
for i = 1:5
    plot(t,record(i,:));
end
title('Move 1 Pawn')
xlabel('time(s)')
ylabel('Angle Displacement(deg)')
legend('Q1','Q2','Q3','Q4','Q5')