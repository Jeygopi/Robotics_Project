close all
clear all 
clc
load('Move6_rook.mat')
t = 0.01:0.01:5.03;
figure;
hold on
for i = 1:5
    plot(t,record(i,:));
end
title('Move 6 Rook')
xlabel('time(s)')
ylabel('Angle Displacement(deg)')
legend('Q1','Q2','Q3','Q4','Q5')