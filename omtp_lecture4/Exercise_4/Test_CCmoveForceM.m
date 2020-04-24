%% move test and grasp

%%
clc
clear all
close all
dt=0.01;
path('..\apimex',path)
path('..\Mujoco_lib',path)
path('..\Exercise_1',path)
path('..\Exercise_3',path)
if mj_connected
    mj_close
end
mj_connect;
mj_reset

q1=[0 0 0 -pi/2 0 pi/2 0];
tcp=[0 0 0.14];


DMP_jmove(q1,2);
Data1=CCmoveForceM([0 0 -1],[Inf Inf 100],10,tcp);
DMP_jmove(q1,2);
Data2=CCmoveForceM([0 0 -1],[Inf Inf 100],10,tcp);
DMP_jmove(q1,2);
%plot(Data.Ftcp.')
%legend Fx Fy Fz

mj_close
