%% move test and grasp

%%
clc
clear all
close all
dt=0.01;
path('..\apimex',path)
path('..\Mujoco_lib',path)
path('..\Exercise_1',path)


mj_connect;
%% Robot pos
q0=[0 0 0 0 0 0 0];
q1=[0 0 0 -pi/2 0 pi/2 0];
q3=[-1.5708 0.8960 0 -1.4466 0 0.8481 0];

%% cube pos
C(3,:)=[-0.4, 0,0.02];
C(2,:)=[-0.4, -0.4, 0.02];
C(1,:)=[-0.5, 0.3, 0.02];
C(4,:)=[-0.5, -0.3,0.02];
C(5,:)=[-0.4, 0.4,0.02];
C(6,:)=[-0.6, 0,0.02];

for i = 1:length(C)
    
    
    DMP_jmove(q1,2);
    
    Gripper('open')
    DMP_cmove([0,-1,0;-1,0,0;0,0,-1],C(i,:),4,[0 0 0.14]);
    Gripper('close')
    pause(2)
    
    DMP_jmove(q1,4);
    DMP_jmove(q3,10);
    Gripper('open')
    
    
end
JmoveM(q1,2);
mj_close
