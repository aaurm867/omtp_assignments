%%
clc
clear all
close all
dt=0.01;
path('..\apimex',path)
path('..\Mujoco_lib',path)
path('..\Exercise_1',path)
load test_trj.mat

%% DONE: Implement the encoding step for the Joint DMP from the previous
%exercise. 

[JDMP] = LearnJDMP(qJoints,dt,2);

T_f = (length(Qpath)+1)*JDMP.dt;
Xmin = exp(-JDMP.a_x*T_f/JDMP.tau);

Sj.y = JDMP.y0;
Sj.z = zeros(1,7);
Sj.x = 1;

JDMP.tau=1;

%% Execute in simulation

try
    %connect to sim robot
    mj_connect;
    
    Startjoints = qJoints(1,:);
    
    %% move to init position
    JmoveM(Startjoints,1);
    Con = mj_get_control;
    i=1;
    tn=0;
    tic;
     
    while Sj.x > Xmin
        
        %% TODO: JDMP integration simmilar as in the previous exercise       
        [Sj]=DMP_integrate(JDMP,Sj,0);
        
        %% Send to robot
        Con.ctrl(1:7) = Sj.y;
        Con.ctrl(8:14)= Sj.z;
        mj_set_control(Con);
       
        
        %% sinhronisation 
        tn = tn+dt;
        if tn>toc
            pause(tn-toc)
        end
        i=i+1;
    end
       toc
       
    %% move to start position    
    JmoveM(Startjoints,1);
    
    %close sim connection
    mj_close;
    
catch ME
    mj_close;
    disp(ME.message);
end