function DMP_jmove(q_goal,t)
    dt=0.01;
    
    Con = mj_get_control;
    q0=Con.ctrl(1:7);
    i=1;
    tn=0;
    tic;
    
    JDMP.N = 2;
    JDMP.dt=dt;JDMP.a_z=48;JDMP.a_x=2;
    JDMP.goal=q_goal;
    JDMP.tau=2*t;
    JDMP.y0=q0;
    JDMP.dy0=[0 0 0 0 0 0 0];
    JDMP.w=[0 0 0 0 0 0 0;0 0 0 0 0 0 0];
    JDMP.c=[1 0.1353];
    JDMP.sigma2=[0.4206 0.4206];

    T_f = t;
    Xmin = exp(-JDMP.a_x*T_f/JDMP.tau);

    Sj.y = JDMP.y0;
    Sj.z = zeros(1,7);
    Sj.x = 1;
    
    
     
    while Sj.x > Xmin
        
        %% TODO: JDMP integration simmilar as in the previous exercise       
        [Sj]=DMP_integrate(JDMP,Sj,0);
        
        %% Send to robot
        Con.ctrl(1:7) = Sj.y;
        Con.ctrl(8:14)= Sj.z;
        mj_set_control(Con);
       
        
        %% sincronisation 
        tn = tn+dt;
        if tn>toc
            pause(tn-toc)
        end
        i=i+1;
    end
    toc
    
end