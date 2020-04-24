function DMP_cmove(R_goal,p_goal,t,tcp)
    dt=0.01;
    Con = mj_get_control;
    j0=Con.ctrl(1:7);
    [p0,R0] = kinjac_lwr(j0,tcp);
    q0=quaternion(R0);
    q_goal=quaternion(R_goal);
    i=1;
    tn=0;
    tic;
    
    PDMP.N = 2;
    PDMP.dt=dt;PDMP.a_z=48;PDMP.a_x=2;
    PDMP.goal=p_goal;
    PDMP.tau=2*t;
    PDMP.y0=p0;
    PDMP.dy0=[0 0 0];
    PDMP.w=[0 0 0;0 0 0];
    PDMP.c=[1 0.1353];
    PDMP.sigma2=[0.4206 0.4206];
    
    QDMP.N = 2;
    QDMP.dt=dt;QDMP.a_z=48;QDMP.a_x=2;
    QDMP.qg=q_goal;
    QDMP.tau=2*t;
    QDMP.q0=q0;
    QDMP.w=[0 0 0;0 0 0];
    QDMP.c=[1 0.1353];
    QDMP.diag=[1 1 1];
    QDMP.sigma2=[0.4206 0.4206];

    T_f = t;
    Xmin = exp(-PDMP.a_x*T_f/PDMP.tau);

    Sp.y = PDMP.y0;
    Sp.z = zeros(1,3);
    Sp.x = 1;
    
    Sq.q = QDMP.q0;
    Sq.o = zeros(3,1);
    Sq.x = 1;
    
    
    last_ji=j0.';
    qs=double(q0);
    ps=p0;
    js=j0.';
    while Sp.x > Xmin
        
        %% TODO: PDMP integration simmilar as in the previous exercise       
        [Sp]=DMP_integrate(PDMP,Sp,0);
        [Sq]=qDMP_integrate(QDMP,Sq,0);
        p_i=Sp.y(1:3);
        qi=double(Sq.q);
        Ri=quat2rotm(qi);
        ji = ikin_lwr(p_i,Ri,tcp',last_ji(:));
        
        qs(end+1,:)=qi;
        ps(end+1,:)=p_i;
        js(end+1,:)=ji;
        %% Send to robot
        Con.ctrl(1:7) = ji;
        %Con.ctrl(8:14)= (last_ji-ji)/dt;
        last_ji=ji;
        mj_set_control(Con);
       
        
        %% sincronisation 
        tn = tn+dt;
        if tn>toc
            pause(tn-toc)
        end
        i=i+1;
    end
    for k=size(js,2)
        plot(mod(js,2*pi))
    end
    toc
    
end