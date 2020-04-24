function [DMP,QDMP] = LearnQDMP(path,dt,N)
    %set DMP parameters
    if nargin == 3
        DMP.N = N;
        QDMP.N = N;
    else
        DMP.N = 100;
        QDMP.N = 100;
    end
    DMP.dt=dt;DMP.a_z=48;DMP.a_x=2;
    QDMP.dt=dt;QDMP.a_z=48*3;QDMP.a_x=2;
    %learning positions
    ppath = path(:,1:3);
    DMP=DMP_rlearn(ppath,DMP);
    %learning quaternions
    qpath = path(:,4:7);
    QDMP=qDMP_learn(qpath,QDMP);
end