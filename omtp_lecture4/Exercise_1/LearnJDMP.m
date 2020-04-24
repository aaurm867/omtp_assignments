function [JDMP] = LearnJDMP(path,dt,N)
    %set DMP parameters
    if nargin == 3
        JDMP.N = N;
    else
        JDMP.N = 100;
    end
    JDMP.dt=dt;JDMP.a_z=48;JDMP.a_x=2;
    %learning positions
    jpath = path(:,1:7);
    JDMP=DMP_rlearn(jpath,JDMP);
end