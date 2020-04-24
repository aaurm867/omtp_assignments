%%
clc
clear all
close all
dt=0.01;
load test_trj.mat

%encode into DMP
[DMP,QDMP] = LearnQDMP(Qpath,dt);


% DONE: for joint trajectories define a function for calculating a Joint
% DMP parameters Inputs: qJoints and dt Output: JDMP parameters. the
% function should be simmilar as LearnQDMP (without the orientation part)

[JDMP] = LearnJDMP(qJoints,dt);

%DMP.tau=5;
%JDMP.tau=5;
%QDMP.tau=5;
%DMP.goal(:)=[0;0;1];

%%Calculate final phase for the DMP integration
T_f = (length(Qpath)+1)*DMP.dt;
Xmin = exp(-DMP.a_x*T_f/DMP.tau);

%init. states for position dmp
Sp.y = DMP.y0;
Sp.z = zeros(1,7);
Sp.x = 1;

%init. states for joint dmp
Sj.y = JDMP.y0;
Sj.z = zeros(1,7);
Sj.x = 1;

%init. states for quaternion dmp
Sq.q = QDMP.q0;
Sq.o = zeros(3,1);
Sq.x = 1;



%%
i=1;
while Sp.x > Xmin
    % position DMP
    [Sp]=DMP_integrate(DMP,Sp,0);
    xN(i,:) = Sp.y;
    
    %%  DONE: Joint DMP integrate Configure the a simillar function as for the position part of the DMP
    
    [Sj]=DMP_integrate(JDMP,Sj,0);
    jN(i,:) = Sj.y;
   
    %%
    % Quaternion DMP
    [Sq]=qDMP_integrate(QDMP,Sq,0);
    qQ(i,:) = double(Sq.q); % just for plotting
    
    i=i+1;
end

%% Plot Position part of the trajectory
h1=plot(xN,'r');  % plot DMP trajectory
hold on
h2=plot(Qpath(:,1:3),'--b'); % plot example trajectory
for h = [h1(2:3).' h2(2:3).']
    set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
end
title('Tool position')
legend({'DMP trajectory','example trajectory'})

%% Plot Orientation part of the trajectory
figure(2)
h3=plot(qQ,'r'); % plot QDMP trajectory
hold on
h4=plot(Qpath(:,4:7),'--b'); % plot example trajectory
for h = [h3(2:end).' h4(2:end).']
    set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
end
title('Tool orientation (quaternion)')
legend('QDMP trajectory','example trajectory');

% Plot Joint trajectories
figure(3)
h5=plot(jN,'r'); % plot QDMP trajectory
hold on
h6=plot(qJoints,'--b'); % plot example trajectory
for h = [h5(2:end).' h6(2:end).']
    set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
end
title('Joint angles')
legend('JDMP trajectory','example trajectory');


