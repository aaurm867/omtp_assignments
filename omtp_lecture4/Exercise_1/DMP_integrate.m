function [S,psi,fx] = DMP_integrate(DMP,S,ts)

% discrete move DMP realization, 

%% init params for target traj. and fitting

NS = size(DMP.w,2); %number of signals
%disp(NS)
% phase stop signal ts
if nargin < 3
    ts = 0;
end
% phase variable
dx=-1*DMP.a_x*S.x;
dx = dx/DMP.tau;
% phase stop
dx = dx/(1+ts);
S.x=S.x+dx*DMP.dt;

% the weighted sum of the locally weighted regression models
psi=exp(-(S.x-DMP.c).^2./(2*DMP.sigma2))';
for i = 1:NS
    fx = sum((DMP.w(:,i)*S.x).*psi/(sum(psi))); 
    % derivatives
    dz = DMP.a_z *(DMP.a_z/4 *(DMP.goal(i) - S.y(i)) - S.z(i)) + fx;
    dy = S.z(i);
    % temporal scaling
    dz = dz/DMP.tau;
    dy = dy/DMP.tau;
    % Euler integration
    S.z(i)=S.z(i)+dz*DMP.dt;
    S.y(i)=S.y(i)+dy*DMP.dt;
   
end;
S.basis = psi*S.x;


