function [DMP]=DMP_rlearn(y, DMP)


% Locally waighetd regression DMP,  


% input measured values 
%   y matrix of signals
%   y,  ... position(t)
%   dy, ... velocity(t)
%   ddy, ... acceleration(t)
%   dt, ... sample time
%   DMP, ... DMP parameters 

% DMP parameters
%   N,  ... number of Gaussian kernel functions
%   w,  ... weight vector of size(Nx1)
%   c,
%   sigma2
%   tau
%   a_x
%   a_z


%% params - global
[NT,NS] = size(y);
% DONE: Define the initial state of the DMP(y0)- first point of the
% demonstrated trajectory
DMP.y0 = y(1,:);
% DONE: Define the goal state of the DMP(goal)- last point of the
% demonstrated trajecory
DMP.goal = y(NT,:);
% Calculate the length of the recorded trajectory
DMP.tau = (NT-1)*DMP.dt;
%% DONE generate derivatives for the entire trajecory (use diff) and divide with sampling rate (DMP.dt)
%Velocities
dy=diff(y)/DMP.dt;
dy=[zeros(1,NS);dy];
%Acceleration
ddy=diff(dy)/DMP.dt;
ddy=[zeros(1,NS);ddy];

DMP.dy0 = dy(1,:);    % initial value

%% init params for target traj. and fitting
x = 1;
h=-0.5;
dx = 0;
for i = 1:NS,
     P(:,:,i) = eye(DMP.N)*1000;    % initial large covariance
end
DMP.w = zeros(DMP.N,NS);   % initial weights

%%% Definition of the gausian kernel functions
c_lin=linspace(0,1,DMP.N);
DMP.c=exp(-DMP.a_x * c_lin);
DMP.sigma2=(diff(DMP.c)*0.75).^2;
DMP.sigma2=[DMP.sigma2,DMP.sigma2(end)];

cutoff = 0.001;
lambda = 0.995;

%% fit all points of the trajectory
for t=1:NT
    %% DONE: the weighted sum of the locally weighted regression models calculate psi
    psi = exp(-(x-DMP.c).^2./(2*DMP.sigma2)).';
    %% derivatives
    dx=-DMP.a_x*x;
    %% DONE: temporal scaling derivatives/ tau
    dx = dx/DMP.tau;
    %% Euler integration
    x=x+dx*DMP.dt;
    
    for k = 1:NS
        %% DONE: target for fitting - expected fx for perfect fitting DMP line
        % equations - implement the second order DMP equations at this
        % point
        ft = DMP.tau^2*ddy(t,k)+DMP.a_z*DMP.tau*dy(t,k)-DMP.a_z*DMP.a_z/4*(DMP.goal(k)-y(t,k));
        %% recursive regression step
        xx = psi*x/sum(psi);
        % calculation of all weights over entire interval x
        Pk = P(:,:,k);
        P(:,:,k) = (Pk-(Pk*xx*xx'*Pk)/(1+xx'*Pk*xx));
        e = ft - xx'*DMP.w(:,k);
        DMP.w(:,k) = DMP.w(:,k) + e*P(:,:,k)*xx;
    end
end   


