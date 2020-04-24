function S = qDMP_integrate(DMP,S,ts)
%-------------------------------------------------------------------------

% discrete move DMP realization for quaternions
%% init params for target traj. and fitting
    % phase stop signal ts
    if nargin < 3
        ts = 0;
    end
    % phase variable
    dx = -DMP.a_x * S.x;
    % phase stop
    dx = dx/(1+ts);
    dx = dx / DMP.tau;
    S.x = S.x + dx * DMP.dt;

    % the weighted sum of the locally weighted regression models
    if S.x >= exp(-DMP.a_x) 
        psi=exp(-(S.x-DMP.c).^2./(2*DMP.sigma2))';
        fx(1) = sum((DMP.w(:,1)*S.x).*psi)/sum(psi);
        fx(2) = sum((DMP.w(:,2)*S.x).*psi)/sum(psi);
        fx(3) = sum((DMP.w(:,3)*S.x).*psi)/sum(psi);
    else
        fx = [0 0 0]; 
    end
    % angular velocity 
    S.do = DMP.a_z *(DMP.a_z/4 * 2 * log_q(DMP.qg,S.q)' - S.o) + (fx.*DMP.diag)';
    S.do = S.do / DMP.tau;
    S.o = S.o + S.do * DMP.dt;
    % integration of quaternions
    q_w = [ cos(norm(S.o/DMP.tau)*DMP.dt/2) , sin(norm(S.o/DMP.tau)*DMP.dt/2) * S.o'/norm(S.o) ];
    dq = quaternion(q_w);
    S.q = dq*S.q;
end

%-------------------------------------------------------------------------
function e = log_q(q1,q2)
%-------------------------------------------------------------------------
%calculates orientation error between quaternions
% q' is conjugate quaternion
    q = q1 * q2';
    if (norm(q.v) > 1.0e-12)
        log_q = acos(q.s) * q.v / norm(q.v);
        if norm(log_q) > pi
            log_q = (2*pi - 2 * acos(q.s)) * (-q.v / norm(q.v));
        end
    else
        log_q = q1.v*0;
    end
    e = log_q;
end

