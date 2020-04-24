%-------------------------------------------------------------------------
function DMP = qDMP_learn(q,DMP)
%-------------------------------------------------------------------------
% non-recursive regression for quaternion DMPs
% same as DMPQ_estimate, but calculates angular velocities
% and accelerations form quaternion path q
    N = length(q);
    DMP.tau = N*DMP.dt;
    % Calculate derivatives
    for j = 1:4
        dq(:,j) = gradient(q(:,j), DMP.dt);
    end

    % Calculate omega and domega
    for i = 1:N
        dqi = quaternion(dq(i,:));
        qi = quaternion(q(i,:));
        omega_q = dqi * qi';
        omega(i,:) = 2*omega_q.v;
    end
    for j = 1:3
        domega(:,j) = gradient(omega(:,j), DMP.dt);
    end
    %initial conditions
    omega(1,:) = [0; 0; 0];
    omega(N,:) = [0; 0; 0];
    domega(1,:) = [0; 0; 0];
    domega(N,:) = [0; 0; 0]; 

    %%% gausian kernel functions
    c_lin=linspace(0,1,DMP.N);
    DMP.c=exp(-DMP.a_x * c_lin);
    DMP.sigma2=(diff(DMP.c)*0.75).^2;
    DMP.sigma2=[DMP.sigma2,DMP.sigma2(end)];

    DMP = qDMP_estimate(q, omega, domega, DMP);
end

%-------------------------------------------------------------------------
function DMP = qDMP_estimate(q, omega, domega, DMP)
%-------------------------------------------------------------------------
% non-recursive regression for quaternion DMPs
    DMP.qg = quaternion(q(end,:));
    DMP.q0 = quaternion(q(1,:));
%     DMP.diag = 2*log_q(DMP.qg,DMP.q0);
    DMP.diag = [1 1 1];
    d = 1 ./ DMP.diag;
    N = length(q);
    f = zeros(N,3);
    A = zeros(N, DMP.N);
    t = DMP.dt:DMP.dt:DMP.tau;

    for i = 1:N
        qi = quaternion(q(i,:));

        f(i,:) = d.*(DMP.tau^2*domega(i,:) + DMP.a_z*DMP.tau*omega(i,:) - ...
                     DMP.a_z*DMP.a_z/4*2*log_q(DMP.qg,qi));
%         f(i,:) = (DMP.tau^2*domega(i,:) + DMP.a_z*DMP.tau*omega(i,:) - ...
%                      DMP.a_z*DMP.a_z/4*2*log_q(DMP.qg,qi));


        x = exp(-DMP.a_x*t(i)/DMP.tau);
        psi = exp(-(x-DMP.c).^2./(2*DMP.sigma2));
        A(i,:) = x*psi/sum(psi);
    end
    disp(d);
    DMP.w = A \ f;
    
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
