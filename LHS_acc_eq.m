function g = LHS_acc_eq(mbs, q, qd, t)
% Return LHS  of acceleration equation "g"
g = zeros(mbs.nc, 1);
c_idx = 0;

for rj = mbs.joints.revolute
    q1 = q(body_idx(rj.body1));
    q2 = q(body_idx(rj.body2));
    qd1 = qd(body_idx(rj.body1));
    qd2 = qd(body_idx(rj.body2));
    qd1 = qd1(3);
    qd2 = qd2(3);
    A1 = rot(q1(3));
    A2 = rot(q2(3));
    g(c_idx + (1:2)) = A1 * rj.s1 * qd1 .* qd1  -  A2 * rj.s2 * qd2 .* qd2;
    c_idx = c_idx + 2;
end

for pj = mbs.joints.prismatic
    q1 = q(body_idx(pj.body1));
    q2 = q(body_idx(pj.body2));
    qd1 = qd(body_idx(pj.body1));
    qd2 = qd(body_idx(pj.body2));
    
    q0_s2 = pj.s2;
    phi_1 = q1(3);
    A_s2 = rot(phi_1);
    q_s2 = q1(1:2) + A_s2 * q0_s2;
    
    g(c_idx + 1) = - 2 * ((q1(1) - q_s2(1)) * (qd1(1) - qd2(1))... 
                   + (q1(2) - q_s2(2)) * (qd1(2) - qd2(2))) * qd1(3)...
                   - ((q1(1) - q_s2(1)) * (q1(2) - q2(2))...
                   - (q1(2) - q_s2(2)) * (q1(1) - q2(1))) * qd2(3) * qd2(3);
    g(c_idx + 2) = 0;
    c_idx = c_idx + 2;
end

for sj = mbs.joints.simple
    qb = qd(body_idx(sj.body));
    g(c_idx + 1) = qb(sj.coord) - sj.c0;
    c_idx = c_idx + 1;
end

for dj = mbs.joints.driving
    g(c_idx + 1) =  - dj.cfun_dtt(t);
    c_idx = c_idx + 1;
end

