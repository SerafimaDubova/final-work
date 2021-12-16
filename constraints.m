function C = constraints(mbs, q, t)
%CONSTRAINTS Return constraints vector for the system
C = zeros(mbs.nc, 1);
c_idx = 0;

for rj = mbs.joints.revolute
    q1 = q(body_idx(rj.body1));
    q2 = q(body_idx(rj.body2));
    r1 = q1(1:2);
    r2 = q2(1:2);
    A1 = rot(q1(3));
    A2 = rot(q2(3));
    C(c_idx + (1:2)) = r1 + A1 * rj.s1 - r2 - A2 * rj.s2;
    c_idx = c_idx + 2;
end

for pj = mbs.joints.prismatic
    q1 = q(body_idx(pj.body1));
    q2 = q(body_idx(pj.body2));
    
    phi_1 = q1(3);
    phi_2 = q2(3);
    
    % Getting actual coordinates of the additional point 
    % for taking into account potentional rotations of the bodies
    
    % According to the equation 4.12, body1 = i and body2 = j
    % the point P(q1) belongs to the body1
    % the point Q(q_s3) belongs to the body1
    % the point P(q2) belongs to the body2
    
    q0_s2 = pj.s2;
    A_s2 = rot(phi_1);
    q_s3 = q1(1:2) + A_s2 * q0_s2;
    
    n = [-(q1(2) - q_s3(2)); 
        q1(1) - q_s3(1)];
    d = [q2(1) - q1(1); 
        q2(2) - q1(2)];
    
    C(c_idx + 1) = n' * d;
    C(c_idx + 2) = phi_1 - phi_2;
    c_idx = c_idx + 2;
end

for sj = mbs.joints.simple
    qb = q(body_idx(sj.body));
    C(c_idx + 1) = qb(sj.coord) - sj.c0;
    c_idx = c_idx + 1;
end

for dj = mbs.joints.driving
    qb = q(body_idx(dj.body));
    C(c_idx + 1) = qb(dj.coord) - dj.cfun(t);
    c_idx = c_idx + 1;
end

