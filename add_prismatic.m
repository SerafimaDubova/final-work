function mbs = add_prismatic(mbs, name, body1_name, s1, s2, body2_name, s3)
%  ADD_PRISMATIC Function that adds prismatic joints.
 
% Prismatic joint definition: name, body names, local vector coordinates.
% According to the equation 4.12, body1 = i and body2 = j
% the point P(s1) belongs to the body1
% the point Q(s2) belongs to the body1 and lies on the axis of movement
% the point P(s3) belongs to the body2

p = inputParser;
is_2vec = @(x) isvector(x) && length(x) == 2;
addRequired(p,'name', @isstring);
addRequired(p, 'body1_name', @isstring);
addRequired(p, 's1', is_2vec);
addRequired(p, 's2', is_2vec);
addRequired(p, 'body2_name', @isstring);
addRequired(p, 's3', is_2vec);
parse(p, name, body1_name, s1, s2, body2_name, s3);

% Get bodies ids
b1 = get_body_id(mbs, body1_name);
b2 = get_body_id(mbs, body2_name);

lin = struct("name", name, "body1", b1, "s1", s1(:), "s2", s2(:), ...
    "body2", b2, "s3", s3(:));
mbs.joints.prismatic = [mbs.joints.prismatic, lin];
mbs.nc = mbs.nc + 2;
end