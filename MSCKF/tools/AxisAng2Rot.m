function Matrix = AxisAng2Rot(vector)

%% Create the rotation matrix from 3x1 axis-angle representation
angle = norm(vector);
if angle == 0
    Matrix = eye(3);
else
    norm_vec = vector/angle;
    Matrix = cos(angle)*eye(3) + (1 - cos(angle))*(norm_vec)*(norm_vec)' - ...
        sin(angle) * Vec2Skew(norm_vec);
end