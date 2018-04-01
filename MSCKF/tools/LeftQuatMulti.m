function R = LeftQuatMulti(qi)

%% Convert left quaternion multiplication to a matrix
vec = qi(1:3);
w = qi(4);
R = [w*eye(3)-Vec2Skew(vec), vec; -vec', w];
end