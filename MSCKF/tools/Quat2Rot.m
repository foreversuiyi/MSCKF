function rot = Quat2Rot(quat)

%% Convert the quaternion to rotation matrix

if(abs(norm(quat) - 1) > eps)
    quat = quat/norm(quat);
end

tmp = RightQuatMulti(quat)' * LeftQuatMulti(quat);
[U,S,V] = svd(tmp);
tmp = U*eye(size(S,1))*V';
rot = tmp(1:3,1:3);

end