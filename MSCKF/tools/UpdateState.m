function msckf_state_up = UpdateState(msckf_state, deltax)
%% Update MSCKF State with deltax

msckf_state_up = msckf_state;

delta_theta = deltax(1:3);
delta_gb = deltax(4:6);
delta_vb = deltax(7:9);
delta_pos = deltax(10:12);

delta_quat = Theta2Quat(delta_theta);
msckf_state_up.imu_state.quat = LeftQuatMulti(delta_quat)*msckf_state.imu_state.quat;
msckf_state_up.imu_state.gb = msckf_state.imu_state.gb + delta_gb;
msckf_state_up.imu_state.vb = msckf_state.imu_state.vb + delta_vb;
msckf_state_up.imu_state.pos = msckf_state.imu_state.pos + delta_pos;

for idx = 1:size(msckf_state.cam_states, 2)
    qstart = 12+6*(idx-1)+1;
    delta_ctheta = deltax(qstart:qstart+2);
    delta_cpos = deltax(qstart+3:qstart+5);
    delta_cquat = Theta2Quat(delta_ctheta);
    msckf_state_up.cam_states{idx}.quat = LeftQuatMulti(delta_cquat)*msckf_state.cam_states{idx}.quat;
    msckf_state_up.cam_states{idx}.pos = msckf_state.cam_states{idx}.pos + delta_cpos;
end

    function quat = Theta2Quat(theta)
    %% Build delta_quat from delta_theta when delta_theta is small enough

    delta_q = 0.5*theta;
    qnorm = delta_q' * delta_q;
    if qnorm > 1
        quat = [delta_q; 1];
        quat = quat/sqrt(1+qnorm);
    else
        quat = [delta_q; sqrt(1-qnorm)];
    end
    quat = quat/norm(quat);
    end

end