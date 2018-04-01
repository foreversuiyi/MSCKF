function imu_states_up = UpdateHistory(imu_states, msckf_state, cam, state_idx)

%% Store the current msckf state
% IMU State
imu_states_up = imu_states;
imu_states_up{state_idx}.quat = msckf_state.imu_state.quat;
imu_states_up{state_idx}.pos = msckf_state.imu_state.pos;

c_R_v = Quat2Rot(cam.c_quat_v);
v_quat_c = Rot2Quat(c_R_v');
c_P_v = - c_R_v * cam.v_P_c;

% Update Other imu_states in the msckf
for cam_idx = 1:size(msckf_state.cam_states, 2)
    v_quat_o = LeftQuatMulti(v_quat_c) * msckf_state.cam_states{cam_idx}.quat;
    v_R_o = Quat2Rot(v_quat_o);
    o_P_c = msckf_state.cam_states{cam_idx}.pos;
    o_P_v = o_P_c + v_R_o'*c_R_v'*c_P_v;
    tmp_idx = msckf_state.cam_states{cam_idx}.idx;
    imu_states_up{tmp_idx}.quat = v_quat_o;
    imu_states_up{tmp_idx}.pos = o_P_v;
end

end