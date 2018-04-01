function msckf_state_up = AddCamState(msckf_state, cam, state_idx)

%% Add the new camera state to the MSCKF state vector
v_R_o = Quat2Rot(msckf_state.imu_state.quat);
c_quat_o = LeftQuatMulti(cam.c_quat_v) * msckf_state.imu_state.quat;
o_P_c = msckf_state.imu_state.pos + v_R_o' * cam.v_P_c;

% MSCKF Covariance Matrix
Sigma = [msckf_state.imu_cov, msckf_state.imucam_cov; msckf_state.imucam_cov', msckf_state.cam_cov];

% Camera State Jacobian
c_R_v = Quat2Rot(cam.c_quat_v);
jac = zeros(6, 12 + 6*size(msckf_state.cam_states, 2));
jac(1:3,1:3) = c_R_v;
jac(4:6,1:3) = Vec2Skew(v_R_o' * cam.v_P_c);
jac(4:6,10:12) = eye(3);

% Covariance Matrix Update
N = size(msckf_state.cam_states, 2);
tmp = [eye(12+6*N); jac];
Sigma_up = tmp * Sigma * tmp';

% MSCKF State and Covariance Update
msckf_state_up = msckf_state;
msckf_state_up.cam_states{N+1}.pos = o_P_c;
msckf_state_up.cam_states{N+1}.quat = c_quat_o;
msckf_state_up.cam_states{N+1}.idx = state_idx;
msckf_state_up.cam_states{N+1}.obs_lm_ids = [];
msckf_state_up.imu_cov = Sigma_up(1:12,1:12);
msckf_state_up.cam_cov = Sigma_up(13:end,13:end);
msckf_state_up.imucam_cov = Sigma_up(1:12,13:end);

end