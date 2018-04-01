function [obs_error, Jacoj, Jacxj, Aj] = CalErrorJac(lm_pos, tmp_lms, msckf_state)
%% Calculate the error and Jacobian of observation
cam_states = tmp_lms.cam_states;
obs = tmp_lms.obs;
cam_states_idx = tmp_lms.cam_states_idx;
obs_error = NaN(2*size(cam_states, 2), 1);

% Calculate the Error
for idx = 1:size(cam_states, 2)
    c_R_o = Quat2Rot(cam_states{idx}.quat);
    c_P_l = c_R_o * (lm_pos - cam_states{idx}.pos);
    z_pre = c_P_l(1:2)/c_P_l(3);
    obs_error(2*idx-1:2*idx) = obs(:,idx) - z_pre;
end

% Calculate the Error Jacobian
N = length(msckf_state.cam_states);
M = length(cam_states_idx);
Jaclj = zeros(2*M, 3);
Jacxj = zeros(2*M, 12 + 6*N);

cam_idx = 1;
for idx = cam_states_idx
    tmp_cam = msckf_state.cam_states{idx};
    c_R_o = Quat2Rot(tmp_cam.quat);
    c_P_l = c_R_o * (lm_pos - tmp_cam.pos);
    X = c_P_l(1); Y = c_P_l(2); Z = c_P_l(3);
    jac = (1/Z)*[1, 0, -X/Z; 0, 1, -Y/Z];
    Jaclj(2*cam_idx-1:2*cam_idx, :) = jac*c_R_o;
    Jacxj(2*cam_idx-1:2*cam_idx, 12+6*(idx-1)+1:12+6*(idx-1)+3) = jac*Vec2Skew(c_P_l);
    Jacxj(2*cam_idx-1:2*cam_idx, 12+6*(idx-1)+4:12+6*(idx-1)+6) = -jac*c_R_o;
    cam_idx = cam_idx + 1;
end

Aj = null(Jaclj');
Jacoj = Aj'*Jacxj;

end