function [msckf_state, lm_obs, obs_lm_ids] = InitMSCKF(first_state, first_measure, cam, state_idx, imu_cov)

%% Initialization of the MSCKF with landmarks and ground truth
% The first msckf_state
msckf_state.imu_state = first_state;
msckf_state.imu_cov = imu_cov;
msckf_state.cam_cov = [];
msckf_state.imucam_cov = [];
msckf_state.cam_states = {};

msckf_state = AddCamState(msckf_state, cam, state_idx);

% Compute all the landmarks
lm_obs = {};
obs_lm_ids = [];

for idx = 1:size(first_measure.cam, 2)
    cam_ob = first_measure.cam(:,idx);
    if ~isnan(cam_ob(1,1))
        lm.lm_id = idx;
        lm.obs = cam_ob;
        lm_obs{end+1} = lm;
        obs_lm_ids(end+1) = idx;
        msckf_state.cam_states{end}.obs_lm_ids(end+1) = idx;
    end
end
end
    