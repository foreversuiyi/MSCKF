function [msckf_state_pruned, del_cam_states] = PruneState(msckf_state)
%% Prunes any states that have no observed landmarks and done state estimation

msckf_state_pruned.imu_state = msckf_state.imu_state;
msckf_state_pruned.imu_cov = msckf_state.imu_cov;

% Finde camera states with no landmarks
del_idx = [];
for idx = 1:length(msckf_state.cam_states)
    if isempty(msckf_state.cam_states{idx}.obs_lm_ids)
        del_idx(end+1) = idx;
    end
end

% Prune the Camera States
del_cam_states = msckf_state.cam_states(del_idx);
msckf_state_pruned.cam_states = msckf_state.cam_states;
msckf_state_pruned.cam_states(del_idx) = [];
msckf_state_pruned.cam_states(cellfun(@isempty, msckf_state_pruned.cam_states)) = [];

state_idx = 1:size(msckf_state.cam_cov, 1);
cov_mask = true(1, numel(state_idx));

for idx = del_idx
    cov_mask(6*idx-5:6*idx) = false(1,6);
end

cov_idx = state_idx(cov_mask);
del_cov_idx = state_idx(~cov_mask);

msckf_state_pruned.cam_cov = msckf_state.cam_cov(cov_idx, cov_idx);
msckf_state_pruned.imucam_cov = msckf_state.imucam_cov(:, cov_idx);

% Log the Standard Variance of the deleted Cameara states
del_cam_cov = msckf_state.cam_cov(del_cov_idx, del_cov_idx);
del_cam_sig = sqrt(diag(del_cam_cov));

for idx = 1:size(del_cam_states, 2)
    del_cam_states{idx}.sigma = del_cam_sig(6*idx-5:6*idx);
end

end