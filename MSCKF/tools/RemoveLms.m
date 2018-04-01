function [msckf_state_up, include_cam_states, include_cam_states_idx] = RemoveLms(msckf_state, lm_idx)
%% Remove landmarks from current camera states and extract all camera states that include it

cam_states_up = msckf_state.cam_states;
include_cam_states = {};
include_cam_states_idx = [];
for idx = 1:length(cam_states_up)
    lm_id = find(lm_idx == cam_states_up{idx}.obs_lm_ids);
    if ~isempty(lm_id)
        cam_states_up{idx}.obs_lm_ids(lm_id) = [];
        include_cam_states_idx(end+1) = idx;
        include_cam_states{end+1} = cam_states_up{idx};
    end
end

msckf_state_up = msckf_state;
msckf_state_up.cam_states = cam_states_up;

end