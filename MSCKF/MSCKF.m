clear;
close all;
clc;
addpath('tools')
fileName = '2011_09_26_drive_0039_sync_KLT.mat';
load(fileName);

%% Initialize Parameters
% Camera Parameters
cam.cx = cx;
cam.cy = cy;
cam.fx = fx;
cam.fy = fy;
cam.c_quat_v = Rot2Quat(c_R_v);
cam.v_P_c = v_P_c;
cam.cov = 121*ones(2,1);
cam.x_cov = cam.cov(1)/cam.fx^2;
cam.y_cov = cam.cov(2)/cam.fy^2;
cam.K = [cam.fx, 0, cam.cx; 0, cam.fy, cam.cy; 0, 0, 1];
% IMU Parameters
imu.wcov = 4e-2*ones(3,1);
imu.gbcov = 1e-6*ones(3,1);
imu.vbcov = 1e-6*ones(3,1);
imu.vcov = 4e-2*ones(3,1);
imu.cov = diag([imu.wcov; imu.gbcov; imu.vcov; imu.vbcov]);
imu.init_wcov = 1e-6*ones(3,1);
imu.init_gbcov = 1e-6*ones(3,1);
imu.init_vbcov = 1e-6*ones(3,1);
imu.init_vcov = 1e-6*ones(3,1);
imu.init_cov = diag([imu.init_wcov; imu.init_gbcov; imu.init_vbcov; imu.init_vcov]);
% MSCKF Parameters
msckf.min_len = 10;
msckf.max_len = Inf;
msckf.max_GNcost = 1e-2;
msckf.min_RCOND = 1e-12;
start_idx = 2;
end_idx = size(all_cam_obs, 2) - msckf.min_len -1;

%% Measurements Loading
% ===== measure: ts cam omega vel =====
ts = [0, diff(time_stamp)];
measure = cell(1,length(time_stamp));
all_cam_obs(all_cam_obs == -1) = NaN;
for sidx = start_idx:end_idx
    measure{sidx}.ts = ts(sidx);
    measure{sidx}.cam = squeeze(all_cam_obs(1:2, sidx, :));
    valid_measures = ~isnan(measure{sidx}.cam(1,:));
    % Turn the Raw Camera Observation to normalized [x; y; 1];
    measure{sidx}.cam(1,valid_measures) = (measure{sidx}.cam(1, valid_measures) - cam.cx)/cam.fx;
    measure{sidx}.cam(2,valid_measures) = (measure{sidx}.cam(2, valid_measures) - cam.cy)/cam.fy;
    measure{sidx}.omega = all_imu_measure.omega(:,sidx);
    measure{sidx}.vel = all_imu_measure.vel(:,sidx);
    % GroundTruth true_state
    v_R_o = AxisAng2Rot(groundtruth.ang_axis(:,sidx));
    v_quat_o = Rot2Quat(v_R_o);
    true_state{sidx}.quat = v_quat_o;
    true_state{sidx}.pos = groundtruth.pos(:,sidx);
    true_state{sidx}.cam_pos = groundtruth.pos(:,sidx) + v_R_o'*cam.v_P_c;
    true_state{sidx}.cam_quat = LeftQuatMulti(cam.c_quat_v)*v_quat_o;
end

%% Initialize the First MSCKF States
lm_obs = {}; % landmarks observations
obs_lm_ids = []; % Observed landmarks index
% ===== first imu state: quat pos gb vb =====
first_state.quat = Rot2Quat(AxisAng2Rot(groundtruth.ang_axis(:, start_idx)));
first_state.pos = groundtruth.pos(:,start_idx);
first_state.gb = zeros(3,1);
first_state.vb = zeros(3,1);
% ===== msckf_state: imu_state cam_states imu_cov cam_cov imucam_cov =====
[msckf_state, lm_obs, obs_lm_ids] = InitMSCKF(first_state, measure{start_idx}, cam, start_idx, imu.init_cov);
% Log the Calculated State of MSCKF
imu_states = cell(1,length(time_stamp));  %% Calculated the MSCKF IMU state update
pruned_states = {};  %% Camera States which have done correction
imu_states = UpdateHistory(imu_states, msckf_state, cam, start_idx);
% Log the IMU only Calculated State
imuonly_state{start_idx} = msckf_state;

%% MSCKF Iterative Update
num_lm_cor = 0;  % number of landmarks in the correction step
map = [];  % landmark positions in the correction step
num_lms = size(all_cam_obs, 3); % total number of landmarks

%% Plot
% fps = 1;
% video_name = 'msckf.avi';
% writerObj = VideoWriter(video_name);
% writerObj.open();
% fig_handle = figure();
% set(gcf, 'color', 'white')
t_start = clock;
for sidx = start_idx:(end_idx -1)
    fprintf('State index = %d\n', sidx)
    %% Propagate State Using IMU Motion Model
    msckf_state = PropagateMSCKF(msckf_state, measure{sidx}, imu.cov);
    imuonly_state{sidx+1} = PropagateMSCKF(imuonly_state{sidx}, measure{sidx}, imu.cov);
    msckf_state = AddCamState(msckf_state, cam, sidx+1);
    
    %% Loading Landmark Observations
    % Select qualified landmarks to generate observation correction
    lms_cor = {};
    % Find in all landmarks
    for lm_idx = 1:num_lms
        % Load the measurement of the landmark in current camera state
        idx_measure = measure{sidx+1}.cam(:,lm_idx);
        out_of_view = isnan(idx_measure(1,1));
        
        % If the landmark has been observed
        if ismember(lm_idx, obs_lm_ids)
            if ~out_of_view
                % Put the new observation in the lm_obs
                lm_obs{obs_lm_ids == lm_idx}.obs(:,end+1) = idx_measure;
                % Update the camera observations in the MSCKF
                msckf_state.cam_states{end}.obs_lm_ids(end+1) = lm_idx;
            end
            % Get all the observations of this landmark
            tmp_lms = lm_obs{obs_lm_ids == lm_idx};
            
            if out_of_view || size(tmp_lms.obs, 2) >= msckf.max_len || sidx+1 == end_idx
                % Remove the landmark from the MSCKF cam_states
                [msckf_state, include_cam_states, include_cam_states_idx] = RemoveLms(msckf_state, lm_idx);
                % Put the landmark in to the correction list
                if length(include_cam_states) >= msckf.min_len
                    tmp_lms.cam_states = include_cam_states;
                    tmp_lms.cam_states_idx = include_cam_states_idx;
                    lms_cor{end+1} = tmp_lms;
                end
                % Remove the landmark from the observations
                lm_obs = lm_obs(obs_lm_ids ~= lm_idx);
                obs_lm_ids(obs_lm_ids == lm_idx) = [];
            end
            
        % If the landmark has not been observed
        elseif ~out_of_view && sidx+1 < end_idx
            tmp_lms.lm_id = lm_idx;
            tmp_lms.obs = idx_measure;
            % Put the landmark in the landmark observations
            lm_obs{end+1} = tmp_lms;
            % Log the landmark id
            obs_lm_ids(end+1) = lm_idx;
            % Update the camera observations in the MSCKF
            msckf_state.cam_states{end}.obs_lm_ids(end+1) = lm_idx;
        end
    end
    
    %% Landmark Observation Correction
    if ~isempty(lms_cor)
        Jaco = []; % error jacobian
        error_o = []; % error between prediction and measurement
        Ro = []; % measurement covariance
        
        % Using all the correction landmarks to update the msckf_state
        for res_idx = 1:length(lms_cor)
            
            tmp_lms = lms_cor{res_idx}; % Include all the cam_state which has observed this landmark
            
            % Triangulate the landmark position
            [lm_pos, Jcost, RCOND] = GNPointEst(tmp_lms.cam_states, tmp_lms.obs, [cam.x_cov; cam.y_cov]);
            num_obs = size(tmp_lms.obs, 2);
            if Jcost/num_obs^2 > msckf.max_GNcost || RCOND < msckf.min_RCOND
                break;
            else
                map(:,end+1) = lm_pos;
                num_lm_cor =  num_lm_cor + 1;
            end
            
            % Calculate Error and Jacobian
            [obs_error, Jacoj, Jacxj, Aj] = CalErrorJac(lm_pos, tmp_lms, msckf_state);
            
            % Measurement Covariance
            Rj = diag(repmat([cam.x_cov, cam.y_cov], [1, numel(obs_error)/2]));
            
            % Stacked Errors and Covariance
            Jaco = [Jaco;Jacoj];
            if ~isempty(Aj)
                error_oj = Aj'*obs_error;
                error_o = [error_o; error_oj];
                Roj = Aj'*Rj*Aj;
                Ro(end+1:end+size(Roj, 1), end+1:end+size(Roj,2)) = Roj;
            end
        end
        
        if ~isempty(error_o)
            % Final update
            
            % QR Decomposition
            [d_Q, d_R] = qr(Jaco);
            zeros_row = all(d_R==0, 2);
            T_H = d_R(~zeros_row, :);
            Q_1 = d_Q(:, ~zeros_row);
            error_n = Q_1' * error_o;
            Rn = Q_1' * Ro * Q_1;
            
            % Build MSCKF Covariance Matirx
            P = [msckf_state.imu_cov, msckf_state.imucam_cov;msckf_state.imucam_cov', msckf_state.cam_cov];
            
            % Calculate the Kalman Gain
            K = (P*T_H')/(T_H*P*T_H' + Rn);
            
            % State Correction
            deltax = K*error_n;            
            msckf_state = UpdateState(msckf_state, deltax);
            
            % Covariance Correction
            tmp_mat = eye(12 + 6*size(msckf_state.cam_states, 2)) - K*T_H;
            P_cor = tmp_mat*P*tmp_mat' + K*Rn*K';   
            msckf_state.imu_cov = P_cor(1:12,1:12);
            msckf_state.imucam_cov = P_cor(1:12,13:end);
            msckf_state.cam_cov = P_cor(13:end,13:end);
        end
    end
    
    % LOG the MSCKF State Update History
    imu_states = UpdateHistory(imu_states, msckf_state, cam, sidx+1);
    [msckf_state, del_cam_states] = PruneState(msckf_state);
    
    % Remove Camera States Done State Estimation
    if ~ isempty(del_cam_states)
        pruned_states(end+1:end+length(del_cam_states)) = del_cam_states;
    end
    
    %% Plot Trajectory
    knum = length(pruned_states);
    if knum > 0
        est_pos = zeros(3, knum);
        imu_pos = zeros(3, knum);
        kplot = zeros(1, knum);        
        for k = 1:knum
            kidx = pruned_states{k}.idx;
            c_R_o = Quat2Rot(pruned_states{k}.quat);
            est_pos(:,k) = pruned_states{k}.pos - c_R_o'*c_R_v*cam.v_P_c;
            imu_pos(:,k) = imuonly_state{kidx}.imu_state.pos;
            kplot(k) = kidx;
        end     
    end
    est_state{sidx}.pos = msckf_state.imu_state.pos;
    est_state{sidx}.quat = msckf_state.imu_state.quat;
%     figure(fig_handle); 
%     hold on;
%     plot3(msckf_state.imu_state.pos(1), msckf_state.imu_state.pos(2,:), msckf_state.imu_state.pos(3,:), 'ob');
%     plot3(imuonly_state{sidx}.imu_state.pos(1), imuonly_state{sidx}.imu_state.pos(2,:), imuonly_state{sidx}.imu_state.pos(3,:), 'or');
%     plot3(groundtruth.pos(1,sidx), groundtruth.pos(2,sidx), groundtruth.pos(3,sidx), 'og');
%     xlabel('x'); ylabel('y'); zlabel('z');
%     legend('MSCKF Estimation', 'IMU Only', 'Groundtruth');
%     grid on;
%     axis([0, 150, -150, 0, 0, 6])
%     view(3)
%     if mod(sidx, fps) == 0
%             frame = getframe(fig_handle);
%             writerObj.writeVideo(frame);
%     end
end
% writerObj.close();
% close(fig_handle);
t_end = clock;
duration = etime(t_end, t_start);
%% Calculate Error
knum = length(pruned_states);
mse_idx = ones(1, knum);
for k = 1:knum
    state_idx = pruned_states{k}.idx;
    real_cam_pos(:,k) = true_state{state_idx}.cam_pos;
    est_cam_pos(:,k) = pruned_states{k}.pos;
    imu_cam_pos(:,k) = imuonly_state{state_idx}.imu_state.pos + Quat2Rot(imuonly_state{state_idx}.imu_state.quat)'*cam.v_P_c;
    mse_idx(k) = state_idx;
end
cam_pos_err = est_cam_pos - real_cam_pos;
cam_pos_mse = sqrt(mean(cam_pos_err.^2));

imu_cam_pos_err = imu_cam_pos - real_cam_pos;
imu_cam_pos_mse = sqrt(mean(imu_cam_pos_err.^2));

save([fileName, 'msckf.mat'], 'duration', 'mse_idx', 'cam_pos_mse', 'imu_cam_pos_mse')

% 
% plot(mse_idx, cam_pos_mse, mse_idx, imu_cam_pos_mse);