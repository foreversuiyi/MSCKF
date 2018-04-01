function msckf_state_prop = PropagateMSCKF(msckf_state, measure, imu_cov)
%% Motion Model Propagation of the MSCKF
% Motion Model Jacobians
ts = measure.ts;
[F, G] = CalFG(msckf_state.imu_state, measure);

% Propagate State
msckf_state_prop.imu_state = PropagateIMU(msckf_state.imu_state, measure);

% Propagate Covariance
phi = eye(size(F,1)) + F*ts;
prop_cov = phi*msckf_state.imu_cov*phi' + G*imu_cov*G'*ts;

% Force Positive Definite
for r = 1:size(prop_cov,1)
    for c = 1:size(prop_cov,2)
        if r == c
            prop_cov(r,c) = abs(prop_cov(r,c));
        else
             offDiagElement = mean([prop_cov(r,c),prop_cov(c,r)]);
             prop_cov(c,r) = offDiagElement;
             prop_cov(r,c) = offDiagElement;
        end
    end
end

% Update the Covariance
msckf_state_prop.imu_cov = prop_cov;
msckf_state_prop.cam_cov = msckf_state.cam_cov;
msckf_state_prop.imucam_cov = phi*msckf_state.imucam_cov;
msckf_state_prop.cam_states = msckf_state.cam_states;

    function [F,G] = CalFG(imu_state, measure)
    %% Error State Model F, G
    F = zeros(12, 12);

    omega_hat = measure.omega - imu_state.gb;
    vel_hat = measure.vel - imu_state.vb;
    v_R_o = Quat2Rot(imu_state.quat);

    F(1:3,1:3) = -Vec2Skew(omega_hat);
    F(1:3,4:6) = -eye(3);
    F(10:12,1:3) = -v_R_o' * Vec2Skew(vel_hat);
    F(10:12,7:9) = -v_R_o';

    G = zeros(12, 12);
    G(1:3,1:3) = -eye(3);
    G(4:6,4:6) = eye(3);
    G(7:9,10:12) = eye(3);
    G(10:12,7:9) = -v_R_o';
    end

    function imu_state_prop = PropagateIMU(imu_state, measure)
    %% IMU State Propagation
    ts = measure.ts;
    v_R_o = Quat2Rot(imu_state.quat);
    ang = (measure.omega - imu_state.gb) * ts;
    Omega = [ -Vec2Skew(ang),  ang; -ang', 0 ];
    % Quaternion Propagation
    imu_state_prop.quat = imu_state.quat + 0.5*Omega*imu_state.quat;
    imu_state_prop.quat = imu_state_prop.quat/norm(imu_state_prop.quat);

    % Bias Propagation
    imu_state_prop.gb = imu_state.gb;
    imu_state_prop.vb = imu_state.vb;

    % Position Propagation
    trans = (measure.vel - imu_state.vb)*ts;
    imu_state_prop.pos = v_R_o' * trans + imu_state.pos;
    end

end