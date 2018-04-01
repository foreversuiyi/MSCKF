function [pos, cost, RCOND] = GNPointEst(cam_states, cam_obs, cov)
%% Calculate Position of Landmark Through Multiview Observations
% Initial estimate through triangulation using the first and last cam_states
last_idx = length(cam_states);
f_R_l = Quat2Rot(cam_states{1}.quat)*Quat2Rot(cam_states{last_idx}.quat)';
f_P_l = Quat2Rot(cam_states{1}.quat)*(cam_states{last_idx}.pos - cam_states{1}.pos);
init_pos = triangulate(cam_obs(:,1), cam_obs(:,last_idx), f_R_l, f_P_l);

%% TESTING
xBar = init_pos(1);
yBar = init_pos(2);
zBar = init_pos(3);

alphaBar = xBar/zBar;
betaBar = yBar/zBar;
rhoBar = 1/zBar;
est_pos = [alphaBar; betaBar; rhoBar];
%% TESTING
num_cams = length(cam_states);
% Optimize
maxIter = 10;
pre_cost = Inf;

for optI = 1:maxIter
    Jacobian = zeros(2*num_cams, 3);
    Sigma = zeros(2*num_cams, 2*num_cams);
    error = zeros(2*num_cams, 1);
    
    for istate = 1:num_cams
        % Weighted Matrix
        Sigma((2*istate-1):(2*istate),(2*istate-1):(2*istate)) = diag([cov]);
        
        i_R_1 = Quat2Rot(cam_states{istate}.quat)*Quat2Rot(cam_states{1}.quat)';
        i_P_1 = Quat2Rot(cam_states{istate}.quat)*(cam_states{1}.pos-cam_states{istate}.pos);
        
        % Error
        z_obs = cam_obs(1:2,istate);
        h = i_R_1 * [alphaBar; betaBar; 1] + rhoBar*i_P_1;
        error((2*istate-1):(2*istate),1) = z_obs - [h(1);h(2)]/h(3);
        
        % Jacobian
        jac = [-i_R_1(1,1)/h(3)+i_R_1(3,1)*h(1)/h(3)^2, -i_R_1(1,2)/h(3)+i_R_1(3,2)*h(1)/h(3)^2,...
               -i_P_1(1)/h(3)+i_P_1(3)*h(1)/h(3)^2;...
               -i_R_1(2,1)/h(3)+i_R_1(3,1)*h(2)/h(3)^2, -i_R_1(2,2)/h(3)+i_R_1(3,2)*h(2)/h(3)^2,...
               -i_P_1(2)/h(3)+i_P_1(3)*h(2)/h(3)^2;];
        Jacobian((2*istate-1):(2*istate),:) = jac;
    end
    
    % Cost Function
    cost = 0.5*error'*(Sigma\error);
    
    % Solve the Equation
    H = Jacobian'*(Sigma\Jacobian);
    RCOND = rcond(H);
    b = Jacobian'*(Sigma\error);
    delta_pos = -H\b;
    est_pos = est_pos + delta_pos;
    
    delta_cost = abs((cost - pre_cost)/cost);
    pre_cost = cost;
    
    if delta_cost < 1e-2
        break;
    else
        alphaBar = est_pos(1);
        betaBar = est_pos(2);
        rhoBar = est_pos(3);
    end
end
pos = (1/est_pos(3))*Quat2Rot(cam_states{1}.quat)'*[est_pos(1:2);1] + cam_states{1}.pos;

function init_pos = triangulate(obsf, obsl, f_R_l, f_P_l)
%% Triangulate 3D points from two sets of observations and transformation
obsf = [obsf;1];
obsl = [obsl;1];
vf = obsf/norm(obsf);
vl = obsl/norm(obsl);
translate = [vf, -f_R_l*vl];
scale_const = translate\f_P_l;
init_pos = scale_const(1)*vf;
end
end