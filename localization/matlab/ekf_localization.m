% Tier 3: Extended Kalman Filter Reference Implementation
%
% REFERENCE THEORY ONLY. This tier is not currently integrated in the
% firmware/server runtime. The UWB reader, EKF fusion, and anchor
% handling were removed from the runtime on 2026-04-14 pending
% validation of the lower tiers. This file is published as the
% planned reference math for a future release — not a drop-in
% working module.
%
% Full sensor fusion with odometry, IMU, and UWB measurements.

classdef ekf_localization < handle
    properties
        % State: [px, py, theta, vx, vy, omega]
        x           % State vector (6x1)
        P           % Covariance matrix (6x6)
        Q           % Process noise (6x6)
        R_uwb       % UWB measurement noise (scalar)
        R_imu       % IMU measurement noise (3x3)
        R_odom      % Odometry measurement noise (3x3)
        anchors     % UWB anchor positions (Mx2)
        dt          % Time step
    end

    methods
        function obj = ekf_localization(params)
            % Constructor
            obj.x = zeros(6, 1);
            obj.P = eye(6) * 0.1;
            obj.Q = params.Q;
            obj.R_uwb = params.R_uwb;
            obj.R_imu = params.R_imu;
            obj.R_odom = params.R_odom;
            obj.anchors = params.anchors;
            obj.dt = params.dt;
        end

        function predict(obj, u)
            % Prediction step
            % u = [ax, ay, alpha] control/acceleration input

            dt = obj.dt;
            px = obj.x(1); py = obj.x(2); theta = obj.x(3);
            vx = obj.x(4); vy = obj.x(5); omega = obj.x(6);

            ct = cos(theta);
            st = sin(theta);

            % Process model
            obj.x(1) = px + (vx * ct - vy * st) * dt;
            obj.x(2) = py + (vx * st + vy * ct) * dt;
            obj.x(3) = theta + omega * dt;
            obj.x(4) = vx + u(1) * dt;
            obj.x(5) = vy + u(2) * dt;
            obj.x(6) = omega + u(3) * dt;

            % Normalize theta
            obj.x(3) = atan2(sin(obj.x(3)), cos(obj.x(3)));

            % Jacobian of process model
            F = eye(6);
            F(1, 3) = (-vx * st - vy * ct) * dt;
            F(1, 4) = ct * dt;
            F(1, 5) = -st * dt;
            F(2, 3) = (vx * ct - vy * st) * dt;
            F(2, 4) = st * dt;
            F(2, 5) = ct * dt;
            F(3, 6) = dt;

            % Covariance prediction
            obj.P = F * obj.P * F' + obj.Q;
        end

        function update_uwb(obj, range, anchor_id)
            % UWB range measurement update

            ax = obj.anchors(anchor_id, 1);
            ay = obj.anchors(anchor_id, 2);
            px = obj.x(1);
            py = obj.x(2);

            % Predicted range
            dx = px - ax;
            dy = py - ay;
            range_pred = sqrt(dx^2 + dy^2);

            % Avoid division by zero
            if range_pred < 0.001
                range_pred = 0.001;
            end

            % Measurement Jacobian
            H = zeros(1, 6);
            H(1) = dx / range_pred;
            H(2) = dy / range_pred;

            % Innovation
            y = range - range_pred;

            % Kalman update
            S = H * obj.P * H' + obj.R_uwb;
            K = obj.P * H' / S;
            obj.x = obj.x + K * y;
            obj.P = (eye(6) - K * H) * obj.P;

            % Normalize theta
            obj.x(3) = atan2(sin(obj.x(3)), cos(obj.x(3)));
        end

        function update_imu(obj, imu_meas)
            % IMU measurement update — gyro only.
            %
            % imu_meas = [ax, ay, gyro_z]
            %
            % The accelerometer channels (ax, ay) are intentionally
            % ignored here. A full update would require gravity-vector
            % compensation against the current orientation estimate;
            % that is left as an extension for a future release.

            H_gyro = [0 0 0 0 0 1];
            y = imu_meas(3) - obj.x(6);

            S = H_gyro * obj.P * H_gyro' + obj.R_imu(3,3);
            K = obj.P * H_gyro' / S;
            obj.x = obj.x + K * y;
            obj.P = (eye(6) - K * H_gyro) * obj.P;
        end

        function update_odom(obj, odom_meas)
            % Odometry velocity measurement update
            % odom_meas = [vx, vy, omega] in body frame

            H = zeros(3, 6);
            H(1, 4) = 1;  % vx
            H(2, 5) = 1;  % vy
            H(3, 6) = 1;  % omega

            y = odom_meas - obj.x(4:6);

            S = H * obj.P * H' + obj.R_odom;
            K = obj.P * H' / S;
            obj.x = obj.x + K * y;
            obj.P = (eye(6) - K * H) * obj.P;
        end

        function [pos, cov] = get_position(obj)
            pos = obj.x(1:2);
            cov = obj.P(1:2, 1:2);
        end
    end
end

%% Example usage
% params.Q = diag([0.01, 0.01, 0.001, 0.1, 0.1, 0.01]);
% params.R_uwb = 0.1^2;
% params.R_imu = diag([0.1, 0.1, 0.01]);
% params.R_odom = diag([0.05, 0.05, 0.01]);
% params.anchors = [0, 0; 3, 0; 3, 3; 0, 3];  % 4 anchors
% params.dt = 0.01;
%
% ekf = ekf_localization(params);
%
% for i = 1:N
%     ekf.predict([0, 0, 0]);
%     ekf.update_odom(odom_data(i, :));
%     ekf.update_uwb(uwb_range(i), uwb_anchor_id(i));
%     [pos, cov] = ekf.get_position();
% end
