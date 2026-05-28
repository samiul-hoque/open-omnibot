% Tier 1: Wheel Odometry Reference Implementation
%
% Dead reckoning using wheel encoder measurements for
% a 4-wheel mecanum omnidirectional robot.

function [x, y, theta] = odometry(encoder_counts, params, x0, y0, theta0)
    % ODOMETRY Compute position from wheel encoders
    %
    % Inputs:
    %   encoder_counts - Nx4 matrix of encoder counts [w1, w2, w3, w4]
    %   params - struct with robot parameters
    %   x0, y0, theta0 - initial pose
    %
    % Outputs:
    %   x, y, theta - position and heading arrays

    % Robot parameters
    r = params.wheel_radius;      % Wheel radius (m)
    cpr = params.encoder_cpr;     % Counts per revolution
    lx = params.wheel_lx;         % Wheel x distance from center
    ly = params.wheel_ly;         % Wheel y distance from center
    dt = params.dt;               % Time step (s)

    % Initialize state
    N = size(encoder_counts, 1);
    x = zeros(N+1, 1);
    y = zeros(N+1, 1);
    theta = zeros(N+1, 1);

    x(1) = x0;
    y(1) = y0;
    theta(1) = theta0;

    % Process each measurement
    for i = 1:N
        % Convert encoder counts to wheel velocities (rad/s).
        % cpr is counts per wheel revolution (post-gearbox), so this
        % already accounts for the motor-to-wheel gear ratio.
        w = encoder_counts(i, :) * (2 * pi / cpr) / dt;

        % Mecanum forward kinematics: body-frame velocity from
        % wheel angular velocities in wire order [L1, R1, R2, L2].
        % Conventions: vx forward, vy body-left, wz CCW.
        vx = (w(1) + w(2) + w(3) + w(4)) * r / 4;
        vy = (w(1) - w(2) + w(3) - w(4)) * r / 4;
        wz = (-w(1) + w(2) + w(3) - w(4)) * r / (4 * (lx + ly));

        % Transform to world frame and integrate
        ct = cos(theta(i));
        st = sin(theta(i));

        x(i+1) = x(i) + (vx * ct - vy * st) * dt;
        y(i+1) = y(i) + (vx * st + vy * ct) * dt;
        theta(i+1) = theta(i) + wz * dt;

        % Normalize theta to [-pi, pi]
        theta(i+1) = atan2(sin(theta(i+1)), cos(theta(i+1)));
    end
end

%% Example usage — values match the Open Omnibot reference platform.
% params.wheel_radius = 0.04;   % 40 mm
% params.encoder_cpr  = 1092;   % counts per wheel revolution
% params.wheel_lx     = 0.1175; % half-track width (m)
% params.wheel_ly     = 0.0953; % half-wheelbase (m)
% params.dt           = 0.05;   % 20 Hz sensor stream
%
% [x, y, theta] = odometry(encoder_data, params, 0, 0, 0);
% plot(x, y); axis equal;
