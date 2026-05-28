% Tier 2: Complementary Filter Reference Implementation
%
% Sensor fusion combining odometry and IMU measurements.

function [x, y, theta] = complementary_filter(odom_vel, imu_data, params, x0, y0, theta0)
    % COMPLEMENTARY_FILTER Fuse odometry and IMU
    %
    % Inputs:
    %   odom_vel - Nx3 matrix [vx, vy, omega] from odometry
    %   imu_data - Nx3 matrix [ax, ay, gyro_z] from IMU
    %   params - struct with filter parameters
    %   x0, y0, theta0 - initial pose
    %
    % Outputs:
    %   x, y, theta - position and heading arrays

    % Parameters
    alpha = params.alpha;         % Filter coefficient (0.95-0.98)
    dt = params.dt;               % Time step
    gyro_bias = params.gyro_bias; % Gyroscope bias (rad/s)

    % Initialize
    N = size(odom_vel, 1);
    x = zeros(N+1, 1);
    y = zeros(N+1, 1);
    theta = zeros(N+1, 1);

    x(1) = x0;
    y(1) = y0;
    theta(1) = theta0;

    % Fused velocity estimate
    vx_fused = 0;
    vy_fused = 0;

    for i = 1:N
        % Odometry velocities
        vx_odom = odom_vel(i, 1);
        vy_odom = odom_vel(i, 2);
        omega_odom = odom_vel(i, 3);

        % IMU measurements
        ax = imu_data(i, 1);
        ay = imu_data(i, 2);
        gyro_z = imu_data(i, 3) - gyro_bias;

        % Complementary filter for heading.
        % High-pass: gyro for short-term yaw.
        % NOTE: the BNO055 magnetometer is deliberately disabled on
        % Open Omnibot (motor magnetic interference within ~10 cm of
        % the IMU mount). The IMU runs in IMUPLUS mode, not NDOF.
        % Do not introduce magnetometer fusion in this loop.
        theta_gyro = theta(i) + gyro_z * dt;
        theta(i+1) = alpha * theta_gyro + (1 - alpha) * theta(i);

        % Complementary filter for velocity
        % Integrate accelerometer (body frame)
        vx_imu = vx_fused + ax * dt;
        vy_imu = vy_fused + ay * dt;

        % Fuse odometry and IMU velocities
        vx_fused = alpha * vx_odom + (1 - alpha) * vx_imu;
        vy_fused = alpha * vy_odom + (1 - alpha) * vy_imu;

        % Transform to world frame and integrate position
        ct = cos(theta(i+1));
        st = sin(theta(i+1));

        x(i+1) = x(i) + (vx_fused * ct - vy_fused * st) * dt;
        y(i+1) = y(i) + (vx_fused * st + vy_fused * ct) * dt;

        % Normalize theta
        theta(i+1) = atan2(sin(theta(i+1)), cos(theta(i+1)));
    end
end

%% Example usage — values match the Open Omnibot reference platform.
% params.alpha     = 0.98;
% params.dt        = 0.05;   % 20 Hz sensor stream
% params.gyro_bias = 0.001;  % rad/s, measured at startup
%
% [x, y, theta] = complementary_filter(odom_vel, imu_data, params, 0, 0, 0);
