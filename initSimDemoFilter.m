function filter = initSimDemoFilter(detection)
% initSimDemoFilter.m
% Khởi tạo bộ lọc trackingUKF cho trạng thái [x; y; v; theta]
% Phù hợp để dùng với multiObjectTracker

% === Lấy thông tin đo lường từ detection ===
position = detection.Measurement(1:2);         % [x, y]
velocity = detection.Measurement(3:4);         % [vx, vy]
v0 = norm(velocity);                           % tính tốc độ
theta0 = atan2(velocity(2), velocity(1));      % tính hướng chuyển động

% === Trạng thái ban đầu ===
initialState = [position(1); position(2); v0; theta0];

% === Hàm chuyển trạng thái (motion model) ===
f = @(x, dt)[
    x(1) + dt * x(3) * cos(x(4));   % x(t+1)
    x(2) + dt * x(3) * sin(x(4));   % y(t+1)
    x(3);                           % tốc độ giữ nguyên
    x(4)                            % hướng giữ nguyên
];

% === Hàm đo lường: từ [x; y; v; theta] → [x; y; vx; vy] ===
h = @(x, varargin)[
    x(1);
    x(2);
    x(3) * cos(x(4));
    x(3) * sin(x(4))
];

% === Tạo bộ lọc trackingUKF ===
filter = trackingUKF(...
    'StateTransitionFcn', f, ...
    'MeasurementFcn', h, ...
    'State', initialState, ...
    'StateCovariance', diag([0.5, 0.5, 2, 0.5]), ...
    'ProcessNoise', diag([0.1, 0.1, 0.5, 0.2]), ...
    'MeasurementNoise', diag([0.5, 0.5, 1, 1]), ...
    'Alpha', 1e-2);
end
