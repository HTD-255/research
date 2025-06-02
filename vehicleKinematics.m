function x_next = vehicleKinematics(x, u, dt)
% Mô hình động học dạng bicycle model
% x = [x; y; theta; v] là trạng thái
% u = [a; delta] là điều khiển (gia tốc, góc lái)
% dt là chu kỳ mẫu

    L = 2.5;  % chiều dài trục cơ sở xe
    theta = x(3); 
    v = x(4);
    a = u(1); 
    delta = u(2);

    % Tính trạng thái tiếp theo
    x_next = zeros(4,1);
    x_next(1) = x(1) + v * cos(theta) * dt;              % x
    x_next(2) = x(2) + v * sin(theta) * dt;              % y
    x_next(3) = x(3) + v / L * tan(delta) * dt;          % theta
    x_next(4) = x(4) + a * dt;                           % v
end