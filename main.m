%% clear
clear
clc
% close all

%% init_params
dt = 0.001;
Max_Step_Counter = 50000;
T_max = Max_Step_Counter * dt;
Step_Counter = 0;
Real_Timer = zeros(1, Max_Step_Counter);
Sim_Timer = zeros(1, Max_Step_Counter);
t = 0;

M = 1; % Mass of cart (kg)
m = 1; % Mass of pole (kg)
l = 0.5; % Length of pole (m)
h = 1.1; % Custom parameter for target angle calculation
g = 9.81; % Gravitational acceleration (m/s^2)

% Initial conditions
x0 = 0;    % Cart position (m)
xd0 = 0;   % Cart velocity (m/s)
xdd0 = 0;  % Cart acceleration (m/s^2)
xdd = 0;

p0 = 0;    % Pole angle (rad)
pd0 = 0;   % Pole angular velocity (rad/s)
pdd0 = 0;  % Pole angular acceleration (rad/s^2)
pdd = 0;

xs = 1; % Desired cart position (m)
ps = 0; % Desired pole angle (rad)

X0 = [x0; xd0; xdd0; p0; pd0; pdd0];
Xd = [xs; ps];

X = zeros(size(X0, 1), Max_Step_Counter);
X(:, 1) = X0;

% PID parameters for pole angle
Kp_p = 2;  % Proportional gain for pole angle
Ki_p = 1;   % Integral gain for pole angle
Kd_p = 5;   % Derivative gain for pole angle

integral_error_p = 0;
prev_error_p = 0;
derivative_error_p = 0;

%% loop
tic
while t < T_max
    Step_Counter = Step_Counter + 1;
    Real_Timer(Step_Counter) = toc;
    t = Step_Counter * dt;
    Sim_Timer(Step_Counter) = t;
    
    % Read states
    x = X(1, Step_Counter);
    xd = X(2, Step_Counter); 
    p = X(4, Step_Counter);
    pd = X(5, Step_Counter); 

    % Desired pole angle based on cart position error
    error_x = xs - x;
    ps = atan(error_x / h); % Dynamically calculated target angle
    ps = sign(ps) * min(0.2, abs(ps))
    % Error for pole angle
    error_p = ps - p;

    % Update integral error
    integral_error_p = integral_error_p + error_p * dt

    % Derivative error
    derivative_error_p = -1 * pd / dt

    % PID control force
    F_p = Kp_p * error_p + Ki_p * integral_error_p + Kd_p * derivative_error_p;

    % F_p = -sign(F_p) * min(15, abs(F_p))
    F_p = -F_p

    % Update previous error
    prev_error_p = error_p;

    % Simulate dynamics
    xdd = (F_p - m * l * pdd * cos(p) + m * pd^2 * sin(p)) / (M + m);
    pdd = (g * sin(p) - cos(p) * xdd) / cos(p);

    % Update velocities and positions
    xd = xd + dt * xdd;
    pd = pd + dt * pdd;
    x = x + dt * xd;
    p = p + dt * pd;

    % Store states
    X(:, Step_Counter + 1) = [x; xd; xdd; p; pd; pdd];

    % Report
    if mod(Step_Counter, 113) == 0
        % Visualize cart and pole
        figure(1);
        clf;
        hold on;
        axis equal;
        xlim([-3, 4]);
        ylim([-0.6, 1.1]);

        % Draw cart
        rectangle('Position', [x - 0.1, -0.05, 0.2, 0.1], 'FaceColor', 'blue');

        % Draw pole
        pole_x = x + l * sin(p);
        pole_y = l * cos(p);
        plot([x, pole_x], [0, pole_y], 'k-', 'LineWidth', 2);

        % Draw pivot point
        plot(x, 0, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'black');

        % Draw pendulum mass (ball)
        plot(pole_x, pole_y, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'red');

        title('Cart and Pole Simulation');
        xlabel('Cart Position (m)');
        ylabel('Pole Position (m)');
        drawnow;
    end

    % Stop condition
    if Step_Counter >= Max_Step_Counter
        break;
    end
end

%% Finalize
plot(Sim_Timer(1:Step_Counter), X(4, 1:Step_Counter), 'r', 'DisplayName', 'Pole Angle');
xlabel('Time (s)');
ylabel('Pole Angle (rad)');
title('Pole Angle over Time');
legend;
