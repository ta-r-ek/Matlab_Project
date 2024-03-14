clc;
clear;
close all;

% Take input for m1, m2, l01, l02, k1, and k2 from the user
m1 = input('Enter mass m1: ');
m2 = input('Enter mass m2: ');
l01 = input('Enter initial length l01: ');
l02 = input('Enter initial length l02: ');
k1 = input('Enter spring constant k1: ');
k2 = input('Enter spring constant k2: ');
T = input('Enter simulation time: ');

% Adjusted initial conditions with lateral velocity for the first mass
offset = [0, 1];
% Applying different initial conditions for enhanced chaotic motion
y1 = [0.8, 0.2, 0.3, pi / 6]; % First mass [velocity_x, angular_velocity, displacement_x, angle]
y2 = [0.4, 0.6, 0.15, pi / 10]; % Second mass with slightly different conditions
delta_t = 0.02;
simulation_duration = T+1;

% Figure and Axes Setup
fig = figure;
ax = axes('Parent', fig);
xlim(ax, [-3, 3]);
ylim(ax, [-3, 2]);
hold(ax, 'on');
grid(ax,'on');

% Update positions
point1 = update_position(y1(3), y1(4), l01, offset);
point2 = update_position(y2(3), y2(4), l02, point1);

% Update springs
[spring1_x, spring1_y] = calculate_spring(offset, point1, 25, 0.2, 0.5, 0.5);
[spring2_x, spring2_y] = calculate_spring(point1, point2, 20, 0.15, 0.4, 0.4);

% Graphical Objects
line1 = line('XData', spring1_x, 'YData', spring1_y, 'Color', 'b', 'Parent', ax);
line2 = line('XData', spring2_x, 'YData', spring2_y, 'Color', 'r', 'Parent', ax);
mass1 = scatter(ax, point1(1), point1(2), 'bo', 'filled', 'SizeData', 25*m1);
mass2 = scatter(ax, point2(1), point2(2), 'ro', 'filled', 'SizeData', 25*m2);

%% Before the simulation loop, initialize arrays for energies and time
time_array = 0:delta_t:simulation_duration-1;
kinetic_energy_array = zeros(size(time_array));
potential_energy_array = zeros(size(time_array));


index = 1;
% Simulation Loop
for t = 0:delta_t:simulation_duration-1
    
    % Update positions
    point1 = update_position(y1(3), y1(4), l01, offset);
    point2 = update_position(y2(3), y2(4), l02, point1);
    
    % Update springs
    [spring1_x, spring1_y] = calculate_spring(offset, point1, 25, 0.2, 0.5, 0.5);
    [spring2_x, spring2_y] = calculate_spring(point1, point2, 20, 0.15, 0.4, 0.4);
    
    % Update graphical objects
    set(line1, 'XData', spring1_x, 'YData', spring1_y);
    set(line2, 'XData', spring2_x, 'YData', spring2_y);
    set(mass1, 'XData', point1(1), 'YData', point1(2));
    set(mass2, 'XData', point2(1), 'YData', point2(2));
    title(ax,'Remaining Time',simulation_duration-t-1);
    
    % Calculate next state
    y1 = RK4_step(y1, 0, delta_t, l01, m1, k1, 9.81);
    y2 = RK4_step(y2, 0, delta_t, l02, m2, k2, 9.81);
    
    %% energy code
    v1 = sqrt(y1(1)^2 + (l01 + y1(3))^2 * y1(2)^2); % velocity of mass 1
    v2 = sqrt(y2(1)^2 + (l02 + y2(3))^2 * y2(2)^2); % velocity of mass 2
    KE1 = 0.5 * m1 * v1^2;
    KE2 = 0.5 * m2 * v2^2;
    
    h1 = offset(2) - (l01 + y1(3)) * cos(y1(4)); % height of mass 1 from the reference
    h2 = point1(2) - (l02 + y2(3)) * cos(y2(4)); % height of mass 2 from the reference
    PE1 = m1 * 9.81 * h1;
    PE2 = m2 * 9.81 * h2;
    
    kinetic_energy_array(index) = KE1 + KE2;
    potential_energy_array(index) = PE1 + PE2;
    
    index = index + 1;

    drawnow; % Update the figure window
    pause(0.001); % Pause to control the speed of the animation
end

%% plot the energy graph
figure;
plot(time_array, kinetic_energy_array , 'LineWidth', 2);
title('Time vs Kinetic Energy');
xlabel('Time (s)');
ylabel('Energy (J)');
grid on;

% Define necessary functions below the main code

function [spring_x, spring_y] = calculate_spring(start, finish, nodes, width, lead1, lead2)
    nodes = max(floor(nodes), 1);
    start = reshape(start, [], 1); % Ensuring column vector
    finish = reshape(finish, [], 1); % Ensuring column vector

    if isequal(start, finish)
        spring_x = [];
        spring_y = [];
        return;
    end

    length = norm(finish - start);
    u_t = (finish - start) / length;
    u_n = [0, -1; 1, 0] * u_t;

    p1 = start + lead1 * u_t;
    p2 = finish - lead2 * u_t;
    length = length - (lead1 + lead2);
    spring_coords = zeros(nodes + 2, 2);
    spring_coords(1, :) = p1';
    spring_coords(end, :) = p2';
    normal_dist = sqrt(max(0, width^2 - (length^2 / nodes^2))) / 2;

    for i = 1:nodes
        spring_coords(i + 1, :) = p1 + (length * (2 * i - 1) * u_t / (2 * nodes)) + (normal_dist * (-1)^i * u_n);
    end

    spring_x = spring_coords(:, 1);
    spring_y = spring_coords(:, 2);
end

function dy = G(y, ~, l0, m, k, g)
    x_d = y(1);
    theta_d = y(2);
    x = y(3);
    theta = y(4);

    x_dd = (l0 + x) * theta_d^2 - (k / m) * x + g * cos(theta);
    theta_dd = -2.0 / (l0 + x) * x_d * theta_d - g / (l0 + x) * sin(theta);

    dy = [x_dd; theta_dd; x_d; theta_d];
end

function y_next = RK4_step(y, t, dt, l0, m, k, g)
   
    k1 = G(y, t, l0, m, k, g);
    k2 = G(y + 0.5 * k1 * dt, t + 0.5 * dt, l0, m, k, g);
    k3 = G(y + 0.5 * k2 * dt, t + 0.5 * dt, l0, m, k, g);
    k4 = G(y + k3 * dt, t + dt, l0, m, k, g);
    
    y_next = y + dt * (k1 + 2*k2 + 2*k3 + k4) / 6;
end

function position = update_position(x, theta, l0, anchor_point)
    x_coord = anchor_point(1) + (l0 + x) * sin(theta);
    y_coord = anchor_point(2) - (l0 + x) * cos(theta);
    position = [max(x_coord), max(y_coord)];
end
