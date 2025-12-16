%% Setup and run simulation
clc;
clear;

% Build track
track = Track();

% Make car
car = Car();

% Load tire test data
car.load_tire_data("B1965run52.mat");

% Initialise simulator
sim = Simulator(car, track);

% Get initial conditions (starting coordinates)
x0 = track.x(1);
y0 = track.y(1);
Z0 = [x0, y0, 0, 0];    % State vector is Z = [x, y, vx, vy]

% Simulation constraints
tspan = linspace(0, 500, track.N_total * 5); % enough time for 5 laps

% Execute simution - run 5 laps (1 warm-up + 4 fast)
sol = sim.simulate(Z0, tspan);

%% Calculate Statistics
% Calculate lap times and statistics for the 4 fast laps
fprintf('\n=== Lap Times Summary ===\n');
warmup_lap_time = sol.lap_times(1);  % First lap time (warm-up)

% Calculate individual fast lap times (laps 2-5)
fast_lap_times = zeros(4, 1);
for i = 1:4
    fast_lap_times(i) = sol.lap_times(i+1) - sol.lap_times(i);
end

% Calculate total time for the 4 fast laps
total_fast_lap_time = sol.lap_times(5) - sol.lap_times(1);

fprintf('Warm-up lap (Lap 1): %.2f seconds\n', warmup_lap_time);
fprintf('\nFast lap times:\n');
for i = 1:4
    fprintf('Lap %d: %.2f seconds\n', i+1, fast_lap_times(i));
end

fprintf('\n=== Fast Laps Statistics ===\n');
fprintf('Total time for 4 fast laps: %.2f seconds\n', total_fast_lap_time);
fprintf('Average lap time: %.2f seconds\n', mean(fast_lap_times));
fprintf('Best lap time: %.2f seconds (Lap %d)\n', min(fast_lap_times), find(fast_lap_times == min(fast_lap_times), 1) + 1);
fprintf('Worst lap time: %.2f seconds (Lap %d)\n', max(fast_lap_times), find(fast_lap_times == max(fast_lap_times), 1) + 1);

% Calculate average speed for the fast laps
avg_speed = track.L_lap / mean(fast_lap_times);
fprintf('Average speed: %.2f m/s (%.2f km/h)\n', avg_speed, avg_speed * 3.6);

%% Plots

% Animate car's trajectory
figure(1); clf;
sim.animate_trajectory(sol);

% Plot x speed and y speed
figure(2); clf;
subplot(2, 1, 1);
plot(sol.t, sol.Z(:,3), 'LineWidth', 2)
xlabel('Time (s)')
ylabel('vx (m/s)')
title('Velocity in x-direction')
grid on

subplot(2, 1, 2);
plot(sol.t, sol.Z(:,4), 'LineWidth', 2)
xlabel('Time (s)')
ylabel('vy (m/s)')
title('Velocity in y-direction')
grid on

% Plot speed against time
figure(3); clf;
plot(sol.t, hypot(sol.Z(:,3),sol.Z(:,4)), 'LineWidth', 2)
xlabel('Time (s)')
ylabel('|v| (m/s)')
title('Speed')
grid on

% Plot tyre state over time
figure(4); clf;
subplot(2,1,1);
plot(sol.tire_t, sol.tire_W, 'b-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Tyre State W(t) (%)');
title('Tyre State Over Time');
ylim([0, 100]);
grid on;

subplot(2,1,2);
% Calculate degradation rate from tire state
dW_dt_plot = diff(sol.tire_W) ./ diff(sol.tire_t);
t_plot = sol.tire_t(1:end-1) + diff(sol.tire_t)/2;
% Plot tyre degradation over time
plot(t_plot, dW_dt_plot, 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Wear Rate (%/s)');
title('Tyre Degradation Rate Over Time');
grid on;

% Plot drive and brake forces over time
figure(5); clf;
subplot(2, 1, 1);
plot(sol.force_t, sol.F_drive, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('F_{drive} (N)');
title('Drive Force Over Time');
grid on;

subplot(2, 1, 2);
plot(sol.force_t, sol.F_brake, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('F_{brake} (N)');
title('Brake Force Over Time');
grid on;

% Display tyre wear summary
fprintf('Final tyre state: W = %.2f%%\n', car.get_tire_state());
fprintf('Tyre degradation: %.2f%%\n', 100 - car.get_tire_state());
fprintf('Simulation duration: %.2f seconds\n', sol.tire_t(end));

% Plot braking delay histogram
figure(6); clf;
histogram(sol.brake_delays, 20, 'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'black');
xlabel('Braking Delay Δs (m)');
ylabel('Frequency');
title(sprintf('Distribution of Braking Delays (λ = %.2f) \nMean = %.2f m, Std = %.2f m', ...
      car.lambda_brake, mean(sol.brake_delays), std(sol.brake_delays)));
grid on;

% Add theoretical exponential distribution overlay
hold on;
x_theory = linspace(0, car.brake_delay_max, 100);
% Theoretical PDF: λ * exp(-λ*x) for exponential distribution
lambda_theory = car.lambda_brake;
pdf_theory = lambda_theory * exp(-lambda_theory * x_theory);
% Scale to match histogram
max_count = max(histcounts(sol.brake_delays, 20));
pdf_scaled = pdf_theory * max_count / max(pdf_theory);
plot(x_theory, pdf_scaled, 'r--', 'LineWidth', 2);
legend('Sampled Delays', 'Theoretical PDF (scaled)');
hold off;

% Plot fast lap times as a line graph
figure(7); clf;
plot(2:5, fast_lap_times, 'b-o', 'LineWidth', 2, 'MarkerSize', 8);
xlabel('Lap Number');
ylabel('Lap Time (seconds)');
title('Fast Lap Times (Laps 2-5)');
grid on;
xlim([1.5, 5.5]);
xticks(2:5);
% Add value labels on each point
for i = 1:4
    text(i+1, fast_lap_times(i), sprintf('%.2f', fast_lap_times(i)), ...
         'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center', ...
         'FontSize', 10);
end
