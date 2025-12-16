%% Load real data and setup simulations
clc;
clear;

% Real data from the provided table (9 runs)
% Format: [Lap1, Lap2, Lap3, Lap4, TotalTime_seconds, AvgSpeed]
real_data = [
    38.5991, 38.6380, 38.7465, 38.6623, 154.6459, 232.790;
    38.6226, 38.7417, 38.7010, 38.7881, 154.8534, 232.478;
    38.7113, 38.7830, 38.8017, 38.8109, 155.1069, 232.098;
    38.7036, 38.7778, 38.8481, 38.8082, 155.1377, 232.052;
    38.7088, 38.7768, 38.8721, 38.8232, 155.1809, 231.987;
    38.8022, 38.8064, 38.9369, 39.0439, 155.5894, 231.378;
    38.8060, 38.8342, 38.8859, 38.9150, 155.4411, 231.599;
    38.8648, 38.8601, 38.9319, 38.9447, 155.6015, 231.360;
    38.9560, 38.9762, 38.9756, 38.9269, 155.8347, 231.014;
];

% Extract real data metrics
real_lap1 = real_data(:, 1);
real_lap2 = real_data(:, 2);
real_lap3 = real_data(:, 3);
real_lap4 = real_data(:, 4);
real_total_time = real_data(:, 5);
real_speed = real_data(:, 6);

% Number of simulation runs
num_simulations = 10; 

% Initialize arrays to store simulated data
sim_lap1 = zeros(num_simulations, 1);
sim_lap2 = zeros(num_simulations, 1);
sim_lap3 = zeros(num_simulations, 1);
sim_lap4 = zeros(num_simulations, 1);
sim_total_time = zeros(num_simulations, 1);
sim_speed = zeros(num_simulations, 1);

% Build track and car (do this once outside the loop for efficiency)
track = Track();
car = Car();
car.load_tire_data("B1965run52.mat");
sim = Simulator(car, track);
x0 = track.x(1);
y0 = track.y(1);
Z0 = [x0, y0, 0, 0];
tspan = linspace(0, 300, track.N_total * 5);

%% Run simulations
for i = 1:num_simulations
    fprintf('Running simulation %d/%d...\n', i, num_simulations);
    
    % Reset simulator and car state
    sim.lap_times = [];
    sim.lap_count = 0;
    car.reset_state();
    
    % Run simulation
    sol = sim.simulate(Z0, tspan);
    
    % Calculate individual fast lap times (laps 2-5)
    sim_lap1(i) = sol.lap_times(2) - sol.lap_times(1);  % Fast lap 1 (lap 2)
    sim_lap2(i) = sol.lap_times(3) - sol.lap_times(2);  % Fast lap 2 (lap 3)
    sim_lap3(i) = sol.lap_times(4) - sol.lap_times(3);  % Fast lap 3 (lap 4)
    sim_lap4(i) = sol.lap_times(5) - sol.lap_times(4);  % Fast lap 4 (lap 5)
    
    % Total time for 4 fast laps
    sim_total_time(i) = sol.lap_times(5) - sol.lap_times(1);
    
    % Average speed for fast laps
    sim_speed(i) = track.L_lap / mean([sim_lap1(i), sim_lap2(i), sim_lap3(i), sim_lap4(i)]);
    sim_speed(i) = sim_speed(i) * 2.23694; % Convert to mph
end

% Remove any NaN values
valid_idx = ~isnan(sim_lap1);
sim_lap1 = sim_lap1(valid_idx);
sim_lap2 = sim_lap2(valid_idx);
sim_lap3 = sim_lap3(valid_idx);
sim_lap4 = sim_lap4(valid_idx);
sim_total_time = sim_total_time(valid_idx);
sim_speed = sim_speed(valid_idx);

%% t-tests

% Perform two-sample t-tests for each metric
% H0: Means are equal (no significant difference)
% H1: Means are different (significant difference)
% Alpha = 0.05 (95% confidence level)

% Test 1: Lap 1
[h1, p1, ci1, stats1] = ttest2(real_lap1, sim_lap1);

% Test 2: Lap 2
[h2, p2, ci2, stats2] = ttest2(real_lap2, sim_lap2);

% Test 3: Lap 3
[h3, p3, ci3, stats3] = ttest2(real_lap3, sim_lap3);

% Test 4: Lap 4
[h4, p4, ci4, stats4] = ttest2(real_lap4, sim_lap4);

% Test 5: Total Time
[h5, p5, ci5, stats5] = ttest2(real_total_time, sim_total_time);

% Test 6: Speed
[h6, p6, ci6, stats6] = ttest2(real_speed, sim_speed);

% Summary table
fprintf('=== Summary Table ===\n');
fprintf('%-15s %12s %12s %12s %10s\n', 'Metric', 'Real Mean', 'Sim Mean', 'p-value', 'Significant?');
fprintf('%s\n', repmat('-', 1, 70));
fprintf('%-15s %12.4f %12.4f %12.6f %10s\n', 'Lap 1', mean(real_lap1), mean(sim_lap1), p1, char(string(p1 < alpha)));
fprintf('%-15s %12.4f %12.4f %12.6f %10s\n', 'Lap 2', mean(real_lap2), mean(sim_lap2), p2, char(string(p2 < alpha)));
fprintf('%-15s %12.4f %12.4f %12.6f %10s\n', 'Lap 3', mean(real_lap3), mean(sim_lap3), p3, char(string(p3 < alpha)));
fprintf('%-15s %12.4f %12.4f %12.6f %10s\n', 'Lap 4', mean(real_lap4), mean(sim_lap4), p4, char(string(p4 < alpha)));
fprintf('%-15s %12.4f %12.4f %12.6f %10s\n', 'Total Time', mean(real_total_time), mean(sim_total_time), p5, char(string(p5 < alpha)));
fprintf('%-15s %12.3f %12.3f %12.6f %10s\n', 'Speed', mean(real_speed), mean(sim_speed), p6, char(string(p6 < alpha)));
fprintf('\n');

%% Visualisation


% Plot 1: Lap times comparison
figure(1); clf;
hold on;
plot(1:4, [mean(real_lap1), mean(real_lap2), mean(real_lap3), mean(real_lap4)], ...
     'b-o', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Real Data');
plot(1:4, [mean(sim_lap1), mean(sim_lap2), mean(sim_lap3), mean(sim_lap4)], ...
     'r--s', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Simulated Data');
xlabel('Lap Number');
ylabel('Lap Time (s)');
title('Lap Times Comparison');
legend('Location', 'best');
grid on;
xticks(1:4);

% Plot 2: Box plots for each real lap
figure(2); clf;
boxplot([real_lap1, real_lap2, real_lap3, real_lap4]);
xlabel('"Fast" Laps')
ylabel('Lap Time (s)');
title('Lap Times Distribution for Real Data');
grid on;
% Make boxplot lines bolder
h = findobj(gca, 'Type', 'Line');
set(h, 'LineWidth', 1.5)

% Plot 3: Box plots for each simulated lap
figure(3); clf;
boxplot([sim_lap1, sim_lap2, sim_lap3, sim_lap4]);
ylim([38.91, 39.06]);
xlabel('"Fast" Laps')
ylabel('Lap Time (s)');
title('Lap Times Distribution for Simulated Data');
grid on;
% Make boxplot lines bolder
h = findobj(gca, 'Type', 'Line');
set(h, 'LineWidth', 1.5)