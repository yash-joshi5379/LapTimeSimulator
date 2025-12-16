classdef Simulator < handle
    properties
        car
        track
        initial_s = 0  % initial position along track
        lap_completed = false  % flag to track if lap is completed
        cumulative_distance = 0  % cumulative distance traveled
        last_s_event = 0  % last s value used in event function
        lap_times = []  % array to store lap completion times
        lap_count = 0  % current lap number
    end

    methods
        function self = Simulator(car, track)
            self.car = car;
            self.track = track;
        end

        function solution = simulate(self, Z0, tspan)
            % Simulates car motion around the track using numerical integration
            % Z0: Initial state [x, y, vx, vy]
            % tspan: Time span for simulation (vector or [t0, tf])
            % Returns: solution structure from ode45
            
            % Reset lap completion flag and distance tracking
            self.lap_completed = false;
            self.cumulative_distance = 0;
            self.lap_times = [];
            self.lap_count = 0;
            
            % Reset car state for new simulation
            self.car.reset_state();
            
            % Get initial position along track
            [~, self.initial_s] = self.track.get_nearest_point(Z0(1), Z0(2));
            self.last_s_event = self.initial_s;
            
            % Reset track's last_s for fresh start
            self.track.last_s = self.initial_s;
            
            % Define the ODE function
            function dZ = ode_func(t, Z)
                % State derivative: dZ/dt = [vx, vy, ax, ay]
                % Get control input at current time and state
                control_input = self.track.control_input_func(t, Z);
                
                % Compute dynamics (velocities and accelerations)
                [vx, vy, ax, ay] = self.car.dynamics(Z, self.track, control_input, t);
                
                % Return state derivative
                dZ = [vx; vy; ax; ay];
            end
            
            % Event function to detect lap completion
            function [value, isterminal, direction] = lap_event(t, Z)
                % Get current position along track
                [~, s_current] = self.track.get_nearest_point(Z(1), Z(2));
                
                % Calculate incremental distance traveled (handle wrapping)
                s_total = self.track.cum_dist(end);
                ds = s_current - self.last_s_event;
                
                % Handle wrapping: if we jump backward significantly, we've wrapped around
                if ds < -s_total/2
                    % Wrapped around forward
                    ds = ds + s_total;
                elseif ds > s_total/2
                    % Wrapped around backward (shouldn't happen, but handle it)
                    ds = ds - s_total;
                end
                
                % Update cumulative distance
                self.cumulative_distance = self.cumulative_distance + ds;
                self.last_s_event = s_current;
                
                % Calculate how many complete laps have been done
                laps_completed = floor(self.cumulative_distance / self.track.L_lap);
                distance_in_current_lap = self.cumulative_distance - laps_completed * self.track.L_lap;
                
                % Track lap completion times when crossing each lap boundary
                if laps_completed > self.lap_count && laps_completed <= 5
                    self.lap_count = laps_completed;
                    self.lap_times = [self.lap_times; t];
                    % Get current tire state for diagnostics
                    current_W = self.car.W;
                    fprintf('Lap %d completed at t = %.2f seconds\n', ...
                            self.lap_count, t);
                end
                
                % Event value: distance remaining until 5 laps completed
                % This will cross zero when we complete 5 laps
                value = 5 * self.track.L_lap - self.cumulative_distance;
                
                % Stop after completing 5 laps (when value crosses zero from positive to negative)
                if self.cumulative_distance >= 5 * self.track.L_lap
                    isterminal = 1;  % Stop integration after 5 laps
                else
                    isterminal = 0;  % Continue integration
                end
                direction = -1;   % Trigger when crossing from positive to negative (completing 5 laps)
            end
            
            % ODE options for numerical integration with event detection
            opts = odeset('RelTol', 1e-4, 'AbsTol', 1e-4, 'MaxStep', 0.2, ...
                          'Events', @lap_event);
            
            % Integrate the ODE for 5 laps
            % The event function will track laps and stop after 5 laps
            [t_sol, Z_sol, te, Ze, ie] = ode23s(@ode_func, tspan, Z0, opts);
            
            % Check if all 5 laps were completed
            if self.lap_count >= 5
                self.lap_completed = true;
                fprintf('\nAll 5 laps completed!\n');
            elseif ~isempty(te)
                fprintf('Simulation stopped after %d lap(s)\n', self.lap_count);
            end
            
            % Project states onto track to prevent drift
            for i = 1:size(Z_sol, 1)
                Z_sol(i, :) = self.project_state(t_sol(i), Z_sol(i, :))';
            end
            
            % Solve tire degradation ODE separately using car's trajectory
            % This ensures smooth, continuous degradation like the user's example
            [tire_t, tire_W] = self.solve_tire_degradation_ode(t_sol, Z_sol);
            
            % Update car's tire state to final value
            if ~isempty(tire_W)
                self.car.W = tire_W(end);
            end
            
            % Get force history from car
            [force_t, F_drive_history, F_brake_history] = self.car.get_force_history();
            
            % Get braking delay history from car
            brake_delays = self.car.get_brake_delay_history();
            
            % Return solution structure
            solution = struct('t', t_sol, 'Z', Z_sol, ...
                            'tire_t', tire_t, 'tire_W', tire_W, ...
                            'force_t', force_t, 'F_drive', F_drive_history, 'F_brake', F_brake_history, ...
                            'brake_delays', brake_delays, ...
                            'lap_times', self.lap_times, 'lap_count', self.lap_count);
        end
        
        function Z_proj = project_state(self, t, Z)
            % Projects car's position onto track to prevent drifting
            % off track during numerical integration
            % Z: State vector [x; y; vx; vy] (column vector)
            % Returns: Projected state vector

            x = Z(1); y = Z(2); vx = Z(3); vy = Z(4);

            % Get nearest point on track
            [idx, s] = self.track.get_nearest_point(x, y);
            [x_track, y_track, theta_track] = self.track.get_point_at_dist(s);

            % Set car position directly on track point to remove any
            % lateral error
            alpha = 0.2;  % small correction factor
            x = (1-alpha)*x + alpha*x_track;
            y = (1-alpha)*y + alpha*y_track;
            
            % Project current velocity onto track tangent
            tangent = [cos(theta_track); sin(theta_track)];
            v_long = dot([vx; vy], tangent);

            v_magnitude = hypot(vx, vy);
            if v_magnitude > 1e-3
                if v_long > 0
                    % If car is moving forward, align it with the track
                    vx = v_long * tangent(1);
                    vy = v_long * tangent(2);
                else
                    % If car is moving backward, give a small forward
                    % push
                    vx = 0.1 * tangent(1);
                    vy = 0.1 * tangent(2);
                end
            else
                vx = 0; 
                vy = 0;
            end
            
            % Return projected state as column vector
            Z_proj = [x; y; vx; vy];
        end
        
        function animate_trajectory(self, sol)
            % Animate the car's trajectory around the track
            % sol: Solution structure from simulator with fields t, Z (where Z = [x, y, vx, vy])
            
            % Plot the track
            plot(self.track.x, self.track.y);
            hold on;
            
            % Set axis properties
            axis equal;
            xlabel('x (m)');
            ylabel('y (m)');
            title('Car Trajectory Animation');
            grid on;
            
            % Get trajectory data
            x_traj = sol.Z(:, 1);
            y_traj = sol.Z(:, 2);
            vx_traj = sol.Z(:, 3);
            vy_traj = sol.Z(:, 4);
            t_traj = sol.t;
            
            % Determine which lap each point belongs to
            % Calculate cumulative distance for each point
            cumulative_dist = zeros(size(x_traj));
            last_s = 0;
            s_total = self.track.cum_dist(end);
            
            for i = 1:length(x_traj)
                [~, s_current] = self.track.get_nearest_point(x_traj(i), y_traj(i));
                ds = s_current - last_s;
                
                % Handle wrapping
                if ds < -s_total/2
                    ds = ds + s_total;
                elseif ds > s_total/2
                    ds = ds - s_total;
                end
                
                cumulative_dist(i) = cumulative_dist(max(1, i-1)) + ds;
                last_s = s_current;
            end
            
            % Determine which lap each point is in
            lap_number = floor(cumulative_dist / self.track.L_lap) + 1;
            lap_number = min(lap_number, 5);  % Cap at 5 for display
            
            % Find the index where lap 2 starts (fast laps begin)
            lap2_start_idx = find(lap_number >= 2, 1, 'first');
            if isempty(lap2_start_idx)
                lap2_start_idx = length(x_traj) + 1;  % No lap 2
            end
            
            % Car size (triangle dimensions)
            car_length = 40;  % meters
            car_width = 30;    % meters
            
            % Initialize car triangle and trails
            car_triangle = [];
            car_trail_lap1 = [];
            car_trail_lap2 = [];
            
            % Animation parameters
            skip_frames = max(1, floor(length(t_traj) / 1000));  % Limit to ~500 frames for smoothness
            frame_delay = 0.01;  % seconds between frames
            
            % Loop through trajectory
            for i = 1:skip_frames:length(t_traj)
                % Get current position and velocity
                x = x_traj(i);
                y = y_traj(i);
                vx = vx_traj(i);
                vy = vy_traj(i);
                
                % Calculate speed and heading
                v = sqrt(vx^2 + vy^2);
                if v > 0.1
                    theta = atan2(vy, vx);
                else
                    % If stopped, use previous heading or default
                    if i > 1
                        vx_prev = vx_traj(i-1);
                        vy_prev = vy_traj(i-1);
                        if sqrt(vx_prev^2 + vy_prev^2) > 0.1
                            theta = atan2(vy_prev, vx_prev);
                        else
                            theta = 0;
                        end
                    else
                        theta = 0;
                    end
                end
                
                % Define triangle vertices (arrowhead pointing in direction of travel)
                % Triangle: front point at car position, two back points
                front_x = x + car_length/2 * cos(theta);
                front_y = y + car_length/2 * sin(theta);
                
                back_left_x = x - car_length/2 * cos(theta) + car_width/2 * cos(theta + pi/2);
                back_left_y = y - car_length/2 * sin(theta) + car_width/2 * sin(theta + pi/2);
                
                back_right_x = x - car_length/2 * cos(theta) + car_width/2 * cos(theta - pi/2);
                back_right_y = y - car_length/2 * sin(theta) + car_width/2 * sin(theta - pi/2);
                
                % Delete previous car triangle
                if ~isempty(car_triangle) && isvalid(car_triangle)
                    delete(car_triangle);
                end
                
                % Draw new car triangle
                car_triangle = fill([front_x, back_left_x, back_right_x], ...
                                   [front_y, back_left_y, back_right_y], ...
                                   'r', 'EdgeColor', 'r', 'LineWidth', 3);
                
                % Draw trail with different colors for different laps
                % Lap 1 (red)
                lap1_end = min(i, lap2_start_idx - 1);
                if lap1_end > 0
                    if isempty(car_trail_lap1)
                        car_trail_lap1 = plot(x_traj(1:lap1_end), y_traj(1:lap1_end), 'r--', 'LineWidth', 2);
                    else
                        set(car_trail_lap1, 'XData', x_traj(1:lap1_end), 'YData', y_traj(1:lap1_end));
                    end
                end
                
                % Lap 2 (green) - fast lap
                if i >= lap2_start_idx
                    lap2_start_plot = lap2_start_idx;
                    if isempty(car_trail_lap2)
                        car_trail_lap2 = plot(x_traj(lap2_start_plot:i), y_traj(lap2_start_plot:i), 'g--', 'LineWidth', 2);
                    else
                        set(car_trail_lap2, 'XData', x_traj(lap2_start_plot:i), 'YData', y_traj(lap2_start_plot:i));
                    end
                end
                
                % Update title with current time and speed every frame
                title(sprintf('Car Trajectory Animation - t = %.2f s, v = %.1f m/s', ...
                      t_traj(i), v));
                
                % Refresh plot
                drawnow;
                pause(frame_delay);
            end
            
            % Final annotation
            title(sprintf('Car Trajectory Animation - Complete (t = %.2f s)', t_traj(end)));
            if ~isempty(car_trail_lap2)
                legend('Track', 'Lap 1 (Warm-up)', 'Laps 2-5 (Fast)');
            else
                legend('Track', 'Lap 1 (Warm-up)');
            end
            hold off;
        end
        
        function [t_tire, W_tire] = solve_tire_degradation_ode(self, t_car, Z_car)
            % Solve tire degradation ODE separately using car's trajectory
            % This ensures smooth, continuous degradation like the user's example
            % t_car: time points from car simulation
            % Z_car: car state at each time point [x, y, vx, vy]
            % Returns: tire_t, tire_W (smooth degradation)
            
            % Calculate speed at each time point
            v_traj = sqrt(Z_car(:,3).^2 + Z_car(:,4).^2);
            
            % Calculate F_total_magnitude at each time point by simulating forces
            % Note: We use W=100 for force calculation here because we're solving
            % the degradation ODE separately. The actual simulation uses the
            % degraded W from Car.dynamics(), which affects lap times.
            F_total_traj = zeros(size(t_car));
            for i = 1:length(t_car)
                Z = Z_car(i, :)';
                control_input = self.track.control_input_func(t_car(i), Z);
                
                % Extract state
                x = Z(1); y = Z(2); vx = Z(3); vy = Z(4);
                v = sqrt(vx^2 + vy^2);
                [idx, ~] = self.track.get_nearest_point(x, y);
                curvature = self.track.kappa(idx);
                
                % Calculate forces
                % Use W=100 for plotting ODE (actual simulation uses degraded W)
                F_drag = 0.5 * self.car.rho * v^2 * self.car.c_d * self.car.A;
                F_down = 0.5 * self.car.rho * v^2 * self.car.c_l * self.car.A;
                F_normal = self.car.mass * self.car.g + F_down;
                max_F_total = self.car.mu * F_normal;  % Use full grip for plotting ODE
                
                v_desired = control_input.desired_speed;
                [F_drive, F_brake] = self.car.compute_forces(v, v_desired, abs(curvature), max_F_total);
                F_rr = self.car.C_rr * F_normal;
                F_long = F_drive - F_brake - F_drag - F_rr;
                
                if abs(curvature) > 1e-4
                    F_lat = self.car.mass * v^2 * curvature;
                else
                    F_lat = 0;
                end
                
                F_total_traj(i) = sqrt(F_long^2 + F_lat^2);
            end
            
            % Create acceleration magnitude trajectory
            a_traj = F_total_traj / self.car.mass;
            
            % Create temperature trajectory
            T_traj = zeros(size(t_car));
            for i = 1:length(t_car)
                T_traj(i) = self.car.get_tire_temperature(t_car(i), v_traj(i), F_total_traj(i));
            end
            
            % Create interpolation functions for smooth evaluation
            T_interp = @(t) interp1(t_car, T_traj, t, 'linear', 'extrap');
            a_interp = @(t) interp1(t_car, a_traj, t, 'linear', 'extrap');
            
            % Generate noise trajectory for tire degradation
            % For continuous-time noise, we generate samples at fine time intervals
            % and interpolate them in the ODE function
            dt_noise = 0.01;  % Time step for noise generation (fine resolution)
            t_noise = t_car(1):dt_noise:t_car(end);
            noise_traj = self.car.mu_tire + self.car.sigma_tire * randn(size(t_noise));
            noise_interp = @(t) interp1(t_noise, noise_traj, t, 'linear', 'extrap');
            
            % Define tire degradation ODE with noise:
            % dW/dt = -[p1 * |T(t) - T_opt| + p2 * |a(t)|] + noise~N(mu_tire, sigma_tire)
            dw_dt = @(t, W) -(self.car.p1 * abs(T_interp(t) - self.car.optimum_temp) + ...
                              self.car.p2 * a_interp(t)) + noise_interp(t);
            
            % Solve the ODE using ode45 for smooth, continuous solution
            W0 = 100;  % Initial tire state
            [t_tire, W_tire] = ode45(dw_dt, [t_car(1), t_car(end)], W0);
            
            % Clamp W to [10, 100] - minimum 10% to prevent complete loss of grip
            W_tire = max(10, min(100, W_tire));
        end
    end
end

