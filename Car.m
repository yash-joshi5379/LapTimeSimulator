classdef Car < handle
    properties
        mass
        max_F_drive
        max_F_brake
        c_d
        c_l
        A
        C_rr
        rho
        g
        mu
        % Tire wear properties
        W              % Tire state: 100 = new, 0 = fully worn
        p1             % Temperature difference coefficient
        p2             % Acceleration magnitude coefficient
        optimum_temp   % Optimum tire temperature
        temp_interp    % Function handle for tire temperature interpolation
        use_tire_data  % Flag: true if using real tire data, false for simplified model
        last_t         % Last time for tire wear calculation
        mu_tire        % Mean of Gaussian noise for tire degradation
        sigma_tire     % Standard deviation of Gaussian noise for tire degradation
        last_v         % Last speed value for interpolation
        last_F_total   % Last total force magnitude for interpolation
        % Tire state history for plotting
        tire_state_history_t  % Time points for tire state history
        tire_state_history_W  % Tire state values at each time point
        % Stochastic force noise parameters
        mu_drive       % Mean of Gaussian noise for F_drive
        sigma_drive    % Standard deviation of Gaussian noise for F_drive
        mu_brake       % Mean of Gaussian noise for F_brake
        sigma_brake    % Standard deviation of Gaussian noise for F_brake
        % Force history for plotting
        force_history_t       % Time points for force history
        force_history_drive  % F_drive values at each time point
        force_history_brake  % F_brake values at each time point
        % Stochastic braking delay parameters
        lambda_brake         % Rate parameter for exponential distribution (1/mean)
        brake_delay_max      % Maximum braking delay (meters)
        % Braking zone state tracking
        in_braking_zone      % Flag: true if currently in a braking zone
        brake_delay_ds       % Sampled delay distance Δs (meters)
        s_at_zone_entry      % Track position s when entering braking zone
        last_s               % Last track position s for distance tracking
        % Braking delay history for visualization
        brake_delay_history  % Array of sampled Δs values
    end

    methods
        function self = Car()
            % Creates a Car object
            self.mass = 1000;
            self.max_F_drive = 9900;
            self.max_F_brake = 11000;
            self.c_d = 0.3;
            self.c_l = 1.5; % Lift coefficient, +ve = downforce
            self.A = 2.5;
            self.C_rr = 0.02;
            self.rho = 1.225;
            self.g = 9.81;
            self.mu = 1.6;
            
            % Initialize tire wear properties
            self.W = 100; % Start with new tire (100% performance)
            self.p1 = 0.015;  % Temperature difference coefficient (increased for more degradation)
            self.p2 = 0.015;  % Acceleration magnitude coefficient (increased for more degradation)
            self.optimum_temp = 80;  % Default optimum temperature (will be set from data)
            self.temp_interp = [];   % Will be set if tire data is loaded
            self.use_tire_data = false;  % Default to false until data is loaded
            self.last_t = 0;  % Initialize time tracking
            self.last_v = 0;  % Initialize speed tracking
            self.last_F_total = 0;  % Initialize force tracking
            % Initialize tire degradation noise parameters
            self.mu_tire = 0;      % Mean of Gaussian noise for tire degradation
            self.sigma_tire = 0;   % Standard deviation of Gaussian noise for tire degradation
            % Initialize tire state history
            self.tire_state_history_t = [];
            self.tire_state_history_W = [];
            
            % Initialize stochastic noise parameters for forces
            self.mu_drive = 0;        % Mean of Gaussian noise for F_drive
            self.sigma_drive = 2e-7;    % Standard deviation of Gaussian noise for F_drive
            self.mu_brake = 0;       % Mean of Gaussian noise for F_brake
            self.sigma_brake = 2e-7;    % Standard deviation of Gaussian noise for F_brake
            % Initialize force history
            self.force_history_t = [];
            self.force_history_drive = [];
            self.force_history_brake = [];
            % Initialize stochastic braking delay parameters
            self.lambda_brake = 0.1;  % Rate parameter (1/mean), default mean = 10m
            self.brake_delay_max = 10; % Maximum delay in meters
            % Initialize braking zone state
            self.in_braking_zone = false;
            self.brake_delay_ds = 0;
            self.s_at_zone_entry = 0;
            self.last_s = 0;
            % Initialize braking delay history
            self.brake_delay_history = [];
        end
        
        function load_tire_data(self, data_file)
            % Load tire test data and set up temperature interpolation
            % data_file: path to .mat file containing tire test data
            % Expected fields: ET (time), TSTC (tire temperature)
            data = load(data_file);
            time = data.ET;
            idx = find(time > 500, 1);
            if isempty(idx)
                idx = length(time);
            end
            time = data.ET(1:idx);
            tyre_temp = data.TSTC(1:idx);
            
            % Calculate optimum temperature as mean
            self.optimum_temp = mean(tyre_temp);
            
            % Remove duplicate time points for interpolation
            [time_unique, ~, idx_unique] = unique(time, 'stable');
            tyre_temp_unique = accumarray(idx_unique, tyre_temp, [], @mean);
            
            % Create interpolation function
            self.temp_interp = @(t) interp1(time_unique, tyre_temp_unique, t, 'linear', 'extrap');
            self.use_tire_data = true;
            
            fprintf('Tire data loaded. Optimum temperature: %.2f°C\n', self.optimum_temp);
        end
        
        function reset_state(self)
            % Reset car state for new simulation
            % Reset tire state
            self.W = 100;
            self.last_t = 0;
            self.last_v = 0;
            self.last_F_total = 0;
            self.tire_state_history_t = [];
            self.tire_state_history_W = [];
            % Clear force history
            self.force_history_t = [];
            self.force_history_drive = [];
            self.force_history_brake = [];
            % Reset braking zone state
            self.in_braking_zone = false;
            self.brake_delay_ds = 0;
            self.s_at_zone_entry = 0;
            self.last_s = 0;
            % Clear braking delay history
            self.brake_delay_history = [];
        end
        
        function T = get_tire_temperature(self, t, v, F_total_magnitude)
            % Calculate tire temperature at time t
            % If tire data is loaded, use interpolation
            % Otherwise, use simplified model based on speed and forces
            if self.use_tire_data && ~isempty(self.temp_interp)
                T = self.temp_interp(t);
            else
                % Simplified temperature model: increases with speed and force
                % Base temperature + contributions from speed and force
                T_base = 20;  % Ambient temperature (°C)
                T_speed = 0.5 * v;  % Temperature increase from speed
                T_force = 0.01 * F_total_magnitude;  % Temperature increase from forces
                T = T_base + T_speed + T_force;
                % Clamp to reasonable range
                T = max(20, min(120, T));
            end
        end
        
        function [t_history, W_history] = get_tire_state_history(self)
            % Get tire state history for plotting
            % Returns: t_history (time points), W_history (tire state values)
            t_history = self.tire_state_history_t;
            W_history = self.tire_state_history_W;
        end
        
        function W = get_tire_state(self)
            % Get current tire state (0-100%)
            W = self.W;
        end
        
        function [t_history, F_drive_history, F_brake_history] = get_force_history(self)
            % Get force history for plotting
            % Returns: t_history (time points), F_drive_history, F_brake_history
            t_history = self.force_history_t;
            F_drive_history = self.force_history_drive;
            F_brake_history = self.force_history_brake;
        end
        
        function brake_delays = get_brake_delay_history(self)
            % Get braking delay history for visualization
            % Returns: Array of sampled Δs values (braking delays in meters)
            brake_delays = self.brake_delay_history;
        end
        
        function [F_drive, F_brake] = compute_forces(self, v, v_desired, curvature, max_F_total)
            % Compute drive and brake forces based on desired speed
            % Check lateral force constraint for corners - ALWAYS enforce this
            if abs(curvature) > 1e-6
                % Calculate maximum speed for this corner based on available grip
                max_lat_force = max_F_total;
                v_max_corner = sqrt(max_lat_force / (self.mass * abs(curvature)));
                % Don't reduce v_desired below 100 m/s - let the car try to reach 100 m/s
                % Only use physics limit if it's actually constraining (above 100 m/s)
                if v_max_corner < 100
                    % Physics doesn't allow 100 m/s, but still try to reach it
                    % The friction circle will naturally limit the speed
                    % v_desired stays at 100 (from look-ahead)
                else
                    % Physics allows more than 100 m/s, so cap at 100 m/s
                    v_desired = min(v_desired, 100);
                end
                
                % P controller for corners - more aggressive braking
                v_error = v_desired - v;
                if v < 0.5
                    a_desired = min(4.0, self.max_F_drive/self.mass);
                elseif v_error > 0.05
                    % Need to accelerate to reach desired speed
                    % Use higher gain to reach 100 m/s
                    a_desired = self.max_F_drive / self.mass;
                elseif v_error < -0.05
                    % Need to brake - use more aggressive braking for corners
                    % Scale braking gain based on how much we're over the limit
                    if v > v_max_corner
                        % Way over limit - brake very hard
                        brake_gain = 1.2;
                    else
                        % Slightly over desired - moderate braking
                        brake_gain = 0.9;
                    end
                    a_desired = max(v_error * brake_gain, -self.max_F_brake / self.mass);
                else
                    % Very close to desired speed - maintain
                    a_desired = 0;
                end
            else
                % Straight section - normal P controller
                v_error = v_desired - v;
                if v < 0.5
                    a_desired = self.max_F_drive/self.mass;
                elseif abs(v_error) < 0.1
                    a_desired = 0;
                elseif v_error >= 0.1
                    a_desired = self.max_F_drive / self.mass;
                else
                    a_desired = -self.max_F_brake / self.mass;
                end
            end

            F_desired = self.mass * a_desired;
            if F_desired > 0
                F_drive = min(F_desired, self.max_F_drive);
                F_brake = 0;
            else
                F_drive = 0;
                F_brake = min(-F_desired, self.max_F_brake);
            end
            
            % Add Gaussian noise to forces
            % F_drive = F_drive + noise_drive~N(mu_drive, sigma_drive)
            if F_drive > 0
                drive_noise = self.mu_drive + self.sigma_drive * randn();
                F_drive = F_drive + drive_noise;
                % Clamp F_drive to [0, max_F_drive]
                F_drive = max(0, min(self.max_F_drive, F_drive));
            end
            
            % F_brake = F_brake + noise_brake~N(mu_brake, sigma_brake)
            if F_brake > 0
                brake_noise = self.mu_brake + self.sigma_brake * randn();
                F_brake = F_brake + brake_noise;
                % Clamp F_brake to [0, max_F_brake]
                F_brake = max(0, min(self.max_F_brake, F_brake));
            end
        end

        function [vx, vy, ax, ay] = dynamics(self, Z, track, control_input, t)
            % Differential equations for car motion using force equations
            % Z: State vector [x, y, vx, vy]
            % t: current time
            if nargin < 5
                t = 0;  % Default to 0 if not provided
            end
            
            x = Z(1); y = Z(2); vx = Z(3); vy = Z(4);
            
            % Calculate speed at each point
            v = sqrt(vx.^2 + vy.^2);
            
            % Get theoretical heading angle at each point
            [idx, s] = track.get_nearest_point(x, y);
            [x_track, y_track, theta_track] = track.get_point_at_dist(s);

            % If moving, calculate real heading angle, else use theoretical
            if v > 0.1
                theta = atan2(vy, vx);
            else
                theta = theta_track;
            end

            % Get track curvature to calculate lateral force
            curvature = track.kappa(idx);

            % Get desired speed and look-ahead information
            v_desired = control_input.desired_speed;
            turn_coming = control_input.turn_coming;

            % Stochastic braking delay logic
            % Track distance traveled along track
            s_total = track.cum_dist(end);
            if self.last_s > 0
                % Calculate distance traveled (handle wrapping)
                ds_traveled = s - self.last_s;
                if ds_traveled < -s_total/2
                    ds_traveled = ds_traveled + s_total;  % Wrapped forward
                elseif ds_traveled > s_total/2
                    ds_traveled = ds_traveled - s_total;   % Wrapped backward
                end
            else
                ds_traveled = 0;
            end
            self.last_s = s;

            % Check if entering a new braking zone
            if turn_coming && ~self.in_braking_zone
                % Entering a new braking zone - sample braking delay
                % Sample from truncated exponential distribution: Δs ~ Exp(λ) truncated to [0, brake_delay_max]
                % Use inverse CDF method for truncated exponential
                mean_delay = 1 / self.lambda_brake;
                % Sample uniform random variable
                u = rand();
                % Transform to truncated exponential using inverse CDF
                % CDF of truncated exponential: F(x) = (1 - exp(-λx)) / (1 - exp(-λ*max))
                F_max = 1 - exp(-self.lambda_brake * self.brake_delay_max);
                self.brake_delay_ds = -mean_delay * log(1 - u * F_max);
                % Clamp to ensure it's within bounds (numerical precision)
                self.brake_delay_ds = max(0, min(self.brake_delay_max, self.brake_delay_ds));
                % Record the sampled delay for visualization
                self.brake_delay_history = [self.brake_delay_history; self.brake_delay_ds];
                self.s_at_zone_entry = s;
                self.in_braking_zone = true;
            elseif ~turn_coming && self.in_braking_zone
                % Exited braking zone
                self.in_braking_zone = false;
                self.brake_delay_ds = 0;
            end

            % Check if we should delay braking
            should_delay_braking = false;
            if self.in_braking_zone
                % Calculate distance traveled in this braking zone
                s_in_zone = s - self.s_at_zone_entry;
                % Handle wrapping
                if s_in_zone < -s_total/2
                    s_in_zone = s_in_zone + s_total;
                elseif s_in_zone > s_total/2
                    s_in_zone = s_in_zone - s_total;
                end
                
                % If we haven't traveled Δs yet, delay braking
                if s_in_zone < self.brake_delay_ds
                    should_delay_braking = true;
                end
            end

            % If delaying braking, override v_desired to prevent braking
            % Keep current speed (don't brake or accelerate yet)
            % BUT still enforce speed limits
            if should_delay_braking
                v_desired = v;
            end
            
            % Enforce hard speed limits based on current track position
            % Cap speed at 106 m/s on straights and 100 m/s on corners
            if abs(curvature) > 1e-6
                % In a corner - cap at 100 m/s
                v_desired = min(v_desired, 100);
            else
                % On a straight - cap at 106 m/s
                v_desired = min(v_desired, 106);
            end
            

            % Compute aero forces
            F_drag = 0.5 * self.rho * v.^2 * self.c_d * self.A;
            F_down = 0.5 * self.rho * v.^2 * self.c_l * self.A;

            % Calculate max combined force possible using friction circle
            F_normal = self.mass*self.g + F_down;
            max_F_total = self.mu * F_normal;
            
            % Scale maximum tire force by tire state W
            % W = 100% means full grip, W = 0% means no grip
            % Ensure minimum grip to prevent complete loss of control
            W_effective = max(self.W / 100, 0.1);  % Minimum 10% grip to prevent complete loss
            max_F_total = max_F_total * W_effective;

            % Compute drive and brake forces
            % Use current curvature to determine if we're on a straight or corner
            % v_desired is already set by look-ahead logic in control_input_func
            curvature_for_forces = abs(curvature);
            
            [F_drive, F_brake] = self.compute_forces(v, v_desired, curvature_for_forces, max_F_total);
            
            % Record force history at each time step
            % Record initial state at t=0
            if t == 0 && isempty(self.force_history_t)
                self.force_history_t = [0];
                self.force_history_drive = [F_drive];
                self.force_history_brake = [F_brake];
            elseif isempty(self.force_history_t) || t > self.force_history_t(end)
                % Record forces if time has advanced
                self.force_history_t = [self.force_history_t; t];
                self.force_history_drive = [self.force_history_drive; F_drive];
                self.force_history_brake = [self.force_history_brake; F_brake];
            end

            % Rolling resistance
            F_rr = self.C_rr * F_normal;

            % Total longitudinal force
            F_long = F_drive - F_brake - F_drag - F_rr;

            % Total lateral force
            if abs(curvature) > 1e-4
                F_lat = self.mass * v.^2 * curvature;
            else
                F_lat = 0;
            end

            F_total_magnitude = sqrt(F_long.^2 + F_lat.^2);
            if F_total_magnitude > max_F_total
                % Scale both forces proportionally to stay within friction
                % circle
                scale_factor = max_F_total / F_total_magnitude;
                F_lat = F_lat * scale_factor;
                F_long = F_long * scale_factor;
                % Recalculate total force magnitude after scaling
                F_total_magnitude = sqrt(F_long.^2 + F_lat.^2);
            end
            
            % Update tire wear state W during simulation for physics
            % Note: For smooth plotting, tire degradation is also solved separately
            % in Simulator.solve_tire_degradation_ode() using ode45
            if t > self.last_t
                dt = t - self.last_t;
                
                % Get tire temperature
                T = self.get_tire_temperature(t, v, F_total_magnitude);
                
                % Calculate acceleration magnitude
                a_mag = F_total_magnitude / self.mass;
                
                % Calculate wear rate
                dW_dt = -(self.p1 * abs(T - self.optimum_temp) + self.p2 * a_mag);
                
                % Update tire state using simple Euler step (for physics during simulation)
                self.W = self.W + dW_dt * dt;
                
                % Clamp W to [10, 100]
                self.W = max(10, min(100, self.W));
                
                self.last_t = t;
            elseif t == 0
                self.last_t = 0;
            end
            
            % Calculate accelerations in each direction
            ax = (F_long*cos(theta) - F_lat*sin(theta)) / self.mass;
            ay = (F_long*sin(theta) + F_lat*cos(theta)) / self.mass;

            normal  = [-sin(theta_track); cos(theta_track)];

            track_to_car = [x - x_track; y - y_track];
            lateral_error = dot(track_to_car, normal);
            v_lateral = dot([vx; vy], normal);

            k_position = 1e6;  % moderate stiffness (reduced for smoother transitions)
            k_velocity = 5e5;   % moderate damping
            
            position_constraint = -k_position * lateral_error * normal;
            velocity_constraint = -k_velocity * v_lateral * normal;
            
            constraint_force = position_constraint + velocity_constraint;
            
            % Limit constraint force magnitude to prevent runaway acceleration
            % But allow constraint forces to be larger than tire forces to keep car on track
            % Only limit if constraint force is extremely large (more than 10x tire force)
            constraint_force_mag = sqrt(constraint_force(1)^2 + constraint_force(2)^2);
            max_constraint_force = max(max_F_total * 10, 1000);  % Allow up to 10x tire force or minimum 1000N
            if constraint_force_mag > max_constraint_force
                constraint_force = constraint_force * (max_constraint_force / constraint_force_mag);
            end

            ax = ax + constraint_force(1)/self.mass;
            ay = ay + constraint_force(2)/self.mass;
        end
    end
end
