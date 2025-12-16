classdef Track < handle
    properties
        L_straight      % length of each straight
        R               % corner turning radius
        L_lap           % total length of lap
        N_straight      % number of points on straights
        N_turn          % number of points on turns
        N_total         % number of points on self.geometry
        x
        y
        theta
        cum_dist
        kappa
        desired_speed   % in m/s 
        last_s = 0      % last known position along track for progress tracking
    end

    methods
        function self = Track()
            self.L_straight = 1000;        
            self.R = 320;                         
            self.L_lap = 2*self.L_straight + 2*pi*self.R;  
            self.N_straight = 1000;                
            self.N_turn = 1500;                     
            self.N_total = 2*self.N_straight + 2*self.N_turn;
            self.x = zeros(1, self.N_total);
            self.y = zeros(1, self.N_total);
            self.theta = zeros(1, self.N_total);
            self.kappa = zeros(1, self.N_total);

            self.build_track();
            self.calc_cum_dist();
        end

        function self = build_track(self)
            % Straight 1: bottom left to bottom right
            n_start = 1; n_end = self.N_straight;
            self.x(n_start:n_end) = linspace(-self.L_straight/2, self.L_straight/2, self.N_straight);
            self.y(n_start:n_end) = -self.R;
            % theta=0, so no changes needed
            self.kappa(n_start:n_end) = 0;
            self.desired_speed(n_start:n_end) = 106;
            
            % Turn 1: Bottom right to top right
            n_start = n_end + 1 ; n_end = n_end + self.N_turn ;
            self.theta(n_start:n_end) = linspace(0, pi, self.N_turn);
            self.x(n_start:n_end) = self.L_straight/2 + self.R*sin(self.theta(n_start:n_end));
            self.y(n_start:n_end) = -self.R*cos(self.theta(n_start:n_end));
            self.kappa(n_start:n_end) = 1/self.R;
            self.desired_speed(n_start:n_end) = 100;

            % Straight 2: Top right to top left
            n_start = n_end + 1 ; n_end = n_end + self.N_straight ;
            self.x(n_start:n_end) = linspace(self.L_straight/2, -self.L_straight/2, self.N_straight);
            self.y(n_start:n_end) = self.R;
            self.theta(n_start:n_end) = pi;
            self.kappa(n_start:n_end) = 0;
            self.desired_speed(n_start:n_end) = 106;
            
            % Turn 2: Top left to bottom left
            n_start = n_end + 1 ; n_end = n_end + self.N_turn ;
            self.theta(n_start:n_end) = linspace(pi, 2*pi, self.N_turn);
            self.x(n_start:n_end) = -self.L_straight/2 + self.R*sin(self.theta(n_start:n_end));
            self.y(n_start:n_end) = -self.R*cos(self.theta(n_start:n_end));
            self.kappa(n_start:n_end) = 1/self.R;
            self.desired_speed(n_start:n_end) = 100;

            % normalise angles to [-pi, pi]
            self.theta = atan2(sin(self.theta), cos(self.theta));
        end

        
        function self = calc_cum_dist(self)
            dx = diff(self.x);
            dy = diff(self.y);
            ds(1) = 0;
            ds(2:self.N_total) = sqrt(dx.^2 + dy.^2);   % distance between each pair of points
            self.cum_dist = cumsum(ds);
        end

        function [idx, s] = get_nearest_point(self, x_pos, y_pos)
            % Find nearest point using Euclidean distance
            dists = sqrt((self.x - x_pos).^2 + (self.y - y_pos).^2);
            
            % Find candidates within a small threshold of the minimum distance
            min_dist = min(dists);
            threshold = min_dist * 1.1;  % 10% tolerance
            candidates = find(dists <= threshold);
            
            if length(candidates) > 1 && self.last_s > 0
                % Multiple candidates - prefer forward progress
                s_candidates = self.cum_dist(candidates);
                s_total = self.cum_dist(end);
                
                % Calculate forward distances (handling wrapping)
                forward_dists = zeros(size(candidates));
                for i = 1:length(candidates)
                    s_diff = s_candidates(i) - self.last_s;
                    % Handle wrapping
                    if s_diff < -s_total/2
                        s_diff = s_diff + s_total;
                    elseif s_diff > s_total/2
                        s_diff = s_diff - s_total;
                    end
                    forward_dists(i) = s_diff;
                end
                
                % Prefer forward progress, but allow small backward jumps
                % (within 5% of track length) for robustness
                valid = forward_dists > -0.05 * s_total;
                if any(valid)
                    valid_candidates = candidates(valid);
                    [~, best_idx] = max(forward_dists(valid));
                    idx = valid_candidates(best_idx);
                else
                    % All candidates are too far backward, just take closest
                    [~, idx] = min(dists(candidates));
                    idx = candidates(idx);
                end
            else
                % Single candidate or no previous position - use closest
                [~, idx] = min(dists);
            end
            
            s = self.cum_dist(idx);
            self.last_s = s;  % Update last known position
        end

        function [x, y, theta] = get_point_at_dist(self, s)
            % Get track point at a given distance along the track
            % Handle wrapping for continuous lap
            s_total = self.cum_dist(end);
            
            % Wrap s to [0, s_total)
            while s < 0
                s = s + s_total;
            end
            while s >= s_total
                s = s - s_total;
            end
            
            % Find the segment containing distance s
            idx = find(self.cum_dist > s, 1);
            
            % Handle edge cases
            if isempty(idx)
                % s is exactly at the end, use last point
                idx = self.N_total;
                x = self.x(idx);
                y = self.y(idx);
                theta = self.theta(idx);
                return;
            elseif idx == 1
                % s is at or before first point
                idx = 1;
                s0 = 0;
                if self.N_total > 1
                    s1 = self.cum_dist(2);
                else
                    s1 = s_total;
                end
                alpha = max(0, min(1, (s - s0) / (s1 - s0)));  % Clamp to [0, 1]
                
                if self.N_total > 1
                    x = self.x(1) + alpha * (self.x(2) - self.x(1));
                    y = self.y(1) + alpha * (self.y(2) - self.y(1));
                    theta0 = self.theta(1);
                    theta1 = self.theta(2);
                else
                    x = self.x(1);
                    y = self.y(1);
                    theta0 = self.theta(1);
                    theta1 = self.theta(1);
                end
                dtheta = theta1 - theta0;
            else
                % Normal case: s is in the middle
                idx = idx - 1;
                s0 = self.cum_dist(idx);
                s1 = self.cum_dist(idx + 1);
                alpha = (s - s0) / (s1 - s0);
                
                x = self.x(idx) + alpha * (self.x(idx + 1) - self.x(idx));
                y = self.y(idx) + alpha * (self.y(idx + 1) - self.y(idx));
                
                theta0 = self.theta(idx);
                theta1 = self.theta(idx + 1);
                dtheta = theta1 - theta0;
            end
            
            % wrap dtheta into [-pi, pi]
            if dtheta > pi
                dtheta = dtheta - 2*pi;
            elseif dtheta < -pi
                dtheta = dtheta + 2*pi;
            end
        
            theta = theta0 + alpha * dtheta;
        
            % normalize theta to (-pi, pi] if desired:
            theta = atan2(sin(theta), cos(theta));
        end

        function turn_coming = look_ahead(self, idx_current)
            % Simplified look-ahead: check next 20 indices
            % idx_current: current nearest point index
            % Returns: true if turn is coming up (>50% of next 20 points have curvature > 0)
            
            N = length(self.kappa);
            look_ahead_count = 50;
            
            % Count how many of the next 20 points have curvature > 0
            turn_count = 0;
            for i = 1:look_ahead_count
                idx = idx_current + i - 1;
                % Handle wrapping
                if idx > N
                    idx = idx - N;
                end
                if abs(self.kappa(idx)) > 0
                    turn_count = turn_count + 1;
                end
            end
            
            % If more than half (i.e., >10) have curvature > 0, turn is coming
            turn_coming = (turn_count > look_ahead_count / 2);
        end
        
        function control_input = control_input_func(self, t, Z)
            % Z = [x, y, vx, vy]
            x_pos = Z(1);
            y_pos = Z(2);
            
            % Find nearest track point
            [idx, s_current] = self.get_nearest_point(x_pos, y_pos);
            
            % Look ahead 20 indices to see if turn is coming
            turn_coming = self.look_ahead(idx);
            
            % Set desired speed based on look-ahead
            if turn_coming
                % Turn coming up - slow down to cornering speed
                v_desired = 100;  % m/s
            else
                % Straight coming up - go to straight speed
                v_desired = 106;  % m/s
            end
            
            % Return as struct
            control_input = struct('desired_speed', v_desired, ...
                                  'turn_coming', turn_coming);
        end
    end
end
