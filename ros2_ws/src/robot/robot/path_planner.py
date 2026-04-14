"""
path_planner.py — pure-algorithm path planning library
=======================================================
These classes are stateless algorithm helpers. They do NOT own threads or
ROS subscriptions. The Robot class calls compute_velocity() from its own
navigation thread.

To use a planner in your navigation code, just instantiate it and call
compute_velocity() with the current pose and remaining waypoints.
"""

from __future__ import annotations

from collections.abc import Callable
import math
import numpy as np

from rclpy.node import Node

# =============================================================================
# Base class
# =============================================================================

class PathPlanner:
    """
    Abstract base for path planning algorithms.

    Subclasses implement compute_velocity() and optionally get_obstacles().
    """

    def compute_velocity(
        self,
        pose: tuple[float, float, float],
        waypoints: list[tuple[float, float]],
        max_linear: float,
    ) -> tuple[float, float]:
        """
        Return (linear, angular) velocity command.

          pose       — (x, y, theta_rad) in any consistent unit
          waypoints  — remaining waypoints in the same unit, nearest first
          max_linear — maximum forward speed in that unit/s

        Returns (linear, angular_rad_s).
        """
        raise NotImplementedError

    def get_obstacles(self) -> list:
        """
        Return a list of obstacle positions in the robot's frame.
        Override when a 2D lidar topic is available.
        """
        return []


# =============================================================================
# Pure Pursuit
# =============================================================================

class PurePursuitPlanner(PathPlanner):
    """
    Pure-pursuit path follower for differential drive.

    Steers toward a lookahead point on the path. Works well for smooth
    curves. The lookahead_dist controls the trade-off between responsiveness
    (small) and smoothness (large).

    Parameters:
        lookahead_dist — how far ahead on the path to aim at (same units as pose)
        max_angular    — maximum angular rate (rad/s)
    """

    def __init__(
        self,
        lookahead_dist: float = 150,
        max_angular: float = 2.0,
        goal_tolerance: float = 20.0,
    ) -> None:
        self._lookahead  = lookahead_dist
        self._max_angular = max_angular
        self.goal_tolerance = goal_tolerance

    def compute_velocity(
        self,
        pose: tuple[float, float, float],
        waypoints: list[tuple[float, float]],
        max_linear: float,
    ) -> tuple[float, float]:
        x, y, theta = pose
        tx, ty = self._lookahead_point(x, y, waypoints)
        dx = tx - x
        dy = ty - y

        # Transform the lookahead point into the robot frame.
        x_r = math.cos(theta) * dx + math.sin(theta) * dy
        y_r = -math.sin(theta) * dx + math.cos(theta) * dy
        dist = math.hypot(x_r, y_r)

        if dist < 1e-6:
            return 0.0, 0.0

        # Standard pure-pursuit curvature for a differential-drive robot.
        curvature = 2.0 * y_r / (dist * dist)

        # Slow down for high-curvature turns. The lookahead-scaled term is
        # dimensionless and gives a smooth transition between straight driving
        # and tight cornering.
        forward_scale = max(0.0, x_r / dist)
        curvature_scale = 1.0 + abs(curvature) * self._lookahead
        linear = max_linear * forward_scale / curvature_scale

        if linear <= 1e-6:
            angular = self._max_angular * math.tanh(y_r / max(self._lookahead, 1e-6))
            return 0.0, angular

        angular = curvature * linear
        if abs(angular) > self._max_angular:
            angular = math.copysign(self._max_angular, angular)
            linear = min(linear, abs(angular / curvature)) if abs(curvature) > 1e-6 else linear

        return linear, angular

    def _lookahead_point(
        self, x: float, y: float, waypoints: list[tuple[float, float]]
    ) -> tuple[float, float]:
        """
        Return the first ordered waypoint beyond the lookahead distance.

        The caller is expected to pass the remaining path in route order. That
        avoids Euclidean nearest-point jumps around corners, which otherwise
        make the lookahead target chatter between the incoming and outgoing
        path segments.
        """
        for wx, wy in waypoints:
            if math.hypot(wx - x, wy - y) >= self._lookahead:
                return wx, wy
        return waypoints[-1]
    
    def CurrentTargetReached(self, target_x, target_y, x, y):
        dist_to_target = np.hypot(target_x - x, target_y - y)
        return dist_to_target < self.goal_tolerance



# =============================================================================
# Dynamic Window Approach (stub — activated when 2D lidar is ready)
# =============================================================================

class DWAPlanner():
    # Dynamic Window Approach

    def __init__(
        self,
        lookahead_dist: float = 200.0,
        max_linear_speed: float = 200.0,
        max_angular_speed: float = 1.0,
        max_linear_acc: float = 300.0,
        max_angular_acc: float = 2.0,
        goal_tolerance: float = 100.0,
        gains_of_costs: list[float] = [2.0, 0.02, 0.2, 0.8, 0.1],
        dt: float = 0.1,
        predict_time: float = 2.0,
        predict_velocity_samples_resolution: list[float] = [20.0, 0.1],
        robot_radius: float = 150.0,
        obstacles_range: float = 1000.0,
        ttc_weight: float = 0.1,
    ) -> None:
        self.Ld = lookahead_dist
        self.v_max = max_linear_speed  # mm/s
        self.w_max = max_angular_speed  # rad/s
        self.dv_max = max_linear_acc 
        self.dw_max = max_angular_acc
        self.goal_tolerance = goal_tolerance

        self.gain_goal, self.gain_heading, self.gain_obs_base, self.gain_speed, self.gain_path = gains_of_costs
        self.ttc_weight = ttc_weight
        self.dt = dt
        self.predict_time = predict_time
        self.sample_rx = predict_velocity_samples_resolution
        self.robot_radius = robot_radius
        self.obstacles_range = obstacles_range

        self.current_index = 0

    def _lookahead_point(self, path, x, y):
        position = np.array([x, y])

        for i in range(self.current_index, len(path)):
            dist = np.linalg.norm(path[i,:] - position)
            if dist >= self.Ld:
                self.current_index = i
                return path[i]

        return path[-1]

    def TargetReached(self, path, x, y):
        goal_x, goal_y = path[-1]
        dist_to_goal = np.hypot(goal_x - x, goal_y - y)
        return (dist_to_goal < self.goal_tolerance) and (self.current_index==len(path)-1)

    # Get the dynamic window based on current velocity and acceleration limits
    def calc_dynamic_window(self, velocity):
        vx, vy, w = velocity
        v = np.hypot(vx, vy)

        return [
            max(-self.v_max, v - self.dv_max * self.dt), 
            min(self.v_max, v + self.dv_max * self.dt), 
            max(-self.w_max, w - self.dw_max * self.dt),
            min(self.w_max, w + self.dw_max * self.dt)
        ]
    
    def motion(self, pose, v, w, dt):
        x, y, theta = pose
        pose[0] += v * np.cos(theta) * self.dt
        pose[1] += v * np.sin(theta) * self.dt
        pose[2] += w * dt
        return pose
    
    def predict_trajectory(self, pose, velocity):
        vx, vy, w = velocity
        v = np.hypot(vx, vy)
        traj = [] # save the predicted trajectory for cost evaluation
        pose = np.float64(pose)
        time = 0
        while time <= self.predict_time: # Incrementally predict the trajectory over the prediction horizon with different velocity commands
            pose = self.motion(pose.copy(), v, w, self.dt) 
            traj.append(pose.copy()) 
            time += self.dt

        return np.array(traj)

    # calculate the cost of a trajectory based on distance to goal
    def calc_goal_cost(self, traj, target):
        dx = target[0] - traj[-1, 0]
        dy = target[1] - traj[-1, 1]
        dx /= 1000
        dy /= 1000
        return np.hypot(dx, dy)

    # heading
    def calc_heading_cost(self, traj, target, path):
        dx = target[0] - traj[-1, 0]
        dy = target[1] - traj[-1, 1]

        # index = min(len(path)-2, self.current_index)
        # dx = path[index+1, 0] - path[index, 0]
        # dy = path[index+1, 1] - path[index, 1]

        angle_to_goal = math.atan2(dy, dx)
        error = angle_to_goal - traj[-1, 2]
        return abs(math.atan2(math.sin(error), math.cos(error)))

    # calculate the cost of a trajectory based on distance to obstacles, returning both the cost and the minimum distance to any obstacle
    def calc_obstacle_cost(self, traj, obstacles):
        if len(obstacles) == 0: # If there are no obstacles, the cost is zero and the minimum distance is infinite, indicating no risk of collision
            return 0.0, float('inf')

        min_dist = float('inf')

        for i, point in enumerate(traj):
            dists = np.linalg.norm(obstacles - point[:2], axis=1)
            # min_dist = min(min_dist, np.min(dists))
            # BUG (ttc reset inside loop): ttc is reset to self.predict_time at the
            # start of every loop iteration.  If the closest approach happened on
            # iteration k but iteration k+1 does NOT update min_dist, ttc is left as
            # self.predict_time instead of k*dt.  The final returned ttc is therefore
            # wrong whenever the minimum-distance point is not the last trajectory step.
            # Fix: initialize ttc = self.predict_time BEFORE the loop, remove this
            # line from inside the loop, and only overwrite it inside the if block.
            ttc = self.predict_time
            if np.min(dists) < min_dist:
                min_dist = np.min(dists)
                ttc = i * self.dt # time-to-collision

            if min_dist < self.robot_radius:
                return float('inf'), min_dist # If the minimum distance to an obstacle is less than or equal to the robot's radius, return infinite cost to indicate a collision risk

        min_dist /= 1000
        return 1.0 / (min_dist + ttc * self.ttc_weight + 1e-5), min_dist # cost is inversely proportional to the minimum distance to obstacles
        # sigma = 1.0
        # return np.exp(- ((min_dist + ttc * self.ttc_weight) ** 2) / (2 * sigma ** 2)), min_dist # Gaussian obstacle cost

    def calc_path_cost(self, traj, path):
        distances = [np.min(np.linalg.norm(path - p[:2], axis=1)) for p in traj]
        return np.mean(distances) / 1000

    # def obstacle_activation(self, dist):
    #     if dist > self.obstacles_range:
    #         return 0.0
    #     elif dist > self.obstacles_range / 2:
    #         return 0.5
    #     else:
    #         return 1.0

    def pure_velocity_search(self, pose, obstacles):
        # NOTE (self-detection fallback): this function is called when every DWA
        # trajectory has infinite cost (robot appears to be stuck).  However, it
        # receives the same already-world-frame-transformed obstacles as compute_velocity.
        # If self-body lidar readings (e.g. from range == 0 or chassis reflections)
        # survived the forward filter and are within robot_radius of the robot,
        # calc_obstacle_cost will return inf for every trajectory here too, and
        # best_u_when_blocked stays [0., 0.] — the robot stops instead of recovering.
        # The root fix is filtering zero-range and below-range_min readings in
        # _on_lidar before they ever reach the planner.
        best_u_when_blocked = [0., 0.]
        best_min_dist = 0.

        for v in np.arange(-self.v_max, self.v_max+self.sample_rx[0], self.sample_rx[0]): # evaluate a set of linear velocity samples within the dynamic window
            for w in np.arange(-self.w_max, self.w_max+self.sample_rx[1], self.sample_rx[1]):
                traj = self.predict_trajectory(pose, np.array([v,0,w]))
                obs_cost, min_dist = self.calc_obstacle_cost(traj, obstacles)
                # find the best orientation with largest minimal distance with obstacles, we'll use it when the vehicle is blocked by obstacles.
                if min_dist > best_min_dist:
                    best_min_dist = min_dist
                    best_u_when_blocked = [v, w]

        return best_u_when_blocked[0], best_u_when_blocked[1]

    def compute_velocity(self, path, pose, velocity, obstacles, dt): # all parameters should be on world frame!!!
        self.dt = dt
        x, y, theta = pose
        if len(obstacles) > 0:
            # lidar orientation due to installation is 180 deg rotated from robot forward, so rotate obstacles accordingly
            # there is a distance between the lidar and the robot center.
            lidar_offset_mm = 100.0
            obstacles = (np.array([[np.cos(np.pi), -np.sin(np.pi)], [np.sin(np.pi), np.cos(np.pi)]]) @ obstacles.T).T + np.array([[lidar_offset_mm, 0],])
            # since some robot parts (e.g., the arm) may cause obstacles to be detected, we can filter out those obstacles behind the lidar.
            obstacles = obstacles[obstacles[:,0] > 0,:]
            # transform obstacles from robot frame to world frame.
            obstacles = (np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]) @ obstacles.T).T + np.array([[x, y],])
            dists = np.linalg.norm(obstacles-np.float64([[x, y]]), axis=1)
            obstacles = obstacles[dists<self.obstacles_range,:]

            # print(f"Min distance: {np.min(dists):.4f}")

            # BUG (missing inner distance filter): only an upper-bound range filter is
            # applied.  Lidar points on the robot's own body that survived the
            # forward-facing filter (obstacles[:,0] > 0) but are closer than
            # robot_radius in world frame are not removed.  calc_obstacle_cost then
            # returns inf for the very first trajectory step because the robot "starts
            # inside" those self-detected points.  Every (v, w) sample gets cost=inf,
            # best_u stays [0, 0], and the robot stops or falls through to
            # pure_velocity_search (which has the same problem).
            # The primary fix is in _on_lidar: filter out range < msg.range_min so
            # zero-range readings never enter the pipeline.  A secondary defensive fix
            # here: obstacles = obstacles[(dists >= self.robot_radius) & (dists < self.obstacles_range)]
        vx, vy, w = velocity

        if self.TargetReached(path, x, y):
            return 0.0, 0.0  # Stop if within goal tolerance

        target = self._lookahead_point(path, x, y) # pure pursuit target

        # Obstacle avoidance
        dw = self.calc_dynamic_window(velocity) # find the dynamic window based on current velocity and acceleration limits, which defines the feasible velocity space for the next control command, ensuring that the robot can safely decelerate to a stop if necessary

        best_cost = float('inf') 
        best_u = [0.0, 0.0]
        # best_u_when_blocked = [0., 0.]
        # best_min_dist = 0.

        # for v in np.linspace(dw[0], dw[1], self.sample_num): # evaluate a set of linear velocity samples within the dynamic window
        #     for w in np.linspace(dw[2], dw[3], self.sample_num): # evaluate a set of angular velocity samples within the dynamic window
        for v in np.arange(dw[0], dw[1]+self.sample_rx[0], self.sample_rx[0]): # evaluate a set of linear velocity samples within the dynamic window
            for w in np.arange(dw[2], dw[3]+self.sample_rx[1], self.sample_rx[1]):
                traj = self.predict_trajectory(pose, np.array([v,0,w]))
                goal_cost = self.calc_goal_cost(traj, target)
                heading_cost = self.calc_heading_cost(traj, target, path)
                path_cost = self.calc_path_cost(traj, path)
                obs_cost, min_dist = self.calc_obstacle_cost(traj, obstacles)

                # curvature = abs(w / (v + 1e-5))
                # find the best orientation with largest minimal distance with obstacles, we'll use it when the vehicle is blocked by obstacles.
                # if min_dist > best_min_dist:
                #     best_min_dist = min_dist
                #     best_u_when_blocked = [v, w]

                obs_weight = self.gain_obs_base # * self.obstacle_activation(min_dist) # according to the minimum distance to obstacles along the predicted trajectory, dynamically adjust the weight of the obstacle cost, increasing it as the robot gets closer to obstacles to prioritize safety in tight spaces while allowing more aggressive navigation when obstacles are farther away

                cost = (
                    self.gain_goal * goal_cost +
                    self.gain_heading * heading_cost +
                    obs_weight * obs_cost +
                    self.gain_speed * (-v) / 1000 + 
                    self.gain_path * path_cost
                )

                if cost < best_cost:
                    best_cost = cost
                    best_u = [v, w]

        best_v, best_w = best_u
        # print(f"Best cost: {best_cost:.4f}")
        if best_cost == float('inf') or (abs(best_v) < (0.01 * 1000) and abs(best_w) < 0.05):
            # return best_u_when_blocked[0], best_u_when_blocked[1]   # rotate in place
            return self.pure_velocity_search(pose, obstacles)   # rotate in place
        
        return  best_v, best_w


# =============================================================================
# Helper
# =============================================================================

def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi
