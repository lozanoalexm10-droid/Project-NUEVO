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



<<<<<<< HEAD
class PurePursuitPlanner2(PathPlanner):
    def __init__(self, lookahead_distance=150.0, max_linear_speed=50.0, goal_tolerance=20.0):
        self.Ld = lookahead_distance
        self.v_max = max_linear_speed  # mm/s
        self.goal_tolerance = goal_tolerance

    def _lookahead_point(self, path, x, y):
        for point in path:
            dx = point[0] - x
            dy = point[1] - y
            if np.hypot(dx, dy) >= self.Ld:
                return point
        return path[-1]
    
    def TargetReached(self, path, x, y):
        goal_x, goal_y = path[-1]
        dist_to_goal = np.hypot(goal_x - x, goal_y - y)
        return dist_to_goal < self.goal_tolerance
        
    def compute_velocity(self, path, pose):
        x, y, theta = pose
        goal_x, goal_y = path[-1]
        dist_to_goal = np.hypot(goal_x - x, goal_y - y)
        if dist_to_goal < self.goal_tolerance:
            return 0.0, 0.0  # Stop if within goal tolerance

        target = self._lookahead_point(path, x, y)
        tx, ty = target

        dx = tx - x
        dy = ty - y

        # Transform to robot frame
        x_r = np.cos(theta) * dx + np.sin(theta) * dy
        y_r = -np.sin(theta) * dx + np.cos(theta) * dy
        Dist2Target = np.hypot(x_r, y_r)
        #curvature = 2.0 * y_r / (self.Ld ** 2)
        curvature = 2.0 * y_r / (Dist2Target ** 2)

        
        v = self.v_max / (1.0 + 2.0 * abs(curvature))  # mm/s
        w = curvature * v  # rad/s
        #print(f"x_r: {x_r:.2f}, y_r: {y_r:.2f}, curvature: {curvature:.4f}, Dist2Target: {Dist2Target:.2f}")
        #print(f"Pose: ({x:.1f}, {y:.1f}, {math.degrees(theta):.1f}°), ", end="")
        #print(f"Target: ({tx:.1f}, {ty:.1f}), Curvature: {curvature:.4f}, v: {v:.2f} mm/s, w: {w:.2f} rad/s")

        return v, w



# =============================================================================
# APF
=======
# =============================================================================
# Dynamic Window Approach (stub — activated when 2D lidar is ready)
>>>>>>> 15d8b77 (updated with obstacle avoidance)
# =============================================================================

class DWAPlanner():
    """
    Artificial Potential Fields planner.

    Combines an attractive force toward the goal with repulsive forces from
    obstacles. Obstacle data comes from get_obstacles(), which reads the
    lidar topic once it is available.

    This first version is usable today with caller-provided robot-frame
    obstacles. A future lidar/object-detection node can feed live obstacles
    through the Robot obstacle-provider API without changing the planner.
    """

    def __init__(
        self,
<<<<<<< HEAD
        lookahead_dist: float = 200,
        max_linear: float = 200,
        max_angular: float = 2.0,
        repulsion_gain: float = 500.0,
        repulsion_range: float = 300.0,
        goal_tolerance: float = 20.0,
        attraction_gain: float = 1.0,
        heading_gain: float = 2.0,
        obstacle_provider: Callable[[], list[tuple[float, float]]] | None = None,
    ) -> None:
        self._lookahead      = lookahead_dist
        self._max_linear     = max_linear
        self._max_angular    = max_angular
        self._rep_gain       = repulsion_gain
        self._rep_range      = repulsion_range
        self.goal_tolerance  = goal_tolerance
        self._attr_gain      = attraction_gain
        self._heading_gain   = heading_gain
        self._obstacle_provider = obstacle_provider

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

        # Attractive force points toward the ordered lookahead target in robot frame.
        goal_x_r = math.cos(theta) * dx + math.sin(theta) * dy
        goal_y_r = -math.sin(theta) * dx + math.cos(theta) * dy
        goal_dist = math.hypot(goal_x_r, goal_y_r)
        if goal_dist < 1e-6:
            return 0.0, 0.0

        attr_x = self._attr_gain * goal_x_r / goal_dist
        attr_y = self._attr_gain * goal_y_r / goal_dist

        rep_x = 0.0
        rep_y = 0.0
        nearest_obstacle = self._rep_range
        for obs_x_r, obs_y_r in self.get_obstacles():
            dist = math.hypot(obs_x_r, obs_y_r)
            if dist < 1e-6 or dist >= self._rep_range:
                continue
            nearest_obstacle = min(nearest_obstacle, dist)
            repulse = self._rep_gain * ((1.0 / dist) - (1.0 / self._rep_range)) / (dist * dist)
            rep_x += repulse * (-obs_x_r / dist)
            rep_y += repulse * (-obs_y_r / dist)

        force_x = attr_x + rep_x
        force_y = attr_y + rep_y
        if math.hypot(force_x, force_y) < 1e-6:
            return 0.0, 0.0

        heading_error = math.atan2(force_y, force_x)
        forward_scale = max(0.0, math.cos(heading_error))
        linear_limit = min(float(max_linear), self._max_linear)
        goal_scale = min(1.0, goal_dist / max(self.goal_tolerance * 2.0, 1e-6))
        linear = linear_limit * forward_scale * goal_scale
        if nearest_obstacle < self._rep_range:
            linear *= max(0.0, min(1.0, nearest_obstacle / self._rep_range))
        if force_x <= 0.0:
            linear = 0.0

        angular = self._heading_gain * heading_error
        angular = max(-self._max_angular, min(self._max_angular, angular))
        return linear, angular

    def get_obstacles(self) -> list[tuple[float, float]]:
        if self._obstacle_provider is None:
            return []
        return list(self._obstacle_provider())

    def _lookahead_point(
        self, x: float, y: float, waypoints: list[tuple[float, float]]
    ) -> tuple[float, float]:
        for wx, wy in waypoints:
            if math.hypot(wx - x, wy - y) >= self._lookahead:
                return wx, wy
        return waypoints[-1]
=======
        lookahead_dist: float = 0.20,
        max_linear_speed: float = 0.50,
        max_angular_speed: float = 1.0,
        max_linear_acc: float = 0.50,
        max_angular_acc: float = 1.0,
        goal_tolerance: float = 0.10,
        gains_of_costs: list[float] = [2.0, 0.1, 0.2, 0.2, 0.1],
        dt: float = 0.1,
        predict_time: float = 2.0,
        predict_velocity_samples_resolution: list[float] = [0.1, 0.1],
        robot_radius: float = 0.20,
        obstacles_range: float = 3.0,
        ttc_weight: float = 0.1,
    ) -> None:
        self.Ld = lookahead_dist
        self.v_max = max_linear_speed  # m/s
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
            if np.min(dists) < min_dist:
                min_dist = np.min(dists)
                ttc = i * self.dt # time-to-collision

            if min_dist < self.robot_radius:
                return float('inf'), min_dist # If the minimum distance to an obstacle is less than or equal to the robot's radius, return infinite cost to indicate a collision risk

        # return 1.0 / (min_dist + ttc * self.ttc_weight + 1e-5), min_dist # cost is inversely proportional to the minimum distance to obstacles
        sigma = 0.1
        return np.exp(- ((min_dist + ttc * self.ttc_weight) ** 2) / (2 * sigma ** 2)), min_dist # Gaussian obstacle cost

    def calc_path_cost(self, traj, path):
        distances = [np.min(np.linalg.norm(path - p[:2], axis=1)) for p in traj]
        return np.mean(distances)

    def obstacle_activation(self, dist):
        if dist > 4.0:
            return 0.0
        elif dist > 2.0:
            return 0.5
        else:
            return 1.0

    def pure_velocity_search(self, pose, target):
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

    def compute_velocity(self, path, pose, velocity, obstacles): # all parameters should be on world frame!!!
        x, y, theta = pose
        if len(obstacles) > 0:
            obstacles = obstacles[np.linalg.norm(obstacles-np.float64([[x, y]]), axis=1)<self.obstacles_range,:]
        vx, vy, w = velocity

        if self.TargetReached(path, x, y):
            return 0.0, 0.0  # Stop if within goal tolerance

        target = self._lookahead_point(path, x, y) # pure pursuit target

        # Obstacle avoidance
        dw = self.calc_dynamic_window(velocity) # find the dynamic window based on current velocity and acceleration limits, which defines the feasible velocity space for the next control command, ensuring that the robot can safely decelerate to a stop if necessary

        best_score = float('inf') 
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
                curvature = abs(w / (v + 1e-5))
                # find the best orientation with largest minimal distance with obstacles, we'll use it when the vehicle is blocked by obstacles.
                # if min_dist > best_min_dist:
                #     best_min_dist = min_dist
                #     best_u_when_blocked = [v, w]

                obs_weight = self.gain_obs_base * self.obstacle_activation(min_dist) # according to the minimum distance to obstacles along the predicted trajectory, dynamically adjust the weight of the obstacle cost, increasing it as the robot gets closer to obstacles to prioritize safety in tight spaces while allowing more aggressive navigation when obstacles are farther away

                score = (
                    self.gain_goal * goal_cost +
                    self.gain_heading * heading_cost +
                    obs_weight * obs_cost +
                    self.gain_speed * (self.v_max - v) + 
                    self.gain_path * path_cost
                )

                if score < best_score:
                    best_score = score
                    best_u = [v, w]

        best_v, best_w = best_u
        if best_score == float('inf') or (abs(best_v) < 0.01 and abs(best_w) < 0.05):
            # return best_u_when_blocked[0], best_u_when_blocked[1]   # rotate in place
            return self.pure_velocity_search(pose, target)   # rotate in place
        
        return  best_v, best_w
>>>>>>> 15d8b77 (updated with obstacle avoidance)


# =============================================================================
# Helper
# =============================================================================

def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi
