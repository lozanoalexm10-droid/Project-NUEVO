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


class PurePursuitPlannerWithAvoidance(PathPlanner):
    def __init__(self,
            lookahead_distance: float=100.0,
            max_linear_speed: float=100.0,
            goal_tolerance: float=20.0,
            obstacles_range: float=300.0,
            safe_dist: float=200.0,
            sharp_angle: float=np.pi/2,
            alpha: float=0.8):
        self.Ld = lookahead_distance
        self.raw_LD = lookahead_distance
        self.v_max = max_linear_speed  # mm/s
        self.goal_tolerance = goal_tolerance
        self.obstacles_range = obstacles_range
        self.safe_dist = safe_dist
        self.sharp_angle = sharp_angle
        self.alpha = alpha
        self.avoidance_active = False

        self.avoidance_counter = 0

    def set_path(self, path: list[tuple[float, float]]):
        self.remaining_path = path.copy()
        self.raw_path = path.copy()

    def _advance_remaining_path(self,
        x: float,
        y: float,
    ) -> list[tuple[float, float]]:
        
        while len(self.remaining_path) > 1:
            next_x_mm, next_y_mm = self.remaining_path[0]
            if np.hypot(x-next_x_mm, y-next_y_mm) > self.Ld:
                break
            self.remaining_path.pop(0)

            # if self.avoidance_active:
            #     self.avoidance_active = False
            #     self.Ld = self.raw_LD

        return self.remaining_path

    def _lookahead_point(self, path, x, y):
        path = np.array(path)
        position = np.array([x, y])

        for i in range(len(path)):
            dist = np.linalg.norm(path[i,:] - position)
            if dist >= self.Ld:
                return path[i]

        return path[-1]
    
    def TargetReached(self, path, x, y):
        goal_x, goal_y = path[0]
        dist_to_goal = np.hypot(goal_x - x, goal_y - y)
        return (dist_to_goal < self.goal_tolerance)

    def compute_velocity(self, pose, obstacles_r: np.nparray):
        # Note that the input obstacle point cloud is in robot frame
        x, y, theta = pose
        self._advance_remaining_path(x,y)
        
        if self.TargetReached(self.remaining_path,x,y):
            return 0.0, 0.0  # Stop if within goal tolerance

        if len(obstacles_r) > 0:
            # lidar orientation due to installation is 180 deg rotated from robot forward, so rotate obstacles accordingly
            # there is a distance between the lidar and the robot center.
            obstacles_r = (np.array([[np.cos(np.pi), -np.sin(np.pi)], [np.sin(np.pi), np.cos(np.pi)]]) @ obstacles_r.T).T 
            
            # since some robot parts (e.g., the arm) may cause obstacles to be detected, we can filter out those obstacles behind the lidar.
            # obstacles_r = obstacles_r[obstacles_r[:,0]>0]
            obstacles_r = obstacles_r[(obstacles_r[:,0]>0) & (np.abs(obstacles_r[:,1])<self.safe_dist),:]

            # consider the lidar offset from the robot center
            # lidar_offset_mm = 100.0
            # obstacles_r = obstacles_r + np.array([[lidar_offset_mm, 0],])

            dists = np.linalg.norm(obstacles_r, axis=1)
            obstacles_r = obstacles_r[(dists < self.obstacles_range)] # only consider obstacles inside range

            # transform obstacles from robot frame to world frame.
            obstacles = (np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]) @ obstacles_r.T).T + np.array([[x, y],])

            if (len(obstacles_r) > 0)  and (self.avoidance_counter <= 0):
                # if desired path is close to obstacles, remove it.
                if (self.remaining_path[0] in self.raw_path) and np.any(np.sqrt(np.sum((np.float64([self.remaining_path[0]])-obstacles)**2, 1)) < self.safe_dist):
                    self.remaining_path.pop(0)

                dists = np.linalg.norm(obstacles_r, axis=1)
                min_dist = np.min(dists)
                closest_pt = obstacles_r[np.argmin(dists),:] # closest obstacle point in robot frame
                
                angle = np.arctan2(closest_pt[1],closest_pt[0]) + theta # angle of the obstacle in world frame
                if min_dist < self.safe_dist:
                    delta_angle = self.sharp_angle # turn sharply if the robot is close to the obstacle
                else:
                    delta_angle = 2*np.abs(np.arctan2(self.safe_dist/2, np.sqrt(min_dist**2 - (self.safe_dist/2)**2)))
                    # delta_angle = self.sharp_angle * 0.2
                print(delta_angle)

                # find the closest waypoint on the desired path
                target = self.raw_path[-1]
                for i in range(len(self.remaining_path)):
                    if self.remaining_path[i] in self.raw_path:
                        target = self.remaining_path[i]
                        break
                
                avoid_dist = min_dist
                # avoid_dist = max(min_dist, self.safe_dist * 1.2)
                # Two candidate waypoints in +- orientations
                candidate_waypoint1 = (x+avoid_dist*np.cos(angle+delta_angle), y+avoid_dist*np.sin(angle+delta_angle)) # left turn
                candidate_waypoint2 = (x+avoid_dist*np.cos(angle-delta_angle), y+avoid_dist*np.sin(angle-delta_angle)) # left right

                # In avoidance mode, we should romove the previous added waypoint
                if self.avoidance_active:
                    self.remaining_path.pop(0)

                # find the candidate waypoints which is close to the desired path.
                if np.hypot(target[0]-candidate_waypoint1[0], target[1]-candidate_waypoint1[1]) < np.hypot(target[0]-candidate_waypoint2[0], target[1]-candidate_waypoint2[1]):
                    self.remaining_path.insert(0, candidate_waypoint1)
                    self.avoidance_active = True
                else:
                    self.remaining_path.insert(0, candidate_waypoint2)
                    self.avoidance_active = True

                # self.Ld = max(self.raw_LD * 0.3, 50.0)
                self.Ld = self.raw_LD * self.alpha # reduce lookahead distance to track added waypoints more precisely.

                self.avoidance_counter = 20 # keep avoidance active for a few cycles to ensure the robot reacts to the obstacle.
            # else:
            #     self.avoidance_counter -= 1
        else:
            self.Ld = self.raw_LD
            self.avoidance_active = False
        
        if self.avoidance_counter>0:
            self.avoidance_counter -= 1

        target = self._lookahead_point(self.remaining_path, x, y)
        tx, ty = target

        dx = tx - x
        dy = ty - y

        # Transform to robot frame
        # x_r = np.cos(theta) * dx + np.sin(theta) * dy
        y_r = -np.sin(theta) * dx + np.cos(theta) * dy
        # Dist2Target = np.hypot(x_r, y_r)

        curvature = 2.0 * y_r / (self.Ld ** 2)
        # curvature = 2.0 * y_r / (self.raw_LD ** 2)
        # curvature = 2.0 * y_r / (Dist2Target ** 2)

        
        v = self.v_max / (1.0 + 2.0 * abs(curvature))  # m/s
        w = curvature * v  # rad/s

        return v, w

    def motion(self, pose, v, w, dt):
        x, y, theta = pose
        pose[0] += v * np.cos(theta) * dt
        pose[1] += v * np.sin(theta) * dt
        pose[2] += w * dt
        return pose


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
        gains_of_costs: list[float] = [2.0, 0.02, 0.2, 0.5, 0.1],
        dt: float = 0.1,
        predict_time: float = 3.0,
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

        # All remaining path points are within lookahead distance — robot is at
        # or past the final waypoint.  Advance current_index to the last point so
        # TargetReached() (which requires current_index == len(path)-1) can fire.
        self.current_index = len(path) - 1
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
        pose[0] += v * np.cos(theta) * dt
        pose[1] += v * np.sin(theta) * dt
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
        ttc = self.predict_time

        for i, point in enumerate(traj):
            dists = np.linalg.norm(obstacles - point[:2], axis=1)
            # min_dist = min(min_dist, np.min(dists))

            if np.min(dists) < min_dist:
                min_dist = np.min(dists)
                ttc = i * self.dt # time-to-collision

            if min_dist < self.robot_radius:
                return float('inf'), min_dist # If the minimum distance to an obstacle is less than or equal to the robot's radius, return infinite cost to indicate a collision risk

        min_dist /= 1000
        # return 1.0 / (min_dist + ttc * self.ttc_weight + 1e-5), min_dist # cost is inversely proportional to the minimum distance to obstacles
        sigma = 0.1
        return np.exp(- ((min_dist + ttc * self.ttc_weight) ** 2) / (2 * sigma ** 2)), min_dist # Gaussian obstacle cost

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
        best_u_when_blocked = [0., self.v_max/2] # default to a forward velocity when blocked, which can help escape from local minima by pushing the robot forward to get out of the tight spot, while still allowing it to slow down or stop if necessary based on obstacle proximity 
        best_min_dist = 0.

        for v in np.arange(-self.v_max, self.v_max+self.sample_rx[0], self.sample_rx[0]):
            for w in np.arange(-self.w_max, self.w_max+self.sample_rx[1], self.sample_rx[1]):
                traj = self.predict_trajectory(pose, np.array([v,0,w]))
                obs_cost, min_dist = self.calc_obstacle_cost(traj, obstacles)
                # Only accept trajectories with no collision along the full horizon.
                if obs_cost != float('inf') and min_dist > best_min_dist:
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
            
            # obstacles = np.float64([(0,1000.0),]) # virtual obstacle for testing
            dists = np.linalg.norm(obstacles-np.float64([[x, y]]), axis=1)
            obstacles = obstacles[(dists >= self.robot_radius) & (dists < self.obstacles_range)]

            # print(f"Min distance: {np.min(dists):.4f}")
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
                # print(f"goal_cost: {goal_cost:.4f}, heading_cost: {heading_cost:.4f}, obs_cost: {obs_cost:.4f}, path_cost: {path_cost:.4f}, speed_cost: {(-v) / 1000.0:.4f}, total_cost: {cost:.4f}")

                if cost < best_cost:
                    best_cost = cost
                    best_u = [v, w]

        best_v, best_w = best_u
        # print(f"Best cost: {best_cost:.4f}")
        # Only fall back to pure_velocity_search when every trajectory in the
        # dynamic window collides (best_cost == inf).  The previous condition
        # also triggered on abs(best_v) < 10 and abs(best_w) < 0.05, which fired
        # when the DWA correctly chose to slow to near-zero in front of an obstacle
        # — overriding the legitimate stop with a full-space search that almost
        # always returned a non-zero velocity and drove the robot into the obstacle.
        if best_cost == float('inf'):
            return self.pure_velocity_search(pose, obstacles)
        
        return best_v, best_w


# =============================================================================
# Helper
# =============================================================================

def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi
