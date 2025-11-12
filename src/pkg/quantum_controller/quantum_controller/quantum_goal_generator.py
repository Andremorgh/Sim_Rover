import os
import math
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory


class QuantumController(Node):
    def __init__(self):
        super().__init__('quantum_controller')

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom/wheels', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.obstacle_sub = self.create_subscription(Float32, '/obstacle', self.obstacle_callback, 10)

        # Publisher (per local planner)
        self.plan_pub = self.create_publisher(Path, '/plan', 10)

        # Stato interno
        self.goal_received = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.obstacle_angle = 0.0

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Caricamento LUT
        pkg_share = get_package_share_directory('quantum_controller')
        self.LUT_x = self.load_LUT(os.path.join(pkg_share, 'config', 'LUT_POS_X.csv'),
                                   ['dist_error', 'obstacle', 'pos_x'])
        self.LUT_y = self.load_LUT(os.path.join(pkg_share, 'config', 'LUT_POS_Y.csv'),
                                   ['orient_error', 'obstacle', 'pos_y'])

        self.get_logger().info("✅ QuantumController (LUT→plan) avviato — pubblica su /plan per il local planner")

    # ---------------------- CALLBACKS ----------------------

    def odom_callback(self, msg: Odometry):
        """Aggiorna posizione e orientamento del robot."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg: PoseStamped):
        """Riceve un goal globale (in frame mappa)."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_received = True
        self.get_logger().info(f"🎯 Nuovo goal globale: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def obstacle_callback(self, msg: Float32):
        """Aggiorna angolo ostacolo (da lidar_listener o sim)."""
        self.obstacle_angle = msg.data

    # ---------------------- LUT HANDLING ----------------------

    def load_LUT(self, csv_path, columns):
        """Carica LUT da CSV e costruisce una griglia per interpolazione 2D."""
        if not os.path.isfile(csv_path):
            self.get_logger().warn(f"⚠️ File LUT non trovato: {csv_path}")
            return None

        data = []
        with open(csv_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                try:
                    vals = [float(row[c]) for c in columns]
                    data.append(vals)
                except (ValueError, KeyError):
                    continue

        data = np.array(data)
        if data.shape[0] < 4:
            return {"type": "points", "data": data}

        x_unique = np.unique(data[:, 0])
        y_unique = np.unique(data[:, 1])
        z_grid = np.zeros((len(x_unique), len(y_unique)))

        for i, x in enumerate(x_unique):
            for j, y in enumerate(y_unique):
                mask = (np.isclose(data[:, 0], x)) & (np.isclose(data[:, 1], y))
                vals = data[mask, 2]
                z_grid[i, j] = np.mean(vals) if vals.size > 0 else 0.0

        self.get_logger().info(
            f"✅ LUT '{os.path.basename(csv_path)}' caricata ({len(x_unique)}x{len(y_unique)} punti)"
        )
        return {"x": x_unique, "y": y_unique, "z": z_grid, "type": "grid"}

    def interpolate_LUT(self, lut, x, y):
        """Interpolazione bilineare 2D."""
        if lut is None:
            return 0.0
        if lut["type"] == "points":
            data = lut["data"]
            diffs = (data[:, 0] - x) ** 2 + (data[:, 1] - y) ** 2
            return float(data[np.argmin(diffs), 2])

        X, Y, Z = lut["x"], lut["y"], lut["z"]
        x = np.clip(x, X[0], X[-1])
        y = np.clip(y, Y[0], Y[-1])
        i = np.searchsorted(X, x) - 1
        j = np.searchsorted(Y, y) - 1
        i = np.clip(i, 0, len(X) - 2)
        j = np.clip(j, 0, len(Y) - 2)

        x1, x2 = X[i], X[i + 1]
        y1, y2 = Y[j], Y[j + 1]
        Q11, Q12 = Z[i, j], Z[i, j + 1]
        Q21, Q22 = Z[i + 1, j], Z[i + 1, j + 1]
        if (x2 - x1) == 0 or (y2 - y1) == 0:
            return float(Q11)

        val = (
            Q11 * (x2 - x) * (y2 - y)
            + Q21 * (x - x1) * (y2 - y)
            + Q12 * (x2 - x) * (y - y1)
            + Q22 * (x - x1) * (y - y1)
        ) / ((x2 - x1) * (y2 - y1))
        return float(val)

    # ---------------------- CONTROL LOOP ----------------------

    def control_loop(self):
        if not self.goal_received:
            return

        # Distanza e direzione verso il goal globale
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        goal_dist = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        orient_error = self.normalize_angle(target_angle - self.robot_yaw)

        # Offset locale dal robot (frame robot)
        offset_x = self.interpolate_LUT(self.LUT_x, goal_dist, self.obstacle_angle)
        offset_y = self.interpolate_LUT(self.LUT_y, orient_error, self.obstacle_angle)

        # Trasformazione nel frame mappa (globale)
        local_goal_x = self.robot_x + math.cos(self.robot_yaw) * offset_x - math.sin(self.robot_yaw) * offset_y
        local_goal_y = self.robot_y + math.sin(self.robot_yaw) * offset_x + math.cos(self.robot_yaw) * offset_y

        # Costruisci messaggio Path per il local planner
        local_pose = PoseStamped()
        local_pose.header.frame_id = 'map'
        local_pose.header.stamp = self.get_clock().now().to_msg()
        local_pose.pose.position.x = local_goal_x
        local_pose.pose.position.y = local_goal_y
        local_pose.pose.orientation.w = 1.0

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = [local_pose]

        # Pubblica sul topic del local planner
        self.plan_pub.publish(path_msg)

        now = self.get_clock().now().seconds_nanoseconds()[0]
        if not hasattr(self, "_last_log_time"):
            self._last_log_time = 0.0
        if now - self._last_log_time > 0.5:
            self.get_logger().info(
                f"🎯 GoalGlob=({self.goal_x:.2f},{self.goal_y:.2f}) | Obs={self.obstacle_angle:.2f} → "
                f"dist={goal_dist:.2f}, orient_error={orient_error:.2f} → "
                f"LocalGoal=({local_goal_x:.2f},{local_goal_y:.2f}) → /plan"
            )
            self._last_log_time = now

    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = QuantumController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
