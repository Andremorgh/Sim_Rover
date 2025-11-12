import os
import math
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory

from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time

class QuantumController(Node):
    def __init__(self):
        super().__init__('quantum_controller')

        # Parametri per i frame TF 
        self.declare_parameter('map_frame', 'rover/map')
        self.declare_parameter('base_frame', 'rover/base_footprint')
        
        # Legge i parametri (se non specificati, usa i default)
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value or 'rover/map'
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value or 'rover/base_footprint'

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher e Subscriber principali
        self.cmd_pub = self.create_publisher(Twist, 'rover/cmd_vel', 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'rover/goal_pose', self.goal_callback, 10)

        # Subscriber per l'angolo ostacolo (dal lidar_listener)
        self.obstacle_sub = self.create_subscription(Float32, '/obstacle', self.obstacle_callback, 10)
        self.min_dist_sub = self.create_subscription(Float32, '/min_distance', self.min_distance_callback, 10)

        # Stato interno
        self.goal_received = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Ostacolo (inizializzato a 3.14 come fallback)
        self.obstacle_angle = 3.14
        self.min_distance = 10.0

        # Timer di controllo
        self.timer = self.create_timer(0.01, self.control_loop)

        # Percorso cartella config nel package
        pkg_share = get_package_share_directory('quantum_controller')

        # Carica LUT
        self.linear_LUT = self.load_LUT(
            os.path.join(pkg_share, 'config', 'LUT_lin.csv'),
            ['dist_error', 'obstacle', 'linear_velocity']
        )
        self.angular_LUT = self.load_LUT(
            os.path.join(pkg_share, 'config', 'LUT_ang.csv'),
            ['orient_error', 'obstacle', 'angular_velocity']
        )

        self.get_logger().info("✅ Quantum Controller avviato con LIDAR e LUT interpolata")
    
    def update_pose_from_tf(self) -> bool:
        """
        Aggiorna robot_x, robot_y, robot_yaw usando TF map -> base_link.
        Ritorna True se la posa è stata aggiornata, False altrimenti.
        """
        try:
            # Time() senza argomenti = tempo "0" (ultimo disponibile). Aggiungo un timeout.
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,      # target frame
                self.base_frame,     # source frame
                Time(),              # latest
                rclpy.duration.Duration(seconds=0.1)  # timeout
            )
        except TransformException as ex:
            # Log limitato per non spammare
            if not hasattr(self, "_last_tf_warn") or (self.get_clock().now() - self._last_tf_warn).nanoseconds > 5e8:
                self.get_logger().warn(f"TF non disponibile ({self.map_frame}←{self.base_frame}): {ex}")
                self._last_tf_warn = self.get_clock().now()
            return False

        # Posizione
        self.robot_x = trans.transform.translation.x
        self.robot_y = trans.transform.translation.y

        # Orientamento → yaw
        q = trans.transform.rotation
        # yaw da quaternion (z-asse)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        return True

    # ---------------------- CALLBACKS ----------------------

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_received = True
        self.get_logger().info(f"🎯 Nuovo goal: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def obstacle_callback(self, msg: Float32):
        """Riceve l'angolo dell'ostacolo pubblicato da lidar_listener"""
        self.obstacle_angle = msg.data
        # Normalizza a [-pi, pi]
        #self.obstacle_angle = math.atan2(math.sin(self.obstacle_angle), math.cos(self.obstacle_angle))

    def min_distance_callback(self, msg: Float32):
        """Riceve la distanza minima (non usata direttamente ora, ma utile per debug)"""
        self.min_distance = msg.data

    # ---------------------- LUT LOADER ----------------------

    def load_LUT(self, csv_path, columns):
        """Carica LUT CSV e costruisce griglia per interpolazione bilineare."""
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
            self.get_logger().warn(f"⚠️ LUT {os.path.basename(csv_path)} ha pochi punti → nearest neighbor")
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

    # ---------------------- INTERPOLAZIONE ----------------------

    def interpolate_LUT(self, lut, x, y):
        """Interpolazione bilineare 2D (clamped ai bordi)."""
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
        
        # Aggiorna posa da TF; se non disponibile, salta il ciclo
        if not self.update_pose_from_tf():
            return

        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        dist_error = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        orient_error = self.normalize_angle(target_angle - self.robot_yaw)

        # Interpolazione LUT con angolo ostacolo aggiornato dal lidar
        v = self.interpolate_LUT(self.linear_LUT, dist_error, self.obstacle_angle)
        w = self.interpolate_LUT(self.angular_LUT, orient_error, self.obstacle_angle)

        if dist_error < 0.05:
            v, w = 0.0, 0.0

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_pub.publish(twist)

        now = self.get_clock().now().seconds_nanoseconds()[0]
        if not hasattr(self, "_last_log_time"):
            self._last_log_time = 0.0
        if now - self._last_log_time > 0.5:
            self.get_logger().info(
                f"x={self.robot_x:.2f}, y={self.robot_y:.2f}, yaw={self.robot_yaw:.2f} | "
                f"dist={dist_error:.2f}, obs={self.obstacle_angle:.2f}, v={v:.2f}, w={w:.2f}"
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
