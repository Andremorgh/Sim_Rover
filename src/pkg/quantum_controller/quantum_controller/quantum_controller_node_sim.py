import os
import math
import csv
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory

from tf2_ros import Buffer, TransformListener, TransformException

# --- SciPy opzionale (Opzione B) ---
try:
    from scipy.interpolate import griddata
    _HAS_SCIPY = True
except Exception:
    griddata = None
    _HAS_SCIPY = False


class QuantumController(Node):
    def __init__(self):
        super().__init__('quantum_controller')

        # --------------------- Parametri TF ---------------------
        self.declare_parameter('map_frame', 'rover/map')
        self.declare_parameter('base_frame', 'rover/base_footprint')

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value or 'rover/map'
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value or 'rover/base_footprint'

        # --------------------- Parametri LUT (griglia per CSV sparsi) ---------------------
        self.declare_parameter('lut_nx', 100)          # nodi lungo X
        self.declare_parameter('lut_ny', 100)          # nodi lungo Y
        self.declare_parameter('lut_method', 'linear') # 'linear' | 'cubic'

        self.lut_nx = int(self.get_parameter('lut_nx').get_parameter_value().integer_value or 100)
        self.lut_ny = int(self.get_parameter('lut_ny').get_parameter_value().integer_value or 100)
        self.lut_method = self.get_parameter('lut_method').get_parameter_value().string_value or 'linear'

        # --------------------- TF2 ---------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --------------------- Pub/Sub ---------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Dati dal LIDAR
        self.obstacle_sub = self.create_subscription(Float32, '/obstacle', self.obstacle_callback, 10)
        self.min_dist_sub = self.create_subscription(Float32, '/min_distance', self.min_distance_callback, 10)

        # --------------------- Stato interno ---------------------
        self.goal_received = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Ostacoli
        self.obstacle_angle = 3.14
        self.min_distance = 10.0

        # --------------------- Timer di controllo ---------------------
        self.timer = self.create_timer(0.01, self.control_loop)

        # --------------------- Caricamento LUT ---------------------
        pkg_share = get_package_share_directory('quantum_controller')

        self.linear_LUT = self.load_LUT(
            os.path.join(pkg_share, 'config', 'LUT_lin.csv'),
            ['dist_error', 'obstacle', 'linear_velocity']
        )
        self.angular_LUT = self.load_LUT(
            os.path.join(pkg_share, 'config', 'LUT_ang.csv'),
            ['orient_error', 'obstacle', 'angular_velocity']
        )

        self.get_logger().info("✅ Quantum Controller avviato con LIDAR e LUT interpolata")

    # ---------------------- TF pose ----------------------

    def update_pose_from_tf(self) -> bool:
        """
        Aggiorna robot_x, robot_y, robot_yaw usando TF map -> base_link.
        Ritorna True se la posa è stata aggiornata, False altrimenti.
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,      # target frame
                self.base_frame,     # source frame
                Time(),              # latest
                timeout=Duration(seconds=0.1)
            )
        except TransformException as ex:
            if not hasattr(self, "_last_tf_warn") or (self.get_clock().now() - self._last_tf_warn).nanoseconds > 5e8:
                self.get_logger().warn(f"TF non disponibile ({self.map_frame}←{self.base_frame}): {ex}")
                self._last_tf_warn = self.get_clock().now()
            return False

        # Posizione
        self.robot_x = trans.transform.translation.x
        self.robot_y = trans.transform.translation.y

        # Yaw da quaternione
        q = trans.transform.rotation
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
        # Per normalizzare: self.obstacle_angle = math.atan2(math.sin(self.obstacle_angle), math.cos(self.obstacle_angle))

    def min_distance_callback(self, msg: Float32):
        """Riceve la distanza minima (utile per debug)"""
        self.min_distance = msg.data

    # ---------------------- LUT LOADER (Opzione B integrata) ----------------------

    def load_LUT(self, csv_path, columns):
        """Carica LUT CSV.
        - Se i punti formano una griglia perfetta, costruisce la griglia classica.
        - Altrimenti (punti sparsi), usa SciPy griddata per creare una griglia regolare X×Y.
          I NaN (fuori dal convesso) sono riempiti con nearest. Se SciPy manca, fallback nearest puro.
        Ritorna dict: {"x": X, "y": Y, "z": Z, "type": "grid"} oppure {"type":"points","data": Nx3}.
        """
        if not os.path.isfile(csv_path):
            self.get_logger().warn(f"⚠️ File LUT non trovato: {csv_path}")
            return None

        # --- Lettura CSV ---
        rows = []
        with open(csv_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                try:
                    vals = [float(row[c]) for c in columns]
                    rows.append(vals)
                except (ValueError, KeyError):
                    continue

        data = np.array(rows, dtype=float)  # shape (N,3) -> [x,y,z]
        if data.shape[0] < 3:
            self.get_logger().warn(
                f"⚠️ LUT {os.path.basename(csv_path)} ha troppo pochi punti → nearest neighbor"
            )
            return {"type": "points", "data": data}

        # --- Verifica se è già una griglia cartesiana completa ---
        x_unique = np.unique(data[:, 0])
        y_unique = np.unique(data[:, 1])
        expected = len(x_unique) * len(y_unique)
        is_perfect_grid = (expected == data.shape[0])

        if is_perfect_grid:
            # Costruzione Z con media (gestione duplicati con isclose)
            Z = np.zeros((len(x_unique), len(y_unique)), dtype=float)
            for i, x in enumerate(x_unique):
                for j, y in enumerate(y_unique):
                    mask = (np.isclose(data[:, 0], x)) & (np.isclose(data[:, 1], y))
                    vals = data[mask, 2]
                    Z[i, j] = np.mean(vals) if vals.size > 0 else 0.0

            self.get_logger().info(
                f"✅ LUT '{os.path.basename(csv_path)}' caricata come griglia perfetta "
                f"({len(x_unique)}x{len(y_unique)} punti)"
            )
            return {"x": x_unique, "y": y_unique, "z": Z, "type": "grid"}

        # --- Punti sparsi: costruzione griglia regolare (Opzione B) ---
        nx = max(2, int(self.lut_nx))
        ny = max(2, int(self.lut_ny))
        x = data[:, 0]
        y = data[:, 1]
        z = data[:, 2]

        X = np.linspace(np.min(x), np.max(x), nx)
        Y = np.linspace(np.min(y), np.max(y), ny)
        XX, YY = np.meshgrid(X, Y, indexing='ij')  # shape (nx, ny)

        if _HAS_SCIPY:
            method = self.lut_method if self.lut_method in ("linear", "cubic") else "linear"
            # Passaggio principale: interpolazione richiesta (può produrre NaN fuori convesso)
            ZZ_primary = griddata(points=np.c_[x, y], values=z, xi=(XX, YY), method=method)
            # Riempimento buchi con nearest
            ZZ_nearest = griddata(points=np.c_[x, y], values=z, xi=(XX, YY), method="nearest")
            Z = np.where(np.isnan(ZZ_primary), ZZ_nearest, ZZ_primary)

            self.get_logger().info(
                f"✅ LUT '{os.path.basename(csv_path)}' grigliata con SciPy ({method}) "
                f"→ ({nx}x{ny} nodi)"
            )
            return {"x": X, "y": Y, "z": Z, "type": "grid"}
        else:
            # Fallback senza SciPy: nearest ai punti originali
            self.get_logger().warn(
                "⚠️ SciPy non disponibile: fallback su griglia regolare con nearest."
            )
            Z = np.empty((nx, ny), dtype=float)
            pts = np.c_[x, y]
            for i in range(nx):
                for j in range(ny):
                    dx = pts[:, 0] - XX[i, j]
                    dy = pts[:, 1] - YY[i, j]
                    k = np.argmin(dx * dx + dy * dy)
                    Z[i, j] = z[k]

            self.get_logger().info(
                f"✅ LUT '{os.path.basename(csv_path)}' grigliata (fallback nearest) "
                f"({nx}x{ny} nodi)"
            )
            return {"x": X, "y": Y, "z": Z, "type": "grid"}

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
        v = self.interpolate_LUT(self.linear_LUT, dist_error, self.obstacle_angle) *0.75/ 5.0
        w = self.interpolate_LUT(self.angular_LUT, orient_error, self.obstacle_angle) / 5.0

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

    # ---------------------- Utility ----------------------

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
