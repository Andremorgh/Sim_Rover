import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import math
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ClosestObstacleFromPC2(Node):
    def __init__(self):
        super().__init__('closest_obstacle_from_pc2')

        # QoS compatibile con bridge/Gazebo
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Banda di elevazione ±5°
        self.elev_band_rad = math.radians(5.0)

        # Topic PointCloud2 (adatta al tuo)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/scan/points',   # es.: /lidar/points, /velodyne_points, /points
            self.cloud_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(Float32, '/obstacle', 10)
        self.min_dist_publisher = self.create_publisher(Float32, '/min_distance', 10)

        # Log distanza minima (come nel tuo codice)
        self.log_file_path = os.path.expanduser('~/.ros/min_distance_log.txt')
        with open(self.log_file_path, 'w') as f:
            f.write('')

    def cloud_callback(self, msg: PointCloud2):
        min_distance = float('inf')
        min_bearing = None  # angolo orizzontale (yaw) del punto più vicino valido
        global_min_distance = float('inf')

        # Itera solo su x,y,z e salta i NaN
        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            # Distanze utili
            r_xy = math.hypot(x, y)                # proiezione sul piano orizzontale
            distance = math.hypot(r_xy, z)         # distanza 3D

            # Aggiorna minima globale
            if distance < global_min_distance:
                global_min_distance = distance

            # Filtro: banda di elevazione ±5°
            elev = math.atan2(z, r_xy)             # angolo rispetto all’orizzonte
            if abs(elev) > self.elev_band_rad:
                continue

            # Bearing orizzontale (come il tuo curr_angle ma su 3D)
            theta = math.atan2(y, x)               # [-pi, pi]

            # Stessa logica di gating che avevi su LaserScan:
            # - entro 1.0 m
            # - finestra laterale (|y|) che si allarga col crescere della distanza
            # - FOV orizzontale ~±1.6 rad
            lateral_gate = 0.2 + 0.2 * ((0.75 - distance) / 0.75)
            if distance <= 0.75 and abs(y) <= lateral_gate and -1.6 <= theta <= 1.6:
                if distance < min_distance:
                    min_distance = distance
                    min_bearing = theta

        # Se non trovato nulla, pubblica un valore "neutro" come facevi tu
        angle_to_publish = 3.0 if min_bearing is None else float(min_bearing)

        # Pubblica /obstacle (angolo orizzontale)
        msg_out = Float32()
        msg_out.data = angle_to_publish
        self.publisher.publish(msg_out)

        # Pubblica /min_distance (globale), default 10.0 se nessun punto
        min_dist_value = float(global_min_distance) if math.isfinite(global_min_distance) else 10.0
        min_dist_msg = Float32()
        min_dist_msg.data = min_dist_value
        self.min_dist_publisher.publish(min_dist_msg)

        # Log su file
        with open(self.log_file_path, 'a') as f:
            f.write(f"{min_dist_value:.4f}\n")

def main(args=None):
    rclpy.init(args=args)
    node = ClosestObstacleFromPC2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
