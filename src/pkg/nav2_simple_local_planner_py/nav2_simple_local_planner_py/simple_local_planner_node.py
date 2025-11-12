import math
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path

def clamp(v, lo, hi): return max(lo, min(hi, v))

class SimpleLocalPlannerNode(Node):
    def __init__(self):
        super().__init__('simple_local_planner_py')
        self.declare_parameter('odom_topic', '/odom/rover')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('controller_frequency', 20.0)
        self.declare_parameter('k_lin', 0.8)
        self.declare_parameter('k_ang', 2.0)
        self.declare_parameter('max_lin', 0.6)
        self.declare_parameter('max_ang', 1.2)
        self.declare_parameter('goal_xy_tolerance', 0.10)
        self.declare_parameter('lookahead_distance', 0.75)
        self.declare_parameter('reverse_allowed', False)
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.plan_topic = self.get_parameter('plan_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.k_lin = self.get_parameter('k_lin').get_parameter_value().double_value
        self.k_ang = self.get_parameter('k_ang').get_parameter_value().double_value
        self.max_lin = self.get_parameter('max_lin').get_parameter_value().double_value
        self.max_ang = self.get_parameter('max_ang').get_parameter_value().double_value
        self.goal_xy_tol = self.get_parameter('goal_xy_tolerance').get_parameter_value().double_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.reverse_allowed = self.get_parameter('reverse_allowed').get_parameter_value().bool_value
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        self._odom_sub = self.create_subscription(Odometry, self.odom_topic, self._on_odom, qos)
        self._plan_sub = self.create_subscription(Path, self.plan_topic, self._on_plan, qos)
        self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        freq = self.get_parameter('controller_frequency').get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / freq, self._on_timer)
        self._last_odom: Optional[Odometry] = None
        self._last_path: Optional[Path] = None
        self.get_logger().info(f'SimpleLocalPlanner (Python) started. odom={self.odom_topic}, plan={self.plan_topic}, cmd_vel={self.cmd_vel_topic}')

    def _on_odom(self, msg: Odometry): self._last_odom = msg
    def _on_plan(self, msg: Path):
        self._last_path = msg
        #self.get_logger().info(f'Received path with {len(msg.poses)} poses')

    def _on_timer(self):
        twist = Twist()
        if self._last_odom is None or self._last_path is None or len(self._last_path.poses) == 0:
            self._cmd_pub.publish(twist)
            return
        px = self._last_odom.pose.pose.position.x
        py = self._last_odom.pose.pose.position.y
        q = self._last_odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        target = self._last_path.poses[-1].pose
        best_dist = float('inf')
        lookahead = self.lookahead_distance
        for ps in self._last_path.poses:
            dx = ps.pose.position.x - px
            dy = ps.pose.position.y - py
            d = math.hypot(dx, dy)
            if abs(d - lookahead) < best_dist:
                best_dist = abs(d - lookahead)
                target = ps.pose
        dx = target.position.x - px
        dy = target.position.y - py
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_err = math.remainder(target_yaw - yaw, 2.0 * math.pi)
        v_sign = 1.0
        if not self.reverse_allowed:
            forward_component = math.cos(yaw) * dx + math.sin(yaw) * dy
            v_cmd = 0.0 if forward_component < 0.0 else self.k_lin * dist
        else:
            v_cmd = self.k_lin * dist
            if abs(yaw_err) > math.pi / 2: v_sign = -1.0
        w_cmd = self.k_ang * yaw_err
        gx = self._last_path.poses[-1].pose.position.x
        gy = self._last_path.poses[-1].pose.position.y
        gdist = math.hypot(gx - px, gy - py)
        if gdist < 0.3: v_cmd = min(v_cmd, 0.2)
        v_cmd = clamp(v_cmd, 0.0, self.max_lin) * v_sign
        w_cmd = clamp(w_cmd, -self.max_ang, self.max_ang)
        if gdist < self.goal_xy_tol: v_cmd = 0.0
        twist.linear.x = v_cmd
        twist.angular.z = w_cmd
        self._cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleLocalPlannerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
