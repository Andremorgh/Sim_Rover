import math
from geometry_msgs.msg import Twist

def yaw_from_quaternion(q):
    # q = geometry_msgs/Quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a < 0:
        a += 2.0 * math.pi
    return a - math.pi

def build_twist(v, w):
    msg = Twist()
    msg.linear.x = float(v)
    msg.angular.z = float(w)
    return msg
