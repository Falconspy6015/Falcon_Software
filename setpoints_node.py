import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped , PoseArray
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from rclpy.qos import QoSProfile , ReliabilityPolicy , HistoryPolicy

class WaypointNode(Node):

    def __init__(self):
        super().__init__('waypoint_node')

        self.current_state = State()
        self.current_pose = PoseStamped()
        self.pose_recieved = False
        self.last_mode_request_time = self.get_clock().now()
        self.last_arm_request_time = self.get_clock().now()

        # Target threshold in meters
        self.target_threshold = 0.3  

        # Publishers & Subscribers
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        pose_qos = QoSProfile(reliability= ReliabilityPolicy.BEST_EFFORT , history=HistoryPolicy.KEEP_LAST, depth=10)

        # Added subscriber for actual drone position
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            pose_qos
        )
        self.waypoints_sub = self.create_subscription(PoseArray , '/waypoints', self.waypoints_callback, 10)

        # Service Clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Setpoints list (X, Y, Z)
        self.setpoints = []
        self.waypoints_recieved=False
        self.current_setpoint_idx = 0
        self.init_count = 0

        # Control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg
        self.pose_recieved = True

    def set_mode(self, custom_mode):
        req = SetMode.Request()
        req.custom_mode = custom_mode
        if self.set_mode_client.service_is_ready():
            self.set_mode_client.call_async(req)

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        if self.arm_client.service_is_ready():
            self.arm_client.call_async(req)

    def waypoints_callback(self , msg):
        self.setpoints = [( pose.position.x , pose.position.y , pose.position.z) for pose in msg.poses]
        self.current_setpoint_idx = 0
        self.waypoints_recieved = True
        self.get_logger().info(f'Recieved {len(self.setpoints)} waypoints.')

    def control_loop(self):
        if not self.current_state.connected:
            self.get_logger().info('Waiting for PX4 connection...', throttle_duration_sec=2.0)
            return
        if not self.waypoints_recieved:
            self.get_logger().info('Waiting for waypoints...' , throttle_duration_sec=2.0)
            return

        # 1. Target setpoint retrieval
        x_target, y_target, z_target = self.setpoints[self.current_setpoint_idx]

        # 2. Build PoseStamped Message
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x_target
        pose.pose.position.y = y_target
        pose.pose.position.z = z_target
        pose.pose.orientation.w = 1.0

        # Continuously stream setpoint
        self.setpoint_pub.publish(pose)

        # 3. Stream ~100 setpoints BEFORE switching to OFFBOARD (PX4 Requirement)
        if self.init_count < 100:
            self.init_count += 1
            return

        # 4. Request OFFBOARD and Arm if needed
        if self.current_state.mode != 'OFFBOARD':
            elapsed=(self.get_clock().now() - self.last_mode_request_time).nanoseconds/1e9
            if elapsed > 1.0:
                self.get_logger().info('Requesting OFFBOARD mode...')
                self.set_mode('OFFBOARD')
                self.last_mode_request_time = self.get_clock().now()
            return

        if not self.current_state.armed:
            elapsed1 = ( self.get_clock().now() - self.last_arm_request_time ).nanoseconds / 1e9
            if elapsed1 > 1.0:
                self.get_logger().info('Requesting Arming...')
                self.arm()
                self.last_arm_request_time = self.get_clock().now()
            return

        # 5. Distance check to iterate setpoints
        if not self.pose_recieved:
            return
        dx = x_target - self.current_pose.pose.position.x
        dy = y_target - self.current_pose.pose.position.y
        dz = z_target - self.current_pose.pose.position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        self.get_logger().info(
            f'Setpoint {self.current_setpoint_idx + 1}/{len(self.setpoints)} | '
            f'Position:x={self.current_pose.pose.position.x:.2f} '
            f'y={self.current_pose.pose.position.y:.2f}, '
            f'z={self.current_pose.pose.position.z:.2f} | '
            f'Dist to target: {distance:.2f}m',
            throttle_duration_sec=1.0
        )

        if distance < self.target_threshold:
            if self.current_setpoint_idx < len(self.setpoints) - 1:
                self.get_logger().info(f'Reached setpoint {self.current_setpoint_idx + 1}! Moving to next...')
                self.current_setpoint_idx += 1
            else:
                self.get_logger().info('Reached final setpoint! Holding position.', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
