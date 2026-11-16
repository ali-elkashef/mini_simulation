import rclpy
from geometry_msgs.msg import Twist, Vector3, Quaternion, Point
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
import math
import time
from std_msgs.msg import Int32

HALF_DISTANCE_BETWEEN_WHEELS = 0.11
WHEEL_RADIUS = 0.043

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        timestep = int(self.__robot.getBasicTimeStep())

        self.__fl_motor = self.__robot.getDevice('fl_wheel_joint')
        self.__fr_motor = self.__robot.getDevice('fr_wheel_joint')
        self.__rl_motor = self.__robot.getDevice('rl_wheel_joint')
        self.__rr_motor = self.__robot.getDevice('rr_wheel_joint')

        for motor in [self.__fl_motor, self.__fr_motor, self.__rl_motor, self.__rr_motor]:
            if motor:
                motor.setPosition(float('inf'))
                motor.setVelocity(0)

        self.fl_encoder = self.__try_get_device(['fl_wheel_joint_sensor', 'fl_wheel sensor', 'front left wheel sensor'])
        self.fr_encoder = self.__try_get_device(['fr_wheel_joint_sensor', 'fr_wheel sensor', 'front right wheel sensor'])
        if self.fl_encoder: self.fl_encoder.enable(timestep)
        if self.fr_encoder: self.fr_encoder.enable(timestep)

        self.imu   = self.__try_get_device(['imu inertial_unit', 'imu', 'inertial_unit'])
        self.gyro  = self.__try_get_device(['imu gyro', 'gyro'])
        self.accel = self.__try_get_device(['imu accelerometer', 'accelerometer'])
        if self.imu:   self.imu.enable(timestep)
        if self.gyro:  self.gyro.enable(timestep)
        if self.accel: self.accel.enable(timestep)

        self.last_time = time.time()
        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')

        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__odom_pub = self.__node.create_publisher(Odometry, 'odom', 10)
        self.__tf_broadcaster = TransformBroadcaster(self.__node)
        self.imu_pub = self.__node.create_publisher(Imu, 'imu/data', 10)

        if self.fl_encoder and self.fr_encoder:
            self.left_enc_pub  = self.__node.create_publisher(Int32, 'left_wheel/encoder', 10)
            self.right_enc_pub = self.__node.create_publisher(Int32, 'right_wheel/encoder', 10)
        else:
            self.left_enc_pub  = None
            self.right_enc_pub = None
            self.__node.get_logger().warn('Encoders not found - encoder publishing disabled')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.__node.get_logger().info('MyRobotDriver initialized successfully!')

    def __try_get_device(self, names):
        for name in names:
            device = self.__robot.getDevice(name)
            if device:
                return device
        return None

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        left_vel  = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        right_vel = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        if self.__fl_motor: self.__fl_motor.setVelocity(left_vel)
        if self.__rl_motor: self.__rl_motor.setVelocity(left_vel)
        if self.__fr_motor: self.__fr_motor.setVelocity(right_vel)
        if self.__rr_motor: self.__rr_motor.setVelocity(right_vel)

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        v = WHEEL_RADIUS * (right_vel + left_vel) / 2.0
        w = WHEEL_RADIUS * (right_vel - left_vel) / (2.0 * HALF_DISTANCE_BETWEEN_WHEELS)

        self.theta += w * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        q = quaternion_from_euler(0, 0, self.theta)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.__node.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom_msg.twist.twist.linear  = Vector3(x=v, y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=w)
        self.__odom_pub.publish(odom_msg)

        if self.imu:
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            q = quaternion_from_euler(roll, pitch, yaw)
            imu_msg = Imu()
            imu_msg.header.stamp = self.__node.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'base_link'
            imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            if self.gyro:
                av = self.gyro.getValues()
                imu_msg.angular_velocity.x = av[0]
                imu_msg.angular_velocity.y = av[1]
                imu_msg.angular_velocity.z = av[2]
            if self.accel:
                la = self.accel.getValues()
                imu_msg.linear_acceleration.x = la[0]
                imu_msg.linear_acceleration.y = la[1]
                imu_msg.linear_acceleration.z = la[2]
            self.imu_pub.publish(imu_msg)

        if self.fl_encoder and self.fr_encoder and self.left_enc_pub:
            TICKS_PER_REV = 4096
            left_ticks  = int(self.fl_encoder.getValue() * TICKS_PER_REV / (2 * math.pi))
            right_ticks = int(self.fr_encoder.getValue() * TICKS_PER_REV / (2 * math.pi))
            self.left_enc_pub.publish(Int32(data=left_ticks))
            self.right_enc_pub.publish(Int32(data=right_ticks))

        t = TransformStamped()
        t.header.stamp = self.__node.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.__tf_broadcaster.sendTransform(t)