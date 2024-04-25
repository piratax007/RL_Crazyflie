import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from rlcrazyflie.CrazyflieAPI import crazyflie

TOPIC1 = "/tf"
TOPIC2 = "/vicon/fausto_crazyfly/fausto_crazyfly"
TOPIC3 = "/vicon/fausto_crazyfly/fausto_crazyfly/odom"


class MySubscriber(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription1 = self.create_subscription(
            TFMessage,
            TOPIC1,  # Change 'topic1' to the name of the first topic you want to subscribe to
            self.listener_callback,
            10)
        self.subscription2 = self.create_subscription(
            TransformStamped,
            TOPIC2,  # Change 'topic2' to the name of the second topic you want to subscribe to
            self.listener_callback,
            10)
        self.subscription3 = self.create_subscription(
            Odometry,
            TOPIC3,  # Change 'topic3' to the name of the third topic you want to subscribe to
            self.listener_callback,
            10)

        self.tf = None
        self.tf_stamped = None  # position
        self.odom = None  # linear and angular velocities
        self.position = ()
        self.euler_angles = ()
        self.linear_velocities = ()
        self.angular_velocities = ()
        self.crazyflie = crazyflie

    def listener_callback(self, msg):
        if isinstance(msg, TFMessage):
            # Handle TFMessage
            # for transform in msg.transforms:
                # Access transform data as needed
                # child_frame_id = transform.child_frame_id
                # transform_matrix = transform.transform
                # Print transform data for demonstration
                # print("Received TF Message:")
                # print("Child Frame ID:", child_frame_id)
                # print("Transform Matrix:")
                # print("  Translation: [x={}, y={}, z={}]".format(
                #     transform_matrix.translation.x,
                #     transform_matrix.translation.y,
                #     transform_matrix.translation.z))
                # print("  Rotation: [x={}, y={}, z={}, w={}]".format(
                #     transform_matrix.rotation.x,
                #     transform_matrix.rotation.y,
                #     transform_matrix.rotation.z,
                #     transform_matrix.rotation.w))
                
            self.tf = msg
                
        elif isinstance(msg, TransformStamped):
            # Handle TransformStamped
            # translation = msg.transform.translation
            # rotation = msg.transform.rotation
            # Print transform data for demonstration
            # print("Received TransformStamped message:")
            # print("Translation: [x={}, y={}, z={}]".format(
            #     translation.x, translation.y, translation.z))
            # print("Rotation: [x={}, y={}, z={}, w={}]".format(
            #     rotation.x, rotation.y, rotation.z, rotation.w))
            
            self.tf_stamped = msg
            
        elif isinstance(msg, Odometry):
            self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
            quaternion = msg.pose.pose.orientation
            self.euler_angles = tuple(quaternion_to_euler_angles(quaternion))
            self.linear_velocities = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
            self.angular_velocities = (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z)

            print(f"""
            ####### ODOMETRY INFORMATION ##########
            POSITION {self.position}
            ---------------------------------------
            QUATERNION {quaternion}
            ROLL, PITCH, YAW = {self.euler_angles}
            ---------------------------------------
            VELOCITY {self.linear_velocities}
            ANGULAR VELOCITY {self.angular_velocities}
            """)

            self.odom = msg
            
        if self.tf is not None and self.odom is not None and self.tf_stamped is not None:
            self.crazyflie.control.observation_space = np.hstack(
                (
                    self.position,
                    self.euler_angles,
                    self.linear_velocities,
                    self.angular_velocities
                )
            )


def quaternion_to_euler_angles(q: list):
    x, y, z, w = q

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def main():
    my_subscriber = MySubscriber()
    rclpy.spin(my_subscriber)
    my_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
