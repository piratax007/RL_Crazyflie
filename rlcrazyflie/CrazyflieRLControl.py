import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from rlcrazyflie.CrazyflieAPI import CrazyflieAPI

TOPIC_ODOMETRY = "/vicon/fausto_crazyfly/fausto_crazyfly/odom"
TOPIC_START_ENGINES = "/startfly"
TOPIC_STOP_ENGINES = "/stopfly"
TOPIC_LANDING = "/landing"
TOPIC_ACTIONS = "/actions"
TOPIC_OBSERVATIONS = "/observations"
crazyflie = CrazyflieAPI()


class CrazyflieRLControl(Node):
    def __init__(self):
        super().__init__('listener')
        self.odometry_subscription = self.create_subscription(
            Odometry,
            TOPIC_ODOMETRY,
            self.odometry_callback,
            10)
        self.start_subscription = self.create_subscription(
            String,
            TOPIC_START_ENGINES,
            self.start_callback,
            1
        )
        self.stop_subscription = self.create_subscription(
            String,
            TOPIC_STOP_ENGINES,
            self.stop_callback,
            1
        )
        self.landing_subscription = self.create_subscription(
            String,
            TOPIC_LANDING,
            self.landing_callback,
            1
        )
        self.actions_publisher = self.create_publisher(
            Quaternion,
            TOPIC_ACTIONS,
            10
        )
        self.observations_publisher = self.create_publisher(
            Odometry,
            TOPIC_OBSERVATIONS,
            10
        )

        self.odom = None
        self.position = ()
        self.euler_angles = ()
        self.linear_velocities = []
        self.angular_velocities = []
        self.crazyflie = crazyflie
        self.start_ref = (0, 0, 0)
        self.start = False

    def odometry_callback(self, msg):
        self.position = (
            msg.pose.pose.position.x - self.start_ref[0],
            msg.pose.pose.position.y - self.start_ref[1],
            msg.pose.pose.position.z - self.start_ref[2]
        )
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        linear_velocities_body_frame = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        angular_velocities_body_frame = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]

        self.euler_angles = euler_angles_from(quaternion)
        self.linear_velocities, self.angular_velocities = velocities_in_global_frame(list(quaternion), linear_velocities_body_frame, angular_velocities_body_frame)

        self.crazyflie.control.observation_space = np.array([np.hstack(
            (
                self.position,
                self.euler_angles,
                self.linear_velocities,
                self.angular_velocities
            )
        )])

        msg_actions = Quaternion()

        if self.start:
            actions = self.crazyflie.control.applicable_pwm()
            msg_actions.x = actions[3]
            msg_actions.y = actions[2]
            msg_actions.z = actions[1]
            msg_actions.w = actions[0]
            self.actions_publisher.publish(msg_actions)
            fly(actions)
            msg_observations.pose.pose.position.x = self.position[0]
            msg_observations.pose.pose.position.y = self.position[1]
            msg_observations.pose.pose.position.z = self.position[2]
            msg_observations.pose.pose.orientation.x = self.euler_angles[0]
            msg_observations.pose.pose.orientation.y = self.euler_angles[1]
            msg_observations.pose.pose.orientation.z = self.euler_angles[2]
            msg_observations.pose.pose.orientation.w = 0.0
            msg_observations.twist.twist.linear.x = self.linear_velocities[0]
            msg_observations.twist.twist.linear.y = self.linear_velocities[1]
            msg_observations.twist.twist.linear.z = self.linear_velocities[2]
            msg_observations.twist.twist.angular.x = self.angular_velocities[0]
            msg_observations.twist.twist.angular.y = self.angular_velocities[1]
            msg_observations.twist.twist.angular.z = self.angular_velocities[2]
            self.observations_publisher.publish(msg_observations)

    def start_callback(self, msg):
        if not self.start:
            ref_x = self.position[0]
            ref_y = self.position[1]
            ref_z = self.position[2]
            self.start_ref = (ref_x, ref_y, ref_z)
            self.start = True

    def stop_callback(self, msg):
        if self.start:
            self.start = False
            fly((0, 0, 0, 0))
            crazyflie.drone.setParam("motorPowerSet.enable", 0)

    def landing_callback(self, msg):
        crazyflie.drone.land(targetHeight=0.04, duration=2.5)
        crazyflie.drone.setParam("motorPowerSet.enable", 0)
        self.start = False


def fly(actions: tuple):
    crazyflie.drone.setParam("motorPowerSet.m1", actions[0])
    crazyflie.drone.setParam("motorPowerSet.m2", actions[1])
    crazyflie.drone.setParam("motorPowerSet.m3", actions[2])
    crazyflie.drone.setParam("motorPowerSet.m4", actions[3])


def euler_angles_from(q: tuple):
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


def velocities_in_global_frame(quaternion: list, linear_velocities_body: list, angular_velocities_body: list):
    r = R.from_quat(quaternion)

    linear_velocity_global = r.apply(linear_velocities_body)
    angular_velocity_global = r.apply(angular_velocities_body)

    return linear_velocity_global, angular_velocity_global


def main():
    crazyflie.drone.setParam("motorPowerSet.enable", 1)
    vicon_subscriber = CrazyflieRLControl()
    rclpy.spin(vicon_subscriber)
    vicon_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
