import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rlcrazyflie.CrazyflieAPI import crazyflie

TOPIC_ODOMETRY = "/vicon/fausto_crazyfly/fausto_crazyfly/odom"
TOPIC_START_ENGINES = "/startfly"
TOPIC_STOP_ENGINES = "/stopfly"
TOPIC_LANDING = "/landing"


class ODOMETRYSubscriber(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription1 = self.create_subscription(
            Odometry,
            TOPIC_ODOMETRY,
            self.odometry_callback,
            10)
        self.subscription2 = self.create_subscription(
            String,
            TOPIC_START_ENGINES,
            self.start_callback,
            1
        )
        self.subscription3 = self.create_subscription(
            String,
            TOPIC_STOP_ENGINES,
            self.stop_callback,
            1
        )
        self.subscription4 = self.create_subscription(
            String,
            TOPIC_LANDING,
            self.landing_callback,
            1
        )

        self.odom = None
        self.position = ()
        self.euler_angles = ()
        self.linear_velocities = ()
        self.angular_velocities = ()
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
        self.euler_angles = euler_angles_from(quaternion)
        self.linear_velocities = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
        self.angular_velocities = (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z)

        self.crazyflie.control.observation_space = np.array([np.hstack(
            (
                self.position,
                self.euler_angles,
                self.linear_velocities,
                self.angular_velocities
            )
        )])
        if self.start:
            actions = self.crazyflie.control.applicable_pwm()
            fly(actions)

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


def main():
    crazyflie.drone.setParam("motorPowerSet.enable", 1)
    vicon_subscriber = ODOMETRYSubscriber()
    rclpy.spin(vicon_subscriber)
    vicon_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
