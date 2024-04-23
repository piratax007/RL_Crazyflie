import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from CrazyflieAPI import drone

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
        # Add more
        self.drone = drone
        # at some point I need to call self.drone.control.state = blah in order to update the observation space

        self.tf = None 	
        self.tf_stamped = None  # position
        self.odom = None  # linear and angular velocities

    def listener_callback(self, msg):
        if isinstance(msg, TFMessage):
            # Handle TFMessage
            for transform in msg.transforms:
                # Access transform data as needed
                child_frame_id = transform.child_frame_id
                transform_matrix = transform.transform
                # Print transform data for demonstration
                print("Received TF Message:")
                print("Child Frame ID:", child_frame_id)
                print("Transform Matrix:")
                print("  Translation: [x={}, y={}, z={}]".format(
                    transform_matrix.translation.x,
                    transform_matrix.translation.y,
                    transform_matrix.translation.z))
                print("  Rotation: [x={}, y={}, z={}, w={}]".format(
                    transform_matrix.rotation.x,
                    transform_matrix.rotation.y,
                    transform_matrix.rotation.z,
                    transform_matrix.rotation.w))
                
                self.tf = msg
                
        elif isinstance(msg, TransformStamped):
            # Handle TransformStamped
            translation = msg.transform.translation
            rotation = msg.transform.rotation
            # Print transform data for demonstration
            print("Received TransformStamped message:")
            print("Translation: [x={}, y={}, z={}]".format(
                translation.x, translation.y, translation.z))
            print("Rotation: [x={}, y={}, z={}, w={}]".format(
                rotation.x, rotation.y, rotation.z, rotation.w))
            
            self.tf_stamped = msg
            
        elif isinstance(msg, Odometry):
            self.odom = msg
            
        if self.tf is not None and self.odom is not None and self.tf_stamped is not None:
            # Do RL stuff
            pass


def main(args=None):
    rclpy.init(args=args)
    my_subscriber = MySubscriber()
    rclpy.spin(my_subscriber)
    my_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
