#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LifecycleSubscriber(Node):
    def __init__(self):
        super().__init__('lifecycle_subscriber')
        self.subscription = self.create_subscription(
            String,
            'lifecycle_topic',
            self.topic_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Lifecycle subscriber initialized. Waiting for messages...')

    def topic_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    
    subscriber = LifecycleSubscriber()
    
    subscriber.get_logger().info('LifecycleSubscriber node initialized.')

    executor = rclpy.executors.SingleThreadedExecutor()
    # executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
  

if __name__ == '__main__':
    main()
