import os
import cv2
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yasmin_ros import MonitorState, set_ros_loggers
from yasmin import StateMachine, Blackboard

import keyboard

# From yasmin_ros docs: MonitorState signature :contentReference[oaicite:1]{index=1}
# MonitorState(topic_name, outcomes, handler, qos, msg_queue, timeout)

class TakePicturesState(MonitorState):
    def __init__(self, node: Node):
        # Declare and read ROS parameters
        node.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        node.declare_parameter('image_folder', os.path.expanduser('~/object_captures'))
        node.declare_parameter('delay', 3.0)

        topic = node.get_parameter('image_topic').value
        folder = os.path.expanduser(node.get_parameter('image_folder').value)
        delay = float(node.get_parameter('delay').value)

        # Create CvBridge early and state
        self.bridge = CvBridge()
        self.image_folder = folder
        self.delay = delay
        self.last_save = 0.0

        qos = QoSProfile(depth=10)

        super().__init__(
            msg_type=Image,
            topic_name=topic,
            outcomes=['waiting', 'saved', 'abort'],
            monitor_handler=self.monitor_handler,
            qos=qos,
            msg_queue=10,
            #timeout=-1
        )
        self.node = node

    def monitor_handler(self, bb: Blackboard, msg: Image) -> str:
        try:
            os.makedirs(self.image_folder, exist_ok=True)

            now = time.time()
            #if now - self.last_save < self.delay:
            #    return 'waiting'

            print("Press 'enter' to continue...")
            input()
            #keyboard.wait('q')  # Waits until the 'q' key is pressed
            print("You pressed 'q'. Continuing...")


            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ts = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
            filename = os.path.join(self.image_folder, f"img_{ts}.png")
            cv2.imwrite(filename, cv_img)

            self.node.get_logger().info(f"[TakePictures] Saved image: {filename}")

            self.last_save = now
            return 'saved'
        except Exception as e:
            self.node.get_logger().error(f"[TakePictures] Abort: {e}")
            return 'abort'

def main(args=None):
    rclpy.init(args=args)
    set_ros_loggers()

    node = Node('picture_capture_sm')
    bb = Blackboard()
    bb['node'] = node

    sm = StateMachine(outcomes={'abort'})
    take_state = TakePicturesState(node)

    sm.add_state(
        name='CAPTURE',
        state=take_state,
        transitions={
            'waiting': 'CAPTURE',
            'saved': 'CAPTURE',
            'abort': 'abort'
        }
    )
    sm.set_start_state('CAPTURE')
    sm.validate()

    # Launch FSM (blocking is OK here since MonitorState runs callback-based)
    try:
        outcome = sm.execute(bb)
        node.get_logger().info(f"State machine ended with outcome: {outcome}")
    except KeyboardInterrupt:
        take_state.cancel_state()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
