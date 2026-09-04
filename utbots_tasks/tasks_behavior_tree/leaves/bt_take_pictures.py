import os
import cv2
import time
import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import py_trees
import py_trees_ros

class SaveImageBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, folder: str):
        super().__init__(name=name)
        self.folder = os.path.expanduser(folder)
        self.bridge = CvBridge()
        
        self.blackboard = py_trees.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="camera_image", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="camera_image", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        os.makedirs(self.folder, exist_ok=True)
        self.logger.info(f"Image folder configured in: {self.folder}")

    def update(self):
        if not self.blackboard.exists("camera_image") or self.blackboard.camera_image is None:
            return py_trees.common.Status.RUNNING

        msg = self.blackboard.camera_image

        print("Press 'ENTER' to save the captured image (or 'Ctrl+C' to exit)...")
        try:
            input()
        except EOFError:
            pass

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ts = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
            filename = os.path.join(self.folder, f"img_{ts}.png")
            cv2.imwrite(filename, cv_img)
            
            self.logger.info(f"[TakePictures] image saved: {filename}")
            
            self.blackboard.camera_image = None
            
            return py_trees.common.Status.SUCCESS
            
        except Exception as e:
            self.logger.error(f"[TakePictures] Error saving: {e}")
            return py_trees.common.Status.FAILURE


def main(args=None):
    rclpy.init(args=args)
    
    root = py_trees.composites.Sequence(name="Capture_Sequence", memory=False)
    
    image_sub = py_trees_ros.subscribers.ToBlackboard(
        name="Image_To_Blackboard",
        topic_name="/camera/camera/color/image_raw",
        topic_type=Image,
        qos_profile=QoSProfile(depth=10),
        blackboard_variables={"camera_image": None} 
    )
    
    save_behavior = SaveImageBehavior(
        name="Save_Image", 
        folder="~/object_captures"
    )
    
    root.add_children([image_sub, save_behavior])

    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    
    try:
        tree.setup(node_name="picture_capture_bt", timeout=15.0)
        
        tree.tick_tock(period_ms=500)
        
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        tree.interrupt()
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()