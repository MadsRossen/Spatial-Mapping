#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError
from message_filters import Subscriber, TimeSynchronizer
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf2_ros
 
class ThermalToColorOverlay(Node):
    def __init__(self):
        super().__init__('thermal_to_color_overlay')
        self.bridge = CvBridge()
        self.raw_image = None
 
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10  # Adjust depth if needed
        )
        # Create a frame for the Robot base
        self.tf_broadcaster = TransformBroadcaster(self)
        self.thermal_broad = StaticTransformBroadcaster(self)
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.robot_start)
        self.timer = self.create_timer(0.1, self.thermal_cam_pos)
        # self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        # self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
        # self.broadcast_timer_callback()
        # Create publisher with QoS settings
        self.image_pub = self.create_publisher(Image, '/color/image_thermal', qos_profile)
 
        # Create subscriptions with QoS settings
        self.color_sub = Subscriber(self, Image, '/camera/color/image_raw', qos_profile=qos_profile)
        self.thermal_sub = Subscriber(self, Image, '/thermal_image_view', qos_profile=qos_profile)
 
        # Use TimeSynchronizer to synchronize the color and thermal images
        self.ts = ApproximateTimeSynchronizer([self.color_sub, self.thermal_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.image_callback)
 
    def robot_start(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'robot_start'
        t.transform.translation.x = -0.35
        t.transform.translation.y = 0.0
        t.transform.translation.z = -0.67
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = -0.1305262
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.9914449
        self.tf_broadcaster.sendTransform(t)
 
    def thermal_cam_pos(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = 'thermal_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = -0.0231441
        t.transform.translation.z = 0.1193676
        t.transform.rotation.x = -0.0453937
        t.transform.rotation.y = 0.0706877
        t.transform.rotation.z = 0.0193218
        t.transform.rotation.w = 0.9962777
        self.thermal_broad.sendTransform(t)
 
    def image_callback(self, color_data, thermal_data):
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_data, "rgb8")
            thermal_image = self.bridge.imgmsg_to_cv2(thermal_data, "rgb8")
 
            # Resize thermal image to match the color image size
            resized_thermal_image = cv2.resize(thermal_image, (color_image.shape[1], color_image.shape[0]))
 
            # Overlay the thermal image on the color image
            overlaid_image = cv2.addWeighted(color_image, 0, resized_thermal_image, 1, 0)
 
            # Convert the overlaid image back to ROS Image message
            overlaid_msg = self.bridge.cv2_to_imgmsg(overlaid_image, "rgb8")
 
            # Set the correct header frame_id and timestamp
            overlaid_msg.header.frame_id = 'camera_color_optical_frame'
            overlaid_msg.header.stamp = color_data.header.stamp  # Use the color image timestamp
 
            # Publish the image
            # self.get_logger().info('Publishing an image')
            self.image_pub.publish(overlaid_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
 
def main(args=None):
    rclpy.init(args=args)
    overlay = ThermalToColorOverlay()
    try:
        rclpy.spin(overlay)
    except KeyboardInterrupt:
        overlay.get_logger().info('Shutting down thermal to color overlay node')
    finally:
        overlay.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()