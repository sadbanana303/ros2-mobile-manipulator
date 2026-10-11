#!/usr/bin/env python3
import rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

class Det(Node):
    def __init__(self):
        super().__init__('cube_detector')
        self.b = CvBridge()
        self.vis = self.create_publisher(Bool, '/cube_visible', 10)
        self.dbg = self.create_publisher(Image, '/camera/debug', 10)
        self.create_subscription(Image, '/camera/image_raw', self.cb, 10)
    def cb(self, m):
        f = self.b.imgmsg_to_cv2(m, 'bgr8')
        h = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(cv2.inRange(h, (0, 120, 70), (10, 255, 255)), cv2.inRange(h, (170, 120, 70), (180, 255, 255)))
        ok = cv2.countNonZero(mask) > 40
        if ok:
            x, y, w, hh = cv2.boundingRect(mask)
            cv2.rectangle(f, (x, y), (x + w, y + hh), (0, 255, 0), 2)
        self.vis.publish(Bool(data=ok))
        self.dbg.publish(self.b.cv2_to_imgmsg(f, 'bgr8'))

rclpy.init(); rclpy.spin(Det())
