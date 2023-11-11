#!/usr/bin/env python3
import os
import numpy as np
import cv2
import time
import rclpy
from rclpy.node import Node

import threading

from sensor_msgs.msg import Image as SensorImage

import subprocess
rtmp_url = "rtp://192.168.17.78:5000/stream"
width = 800
height = 450
fps = 30

camera_topic = "/vehicle/camera/image_color"

# command and params for ffmpeg
command = ['ffmpeg',
           '-y',
           '-f', 'rawvideo',
           '-vcodec', 'rawvideo',
           '-pix_fmt', 'bgr24',
           '-s', "{}x{}".format(width, height),
           '-r', str(fps),
           '-i', '-',
           '-c:v', 'libx264',
           '-pix_fmt', 'yuv420p',
           '-preset', 'ultrafast',
           '-f', 'rtp',
           rtmp_url]
# using subprocess and pipe to fetch frame data
p = subprocess.Popen(command, stdin=subprocess.PIPE)

frame_count = 0

frame = None

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            SensorImage,
            camera_topic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, data):
        global frame, frame_count, _frame, width, height, p

        if frame is not None:
            _frame = cv2.cvtColor(cv2.resize(frame, (800,450)), cv2.COLOR_RGB2BGR)
            p.stdin.write(_frame.tobytes())

        frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        frame_count += 1

        self.get_logger().info(f"Get frame count: {frame_count}")

def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriber()
    camera_subscriber.get_logger().info("Streamer initiated...")

    rclpy.spin(camera_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
