#!/home/paris/venv/bin/python

import json
import os
from collections import Counter

import cv2
import numpy as np
import rospkg
import rospy
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ObjectDetector:
    def __init__(self):
        # Initialise ROS node
        rospy.init_node("yolov5_object_detector", anonymous=False)

        # Set object detection mode
        as_a_service = rospy.get_param("as_a_service", False)

        # Set YOLOV5 Parameters
        rospack = rospkg.RosPack()
        ROS_YOLOV5_PATH = rospack.get_path("ros_yolov5")
        YOLO_PATH = rospy.get_param("~yolo_path", os.path.join(ROS_YOLOV5_PATH, "yolov5"))
        WEIGHTS_PATH = rospy.get_param(
            "~weights_path", os.path.join(ROS_YOLOV5_PATH, "weights", "yolov5m.pt")
        )
        self.model = torch.hub.load(YOLO_PATH, "custom", WEIGHTS_PATH, source="local")

        # Configure detection parameters and publishers
        detection_queue = 1
        self.image_pub = rospy.Publisher(
            rospy.get_param("~detection_image_topic", "/object_detection/image"),
            Image,
            queue_size=detection_queue,
        )
        self.detection_objects_pub = rospy.Publisher(
            rospy.get_param("~detection_objects_topic", "/object_detection/objects"),
            String,
            queue_size=detection_queue,
        )
        self.detection_counter_pub = rospy.Publisher(
            rospy.get_param("~detection_counter_topic", "/object_detection/counter"),
            String,
            queue_size=detection_queue,
        )
        self.detection_names_pub = rospy.Publisher(
            rospy.get_param("~detection_names_topic", "/object_detection/names"),
            String,
            queue_size=detection_queue,
        )

        # Configure camera subscription
        source_topic = rospy.get_param("~source_topic", "/rgb/image_raw")
        source_queue = rospy.get_param("~source_queue", 1)
        self.image_sub = rospy.Subscriber(
            source_topic,
            Image,
            callback=self.image_callback,
            queue_size=source_queue,
            buff_size=2**24,
        )

        # Output initialization messages
        rospy.loginfo(f"Node {rospy.get_name()} initiated")
        rospy.loginfo(f"Started as a service: {as_a_service}")
        rospy.loginfo(f"Yolov5 path: {YOLO_PATH}")
        rospy.loginfo(f"Yolov5 weights path: {WEIGHTS_PATH}")
        rospy.loginfo(f"Source img topic: {source_topic}")
        rospy.loginfo(
            "Annotated img topic:"
            f" {rospy.get_param('~detection_image_topic', '/object_detection/image')}"
        )
        rospy.loginfo(
            "Detection counter topic:"
            f" {rospy.get_param('~detection_counter_topic', '/object_detection/counter')}"
        )
        rospy.loginfo(
            "Detection names topic:"
            f" {rospy.get_param('~detection_names_topic', '/object_detection/names')}"
        )
        rospy.loginfo(
            "Detection objects topic:"
            f" {rospy.get_param('~detection_objects_topic', '/object_detection/objects')}"
        )

        rospy.spin()

    def draw_text(
        self,
        img,
        text,
        pos=(0, 0),
        font=cv2.FONT_HERSHEY_PLAIN,
        font_scale=1,
        thickness=2,
        color=(0, 0, 0),
        bg_color=(0, 255, 0),
    ):
        x, y = pos
        text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
        cv2.rectangle(img, pos, (x + text_size[0], y + text_size[1]), bg_color, -1)
        cv2.putText(
            img,
            text,
            (x, y + text_size[1] + font_scale - 1),
            font,
            font_scale,
            color,
            thickness,
        )
        return text_size

    def image_callback(self, msg):
        # Get class names from model
        names = self.model.names

        # Convert image message to numpy array
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape((msg.height, msg.width, 3))

        # Perform object detection on the image
        results = self.model(img, size=640)
        detections = results.pred[0].detach().cpu().numpy()
        detected_names = []

        # Draw bounding boxes and labels on image for each detected object
        for detection in detections:
            start, end, conf, cls = (
                detection[:2],
                detection[2:4],
                detection[4],
                int(detection[5]),
            )
            start, end = tuple(start.astype(int)), tuple(end.astype(int))
            cv2.rectangle(img, start, end, (0, 255, 0), 2)
            name = names[cls]
            label = f"{name}: {conf:.2f}"
            self.draw_text(img, label, pos=(start[0], start[1] - 10))

            # Keep track of detected object names
            detected_names.append(name)

        # Publish detected object information
        detected_objects = detections.tolist()
        message = json.dumps(detected_names)
        self.detection_names_pub.publish(message)
        message = json.dumps(detected_objects)
        self.detection_objects_pub.publish(message)
        counter = dict(Counter(detected_names))
        message = json.dumps(counter)
        self.detection_counter_pub.publish(message)

        # Publish the modified image
        img_msg = Image(
            height=480,
            width=640,
            encoding="rgb8",
            is_bigendian=0,
            step=1920,
            data=img.flatten().tobytes(),
        )
        self.image_pub.publish(img_msg)

    def main(self):
        rospy.spin()


if __name__ == "__main__":
    od = ObjectDetector()
    od.main()
