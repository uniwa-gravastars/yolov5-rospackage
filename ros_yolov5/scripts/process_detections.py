#!/home/paris/venv/bin/python

import rospy
from std_msgs.msg import String


class process_detections:
    def __init__(self):
        rospy.init_node("process_detections")
        rospy.loginfo(f"Node {rospy.get_name()} initiated")

        self.detection_counter_sub = rospy.Subscriber(
            rospy.get_param("~detection_counter_topic", "/object_detection/counter"),
            String,
            self.process_detections_callback,
        )

        self.people_counter_pub = rospy.Publisher(
            "/object_detection/people_counter", String, queue_size=1
        )

        self.objs = ["people"]

    def process_detections_callback(self, data):
        try:
            # get people
            detections = eval(data.data)
            people_counter = detections.get("person", 0)
            self.people_counter_pub.publish(str(people_counter))

        except (TypeError, SyntaxError):
            self.people_counter_pub.publish(str(0))
            pass

        detections = eval(data.data)
        objsSet = set(self.objs)
        exclude_keys = ["person"]
        no_people_dictSet = set(
            {k: detections[k] for k in set(list(detections.keys())) - set(exclude_keys)}
        )

        if objsSet.intersection(no_people_dictSet):
            # if any obj is detected
            print(objsSet.intersection(no_people_dictSet))
            print("true")
        else:
            print("false")


if __name__ == "__main__":
    process_detections()
    rospy.spin()
