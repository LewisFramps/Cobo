#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import String
from sdp.srv import getState

class Example:

    pi = "pi"

    def __init__(self):
        rospy.init_node("ExampleNode")
        self.sub1 = rospy.Subscriber("topic1", String, callback=self.callback1, queue_size=20)
        self.pub1 = rospy.Publisher("topic2", String)
        print("Hello World")
        pass

    def callback1(self, data):
        message = data.data
        print(message)
        self.pub1.publish("aaaaaaaaaa")
        self.pi = "e"

    def callback2(self, event):
        rospy.wait_for_service("getState")
        getStateFunc = rospy.ServiceProxy("Controller", getState)
        reponse = getStateFunc("hahahahahah")


def main(args):
    instance = Example()
    rospy.Timer(rospy.Duration(5), instance.callback2)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

if __name__ == "__main__":
    main(sys.argv)