#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import String
from sdp.srv import getState

class SpeechNode:

    pi = "pi"

    def __init__(self):
        rospy.init_node("SpeechNode")
        self.r_to_s_pub = rospy.Publisher("RtoS", String, queue_size=20)
        print("Node initialized")
        pass


    def publishToServer(self, message):
        self.r_to_s_pub.publish(message)

    def getStateClient(self, robot_id):
        rospy.wait_for_service("getState")
        getStateFunc = rospy.ServiceProxy("Controller", getState)
        reponse = getStateFunc(robot_id)
        return reponse.state

    def testCallback(self, num):
        print(num)
        return num

def main(args):
    instance = SpeechNode()
    rospy.Timer(rospy.Duration(5), instance.testCallback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

if __name__ == "__main__":
    main(sys.argv)