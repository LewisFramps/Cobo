#!/usr/bin/python3
import rospy
import sys
from std_msgs.msg import String
from sdp.srv import getState
import time

class SpeechNode:

    def __init__(self):
        self.r_to_s_pub = rospy.Publisher("RtoS", String, queue_size=20)
        rospy.init_node("SpeechNode")
        print("Node initialized")
        pass


    def publishToServer(self, message):
        print(f'sending {message} to server')
        Msg = String(data=message)
        self.r_to_s_pub.publish(Msg)

    def getStateClient(self, robot_id):
        rospy.wait_for_service("getState")
        getStateFunc = rospy.ServiceProxy("getState", getState)
        reponse = getStateFunc(robot_id)
        return reponse.state

    def testCallback(self):
        print("asdas")
        pass

def main(args):
    instance = SpeechNode()
    print('started listening')
    print('noise detected...')
    print('intentName is humanAssistance')
    state = instance.getStateClient('a')
    print(f'state is {state}')
    instance.publishToServer(f'a,helpRequired,{state}')
    time.sleep(5)
    print('noise detected...')
    print('intentName is StaffCommandContinue')
    state = instance.getStateClient('a')
    print(f'state is {state}')
    instance.publishToServer(f'a,helpCompleteContinue,{state}')

    # rospy.Timer(rospy.Duration(5), instance.testCallback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

if __name__ == "__main__":
    main(sys.argv)
