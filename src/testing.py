#Contains a number of tests for the system
import pytest
from subprocess import Popen
from time import sleep
import rospy
from std_msgs.msg import String
from sdp.srv import getState

#Before running these tests the controller should be launched with the following command:
#rosrun sdp Controller.py a,b,c,d 2 which starts 4 robots with ids a-d and 2 booths

class Testing:

    value = 0

    @classmethod
    def setup_class(cls):
        cls.value = 1
        #Set up this Node
        rospy.init_node("testing")
        cls.RtoS_pub = rospy.Publisher("RtoS", String, queue_size=20)
        cls.CtoS_pub = rospy.Publisher("CtoS", String, queue_size=20)
        cls.getStateService = rospy.ServiceProxy('getState', getState)
        sleep(1)

    #Checks that setup has run correctly
    def testInit(self):
        assert 1 == self.value

    def testInitialState(self):
        response = self.getStateService("a")
        assert response.state == "WaitingAtReception"
        response = self.getStateService("b")
        assert response.state == "WaitingAtReception"
        response = self.getStateService("c")
        assert response.state == "WaitingAtReception"
        response = self.getStateService("d")
        assert response.state == "WaitingAtReception"

    #Tests T1-T7 tests the full rotation through a cycle
    def testT1(self):
        self.RtoS_pub.publish("d,checkinComplete")
        sleep(1)
        response = self.getStateService("d")
        assert response.state == "Movement(Booth)"

    def testT2(self):
        self.RtoS_pub.publish("d,movementFinished")
        sleep(1)
        response = self.getStateService("d")
        assert response.state == "Testing"

    def testT3(self):
        self.RtoS_pub.publish("d,testingComplete")
        sleep(1)
        response = self.getStateService("d")
        assert response.state == "Movement(Exit)"

    def testT4(self):
        self.RtoS_pub.publish("d,movementFinished")
        sleep(4)
        response = self.getStateService("d")
        assert response.state == "Movement(DropOff)"

    def testT5(self):
        self.RtoS_pub.publish("d,movementFinished")
        sleep(1)
        response = self.getStateService("d")
        assert response.state == "AtDropOff"

    def testT6(self):
        self.RtoS_pub.publish("d,dropOffComplete")
        sleep(1)
        response = self.getStateService("d")
        assert response.state == "Movement(Reception)"

    def testT7(self):
        self.RtoS_pub.publish("d,movementFinished")
        sleep(1)
        response = self.getStateService("d")
        assert response.state == "WaitingAtReception"

    #Shouldn't change state as the messages doesn't make sense
    def testNonsense(self):
        response1 = self.getStateService("a").state
        self.RtoS_pub.publish("b,dropOffComplete")
        sleep(1)
        response2 = self.getStateService("a").state
        assert response1 == response2

    #Testing the Help subsystem
    def testHelp1(self):
        #First the robot is moved off the reception
        self.RtoS_pub.publish("b,checkinComplete")
        sleep(1)
        self.RtoS_pub.publish("b,helpRequired") #Help is triggered
        sleep(0.5)
        response = self.getStateService("b").state
        assert response == "WaitingForAssistance" #Check of the help state
        
    def testHelp2(self):
        self.CtoS_pub.publish("b,helpCompleteContinue")
        response = self.getStateService("b").state
        assert response == "Movement(Booth)" #Check of the help state


