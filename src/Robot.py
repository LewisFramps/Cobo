#!/usr/bin/env python3
from time import sleep
from States import ReadyForMovement
from time import time
import json

#Represents the robot in our system - Its the context class in the State Design Pattern
class Robot:
    
    robot_id = None #Id to uniquely identify the robot
    _controller = None #Reference to the controller (needed to make room reservations etc.)
    _state = None #The current state the robot is in
    batteryLevel = -1 #The current battery level of the robot
    timeOfLastMessage = None #The time (epoch timestamp) of the last message, float

    def __init__(self, id1, startingState, control):
        self._state = startingState
        self._state._context = self
        self.robot_id = id1
        self._controller = control
        self.timeOfLastMessage = time()

    """ Method handles a message that is passed to the robot, simply passes that
    message to the robot's current state

    :param message: A string containing the message (just the message, do not include the robots ID)
    """
    def handleMessage(self, message):
        self.timeOfLastMessage = time()
        self._state.handleMessage(message)

    """Method transitions the robot from its current state to a provided new state. It also may triggers
    actions that related to the change in state.

    :param newState: The state that is being transitioned into
    """
    def transition(self, newState):
        self._state = newState #Updates the state
        self._state._context = self #Includes reference to this object in the state
        print("[STATE CHANGE] Robot: " + self.robot_id + " has transitions to state: " + newState.toString())

        #Below are any actions that need to be carried out based on a change of state
        if newState.toString() == "ReadyForMovement(Booth)":
            self._controller.requestRoom(self, "Booth", True)
        elif newState.toString() == "ReadyForMovement(Exit)":
            self._controller.requestRoom(self, "ExitArea", True)
        elif newState.toString() == "ReadyForMovement(DropOff)":
            self._controller.requestRoom(self, "DropOff", False)
        elif newState.toString() == "ReadyForMovement(Reception)":
            self._controller.requestRoom(self, "Reception", False)
        elif (newState.toString() == "Testing") or (newState.toString() == "AtDropOff") or (newState.toString()== "WaitingAtReception"):
            self._controller.releaseReservations(self, "TransitArea")
        elif (newState.toString() == "AtExit"): #Special case where once the robot has arrived it will wait for a time then head to the drop off
            self._controller.releaseReservations(self, "TransitArea")
            sleep(2)
            self.transition(ReadyForMovement("DropOff"))
        elif newState.toString() == "Movement(Booth)":
            self._controller.releaseReservations(self, "Reception")
        elif newState.toString() == "Movement(Exit)":
            self._controller.releaseReservations(self, "Booth")
        elif newState.toString() == "Movement(DropOff)":
            self._controller.releaseReservations(self, "DropOff")
        elif newState.toString() == "AtDropOff":
            self._controller.StoC_pub.publish(self.robot_id + ",dropOff")
        elif newState.toString() == "WaitingForAssistance":
            self._controller.StoC_pub.publish(self.robot_id + ",help," + self.getLocation(newState.stateAtCall))

    def toDict(self):
        return {"ID":self.robot_id, "Battery Level":self.batteryLevel, "Last Message Time":self.timeOfLastMessage,
            "State":self._state.toDict()}
        
    #Works out the location based on a state
    def getLocation(self,state):
        if state.toString()[0:7] == "Movement":
            return "Transit Area"
        elif state.toString() == "Testing":
            return "Booth:" + self.whichBooth()
        elif state.toString() == "AtExit":
            return "Exit"
        elif state.toString() == "AtDropOff":
            return "DropOff"
        elif state.toString() == "WaitingAtReception":
            return "Reception"
        elif state.toString() == "ReadyForMovement(Booth)":
            return "Reception"
        elif state.toString() == "ReadyForMovement(Exit)":
            return "Booth:" + self.whichBooth()
        elif state.toString() == "ReadyForMovement(DropOff)":
            return "Exit"
        elif state.toString() == "ReadyForMovement(Reception)":
            return "DropOff"
        else:
            return "N/A"

    #Works out what booth this robot is in
    def whichBooth(self):
        for booth in self._controller.booths:
            if booth.hasReservation(self.robot_id):
                return booth.id
        else: 
            return "unknown"