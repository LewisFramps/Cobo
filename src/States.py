#!/usr/bin/env python3
from abc import ABC, abstractmethod
import json

#The abstract state - Represents the interface in the State Design Pattern
class State(ABC):

    _context = None

    @abstractmethod
    def handleMessage(self, message):
        pass

    #Handles some generic messages that don't changed based on state
    def genericHandleMessage(self, message, currentState):
        if message[0:12] == "batteryLevel":
            print("Battery Update")
            self._context.batteryLevel = int(message[13:16]) #Updates battery level in robot
            return True
        elif message == "helpRequired":
            self._context.transition(WaitingForAssistance(currentState))
            return True
        else:
            return False #If it can't handle the message it returns false

    #Function to get a string representation
    @abstractmethod
    def toString(self):
        pass

    #Builds a dictionary representation used to make JSON
    def toDict(self, name, destination):
        if destination == None:
            return {"State":name, "Destination":"N/A"}
        else:
            return {"State":name, "Destination":destination}

#The below classes are all concrete classes for the State Design Pattern

class WaitingAtReception(State):

    def handleMessage(self, message):
        if message == "checkinComplete":
            self._context.transition(ReadyForMovement("Booth"))
        elif message[0:12] == "batteryLevel" and int(message[13:16]) < 20:
            self._context.transition(Charging())
        elif super().genericHandleMessage(message, self): #If the generic answers can handle it then
            return #This can stop
        else:
            print("Message doesn't make sense")

    def toString(self):
        return "WaitingAtReception"

    def toDict(self):
        return super().toDict("WaitingAtReception", None)

class ReadyForMovement(State):

    def __init__(self, dest):
        self.destination = dest

    def handleMessage(self, message):
        if message[0:6] == "moveTo":
            self._context.transition(Movement(self.destination))
        elif super().genericHandleMessage(message,self):
            return
        else:
            print("Message doesn't make sense")

    def toString(self):
        return "ReadyForMovement(" + self.destination + ")"

    def toDict(self):
        return super().toDict("ReadyForMovement", self.destination)

class Charging(State):

    def handleMessage(self, message):
        if message[0:12] == "batteryLevel" and int(message[13:16]) >= 100:
            self._context.transition(WaitingAtReception())
        elif super().genericHandleMessage(message,self):
            return
        else:
            print("Message doesn't make sense")

    def toString(self):
        return "Charging"

    def toDict(self):
        return super().toDict("Charging", None)

class Movement(State):

    def __init__(self, dest):
        self.destination = dest

    def handleMessage(self, message):
        if message == "movementFinished":
            if self.destination == "Booth":
                self._context.transition(Testing())
            elif self.destination == "Exit":
                self._context.transition(AtExit())
            elif self.destination == "DropOff":
                self._context.transition(AtDropOff())
            elif self.destination == "Reception":
                self._context.transition(WaitingAtReception())
            else:
                print("Error in Movement")
        elif super().genericHandleMessage(message, self):
            return
        else:
            print("Message doesn't make sense") 


    def toString(self):
        return "Movement(" + self.destination + ")"

    def toDict(self):
        return super().toDict("Movement", self.destination)

class Testing(State):

    def handleMessage(self,message):
        if message == "testingComplete":
            self._context.transition(ReadyForMovement("Exit"))
        elif super().genericHandleMessage(message, self):
            return
        else:
            print("Message doesn't make sense") 

    def toString(self):
        return "Testing"

    def toDict(self):
        return super().toDict("Testing", None)

class AtExit(State):

    #Note that this state has no specific messages it handles, this is because it is only held here for a few seconds
    #The handleMessage function in the robot deals with this
    def handleMessage(self,message):
        if super().genericHandleMessage(message, self):
            return
        else:
            print("Message doesn't make sense") 

    def toString(self):
        return "AtExit"

    def toDict(self):
        return super().toDict("AtExit", None)

class AtDropOff(State):

    def handleMessage(self,message):
        if message == "dropOffComplete":
            self._context.transition(ReadyForMovement("Reception"))
        elif super().genericHandleMessage(message, self):
            return
        else:
            print("Message doesn't make sense") 

    def toString(self):
        return "AtDropOff"

    def toDict(self):
        return super().toDict("AtDropOff", None)

class WaitingForAssistance(State):

    def __init__(self, stateAtCall):
        self.stateAtCall = stateAtCall

    def handleMessage(self, message):
        if message == "helpCompleteRestart":
            self._context.transition(ReadyForMovement("Reception"))
        elif message == "helpCompleteContinue":
            self._context.transition(self.stateAtCall)
        elif super().genericHandleMessage(message, self):
            return
        else:
            print("Message doesn't make sense")

    def toString(self):
        return "WaitingForAssistance"

    def toDict(self):
        return super().toDict("WaitingForAssistance", None)
