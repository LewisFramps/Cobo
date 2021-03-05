#!/usr/bin/env python3
from threading import Lock
from abc import ABC, abstractmethod
class Room(ABC):

    id = None
    capacityHuman = -1
    capacityRobot = -1
    transitArea = None
    humans = None #Holds the robot that has reserved the area with a human
    robots = None #Holds the robots that have reserved the area without a human
    lock = None   #Lock that prevents a room from having more than 1 thread reserving a room which can cause issues

    """Reserves the room for the given robot if there is capcity

    :param robot: A Robot object that want to reserve the room
    :param hasHuman: Boolean, true if there is a human following the robot, false if robot is alone
    """
    def reserve(self, robot, hasHuman):
        self.lock.acquire()
        if (robot in self.humans) or (robot in self.robots): #If it is already reserved
            return True
        if hasHuman:
            if len(self.humans) < self.capacityHuman: #If there is capcity for a human
                self.humans.append(robot)
                self.lock.release()
                return True
            else:
                self.lock.release()
                return False
        else:
            if len(self.robots) + len(self.humans) < self.capacityRobot: #If there is room for a robot, must consider the robots with humans
                self.robots.append(robot)
                self.lock.release()
                return True
            else:
                self.lock.release()
                return False

    def clearReservation(self, robot):
        self.lock.acquire()
        if robot in self.humans:
            self.humans.remove(robot)
        if robot in self.robots:
            self.robots.remove(robot)
        self.lock.release()

    def hasCapcity(self, robot, hasHuman):
        if hasHuman:
            if len(self.humans) < self.capacityHuman:
                return True
            else:
                return False
        else:
            if len(self.robots) + len(self.humans) < self.capacityRobot:
                return True
            else:
                return False

    def hasReservation(self, robot, hasHuman):
        if hasHuman:
            if (robot in self.humans):
                return True
            else:
                return False
        else:
            if robot in self.robots:
                return True
            else:
                return False

    def toDict(self):
        robotsWithHumans = list()
        robotsWithoutHumans = list()
        for robot in self.humans:
            robotsWithHumans.append(robot.robot_id)
        for robot in self.robots:
            robotsWithoutHumans.append(robot.robot_id)
        return {"ID":self.id, "Capcity, Humans":self.capacityHuman, "Capacity Robots":self.capacityRobot,
            "Robots with humans":robotsWithHumans, "Robots without Humans":robotsWithoutHumans}

class Booth(Room):

    def __init__(self, id1, transit):
        self.id = id1
        self.capacityHuman = 1
        self.capacityRobot = 1
        self.transitArea = transit
        self.humans = list()
        self.robots = list()
        self.lock = Lock()

class Reception(Room):

    def __init__(self, id1, transit):
        self.id = id1
        self.capacityHuman = 100
        self.capacityRobot = 100
        self.transitArea = transit
        self.humans = list()
        self.robots = list()
        self.lock = Lock()


class DropOff(Room):

    def __init__(self, id1, transit):
        self.id = id1
        self.capacityHuman = 0
        self.capacityRobot = 100
        self.transitArea = transit
        self.humans = list()
        self.robots = list()
        self.lock = Lock()

class TransitArea(Room):

    def __init__(self, id1):
        self.id = id1
        self.capacityHuman = 1
        self.capacityRobot = 100
        self.humans = list()
        self.robots = list()
        self.lock = Lock()

class ExitArea(Room):

    def __init__(self, id1, transit):
        self.id = id1
        self.capacityHuman = 1
        self.capacityRobot = 100
        self.transitArea = transit
        self.humans = list()
        self.robots = list()
        self.lock = Lock()