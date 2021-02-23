#!/usr/bin/env python3
import rospy
import sys
from sdp.srv import booking
from hashlib import sha512
import pyqrcode

#System for checking the bookings
class Bookings:

    bookingsList = list()

    def __init__(self):
        rospy.init_node("bookings")
        self.bookingCheck = rospy.Service("checkBooking", booking, self.checkBooking)
        self.generateHashAndQRCodes()

    def checkBooking(self,data):
        database = open("/home/martin/hashes.txt", 'r')
        for line in database:
            if data.bookingDetails == str.rstrip(line): #Checks if the given data is on the list
                return True
        return False

    def generateHashAndQRCodes(self):
        details = open("/home/martin/bookings.csv", 'r') #Input of booking details
        database = open("/home/martin/hashes.txt", 'w') #Where the resulting hashes will be stored
        for index,line in enumerate(details): #For each booking
            print(index)
            booking = str.rstrip(line)
            hashObject = sha512(bytes(booking, 'utf-8')) #Hashing object is passed the bytes of the booking details
            hexCode = hashObject.hexdigest() #Hex of the hash is created
            print(hexCode)
            database.write(hexCode + "\n") #Hex written to database
            qr = pyqrcode.create(hexCode) #QR code created
            with open(str(index) + '.png', 'wb') as fstream:
                qr.png(fstream, scale=10) #QR code .png created and stored
        details.close()
        database.close()
        #DELETE THE DETAILS


def main(args):
    books = Bookings()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

if __name__ == "__main__":
    main(sys.argv)