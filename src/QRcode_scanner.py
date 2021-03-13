import cv2
import pyqrcode

#############################################
# below this line is the code for QRCode Scanning
#############################################
# do a bit of cleanup
cv2.destroyAllWindows()
cam = cv2.VideoCapture(0)
detector = cv2.QRCodeDetector()
while True:
    #frame = vs.read()
    #frame = imutils.resize(frame, width=1200)
    _, img = cam.read()
    out, data, _ = detector.detectAndDecode(img)
    #if data:
    print(out)
    cv2.imshow("img",img)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("f"):
        break

cam.release()
cv2.destroyAllWindows()
