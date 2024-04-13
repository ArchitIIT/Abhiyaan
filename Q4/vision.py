import cv2 as cv

#img = cv.imread("")
#cv.imshow("cat",img)

capture = cv.VideoCapture("vid")

while True:
    isTrue,frame= capture.read()

    frame_resized = rescaleFrame(frame)
    cv.imshow("video",frame)
    if cv.waitKey(20) & 0xFF==ord('d'):
        break

capture.release()
cv.destroyAllWindows()
#-215:Assertion failed =bcz open cv could not find frame once video ends

def rescaleFrame(frame,scale=0.75):
    width = int(frame.shape[1]* scale)
    height = int(frame.shape[0]* scale)

    dimensions =(width,height)

    return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)

#readong videos
capture = cv.VideoCapture()