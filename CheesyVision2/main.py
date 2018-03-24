import numpy as np
import cv2
from networktables import NetworkTables

import math
import time
import collections
import socket

def fadeHSV(image, mask):
    fade = cv2.multiply(image, (0.3,))
    cv2.subtract(image, fade, image, cv2.bitwise_not(mask))

VIEW_SCALE = 2.0

def process(input):
    height, width = input.shape[:2]
    #cv2.imshow("input", input)
    
    # downsample
    #DOWNSCALE = 0.4
    #input = cv2.resize(input, (0,0), fx=DOWNSCALE, fy=DOWNSCALE)
    #print(f"downsampled input size: [{width}, {height}]")
    
    # convert to HSV
    hsv = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
    cv2.medianBlur(hsv, 5, hsv)
    global curFrame
    curFrame = hsv
    
    # threshold
    global minColor, maxColor
    mask = cv2.inRange(hsv, minColor, maxColor)
    #mask = cv2.inRange(hsv, (110, 30, 30), (140, 255, 255))
    #mask = cv2.inRange(hsv, (130, 130, 80), (160, 255, 255)) # actual
    #cv2.imshow("raw mask", mask)
    
    # reduce noise with morphology
    def getKernel(size):
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size,size))
    KERNEL_SIZE = 4#10
    cv2.morphologyEx(mask, cv2.MORPH_CLOSE, getKernel(KERNEL_SIZE), mask)
    cv2.morphologyEx(mask, cv2.MORPH_OPEN,  getKernel(KERNEL_SIZE//2), mask)
    #cv2.imshow("mask", mask)
    
    # find & filter blobs in the mask
    def getMedian(contour):
        xs, ys = zip(*(p[0] for p in contour))
        medianX = (max(xs) + min(xs)) / 2
        medianY = (max(ys) + min(ys)) / 2
        return (medianX, medianY)
    def getBlobs(mask):
        # find contours
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # construct list of blobs
        blobs = []
        maxArea = 0
        for contour in contours:
            m = cv2.moments(contour)
            area = m["m00"]
            centroid = (m["m10"]/m["m00"], m["m01"]/m["m00"]) if area>0 else (math.inf, math.inf)
            median = getMedian(contour)
            blobs.append({"contour": contour, "area": area, "centroid": centroid, "median": median})
            maxArea = max(maxArea, area)
        
        # prune blobs
        blobs = [b for b in blobs if b["area"] > maxArea*0.30]
        
        # return the final list
        return blobs
    blobs = getBlobs(mask)
    
    # compute angles from blob centers to the pivot
    centerAngles = []
    for b in blobs:
        x, y = b["median"]
        dx, dy = x-pivotLoc[0], y-pivotLoc[1]
        if dx == 0: continue
        angle = -math.degrees(math.atan(dy/dx))
        centerAngles.append(angle)
    
    # convert contours to a mask for line detection
    contourMask = np.zeros((height-2,width-2,1), np.uint8)
    for b in blobs:
        cv2.drawContours(contourMask, [b["contour"]], 0, (255), offset=(-1,-1))
    contourMask = cv2.copyMakeBorder(contourMask, 1,1,1,1, cv2.BORDER_CONSTANT)
    #cv2.imshow("contourMask", contourMask)
    
    # binary search & Hough transform to find the two best lines
    NUM_LINES = 2
    lines = []
    minT = 70
    maxT = 266
    numIters = 0
    while maxT > minT+1:
        # detect lines with the current threshold
        midT = (minT+maxT)//2
        lines = cv2.HoughLines(contourMask, 5, np.pi*0.1/180, midT)
        lines = [] if lines is None else [a[0] for a in lines]
        
        # remove clearly invalid lines
        lines = [l for l in lines if l[1] > np.pi/4 and l[1] < np.pi*3/4]
        
        # remove close-dulplicate lines
        BUCKET_SIZE = 15
        rBuckets = [False]*(1 + int(math.hypot(width, height)/BUCKET_SIZE))
        def keepLine(l):
            bi1 = int(l[0]//BUCKET_SIZE)+1
            bi2 = bi1-1 if l[0]%BUCKET_SIZE<BUCKET_SIZE/2 else bi1+1
            if rBuckets[bi1] or rBuckets[bi2]:
                return False
            rBuckets[bi1] = True
            rBuckets[bi2] = True
            return True
        lines = [l for l in lines if keepLine(l)]
        
        # update/end
        numIters += 1
        if len(lines) < NUM_LINES:
            maxT = midT
        elif len(lines) > NUM_LINES:
            minT = midT
        else:
            # print(numIters, midT)
            break
    
    # send angle data to be processed
    lineAngles = None
    if len(lines) == NUM_LINES:
        lineAngles = (90 - math.degrees(l[1]) for l in lines)
    updateAngle(lineAngles, centerAngles)
    
    
    ### draw debug info onto the input image and show it ###
    output = input
    fadeHSV(output, mask)
    
    # detected lines
    for l in lines:
        a = math.cos(l[1])
        b = math.sin(l[1])
        x0 = a*l[0]
        y0 = b*l[0]
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
        cv2.line(output, (x1,y1), (x2,y2), (255, 230, 0), 1)
    
    # blobs/contours being used for detection
    for b in blobs:
        cv2.drawContours(output, [b["contour"]], 0, (0, 255, 0), 1)
        cv2.drawMarker(output, tuple(int(v) for v in b["centroid"]), (255,0,255), cv2.MARKER_CROSS)
        cv2.drawMarker(output, tuple(int(v) for v in b["median"]), (0,255,255), cv2.MARKER_CROSS)
    
    # scale pivot location
    cv2.circle(output, pivotLoc, 3, (255,230,0), lineType=cv2.LINE_AA)
    
    # blow up image for easier viewing
    output = cv2.resize(output, (0,0), fx=VIEW_SCALE, fy=VIEW_SCALE, interpolation=cv2.INTER_NEAREST)
    
    def drawText(text, x, y, color, size=0.4, fromM=0):
        textSz, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size, 1)
        y += int(textSz[1]*fromM)
        cv2.putText(output, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    size, color, 1, cv2.LINE_AA)
    
    # FPS/debug text
    global dt, fps
    debugStr = ""
    if dt is not None:
        debugStr += f"{int(dt*1000)} ms"
    if fps is not None:
        debugStr += f" ({int(fps)} FPS)"
    drawText(debugStr, 10, int(height*VIEW_SCALE)-10, (0,255,0))
    
    # visualize the detected angle and state
    angle = getAngle()
    if angle is not None:
        PRE_SZ = 50
        cv2.rectangle(output, (0, 0), (PRE_SZ, PRE_SZ), (255,255,255), cv2.FILLED)
        rads = math.radians(angle)
        offX = 200*math.cos(rads)
        offY = -200*math.sin(rads)
        cv2.line(output[0:PRE_SZ, 0:PRE_SZ], (int(PRE_SZ/2-offX),int(PRE_SZ/2-offY)), (int(PRE_SZ/2+offX),int(PRE_SZ/2+offY)), (0,0,0), lineType=cv2.LINE_AA)
        drawText(f"angle = {int(angle*10)/10} deg  (tip = {getTip()})", 60, PRE_SZ//2, (0,255,0), fromM=0.5)
        if errorMsg is not None:
            drawText(errorMsg, 5, 60, (0,0,255), fromM=1)
    
    cv2.imshow("output", output)



#########################################
############ angle filtering ############
#########################################

# constants (TODO: tune these)
MAX_SCALE_SPEED = 60.0 # maximum normal movement speed (degrees per second)
STEADY_HISTORY = 1.0 # amount of history to consider (seconds)
STEADY_THRESHOLD = 4.0 # angle variation considered "steady" (degrees)
MAX_SKEW = 10.0 # maximum skew between the top & bottom lines (degrees)
CENTER_ANGLE_REJECT_THRESHOLD = 7.0 # reject blob-center-angles farther off than this (degrees)
TIPPED_THRESHOLD = 5.0 # angle at which the scale is "tipped" (degrees)

curAngle = 0
zeroPoint = 0
lastUpdate = None
lastAngles = collections.deque()
waitingForSteady = True

errorMsg = None

def isSteady():
    if len(lastAngles) == 0: return False
    angles = [e[1] for e in lastAngles]
    minA = min(angles)
    maxA = max(angles)
    return maxA - minA < STEADY_THRESHOLD

def updateAngle(lineAngles, centerAngles):
    global curAngle, zeroPoint, lastUpdate, lastAngles, waitingForSteady, errorMsg
    errorMsg = None
    
    # calculate dt
    now = time.perf_counter()
    if lastUpdate is None: lastUpdate = now
    dt = now - lastUpdate
    lastUpdate = now
    
    maxDelta = MAX_SCALE_SPEED*dt
    newAngle = None
    
    # use the the angles from the lines, if available
    if lineAngles is None:
        errorMsg = "LINES FAILED"
    else:
        a1, a2 = lineAngles
        if abs(a1 - a2) > MAX_SKEW:
            errorMsg = "SKEWED"
        else:
            newAngle = (a1 + a2) / 2
    
    # fall back to angles from centers of blobs, if necessary
    if newAngle is None:
        centerAngles = [a for a in centerAngles if abs(a-curAngle) < CENTER_ANGLE_REJECT_THRESHOLD]
        if len(centerAngles) > 0:
            newAngle = sum(centerAngles)/len(centerAngles)
        else:
            errorMsg = "NO GOOD DATA"
            return
    
    delta = newAngle - curAngle
    
    # update history
    lastAngles.append((now, newAngle))
    while now - lastAngles[0][0] > STEADY_HISTORY:
        lastAngles.popleft()
    
    # if it's moving too fast, stop updating until it's steady again
    if abs(delta) > maxDelta:
        waitingForSteady = True
    if waitingForSteady and not isSteady():
        errorMsg = "STEADYING"
        return
    waitingForSteady = False
    
    curAngle += min(max(delta, -maxDelta), +maxDelta)

def getRawAngle():
    return curAngle

SCALE_VIEW_ANGLE = 0 # degrees
COS_SCALE_VIEW_ANGLE = math.cos(math.radians(SCALE_VIEW_ANGLE))
def getAngle():
    screenAngle = math.radians(curAngle - zeroPoint)
    return math.degrees(math.atan(COS_SCALE_VIEW_ANGLE*math.tan(screenAngle)))

def getTip(): # TODO: hysteresis?
    angle = getAngle()
    if angle > +TIPPED_THRESHOLD: return +1
    if angle < -TIPPED_THRESHOLD: return -1
    return 0

def zeroAngle():
    global zeroPoint
    angle = getRawAngle()
    if angle is not None:
        zeroPoint = angle



#########################################
########## robot communication ##########
#########################################

robotIP = None
print("Resolving robot IP...")
try:
    robotIP = socket.gethostbyname("roborio-254-frc.local")
    print(f"    robot IP: {robotIP}")
except:
    print("    failed.")
NetworkTables.initialize(server=robotIP)
smartDashboard = NetworkTables.getTable("SmartDashboard")



#########################################
############### main code ###############
#########################################

global minColor, maxColor
minColor = (0,0,0)
maxColor = (0,0,0)
H_PAD = 10
S_PAD = 30
V_PAD = 30

roi = None
gotROI = False

pivotLoc = None
gotPivotLoc = False

global width, height
def onMouse_raw(event, x, y, flags, param):
    global roi, gotROI, width, height, rawViewScale
    x = int(x/rawViewScale)
    y = int(y/rawViewScale)
    x = min(max(x, 0), width-1)
    y = min(max(y, 0), height-1)
    if event == cv2.EVENT_LBUTTONDOWN:
        roi = (x, y, x, y)
        gotROI = False
    elif event == cv2.EVENT_LBUTTONUP:
        if roi[0] > roi[2]: roi = (roi[2], roi[1], roi[0], roi[3])
        if roi[1] > roi[3]: roi = (roi[0], roi[3], roi[2], roi[1])
        if roi[0] < roi[2] and roi[1] < roi[3]:
            gotROI = True
    
    if flags & cv2.EVENT_FLAG_LBUTTON:
        roi = roi[:2] + (x, y)

def onMouse(event, x, y, flags, param):
    global curFrame, minColor, maxColor, pivotLoc, gotPivotLoc
    h, w = curFrame.shape[:2]
    x = int(x/VIEW_SCALE)
    y = int(y/VIEW_SCALE)
    if x >= w or y >= h:
        return
    
    leftDown  = flags & cv2.EVENT_FLAG_LBUTTON != 0
    shiftDown = flags & cv2.EVENT_FLAG_SHIFTKEY != 0
    if not gotPivotLoc or shiftDown:
        if leftDown:
            pivotLoc = (x, y)
        if event == cv2.EVENT_LBUTTONUP:
            gotPivotLoc = True
    else:
        color = curFrame[y, x]
        newMinColor = (float(color[0])-H_PAD, float(color[1])-S_PAD, float(color[2])-V_PAD)
        newMaxColor = (float(color[0])+H_PAD, float(color[1])+S_PAD, float(color[2])+V_PAD)
        if event == cv2.EVENT_RBUTTONDOWN or ((event == cv2.EVENT_LBUTTONDOWN) and maxColor == (0,0,0)):
            minColor = newMinColor
            maxColor = newMaxColor
        if event == cv2.EVENT_LBUTTONDOWN or leftDown:
            minColor = tuple(map(min, minColor, newMinColor))
            maxColor = tuple(map(max, maxColor, newMaxColor))

def onKey(key):
    global minColor, maxColor
    print(f"key: {key}")
    if key == 32:
        minColor = (0,0,0)
        maxColor = (0,0,0)
    if key == ord("z") or key == ord("Z"):
        zeroAngle()

######### VideoCapture #########
CAPTURE_DEVICE = 2
def initCapture():
    cap = cv2.VideoCapture(CAPTURE_DEVICE)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cap
cap = initCapture()
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

global dt, fps
dt = fps = None
frameCount = 0
lastSecond = time.perf_counter()

global rawViewScale
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # frame = cv2.imread("inputP1.jpg")
    height, width = frame.shape[:2]
    # print([width, height])
    
    def isFrameOK():
        if not ret:
            return False
        for i in [0,1,2]:
            if cv2.countNonZero(frame[:,:,i]) > 0:
                return True
        return False
    if not isFrameOK():
        print("FRAME IS NOT OK, ret:", ret)
        cap.release()
        cap = initCapture() # reopen the VideoCapture
    
    # show the raw frame (with ROI rect)
    frameDisp = frame.copy()
    if roi is not None:
        cv2.rectangle(frameDisp, roi[:2], roi[2:], (0, 0, 255), 2)
    rawViewScale = 1.0#800/width
    # frameDisp = cv2.resize(frameDisp, (800, 450), interpolation=cv2.INTER_NEAREST)
    cv2.imshow("raw", frameDisp)
    cv2.setMouseCallback("raw", onMouse_raw)
    
    if gotROI:
        start = time.perf_counter()
        process(frame[roi[1]:roi[3], roi[0]:roi[2]])
        end = time.perf_counter()
        dt = end - start
        
        frameCount += 1
        if time.perf_counter() - lastSecond > 1.0:
            lastSecond += 1.0
            fps = frameCount
            frameCount = 0
        
        cv2.setMouseCallback("output", onMouse)
    
    smartDashboard.putNumber("scaleAngle", getAngle())
    smartDashboard.putNumber("scaleTip", getTip())
    if not NetworkTables.isConnected():
        print("NetworkTables not connected")
    
    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break
    elif key != 0xFF:
        onKey(key)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()