import numpy as np
import cv2
import math
import time

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
            blobs.append({"contour": contour, "area": area, "centroid": centroid})
            maxArea = max(maxArea, area)
        
        # prune blobs
        blobs = [b for b in blobs if b["area"] > maxArea*0.30]
        
        # return the final list
        return blobs
    blobs = getBlobs(mask)
    
    # convert contours to a mask for line detection
    contourMask = np.zeros((height-2,width-2,1), np.uint8)
    for b in blobs:
        cv2.drawContours(contourMask, [b["contour"]], 0, (255), offset=(-1,-1))
    contourMask = cv2.copyMakeBorder(contourMask, 1,1,1,1, cv2.BORDER_CONSTANT)
    #cv2.imshow("contourMask", contourMask)
    
    # binary search & Hough transform to find the two best lines
    NUM_LINES = 2
    lines = []
    minT = 20
    maxT = 200
    numIters = 0
    while maxT > minT+1:
        # detect lines with the current threshold
        midT = (minT+maxT)//2
        lines = cv2.HoughLines(contourMask, 3, np.pi*0.1/180, midT)
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
            print(numIters, flush=True)
            break
    
    if len(lines) == NUM_LINES:
        updateAngle(*[90 - l[1]*180/math.pi for l in lines])
    
    
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
    
    # blow up image for easier viewing
    output = cv2.resize(output, (0,0), fx=VIEW_SCALE, fy=VIEW_SCALE, interpolation=cv2.INTER_NEAREST)
    
    # FPS/debug text
    global dt, fps
    debugStr = ""
    if dt is not None:
        debugStr += f"{int(dt*1000)} ms"
    if fps is not None:
        debugStr += f" ({int(fps)} FPS)"
    cv2.putText(output, debugStr, (10,int(height*VIEW_SCALE)-10), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, (0,255,0), lineType=cv2.LINE_AA)
    
    def drawText(text, x, y, color, size=0.4, fromM=0):
        textSz, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size, 1)
        y += int(textSz[1]*fromM)
        cv2.putText(output, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    size, color, 1, cv2.LINE_AA)
    
    # RESET text
    # resetText = "[Space] to reset"
    # textSz, _ = cv2.getTextSize(resetText, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
    # cv2.rectangle(output, (0, 0), tuple(v + 10 for v in textSz), (255,255,255), cv2.FILLED)
    # cv2.putText(output, resetText, (5, 5+textSz[1]), cv2.FONT_HERSHEY_SIMPLEX,
    #             0.4, (0,0,0), 1, cv2.LINE_AA)
    
    # visualize the detected angle and state
    angle = getAngle()
    if angle is not None:
        cv2.rectangle(output, (0, 0), (50, 50), (255,255,255), cv2.FILLED)
        rads = angle*math.pi/180
        offX = 25*math.cos(rads)
        offY = -25*math.sin(rads)
        cv2.line(output, (int(25-offX),int(25-offY)), (int(25+offX),int(25+offY)), (0,0,0), 1, cv2.LINE_AA)
        drawText(f"angle = {int(angle*10)/10} deg", 60, 25, (0,255,0), fromM=0.5)
        if waitingForSteady:
            drawText(f"STEADYING", 5, 60, (0,0,255), fromM=1)
    
    cv2.imshow("output", output)



#########################################
############ angle filtering ############
#########################################

# constants (TODO: tune these)
MAX_SCALE_SPEED = 10.0 # maximum normal movement speed (degrees per second)
STEADY_HISTORY = 2.0 # amount of history to consider (seconds)
STEADY_THRESHOLD = 4.0 # angle variation considered "steady" (degrees)

curAngle = 0
zeroPoint = 0
lastUpdate = None
lastAngles = []
waitingForSteady = True

def isSteady():
    if len(lastAngles) == 0: return False
    angles = [e[1] for e in lastAngles]
    minA = min(angles)
    maxA = max(angles)
    return maxA - minA < STEADY_THRESHOLD

def updateAngle(a1, a2):
    global curAngle, zeroPoint, lastUpdate, lastAngles, waitingForSteady
    now = time.perf_counter()
    if lastUpdate is None: lastUpdate = now
    dt = now - lastUpdate
    lastUpdate = now
    
    newAngle = (a1+a2)/2
    delta = newAngle - curAngle
    
    # update history
    lastAngles.append((now, newAngle))
    while now - lastAngles[0][0] > STEADY_HISTORY:
        lastAngles.pop(0)
    
    # if it's moving too fast, stop updating until it's steady again
    if abs(delta) > MAX_SCALE_SPEED*dt:
        waitingForSteady = True
    if waitingForSteady and not isSteady():
        return
    waitingForSteady = False
    
    curAngle += min(max(delta, -MAX_SCALE_SPEED*dt), +MAX_SCALE_SPEED*dt)

def getRawAngle():
    return curAngle

def getAngle():
    return curAngle - zeroPoint

def zeroAngle():
    global zeroPoint
    angle = getRawAngle()
    if angle is not None:
        zeroPoint = angle



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
    global curFrame, minColor, maxColor
    h, w = curFrame.shape[:2]
    x = int(x/VIEW_SCALE)
    y = int(y/VIEW_SCALE)
    if x >= w or y >= h:
        return
    
    color = curFrame[y, x]
    newMinColor = (float(color[0])-H_PAD, float(color[1])-S_PAD, float(color[2])-V_PAD)
    newMaxColor = (float(color[0])+H_PAD, float(color[1])+S_PAD, float(color[2])+V_PAD)
    if event == cv2.EVENT_RBUTTONDOWN or ((event == cv2.EVENT_LBUTTONDOWN) and maxColor == (0,0,0)):
        minColor = newMinColor
        maxColor = newMaxColor
    if event == cv2.EVENT_LBUTTONDOWN or flags & cv2.EVENT_FLAG_LBUTTON != 0:
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
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
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
    
    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break
    elif key != 0xFF:
        onKey(key)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()