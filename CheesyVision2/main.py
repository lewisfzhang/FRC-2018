import numpy as np
import cv2
from networktables import NetworkTables

import math
import time
import collections
import socket
import argparse
import sys

def fadeHSV(image, mask):
    fade = cv2.multiply(image, (0.3,))
    cv2.subtract(image, fade, image, cv2.bitwise_not(mask))

def processMask(mask, closeSize=4, openSize=2):
    def getKernel(size):
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size,size))
    cv2.morphologyEx(mask, cv2.MORPH_CLOSE, getKernel(closeSize), mask)
    cv2.morphologyEx(mask, cv2.MORPH_OPEN,  getKernel(openSize), mask)

# returns: blobs, hsv, mask
def getBlobs(input):
    # convert to HSV
    hsv = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
    cv2.medianBlur(hsv, 5, hsv)
    
    # threshold
    global minColor, maxColor
    mask = cv2.inRange(hsv, minColor, maxColor)
    
    # reduce noise with morphological operations
    processMask(mask)
    
    # find contours
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # construct list of blobs
    blobs = []
    maxArea = 0
    def getMedian(contour):
        xs, ys = zip(*(p[0] for p in contour))
        medianX = (max(xs) + min(xs)) / 2
        medianY = (max(ys) + min(ys)) / 2
        return (medianX, medianY)
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
    return blobs, hsv, mask

def process(input):
    autoSetColor(input)
    height, width = input.shape[:2]
    
    global curFrame
    blobs, curFrame, mask = getBlobs(input)
    
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
    updateAngle(lineAngles)
    
    
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
    
    # blow up image for easier viewing
    if args.roi_scale != 1.0:
        output = cv2.resize(output, (0,0), fx=args.roi_scale, fy=args.roi_scale, interpolation=cv2.INTER_NEAREST)
    
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
    drawText(debugStr, 10, int(height*args.roi_scale)-10, (0,255,0))
    
    # visualize the detected angle and state
    angle = getAngle()
    if angle is not None:
        PRE_SZ = 50
        cv2.rectangle(output, (0, 0), (PRE_SZ, PRE_SZ), (255,255,255), cv2.FILLED)
        rads = math.radians(angle)
        offX = 200*math.cos(rads)
        offY = -200*math.sin(rads)
        cv2.line(output[0:PRE_SZ, 0:PRE_SZ], (int(PRE_SZ/2-offX),int(PRE_SZ/2-offY)), (int(PRE_SZ/2+offX),int(PRE_SZ/2+offY)), (0,0,0), lineType=cv2.LINE_AA)
        drawText(f"angle = {int(angle*100)/100} deg  (tip = {getTip()})", 60, PRE_SZ//2, (0,255,0), fromM=0.5)
        if errorMsg is not None:
            drawText(errorMsg, 5, 60, (0,0,255), fromM=1)
    
    cv2.imshow("output", output)



#########################################
######### automatic calibration #########
#########################################

def getHistogram(hsv, channel, mask, normMax=255, reduce=3):
    maxV = [180,255,255][channel]
    hist = cv2.calcHist([hsv[:, :, channel]], [0], mask, [maxV//reduce], [0,maxV])
    cv2.normalize(hist, hist, 0, normMax, cv2.NORM_MINMAX)
    return hist

def drawHistogram(hist, markers=[], bestMarker=-1):
    n = hist.shape[0]
    m = 180//n
    histImage = np.zeros((256, n*2, 3), np.uint8)
    hist = np.int32(np.around(hist))
    markers = markers + [bestMarker]
    for x,y in enumerate(hist):
        cv2.line(histImage, (x*2,256), (x*2,256-y), (x*m,255,255))
        cv2.line(histImage, (x*2+1,256), (x*2+1,256-y), (x*m+1,255,255))
        if x in markers:
            color = (0,0,255)
            if x == bestMarker: color = (60,255,255)
            cv2.line(histImage, (x*2,0), (x*2,256-y), color)
    histImage = cv2.cvtColor(histImage, cv2.COLOR_HSV2BGR)
    cv2.imshow("histogram", histImage)

VALID_DEPTH = 7
def getMaxima(hist):
    def isValidMax(i):
        mVal = hist[i]
        left, right = False, False
        for dx in range(1, min(i+1, len(hist)-i)):
            vl, vr = hist[i-dx], hist[i+dx]
            if (vl > mVal and not left) or (vr > mVal and not right): return False
            if vl < mVal-VALID_DEPTH: left = True
            if vr < mVal-VALID_DEPTH: right = True
            if left and right: return True
        return False
    return [i for i in range(1, len(hist)-1) if isValidMax(i)]
def getMinima(hist):
    def isValidMin(i):
        mVal = hist[i]
        left, right = False, False
        for dx in range(1, min(i+1, len(hist)-i)):
            vl, vr = hist[i-dx], hist[i+dx]
            if (vl < mVal and not left) or (vr < mVal and not right): return False
            if vl > mVal+VALID_DEPTH: left = True
            if vr > mVal+VALID_DEPTH: right = True
            if left and right: return True
        return False
    return [i for i in range(1, len(hist)-1) if isValidMin(i)]
def getClosestExtremum(extrema, target):
    return extrema[np.argmin([abs(i-target) for i in extrema])]
def getClosestExtremumLeft(extrema, target, default):
    return max([i for i in extrema if i < target], default=default)
def getClosestExtremumRight(extrema, target, default):
    return min([i for i in extrema if i > target], default=default)

def computeHueRange(hsv):
    # compute hue histogram
    mask = cv2.inRange(hsv, (0, 64, 64), (180, 255, 255))
    hist = getHistogram(hsv, 0, mask, reduce=2)
    
    # find hue range
    maxima = getMaxima(hist)
    minima = getMinima(hist)
    bestMax = getClosestExtremum(maxima, 140 / 2)
    min0 = getClosestExtremumLeft(minima, bestMax, 0)
    min1 = getClosestExtremumRight(minima, bestMax, len(hist))
    
    # visualize the histogram
    drawHistogram(hist, [min0, bestMax, min1], bestMax)
    
    return min0*2, min1*2, bestMax*2

SCALE = 8
histVals = np.zeros((256//SCALE, 256//SCALE))
def svHist(hsv, mask, peakHue):
    # compute histogram
    global histVals
    hsv = hsv[mask > 128]
    weights = hsv[:,0] - peakHue
    weights = 1 / (0.1 + weights*weights)
    hist, _, _ = np.histogram2d(hsv[:,2], hsv[:,1], weights=weights, bins=256//SCALE, range=[[0,255],[0,255]])
    histVals += hist
    
    # normalize
    # hist = histVals # uncomment to show accumulated histogram
    hist = (hist*255/np.max(hist)).astype(np.uint8)
    
    # create mask
    cv2.GaussianBlur(hist, (3,3), 2.5, hist)
    mask = cv2.inRange(hist, 20, 255)
    # cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (2,2)), mask)
    
    # find biggest contour and bounding rectangle
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    biggestI = np.argmax([cv2.contourArea(c) for c in contours])
    x,y,w,h = cv2.boundingRect(contours[biggestI])
    
    # display
    hist = cv2.resize(hist, (512,512), interpolation=cv2.INTER_NEAREST)
    hist = cv2.cvtColor(hist, cv2.COLOR_GRAY2BGR)
    cv2.rectangle(hist, (x*SCALE*2,y*SCALE*2), ((x+w)*SCALE*2,(y+h)*SCALE*2), (0,255,0))
    cv2.imshow("SV histogram", hist)
    
    return x*SCALE, (x+w)*SCALE, y*SCALE, (y+h)*SCALE

def autoSetColor(input):
    start = time.perf_counter()
    
    # downsample
    DOWNSCALE = 1.0
    inputSmall = cv2.resize(input, (0,0), fx=DOWNSCALE, fy=DOWNSCALE)
    # height, width = inputSmall.shape[:2]
    # print(f"downsampled input size: [{width}, {height}]")
    
    # convert to HSV
    hsv = cv2.cvtColor(inputSmall, cv2.COLOR_BGR2HSV)
    
    # get the hue range
    minH, maxH, peakHue = computeHueRange(hsv)
    
    # visualize S-V histogram
    mask = cv2.inRange(hsv, (minH, 64, 64), (maxH, 255, 255))
    minS,maxS,minV,maxV = svHist(hsv, mask, peakHue)
    
    # set color range
    global minColor, maxColor
    minColor = (minH, minS, minV)
    maxColor = (maxH, 255, 255)
    print("auto:", minColor, maxColor)
    
    
    end = time.perf_counter()
    print(f"autoSetColor took {int((end-start)*1000)} ms")



#########################################
############ angle filtering ############
#########################################

# constants (TODO: tune these)
MAX_SCALE_SPEED = 40.0 # maximum normal movement speed (degrees per second)
SMOOTH_HISTORY = 1.0 # amount of history to consider for smoothing (seconds)
SMOOTH_FIT_DEGREE = 2 # degree of polynomial fit for smoothing
STEADY_HISTORY = 1.0 # amount of history to consider for steadiness (seconds)
STEADY_THRESHOLD = 4.0 # angle variation considered "steady" (degrees)
MAX_SKEW = 5.0 # maximum skew between the top & bottom lines (degrees)
TIPPED_THRESHOLD = 3.5 # angle at which the scale is "tipped" (degrees)

# ignore RankWarnings from np.polyfit
import warnings
warnings.simplefilter("ignore", np.RankWarning)

curAngle = 0
zeroPoint = 0
zeroed = False
lastUpdate = None
smoothHistory = collections.deque()
steadyHistory = collections.deque()
waitingForSteady = True

errorMsg = None

def isSteady():
    if len(steadyHistory) == 0: return False
    angles = [e[1] for e in steadyHistory]
    minA = min(angles)
    maxA = max(angles)
    return maxA - minA < STEADY_THRESHOLD

def updateAngle(lineAngles):
    global curAngle, zeroPoint, zeroed, lastUpdate, waitingForSteady, errorMsg
    errorMsg = None
    if not zeroed:
        errorMsg = "NOT ZEROED YET"
    
    # calculate dt
    now = time.perf_counter()
    if lastUpdate is None: lastUpdate = now
    dt = now - lastUpdate
    lastUpdate = now
    
    maxDelta = MAX_SCALE_SPEED*dt
    newAngle = None
    
    # history update functions
    def updateHistory(list, history, value):
        list.append((now, value))
        while now - list[0][0] > history:
            list.popleft()
    def updateSmoothHistory():
        updateHistory(smoothHistory, SMOOTH_HISTORY, curAngle)
    
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
        errorMsg = "NO GOOD DATA"
        updateSmoothHistory()
        return
    
    delta = newAngle - curAngle
    
    # update history
    updateHistory(steadyHistory, STEADY_HISTORY, newAngle)
    
    # if it's moving too fast, stop updating until it's steady again
    if abs(delta) > maxDelta:
        waitingForSteady = True
    if waitingForSteady and not isSteady():
        errorMsg = "STEADYING"
        updateSmoothHistory()
        return
    waitingForSteady = False
    
    curAngle += min(max(delta, -maxDelta), +maxDelta)
    updateSmoothHistory()

def getRawAngle():
    return curAngle

SCALE_VIEW_ANGLE = 0 # degrees
COS_SCALE_VIEW_ANGLE = math.cos(math.radians(SCALE_VIEW_ANGLE))
def getAngle():
    if len(smoothHistory) == 0: return 0.0
    # do a polynomial fit on the history, putting more weight on recent data points
    weights = [x**0 for x in range(1, len(smoothHistory)+1)]
    fitFunc = np.poly1d(np.polyfit(*zip(*smoothHistory), SMOOTH_FIT_DEGREE, w=weights))
    lastTime = smoothHistory[-1][0]
    screenAngle = math.radians(fitFunc(lastTime) - zeroPoint)
    return math.degrees(math.atan(COS_SCALE_VIEW_ANGLE*math.tan(screenAngle)))

def getTip(): # TODO: hysteresis?
    angle = getAngle()
    if angle > +TIPPED_THRESHOLD: return +1
    if angle < -TIPPED_THRESHOLD: return -1
    return 0

def zeroAngle():
    global zeroPoint, zeroed
    angle = getRawAngle()
    if angle is not None:
        zeroPoint = angle
        zeroed = True



##########################################
######### command-line arguments #########
##########################################

parser = argparse.ArgumentParser(description="Program to track the scale arm using OpenCV. (by Quinn Tucker '18)")
parser.add_argument("-n", "--no-network", action="store_true", help="don't initialize/output to NetworkTables")
optGroup = parser.add_mutually_exclusive_group()
optGroup.add_argument("-d", "--device", type=int, default=2, metavar="ID",
                    help="device ID of the camera to use (default: %(default)s)")
optGroup.add_argument("-i", "--input-image", metavar="FILE", help="optional image to use instead of a live camera")
optGroup.add_argument("-v", "--input-video", metavar="FILE", help="optional video to use instead of a live camera")
parser.add_argument("-s", "--scale", type=float, default=1.0, metavar="FACTOR",
                      help="amount to up/downsample each frame (optional)")
parser.add_argument("--raw-scale", type=float, default=1.0, metavar="FACTOR",
                    help="amount to scale the raw frame display by (default: %(default)s)")
parser.add_argument("--roi-scale", type=float, default=2.0, metavar="FACTOR",
                    help="amount to scale the region-of-interest display by (default: %(default)s)")
parser.add_argument("--csv-output", type=argparse.FileType("w"), metavar="FILE",
                    help="optional file to write angle data to")
args = parser.parse_args()



#########################################
########## robot communication ##########
#########################################

if args.no_network:
    print("Skipping NetworkTables initialization")
else:
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
S_PAD = 20
V_PAD = 20

roi = None
gotROI = False

global width, height
def onMouse_raw(event, x, y, flags, param):
    global roi, gotROI, width, height
    x = int(x/args.raw_scale)
    y = int(y/args.raw_scale)
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
    x = int(x/args.roi_scale)
    y = int(y/args.roi_scale)
    
    leftDown  = flags & cv2.EVENT_FLAG_LBUTTON != 0
    rightDown = flags & cv2.EVENT_FLAG_RBUTTON != 0
    if not (leftDown or rightDown):
        return
    
    if event == cv2.EVENT_RBUTTONDOWN:
        maxColor = (0,0,0)
    
    def addPixel(x, y):
        global curFrame, minColor, maxColor
        if x >= w or y >= h:
            return
        color = curFrame[y, x]
        newMinColor = (float(color[0])-H_PAD, float(color[1])-S_PAD, float(color[2])-V_PAD)
        newMaxColor = (float(color[0])+H_PAD, float(color[1])+S_PAD, float(color[2])+V_PAD)
        if maxColor == (0,0,0):
            minColor = newMinColor
            maxColor = newMaxColor
        elif leftDown or rightDown:
            minColor = tuple(map(min, minColor, newMinColor))
            maxColor = tuple(map(max, maxColor, newMaxColor))
    
    BRUSH_R = 4 # radius of brush
    for x2 in range(x-BRUSH_R, x+BRUSH_R+1):
        for y2 in range(y-BRUSH_R, y+BRUSH_R+1):
            addPixel(x2, y2)

def onKey(key):
    global minColor, maxColor
    print(f"key: {key}")
    if key == ord("c") or key == ord("C"):
        minColor = (0,0,0)
        maxColor = (0,0,0)
    if key == ord("z") or key == ord("Z"):
        zeroAngle()
    if key == ord("a") or key == ord("A"):
        if frameROI is not None:
            print("user:", minColor, maxColor)
            autoSetColor(frameROI)
            print()

######### VideoCapture #########
def initCapture():
    if args.input_image is not None: return None
    print("Initializing VideoCapture...")
    if args.input_video is not None:
        cap = cv2.VideoCapture(args.input_video)
    else:
        cap = cv2.VideoCapture(args.device)
        if not cap.isOpened():
            print(f"    failed to open camera device {args.device}")
            sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    print("    done.")
    return cap
cap = initCapture()

if args.input_image is not None:
    inputImage = cv2.imread(args.input_image)
    inputImage = cv2.resize(inputImage, (0,0), fx=args.scale, fy=args.scale)

global dt, fps
dt = fps = None
frameCount = 0
heartbeat = 0
lastSecond = time.perf_counter()

while True:
    # read the next frame and make sure it's valid
    if args.input_image is None:
        ret, frame = cap.read()
        def isFrameOK():
            if not ret or frame is None:
                return False
            for i in [0,1,2]:
                if cv2.countNonZero(frame[:,:,i]) > 0:
                    return True
            return False
        if not isFrameOK():
            print("Got a bad frame, reinitializing.")
            cap.release()
            cap = initCapture() # reopen the VideoCapture
            continue
        if args.scale != 1.0:
            frame = cv2.resize(frame, (0,0), fx=args.scale, fy=args.scale)
    else:
        frame = inputImage.copy()
    
    height, width = frame.shape[:2]
    
    # show the raw frame (with ROI rect)
    frameDisp = frame.copy()
    if args.raw_scale != 1.0:
        frameDisp = cv2.resize(frameDisp, (0,0), fx=args.raw_scale, fy=args.raw_scale)
    if roi is not None:
        sROI = tuple(int(v*args.raw_scale) for v in roi)
        cv2.rectangle(frameDisp, sROI[:2], sROI[2:], (0, 0, 255), 2)
    if not NetworkTables.isConnected():
        cv2.putText(frameDisp, "NetworkTables is not connected", (10, height-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 1, cv2.LINE_AA)
    cv2.imshow("raw", frameDisp)
    cv2.setMouseCallback("raw", onMouse_raw)
    
    if gotROI:
        start = time.perf_counter()
        frameROI = frame[roi[1]:roi[3], roi[0]:roi[2]]
        process(frameROI)
        end = time.perf_counter()
        dt = end - start
        
        frameCount += 1
        if time.perf_counter() - lastSecond > 1.0:
            lastSecond += 1.0
            fps = frameCount
            frameCount = 0
        
        cv2.setMouseCallback("output", onMouse)
        
        if args.csv_output is not None:
            args.csv_output.write(f"{start}, {curAngle}, {getAngle()}, {getTip()}\n")
    else:
        frameROI = None
    
    if not args.no_network:
        smartDashboard.putNumber("scaleAngle", getAngle())
        smartDashboard.putNumber("scaleTip", getTip())
        smartDashboard.putBoolean("scaleError", errorMsg is not None)
        smartDashboard.putNumber("scaleHeartbeat", heartbeat)
        heartbeat += 1
    
    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break
    elif key != 0xFF:
        onKey(key)

# cleanup VideoCapture, windows, and CSV output
if args.input_image is None:
    cap.release()
cv2.destroyAllWindows()
if args.csv_output is not None:
    args.csv_output.close()